// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using System.Runtime.InteropServices;
using DotRecast.Detour;
using DotRecast.Recast;
using DotRecast.Recast.Geom;
using DotRecast.Recast.Toolset;
using DotRecast.Recast.Toolset.Builder;
using DotRecast.Core.Numerics;
using Stride.BepuPhysics.Definitions;
using Stride.Engine;
using Stride.Games;
using Stride.Core.Mathematics;
using Stride.BepuPhysics.Systems;
using Stride.Core;
using System.Diagnostics;
using Stride.BepuPhysics.Navigation.Extensions;
using Stride.BepuPhysics.Navigation.Definitions;
using Stride.Engine.Design;

namespace Stride.BepuPhysics.Navigation.Processors;

/// <summary>
/// This system is responsible for processing the navmesh to be used for pathfinding.
/// </summary>
public class RecastMeshProcessor : GameSystemBase
{
    public TimeSpan LastShapeCacheTime { get; private set; }

    /// <summary>
    /// The maximum number of polygons.
    /// </summary>
    public const int MaxPolys = 256;
    /// <summary>
    /// The maximum number of smooth path points.
    /// </summary>
    public const int MaxSmooth = 2048;

    private readonly RcVec3f _polyPickExt = new(2, 4, 2);

    private readonly Stopwatch _stopwatch = new();
    private readonly SceneSystem _sceneSystem;
    private readonly ShapeCacheSystem _shapeCache;

    private DtNavMesh? _navMesh;
    private Task<DtNavMesh>? _runningRebuild;

    private CancellationTokenSource _rebuildingTask = new();
    private RecastNavigationConfiguration _navSettings;

    public RecastMeshProcessor(IServiceRegistry registry) : base(registry)
    {
        UpdateOrder = 20000;
        Enabled = true; //enabled by default

        registry.AddService(this);

        _sceneSystem = registry.GetSafeServiceAs<SceneSystem>();
        _shapeCache = registry.GetSafeServiceAs<ShapeCacheSystem>();

        var gameSettings = registry.GetSafeServiceAs<IGameSettingsService>();
        _navSettings = gameSettings.Settings.Configurations.Get<RecastNavigationConfiguration>() ?? new();
    }

    public override void Update(GameTime time)
    {
        if (_runningRebuild?.Status == TaskStatus.RanToCompletion)
        {
            _navMesh = _runningRebuild.Result;
            _runningRebuild = null;
        }
    }

    /// <summary>
    /// Rebuilds the navigation mesh asynchronously. This needs to be manually called when the scene changes for now.
    /// </summary>
    /// <returns>A task representing the asynchronous operation.</returns>
    public Task RebuildNavMesh()
    {
        #warning Right now nothing no systems calls for a rebuild of the navmesh, users have to explicitly do it from their side, not the best ...
        // The goal of this method is to do the strict minimum here on the main thread, gathering data for the async thread to do the rest on its own

        // Cancel any ongoing rebuild
        _rebuildingTask.Cancel();
        _rebuildingTask = new CancellationTokenSource();

        _stopwatch.Start();

        // Fetch mesh data from the scene - this may be too slow
        // There are a couple of avenues we could go down into to fix this but none of them are easy
        // Something we'll have to investigate later.
        var asyncInput = new AsyncInput();
        var containerProcessor = _sceneSystem.SceneInstance.Processors.Get<CollidableProcessor>();
        for (var e = containerProcessor.ComponentDataEnumerator; e.MoveNext();)
        {
            var collidable = e.Current.Value;

#warning should we really ignore all bodies ?
            if (collidable is BodyComponent)
                continue;

            // No need to store cache, nav mesh recompute should be rare enough were it would waste more memory than necessary
            collidable.Collider.AppendModel(asyncInput.ShapeData, _shapeCache, out object? cache);
            int shapeCount = collidable.Collider.Transforms;
            for (int i = shapeCount - 1; i >= 0; i--)
                asyncInput.TransformsOut.Add(default);
            collidable.Collider.GetLocalTransforms(collidable, CollectionsMarshal.AsSpan(asyncInput.TransformsOut)[^shapeCount..]);
            asyncInput.Matrices.Add((collidable.Entity.Transform.WorldMatrix, shapeCount));
        }

        _stopwatch.Stop();
        LastShapeCacheTime = _stopwatch.Elapsed;
        _stopwatch.Reset();

        var settingsCopy = new RcNavMeshBuildSettings
        {
            cellSize = _navSettings.BuildSettings.CellSize,
            cellHeight = _navSettings.BuildSettings.CellHeight,
            agentHeight = _navSettings.BuildSettings.AgentHeight,
            agentRadius = _navSettings.BuildSettings.AgentRadius,
            agentMaxClimb = _navSettings.BuildSettings.AgentMaxClimb,
            agentMaxSlope = _navSettings.BuildSettings.AgentMaxSlope,
            agentMaxAcceleration = _navSettings.BuildSettings.AgentMaxAcceleration,
            //agentMaxSpeed = _navSettings.BuildSettings.agentMaxSpeed,
            minRegionSize = _navSettings.BuildSettings.MinRegionSize,
            mergedRegionSize = _navSettings.BuildSettings.MergedRegionSize,
            partitioning = _navSettings.BuildSettings.Partitioning,
            filterLowHangingObstacles = _navSettings.BuildSettings.FilterLowHangingObstacles,
            filterLedgeSpans = _navSettings.BuildSettings.FilterLedgeSpans,
            filterWalkableLowHeightSpans = _navSettings.BuildSettings.FilterWalkableLowHeightSpans,
            edgeMaxLen = _navSettings.BuildSettings.EdgeMaxLen,
            edgeMaxError = _navSettings.BuildSettings.EdgeMaxError,
            vertsPerPoly = _navSettings.BuildSettings.VertsPerPoly,
            detailSampleDist = _navSettings.BuildSettings.DetailSampleDist,
            detailSampleMaxError = _navSettings.BuildSettings.DetailSampleMaxError,
            tiled = _navSettings.BuildSettings.Tiled,
            tileSize = _navSettings.BuildSettings.TileSize,
        };
        var token = _rebuildingTask.Token;
        var task = Task.Run(() => _navMesh = CreateNavMesh(settingsCopy, asyncInput, _navSettings.UsableThreadCount, token), token);
        _runningRebuild = task;
        return task;
    }

    /// <summary>
    /// Creates a new navigation mesh using the specified settings, input, threads, and cancellation token.
    /// </summary>
    /// <param name="navSettings">The navigation mesh build settings.</param>
    /// <param name="input">The input geometry provider.</param>
    /// <param name="threads">The number of threads to use.</param>
    /// <param name="cancelToken">The cancellation token.</param>
    /// <returns>The created navigation mesh.</returns>
    private static DtNavMesh CreateNavMesh(RcNavMeshBuildSettings navSettings, AsyncInput input, int threads, CancellationToken cancelToken)
    {
        // /!\ THIS IS NOT RUNNING ON THE MAIN THREAD /!\

        var verts = new List<VertexPosition3>();
        var indices = new List<int>();
        for (int collidableI = 0, shapeI = 0; collidableI < input.Matrices.Count; collidableI++)
        {
            var (collidableMatrix, shapeCount) = input.Matrices[collidableI];
            collidableMatrix.Decompose(out _, out Matrix worldMatrix, out var translation);
            worldMatrix.TranslationVector = translation;

            for (int j = 0; j < shapeCount; j++, shapeI++)
            {
                var transform = input.TransformsOut[shapeI];
                Matrix.Transformation(ref transform.Scale, ref transform.RotationLocal, ref transform.PositionLocal, out var localMatrix);
                var finalMatrix = localMatrix * worldMatrix;

                var shape = input.ShapeData[shapeI];
                verts.EnsureCapacity(verts.Count + shape.Vertices.Length);
                indices.EnsureCapacity(indices.Count + shape.Indices.Length);

                int vertexBufferStart = verts.Count;

                for (int i = 0; i < shape.Indices.Length; i += 3)
                {
                    var index0 = shape.Indices[i];
                    var index1 = shape.Indices[i + 1];
                    var index2 = shape.Indices[i + 2];
                    indices.Add(vertexBufferStart + index0);
                    indices.Add(vertexBufferStart + index2);
                    indices.Add(vertexBufferStart + index1);
                }

                //foreach (int index in shape.Indices)
                //    indices.Add(vertexBufferStart + index);

                for (int l = 0; l < shape.Vertices.Length; l++)
                {
                    var vertex = shape.Vertices[l].Position;
                    Vector3.Transform(ref vertex, ref finalMatrix, out Vector3 transformedVertex);
                    verts.Add(new(transformedVertex));
                }
            }
        }

        // Get the backing array of this list,
        // get a span to that backing array,
        var spanToPoints = CollectionsMarshal.AsSpan(verts);
        // cast the type of span to read it as if it was a series of contiguous floats instead of contiguous vectors
        var reinterpretedPoints = MemoryMarshal.Cast<VertexPosition3, float>(spanToPoints);
        StrideGeomProvider geom = new(reinterpretedPoints.ToArray(), [.. indices]);

        cancelToken.ThrowIfCancellationRequested();

        RcPartition partitionType = RcPartitionType.OfValue(navSettings.partitioning);
        RcConfig cfg = new(
            useTiles: true,
            navSettings.tileSize,
            navSettings.tileSize,
            RcConfig.CalcBorder(navSettings.agentRadius, navSettings.cellSize),
            partitionType,
            navSettings.cellSize,
            navSettings.cellHeight,
            navSettings.agentMaxSlope,
            navSettings.agentHeight,
            navSettings.agentRadius,
            navSettings.agentMaxClimb,
            (navSettings.minRegionSize * navSettings.minRegionSize) * navSettings.cellSize * navSettings.cellSize,
            (navSettings.mergedRegionSize * navSettings.mergedRegionSize) * navSettings.cellSize * navSettings.cellSize,
            navSettings.edgeMaxLen,
            navSettings.edgeMaxError,
            navSettings.vertsPerPoly,
            navSettings.detailSampleDist,
            navSettings.detailSampleMaxError,
            navSettings.filterLowHangingObstacles,
            navSettings.filterLedgeSpans,
            navSettings.filterWalkableLowHeightSpans,
            SampleAreaModifications.SAMPLE_AREAMOD_WALKABLE,
            buildMeshDetail: true);

        cancelToken.ThrowIfCancellationRequested();

        List<DtMeshData> dtMeshes = [];
        foreach (RcBuilderResult result in new RcBuilder().BuildTiles(geom, cfg, true, false, threads, cancellation: cancelToken))
        {
            DtNavMeshCreateParams navMeshCreateParams = DemoNavMeshBuilder.GetNavMeshCreateParams(geom, navSettings.cellSize, navSettings.cellHeight, navSettings.agentHeight, navSettings.agentRadius, navSettings.agentMaxClimb, result);
            navMeshCreateParams.tileX = result.TileX;
            navMeshCreateParams.tileZ = result.TileZ;
            DtMeshData dtMeshData = DtNavMeshBuilder.CreateNavMeshData(navMeshCreateParams);
            if (dtMeshData != null)
            {
                dtMeshes.Add(DemoNavMeshBuilder.UpdateAreaAndFlags(dtMeshData));
            }

            cancelToken.ThrowIfCancellationRequested();
        }

        cancelToken.ThrowIfCancellationRequested();

        DtNavMeshParams option = default;
        option.orig = geom.GetMeshBoundsMin();
        option.tileWidth = navSettings.tileSize * navSettings.cellSize;
        option.tileHeight = navSettings.tileSize * navSettings.cellSize;
        option.maxTiles = GetMaxTiles(geom, navSettings.cellSize, navSettings.tileSize);
        option.maxPolys = GetMaxPolysPerTile(geom, navSettings.cellSize, navSettings.tileSize);
        DtNavMesh navMesh = new(option, navSettings.vertsPerPoly);
        foreach (DtMeshData dtMeshData1 in dtMeshes)
        {
            navMesh.AddTile(dtMeshData1, 0, 0L);
        }

        cancelToken.ThrowIfCancellationRequested();

        return navMesh;
    }

    /// <summary>
    /// Gets the maximum number of tiles based on the input geometry, cell size, and tile size.
    /// </summary>
    /// <param name="geom">The input geometry provider.</param>
    /// <param name="cellSize">The cell size.</param>
    /// <param name="tileSize">The tile size.</param>
    /// <returns>The maximum number of tiles.</returns>
    private static int GetMaxTiles(IInputGeomProvider geom, float cellSize, int tileSize)
    {
        int tileBits = GetTileBits(geom, cellSize, tileSize);
        return 1 << tileBits;
    }

    /// <summary>
    /// Gets the maximum number of polygons per tile based on the input geometry, cell size, and tile size.
    /// </summary>
    /// <param name="geom">The input geometry provider.</param>
    /// <param name="cellSize">The cell size.</param>
    /// <param name="tileSize">The tile size.</param>
    /// <returns>The maximum number of polygons per tile.</returns>
    private static int GetMaxPolysPerTile(IInputGeomProvider geom, float cellSize, int tileSize)
    {
        int num = 22 - GetTileBits(geom, cellSize, tileSize);
        return 1 << num;
    }

    /// <summary>
    /// Gets the number of bits required to represent a tile based on the input geometry, cell size, and tile size.
    /// </summary>
    /// <param name="geom">The input geometry provider.</param>
    /// <param name="cellSize">The cell size.</param>
    /// <param name="tileSize">The tile size.</param>
    /// <returns>The number of bits required to represent a tile.</returns>
    private static int GetTileBits(IInputGeomProvider geom, float cellSize, int tileSize)
    {
        RcCommons.CalcGridSize(geom.GetMeshBoundsMin(), geom.GetMeshBoundsMax(), cellSize, out var sizeX, out var sizeZ);
        int num = (sizeX + tileSize - 1) / tileSize;
        int num2 = (sizeZ + tileSize - 1) / tileSize;
        return Math.Min(DtUtils.Ilog2(DtUtils.NextPow2(num * num2)), 14);
    }

    /// <summary>
    /// Tries to find a path between the specified start and end points.
    /// </summary>
    /// <param name="start">The start point.</param>
    /// <param name="end">The end point.</param>
    /// <param name="polys">The list of polygons.</param>
    /// <param name="smoothPath">The list of smooth path points.</param>
    /// <returns>True if a path is found, false otherwise.</returns>
    public bool TryFindPath(Vector3 start, Vector3 end, ref List<long> polys, ref List<Vector3> smoothPath)
    {
        if (_navMesh is null) return false;

        var queryFilter = new DtQueryDefaultFilter();
        var dtNavMeshQuery = new DtNavMeshQuery(_navMesh);

        dtNavMeshQuery.FindNearestPoly(start.ToDotRecastVector(), _polyPickExt, queryFilter, out long startRef, out _, out _);

        dtNavMeshQuery.FindNearestPoly(end.ToDotRecastVector(), _polyPickExt, queryFilter, out long endRef, out _, out _);
        // find the nearest point on the navmesh to the start and end points
        var result = FindFollowPath(dtNavMeshQuery, startRef, endRef, start.ToDotRecastVector(), end.ToDotRecastVector(), queryFilter, true, ref polys, ref smoothPath);

        return result.Succeeded();
    }

    /// <summary>
    /// Gets the list of navigation mesh tiles.
    /// </summary>
    /// <returns>The list of navigation mesh tiles.</returns>
    public List<Vector3>? GetNavMeshTiles()
    {
        if (_navMesh is null) return null;

        List<Vector3> verts = [];

        for (int i = 0; i < _navMesh.GetMaxTiles(); i++)
        {
            var tile = _navMesh.GetTile(i);
            if (tile?.data != null)
            {
                for (int j = 0; j < tile.data.verts.Length; j += 3)
                {
                    var point = new Vector3(
                        tile.data.verts[j],
                        tile.data.verts[j + 1],
                        tile.data.verts[j + 2]);
                    verts.Add(point);
                }
            }
        }

        return verts;
    }

    /// <summary>
    /// Finds a follow path between the specified start and end references.
    /// </summary>
    /// <param name="navQuery">The navigation mesh query.</param>
    /// <param name="startRef">The start reference.</param>
    /// <param name="endRef">The end reference.</param>
    /// <param name="startPt">The start point.</param>
    /// <param name="endPt">The end point.</param>
    /// <param name="filter">The query filter.</param>
    /// <param name="enableRaycast">True to enable raycast, false otherwise.</param>
    /// <param name="polys">The list of polygons.</param>
    /// <param name="smoothPath">The list of smooth path points.</param>
    /// <returns>The status of the find follow path operation.</returns>
    public static DtStatus FindFollowPath(DtNavMeshQuery navQuery, long startRef, long endRef, RcVec3f startPt, RcVec3f endPt, IDtQueryFilter filter, bool enableRaycast, ref List<long> polys, ref List<Vector3> smoothPath)
    {
        if (startRef == 0 || endRef == 0)
        {
            polys.Clear();
            smoothPath.Clear();

            return DtStatus.DT_FAILURE;
        }

        polys.Clear();
        smoothPath.Clear();

        var opt = new DtFindPathOption(enableRaycast ? DtFindPathOptions.DT_FINDPATH_ANY_ANGLE : 0, float.MaxValue);
        navQuery.FindPath(startRef, endRef, startPt, endPt, filter, ref polys, opt);
        if (0 >= polys.Count)
            return DtStatus.DT_FAILURE;

        // Iterate over the path to find smooth path on the detail mesh surface.
        navQuery.ClosestPointOnPoly(startRef, startPt, out var iterPos, out _);
        navQuery.ClosestPointOnPoly(polys[polys.Count - 1], endPt, out var targetPos, out _);

        const float STEP_SIZE = 0.5f;
        const float SLOP = 0.01f;

        smoothPath.Clear();
        smoothPath.Add(iterPos.ToStrideVector());
        var visited = new List<long>();

        // Move towards target a small advancement at a time until target reached or
        // when ran out of memory to store the path.
        while (0 < polys.Count && smoothPath.Count < MaxSmooth)
        {
            // Find location to steer towards.
            if (!DtPathUtils.GetSteerTarget(navQuery, iterPos, targetPos, SLOP,
                    polys, out var steerPos, out var steerPosFlag, out _))
            {
                break;
            }

            bool endOfPath = (steerPosFlag & DtStraightPathFlags.DT_STRAIGHTPATH_END) != 0;
            bool offMeshConnection = (steerPosFlag & DtStraightPathFlags.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;

            // Find movement delta.
            RcVec3f delta = RcVec3f.Subtract(steerPos, iterPos);
            float len = MathF.Sqrt(RcVec3f.Dot(delta, delta));
            // If the steer target is end of path or off-mesh link, do not move past the location.
            if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
            {
                len = 1;
            }
            else
            {
                len = STEP_SIZE / len;
            }

            RcVec3f moveTgt = RcVecUtils.Mad(iterPos, delta, len);

            // Move
            navQuery.MoveAlongSurface(polys[0], iterPos, moveTgt, filter, out var result, ref visited);

            iterPos = result;

            polys = DtPathUtils.MergeCorridorStartMoved(polys, 0, 256, visited);
            polys = DtPathUtils.FixupShortcuts(polys, navQuery);

            var status = navQuery.GetPolyHeight(polys[0], result, out var h);
            if (status.Succeeded())
            {
                iterPos.Y = h;
            }

            // Store results.
            if (smoothPath.Count < MaxSmooth)
            {
                smoothPath.Add(iterPos.ToStrideVector());
            }
        }

        return DtStatus.DT_SUCCESS;
    }

    class AsyncInput
    {
        public readonly List<BasicMeshBuffers> ShapeData = [];
        public readonly List<ShapeTransform> TransformsOut = [];
        public readonly List<(Matrix entity, int count)> Matrices = [];
    }
}
