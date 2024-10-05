// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using DotRecast.Recast;
using Stride.Core;

namespace Stride.BepuPhysics.Navigation.Definitions;
/// <summary>
/// Represents the settings used for building navigation meshes with Recast and Detour.
/// These settings control various parameters such as cell size, agent properties, region sizes, and filtering options.
/// </summary>
[DataContract("DotRecastBuildSettings")]
public class BuildSettings
{
	/// <summary>
	/// The size of each cell in the navigation mesh.
	/// </summary>
	public float CellSize = 0.3f;

	/// <summary>
	/// The height of each cell in the navigation mesh.
	/// </summary>
	public float CellHeight = 0.2f;

	/// <summary>
	/// The height of the agent used in the navigation mesh.
	/// </summary>
	public float AgentHeight = 2f;

	/// <summary>
	/// The radius of the agent used in the navigation mesh.
	/// </summary>
	public float AgentRadius = 0.6f;

	/// <summary>
	/// The maximum height the agent can climb.
	/// </summary>
	public float AgentMaxClimb = 0.9f;

	/// <summary>
	/// The maximum slope the agent can traverse.
	/// </summary>
	public float AgentMaxSlope = 45f;

	/// <summary>
	/// The maximum acceleration of the agent.
	/// </summary>
	public float AgentMaxAcceleration = 8f;

	/// <summary>
	/// The minimum size of regions in the navigation mesh.
	/// </summary>
	public int MinRegionSize = 8;

	/// <summary>
	/// The size of merged regions in the navigation mesh.
	/// </summary>
	public int MergedRegionSize = 20;

	/// <summary>
	/// The type of partitioning used in the navigation mesh.
	/// </summary>
	public RcPartition PartitionType
	{
		get
		{
			return RcPartitionType.OfValue(Partitioning);
		}
		set
		{
			Partitioning = (int)value;
		}
	}

	/// <summary>
	/// The partitioning value used in the navigation mesh.
	/// </summary>
	[DataMemberIgnore]
	public int Partitioning = RcPartitionType.WATERSHED.Value;

	/// <summary>
	/// Indicates whether to filter low hanging obstacles in the navigation mesh.
	/// </summary>
	public bool FilterLowHangingObstacles = true;

	/// <summary>
	/// Indicates whether to filter ledge spans in the navigation mesh.
	/// </summary>
	public bool FilterLedgeSpans = true;

	/// <summary>
	/// Indicates whether to filter walkable low height spans in the navigation mesh.
	/// </summary>
	public bool FilterWalkableLowHeightSpans = true;

	/// <summary>
	/// The maximum length of edges in the navigation mesh.
	/// </summary>
	public float EdgeMaxLen = 12f;

	/// <summary>
	/// The maximum error allowed for edges in the navigation mesh.
	/// </summary>
	public float EdgeMaxError = 1.3f;

	/// <summary>
	/// The number of vertices per polygon in the navigation mesh.
	/// </summary>
	public int VertsPerPoly = 6;

	/// <summary>
	/// The distance to sample the detail mesh.
	/// </summary>
	public float DetailSampleDist = 6f;

	/// <summary>
	/// The maximum error allowed when sampling the detail mesh.
	/// </summary>
	public float DetailSampleMaxError = 1f;

	/// <summary>
	/// Indicates whether the navigation mesh build is tiled.
	/// </summary>
	public bool Tiled;

	/// <summary>
	/// The size of each tile in the navigation mesh.
	/// </summary>
	public int TileSize = 32;
}
