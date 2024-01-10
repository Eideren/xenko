﻿using Stride.BepuPhysics._2D.Components.Containers;
using Stride.BepuPhysics.Components.Containers;
using Stride.BepuPhysics.Components.Containers.Interfaces;
using Stride.BepuPhysics.DebugRender.Components;
using Stride.BepuPhysics.DebugRender.Effects;
using Stride.BepuPhysics.DebugRender.Effects.RenderFeatures;
using Stride.BepuPhysics.Definitions;
using Stride.BepuPhysics.Extensions;
using Stride.BepuPhysics.Processors;
using Stride.Core.Annotations;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Input;
using Stride.Rendering;

namespace Stride.BepuPhysics.DebugRender.Processors
{
    public class DebugRenderProcessor : EntityProcessor<DebugRenderComponent>
    {
        private IGame? _game = null;
        private SceneSystem _sceneSystem;
        private BepuShapeCacheSystem _bepuShapeCacheSystem;
        private InputManager _input;
        private SinglePassWireframeRenderFeature _wireframeRenderFeature;
        private VisibilityGroup _visibilityGroup;
        private Dictionary<ContainerComponent, WireFrameRenderObject[]> _wireFrameRenderObject = new();

        private bool _alwaysOn = false;
        private bool _isOn = false;

        public DebugRenderProcessor()
        {
            Order = 10200; // Transform processor operates at -200, as long as we're after it we're working optimally
        }

        protected override void OnSystemAdd()
        {
            BepuServicesHelper.LoadBepuServices(Services);
            _game = Services.GetService<IGame>();
            _sceneSystem = Services.GetService<SceneSystem>();
            _bepuShapeCacheSystem = Services.GetService<BepuShapeCacheSystem>();
            _input = Services.GetService<InputManager>();

            if (_sceneSystem.GraphicsCompositor.RenderFeatures.OfType<SinglePassWireframeRenderFeature>().FirstOrDefault() is { } wireframeFeature)
            {
                _wireframeRenderFeature = wireframeFeature;
            }
            else
            {
                _wireframeRenderFeature = new();
                _sceneSystem.GraphicsCompositor.RenderFeatures.Add(_wireframeRenderFeature);
            }

            _visibilityGroup = _sceneSystem.SceneInstance.VisibilityGroups.First();
        }

        protected override void OnEntityComponentAdding(Entity entity, [NotNull] DebugRenderComponent component, [NotNull] DebugRenderComponent data)
        {
            _alwaysOn = component.AlwaysRender;
            component.SetFunc = (e) => _alwaysOn = e;
            base.OnEntityComponentAdding(entity, component, data);
        }

        public override void Update(GameTime time)
        {
            base.Update(time);

            bool shouldBeOn = _alwaysOn || _input.IsKeyDown(Keys.F10);
            if (_isOn != shouldBeOn) // Changed state
            {
                if (_sceneSystem.SceneInstance.GetProcessor<ContainerProcessor>() is { } proc)
                {
                    if (shouldBeOn)
                    {
                        _isOn = true;
                        proc.OnPostAdd += StartTrackingContainer;
                        proc.OnPreRemove += ClearTrackingForContainer;
                        StartTracking(proc);
                    }
                    else
                    {
                        _isOn = false;
                        proc.OnPostAdd -= StartTrackingContainer;
                        proc.OnPreRemove -= ClearTrackingForContainer;
                        Clear();
                    }
                }
                else
                {
                    _isOn = false;
                    Clear();
                }
            }

            if (_isOn) // Update gizmos transform
            {
                foreach (var kvp in _wireFrameRenderObject)
                {
                    var container = kvp.Key;
                    // We don't need to call UpdateWorldMatrix before reading WorldMatrix as we're running after the TransformProcessor operated,
                    // and we don't expect or care if other processors affect the transform afterwards
                    container.Entity.Transform.WorldMatrix.Decompose(out _, out Matrix rotMatrix, out var translation);
                    rotMatrix.TranslationVector = translation; // rotMatrix is now a translation and rotation matrix
                    foreach (var wireframe in kvp.Value)
                    {
                        wireframe.WorldMatrix = wireframe.ContainerBaseMatrix * rotMatrix;
                    }
                }
            }

            if (_input.IsKeyPressed(Keys.F11))
                _alwaysOn = !_alwaysOn;
        }

        private void StartTracking(ContainerProcessor proc)
        {
            var shapeAndOffsets = new List<(BodyShapeData data, BodyShapeTransform transform)>();
            for (var containers = proc.ComponentDatas; containers.MoveNext();)
            {
                StartTrackingContainer(containers.Current.Key, shapeAndOffsets);
            }
        }

        private void StartTrackingContainer(ContainerComponent container) => StartTrackingContainer(container, new());

        private void StartTrackingContainer(ContainerComponent container, List<(BodyShapeData data, BodyShapeTransform transform)> shapeAndOffsets)
        {
            var color = Color.Black;

#warning replace with I2DContainer
            if (container is _2DBodyContainerComponent)
            {
                color = Color.Green;
            }
            else if (container is IContainerWithColliders)
            {
                color = Color.Red;
            }
            else if (container is IContainerWithMesh)
            {
                color = Color.Blue;
            }

            shapeAndOffsets.Clear();
            _bepuShapeCacheSystem.AppendCachedShapesFor(container, shapeAndOffsets);

            WireFrameRenderObject[] wireframes = new WireFrameRenderObject[shapeAndOffsets.Count];
            for (int i = 0; i < shapeAndOffsets.Count; i++)
            {
                var shapeAndOffset = shapeAndOffsets[i];
                var local = shapeAndOffset;

                var wireframe = WireFrameRenderObject.New(_game.GraphicsDevice, shapeAndOffset.data.Indices, shapeAndOffset.data.Vertices);
                wireframe.Color = color;
                Matrix.Transformation(ref local.transform.Scale, ref local.transform.RotationLocal, ref local.transform.PositionLocal, out wireframe.ContainerBaseMatrix);
                wireframes[i] = wireframe;
                _visibilityGroup.RenderObjects.Add(wireframe);
            }
            _wireFrameRenderObject.Add(container, wireframes);
        }

        private void ClearTrackingForContainer(ContainerComponent container)
        {
            if (_wireFrameRenderObject.TryGetValue(container, out var wfros))
            {
                foreach (var wireframe in wfros)
                {
                    wireframe.Dispose();
                    _visibilityGroup.RenderObjects.Remove(wireframe);
                }
                _wireFrameRenderObject.Remove(container);
            }
        }

        private void Clear()
        {
            foreach (var kvp in _wireFrameRenderObject)
            {
                foreach (var wireframe in kvp.Value)
                {
                    wireframe.Dispose();
                    _visibilityGroup.RenderObjects.Remove(wireframe);
                }
            }
            _wireFrameRenderObject.Clear();
        }
    }
}
