// <auto-generated>
// Do not edit this file yourself!
//
// This code was generated by Stride Shader Mixin Code Generator.
// To generate it yourself, please install Stride.VisualStudio.Package .vsix
// and re-save the associated .sdfx.
// </auto-generated>

using System;
using Stride.Core;
using Stride.Rendering;
using Stride.Graphics;
using Stride.Shaders;
using Stride.Core.Mathematics;
using Buffer = Stride.Graphics.Buffer;

using Stride.Rendering.Data;
using Stride.Rendering.Materials;
namespace Stride.Rendering
{
    internal static partial class ShaderMixins
    {
        internal partial class StrideEffectBase : IShaderMixinBuilder
        {
            public void Generate(ShaderMixinSource mixin, ShaderMixinContext context)
            {
                context.Mixin(mixin, "ShaderBase");
                context.Mixin(mixin, "ShadingBase");
                var extensionPreVertexStageSurfaceShaders = context.GetParam(MaterialKeys.VertexStageSurfaceShaders);
                if (extensionPreVertexStageSurfaceShaders != null)
                {
                    context.Mixin(mixin, "MaterialSurfaceVertexStageCompositor");

                    {
                        var __mixinToCompose__ = (extensionPreVertexStageSurfaceShaders);
                        var __subMixin = new ShaderMixinSource();
                        context.PushComposition(mixin, "materialVertexStage", __subMixin);
                        context.Mixin(__subMixin, __mixinToCompose__);
                        context.PopComposition();
                    }

                    {
                        var __mixinToCompose__ = context.GetParam(MaterialKeys.VertexStageStreamInitializer);
                        var __subMixin = new ShaderMixinSource();
                        context.PushComposition(mixin, "streamInitializerVertexStage", __subMixin);
                        context.Mixin(__subMixin, __mixinToCompose__);
                        context.PopComposition();
                    }
                }
                context.Mixin(mixin, "TransformationBase");
                context.Mixin(mixin, "NormalStream");
                var extensionTessellationShader = context.GetParam(MaterialKeys.TessellationShader);
                if (context.GetParam(StrideEffectBaseKeys.HasInstancing))
                {
                    mixin.AddMacro("ModelTransformUsage", context.GetParam(StrideEffectBaseKeys.ModelTransformUsage));
                    context.Mixin(mixin, "TransformationWAndVPInstanced");
                    if (context.GetParam(MaterialKeys.HasNormalMap))
                    {
                        if (extensionTessellationShader != null)
                        {
                            context.Mixin(mixin, "NormalFromNormalMappingTessellationInstanced");
                        }
                        else
                        {
                            context.Mixin(mixin, "NormalFromNormalMappingInstanced");
                        }
                    }
                    else
                    {
                        context.Mixin(mixin, "NormalFromMeshInstanced");
                    }
                }
                else
                {
                    context.Mixin(mixin, "TransformationWAndVP");
                    if (context.GetParam(MaterialKeys.HasNormalMap))
                    {
                        if (extensionTessellationShader != null)
                        {
                            context.Mixin(mixin, "NormalFromNormalMappingTessellation");
                        }
                        else
                        {
                            context.Mixin(mixin, "NormalFromNormalMapping");
                        }
                    }
                    else
                    {
                        context.Mixin(mixin, "NormalFromMesh");
                    }
                }
                if (context.GetParam(MaterialKeys.HasSkinningPosition))
                {
                    mixin.AddMacro("SkinningMaxBones", context.GetParam(MaterialKeys.SkinningMaxBones));
                    if (context.GetParam(StrideEffectBaseKeys.HasInstancing))
                    {
                        context.Mixin(mixin, "TransformationSkinningInstanced");
                    }
                    else
                    {
                        context.Mixin(mixin, "TransformationSkinning");
                    }
                    if (context.GetParam(MaterialKeys.HasSkinningNormal))
                    {
                        context.Mixin(mixin, "NormalMeshSkinning");
                    }
                    if (context.GetParam(MaterialKeys.HasSkinningTangent))
                    {
                        context.Mixin(mixin, "TangentMeshSkinning");
                    }
                    if (context.GetParam(MaterialKeys.HasSkinningNormal))
                    {
                        if (context.GetParam(MaterialKeys.HasNormalMap))
                        {
                            if (extensionTessellationShader != null)
                            {
                                context.Mixin(mixin, "NormalVSSkinningNormalMappingTessellation");
                            }
                            else
                            {
                                context.Mixin(mixin, "NormalVSSkinningNormalMapping");
                            }
                        }
                        else
                        {
                            context.Mixin(mixin, "NormalVSSkinningFromMesh");
                        }
                    }
                }


                if (context.GetParam(MaterialKeys.HasBlendShape))
                {
                    mixin.AddMacro("MAT_COUNT", context.GetParam(MaterialKeys.MAT_COUNT));
                    mixin.AddMacro("MORPH_TARGETS_COUNT", context.GetParam(MaterialKeys.MORPH_TARGETS_COUNT));
                    context.Mixin(mixin, "TransformationBlendShape");
                }



                if (extensionTessellationShader != null)
                {
                    context.Mixin(mixin, (extensionTessellationShader));
                    var extensionDomainStageSurfaceShaders = context.GetParam(MaterialKeys.DomainStageSurfaceShaders);
                    if (extensionDomainStageSurfaceShaders != null)
                    {
                        context.Mixin(mixin, "MaterialSurfaceDomainStageCompositor");

                        {
                            var __mixinToCompose__ = (extensionDomainStageSurfaceShaders);
                            var __subMixin = new ShaderMixinSource();
                            context.PushComposition(mixin, "materialDomainStage", __subMixin);
                            context.Mixin(__subMixin, __mixinToCompose__);
                            context.PopComposition();
                        }

                        {
                            var __mixinToCompose__ = context.GetParam(MaterialKeys.DomainStageStreamInitializer);
                            var __subMixin = new ShaderMixinSource();
                            context.PushComposition(mixin, "streamInitializerDomainStage", __subMixin);
                            context.Mixin(__subMixin, __mixinToCompose__);
                            context.PopComposition();
                        }
                    }
                }
                var computeVelocityShader = context.GetParam(StrideEffectBaseKeys.ComputeVelocityShader);
                if (computeVelocityShader != null)
                {
                    context.Mixin(mixin, (computeVelocityShader));
                }
                var extensionPostVertexStage = context.GetParam(StrideEffectBaseKeys.ExtensionPostVertexStageShader);
                if (extensionPostVertexStage != null)
                {
                    context.Mixin(mixin, (extensionPostVertexStage));
                }
                var targetExtensions = context.GetParam(StrideEffectBaseKeys.RenderTargetExtensions);
                if (targetExtensions != null)
                {
                    context.Mixin(mixin, (targetExtensions));
                }
            }

            [ModuleInitializer]
            internal static void __Initialize__()

            {
                ShaderMixinManager.Register("StrideEffectBase", new StrideEffectBase());
            }
        }
    }
}
