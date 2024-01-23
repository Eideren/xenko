﻿// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net) and Silicon Studio Corp. (https://www.siliconstudio.co.jp)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.
using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Assets.Editor.Quantum.NodePresenters.Commands;
using Stride.Core.Assets.Editor.Quantum.NodePresenters.Keys;
using Stride.Core.Annotations;
using Stride.Core.Extensions;
using Stride.Core.Reflection;
using Stride.Core.Presentation.Quantum.Presenters;

namespace Stride.Core.Assets.Editor.Quantum.NodePresenters.Updaters
{
    public sealed class AbstractNodeCollectionEntryNodeUpdater : AssetNodePresenterUpdaterBase
    {
        public static IEnumerable<AbstractNodeEntry> FillDefaultAbstractNodeCollectionEntries(IAssetNodePresenter node, Type type)
        {
            IEnumerable<AbstractNodeEntry> abstractNodeMatchingEntries = AbstractNodeType.GetInheritedInstantiableTypes(type);

            if (abstractNodeMatchingEntries != null)
            {
                // Prepend the value that will allow to set the value to null, if this command is allowed.
                if (IsAllowingNull(node))
                    abstractNodeMatchingEntries = AbstractNodeValue.Null.Yield().Concat(abstractNodeMatchingEntries);
            }
            return abstractNodeMatchingEntries;
        }

        /// <summary>
        /// Checks if <see cref="MemberCollectionAttribute.NotNullItems"/> is present and set.
        /// </summary>
        /// <param name="node">The node to check.</param>
        /// <returns>True if null is a possible choice for this node, otherwise false.</returns>
        public static bool IsAllowingNull(IAssetNodePresenter node)
        {
            var abstractNodeAllowNull = true;
            var memberNode = node as MemberNodePresenter ?? (node as ItemNodePresenter)?.Parent as MemberNodePresenter;
            if (memberNode != null)
            {
                var memberCollection = memberNode.MemberAttributes.OfType<MemberCollectionAttribute>().FirstOrDefault()
                                       ?? memberNode.Descriptor.Attributes.OfType<MemberCollectionAttribute>().FirstOrDefault();

                if (memberNode.IsEnumerable && memberCollection != null && memberCollection.NotNullItems)
                {
                    // Collections
                    abstractNodeAllowNull = false;
                }
                else
                {
                    // Members
                    abstractNodeAllowNull = !memberNode.MemberAttributes.OfType<NotNullAttribute>().Any();
                }
            }
            return abstractNodeAllowNull;
        }

        protected override void UpdateNode(IAssetNodePresenter node)
        {
            if (node.ValueIsAnyCollection(out _, out var type, out _) && type.IsAbstract && !IsReferenceType(type) && IsInstantiable(type))
            {
                var abstractNodeEntries = FillDefaultAbstractNodeCollectionEntries(node, type);
                node.AttachedProperties.Add(AbstractNodeCollectionEntriesData.Key, abstractNodeEntries);
            }
        }

        private static bool IsInstantiable(Type type) => TypeDescriptorFactory.Default.AttributeRegistry.GetAttribute<NonInstantiableAttribute>(type) == null;

        private static bool IsReferenceType(Type type) => AssetRegistry.IsContentType(type) || typeof(AssetReference).IsAssignableFrom(type);
    }
}
