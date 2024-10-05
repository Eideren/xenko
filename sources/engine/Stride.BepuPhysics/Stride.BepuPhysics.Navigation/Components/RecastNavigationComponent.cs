using Stride.BepuPhysics.Navigation.Processors;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Engine.Design;

namespace Stride.BepuPhysics.Navigation.Components;
/// <summary>
/// This component is responsible for handling navigation-related properties and behaviors for an entity.
/// </summary>
[DataContract(nameof(RecastNavigationComponent))]
[ComponentCategory("Bepu - Navigation")]
[DefaultEntityComponentProcessor(typeof(RecastNavigationProcessor), ExecutionMode = ExecutionMode.Runtime)]
public class RecastNavigationComponent : EntityComponent
{
	/// <summary>
	/// The speed at which the agent moves.
	/// </summary>
	public float Speed { get; set; } = 5.0f;

    /// <summary>
    /// Tells the <see cref="RecastNavigationProcessor"/> if the agent should move and rotate towards the target in the path.
    /// </summary>
    [DataMemberIgnore]
	public bool ShouldMove { get; set; } = true;

    /// <summary>
    /// Tells the <see cref="RecastNavigationProcessor"/> to set a new path at the next available opportunity.
    /// </summary>
	[DataMemberIgnore]
	public bool SetNewPath { get; set; } = true;

    /// <summary>
    /// Indicates if the component is in the queue to set a new path. This is used internally to prevent multiple path calculations per frame.
    /// </summary>
    [DataMemberIgnore]
	public bool InSetPathQueue { get; internal set; }

	/// <summary>
	/// The target position for the agent to move to.
	/// </summary>
	[DataMemberIgnore]
	public Vector3 Target;

	/// <summary>
	/// The calculated path for the agent to follow.
	/// </summary>
	[DataMemberIgnore]
	public List<Vector3> Path = new();

	/// <summary>
	/// The list of polygons that make up the path.
	/// </summary>
	[DataMemberIgnore]
	public List<long> Polys = new();
}
