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
    /// The current state of the agent.
    /// </summary>
    [DataMemberIgnore]
    public NavigationState State { get; set; }

    /// <summary>
    /// The target position for the agent to move to.
    /// </summary>
    [DataMemberIgnore]
    public Vector3 Target { get; protected set; }

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

    /// <summary>
    /// Sets the target for the agent to move to. This will set the <see cref="State"/> to <see cref="NavigationState.QueuePathPlanning"/>.
    /// </summary>
    /// <param name="target"></param>
    public virtual void SetTarget(Vector3 target)
    {
        Target = target;
        State = NavigationState.QueuePathPlanning;
    }

    /// <summary>
    /// Stops the agent from moving. This will set the <see cref="State"/> to <see cref="NavigationState.Idle"/>.
    /// </summary>
    public virtual void Stop()
    {
        State = NavigationState.Idle;
    }
}

public enum NavigationState
{
    /// <summary>
    /// Tells the <see cref="RecastNavigationProcessor"/> to do nothing.
    /// </summary>
    Idle,
    /// <summary>
    /// Tells the <see cref="RecastNavigationProcessor"/> if the agent should move and rotate towards the target in the path.
    /// </summary>
    FollowingPath,
    /// <summary>
    /// Tells the <see cref="RecastNavigationProcessor"/> a plan needs to be queued. This is used internally to prevent multiple path calculations per frame.
    /// </summary>
    QueuePathPlanning,
    /// <summary>
    /// Tells the <see cref="RecastNavigationProcessor"/> to set a new path at the next available opportunity.
    /// </summary>
    PlanningPath
}
