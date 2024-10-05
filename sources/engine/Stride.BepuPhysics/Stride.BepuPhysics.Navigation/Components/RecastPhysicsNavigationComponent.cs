using Stride.BepuPhysics.Navigation.Processors;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Engine.Design;
using Stride.Core.Annotations;

namespace Stride.BepuPhysics.Navigation.Components;
/// <summary>
/// This component is responsible for handling navigation-related properties and behaviors for an entity.
/// </summary>
[DataContract(nameof(RecastPhysicsNavigationComponent))]
[ComponentCategory("Bepu - Navigation")]
[DefaultEntityComponentProcessor(typeof(RecastPhysicsNavigationProcessor), ExecutionMode = ExecutionMode.Runtime)]
public class RecastPhysicsNavigationComponent : EntityComponent
{
	/// <summary>
	/// The speed at which the agent moves.
	/// </summary>
	public float Speed { get; set; } = 5.0f;

	/// <summary>
	/// The character component that handles the physics for the agent.
	/// </summary>
	[MemberRequired]
	public required CharacterComponent PhysicsComponent { get; set; }

    /// <summary>
    /// Tells the <see cref="RecastPhysicsNavigationProcessor"/> if the agent should move and rotate towards the target in the path.
    /// This will also reset the <see cref="CharacterComponent"/>'s velocity to zero if set to false.
    /// </summary>
	[DataMemberIgnore]
	public bool ShouldMove
	{
		get => _shouldMove;
		set
		{
			_shouldMove = value;
			if (!value)
			{
				PhysicsComponent.Move(Vector3.Zero);
			}
		}
	}

	private bool _shouldMove;

    /// <summary>
    /// Tells the <see cref="RecastPhysicsNavigationProcessor"/> to set a new path at the next available opportunity.
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
