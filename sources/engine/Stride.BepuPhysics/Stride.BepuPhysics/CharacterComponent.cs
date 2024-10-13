// Copyright (c) .NET Foundation and Contributors (https://dotnetfoundation.org/ & https://stride3d.net)
// Distributed under the MIT license. See the LICENSE.md file in the project root for more information.

using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using Stride.BepuPhysics.Components;
using Stride.BepuPhysics.Definitions;
using Stride.BepuPhysics.Definitions.Contacts;
using Stride.Core;
using Stride.Core.Extensions;
using Stride.Core.Mathematics;
using Stride.Engine;
using NVector3 = System.Numerics.Vector3;
using NRigidPose = BepuPhysics.RigidPose;

namespace Stride.BepuPhysics;

[ComponentCategory("Bepu")]
public class CharacterComponent : BodyComponent, ISimulationUpdate, IContactEventHandler
{

    /// <summary> Base speed applied when moving, measured in units per second </summary>
    public float Speed { get; set; } = 10f;

    /// <summary> Force of the impulse applied when calling <see cref="TryJump"/> </summary>
    public float JumpSpeed { get; set; } = 10f;
    /// <summary>
    /// How long the character should be in the air before falling faster.
    /// </summary>
    public float PeakAirTime { get; set; } = 0.5f;
    public float FallSpeedMultiplier { get; set; } = 2.0f;

    [DataMemberIgnore]
    public Vector3 Velocity { get; set; }

    [DataMemberIgnore]
    public bool IsGrounded { get; protected set; }

    /// <summary>
    /// Order is not guaranteed and may change at any moment
    /// </summary>
    [DataMemberIgnore]
    public List<(CollidableReference Source, Contact Contact)> Contacts { get; } = new();

    private bool _tryJump;
    private bool _didJump;

    private float _airTime;

    public CharacterComponent()
    {
        InterpolationMode = InterpolationMode.Interpolated;
    }

    /// <inheritdoc cref="BodyComponent.AttachInner"/>
    protected override void AttachInner(NRigidPose pose, BodyInertia shapeInertia, TypedIndex shapeIndex)
    {
        base.AttachInner(pose, new BodyInertia { InverseMass = 1f }, shapeIndex);
        FrictionCoefficient = 0f;
        ContactEventHandler = this;
    }

    /// <summary>
    /// Sets the velocity based on <paramref name="direction"/> and <see cref="Speed"/>
    /// </summary>
    /// <remarks>
    /// <paramref name="direction"/> does not have to be normalized;
    /// if the vector passed in has a length of 2, the character will go twice as fast
    /// </remarks>
    public virtual void Move(Vector3 direction)
    {
        // Note that this method should be thread safe, see usage in RecastPhysicsNavigationProcessor
        Velocity = direction * Speed;
    }

    /// <summary>
    /// Try to perform a jump on the next physics tick, will fail when not grounded
    /// </summary>
    public virtual void TryJump()
    {
        _tryJump = true;
    }

    /// <summary>
    /// This is called internally right before the physics simulation does a tick
    /// </summary>
    /// <param name="simTimeStep">The amount of time in seconds since the last simulation</param>
    public virtual void SimulationUpdate(float simTimeStep)
    {
        Awake = true; // Keep this body active

        LinearVelocity = new Vector3(Velocity.X, LinearVelocity.Y, Velocity.Z);

        if (_tryJump)
        {
            if (IsGrounded)
            {
                ApplyLinearImpulse(Vector3.UnitY * JumpSpeed);
                _didJump = true;
            }
            _tryJump = false;
        }
    }

    /// <summary>
    /// This is called internally right after the physics simulation does a tick
    /// </summary>
    /// <param name="simTimeStep">The amount of time in seconds since the last simulation</param>
    public virtual void AfterSimulationUpdate(float simTimeStep)
    {
        UpdateFallSpeed(simTimeStep);

        IsGrounded = GroundTest(-Simulation!.PoseGravity.ToNumeric()); // Checking for grounded after simulation ran to compute contacts as soon as possible after they are received
        // If there is no input from the player, and we are grounded, ignore gravity to prevent sliding down the slope we might be on
        // Do not ignore if there is any input to ensure we stick to the surface as much as possible while moving down a slope
        Gravity = !IsGrounded || Velocity.Length() > 0f;

        if (IsGrounded == true && !_didJump)
        {
            LinearVelocity = new Vector3(0, 0, 0);
        }
        _didJump = false;
    }

    /// <summary>
    /// Returns whether this body is in contact with the ground.
    /// </summary>
    /// <remarks>
    /// Goes through the list of <see cref="Contacts"/> to do so
    /// </remarks>
    /// <param name="groundNormal"> Which direction a surface has to be in to be considered as ground </param>
    /// <param name="threshold">
    /// How close to this direction a supporting contact
    /// has to be for it to be considered as ground.
    /// In the [-1,1] range, where -1 would return true for any given surface we are in contact with,
    /// 0 would return true for a surface that is at most 90 degrees away from <paramref name="groundNormal"/>,
    /// and 1 would return true only when a surface matches <paramref name="groundNormal"/> exactly.
    /// </param>
    protected bool GroundTest(NVector3 groundNormal, float threshold = 0f)
    {
        IsGrounded = false;
        if (Simulation == null || Contacts.Count == 0)
            return false;

        foreach (var contact in Contacts)
        {
            var contactNormal = contact.Contact.Normal;

            if (NVector3.Dot(groundNormal, contactNormal) >= threshold) // If the body is supported by a contact whose surface is against gravity
            {
                return true;
            }
        }

        return false;
    }

    private void UpdateFallSpeed(float delta)
    {
        _airTime += delta * FallSpeedMultiplier;
        if (IsGrounded || _airTime <= PeakAirTime)
        {
            _airTime = 0;
        }
        else
        {
            LinearVelocity = new Vector3(0, -9.8f * (_airTime + 1), 0);
        }
    }

    bool IContactEventHandler.NoContactResponse => NoContactResponse;
    void IContactEventHandler.OnStartedTouching<TManifold>(CollidableReference eventSource, CollidableReference other, ref TManifold contactManifold, int contactIndex, BepuSimulation bepuSimulation) => OnStartedTouching(eventSource, other, ref contactManifold, contactIndex, bepuSimulation);
    void IContactEventHandler.OnStoppedTouching<TManifold>(CollidableReference eventSource, CollidableReference other, ref TManifold contactManifold, int contactIndex, BepuSimulation bepuSimulation) => OnStoppedTouching(eventSource, other, ref contactManifold, contactIndex, bepuSimulation);

    protected bool NoContactResponse => false;

    protected virtual void OnStartedTouching<TManifold>(CollidableReference eventSource, CollidableReference other, ref TManifold contactManifold, int contactIndex, BepuSimulation bepuSimulation) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        contactManifold.GetContact(contactIndex, out var contact);
        contact.Offset = contact.Offset + Entity.Transform.WorldMatrix.TranslationVector.ToNumeric() + CenterOfMass.ToNumeric();
        Contacts.Add((other, contact));
    }

    protected virtual void OnStoppedTouching<TManifold>(CollidableReference eventSource, CollidableReference other, ref TManifold contactManifold, int contactIndex, BepuSimulation bepuSimulation) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        for (int i = Contacts.Count - 1; i >= 0; i--)
        {
            if (Contacts[i].Source == other)
                Contacts.SwapRemoveAt(i);
        }
    }

}


