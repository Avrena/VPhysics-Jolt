
#include "cbase.h"

#include "vjolt_layers.h"

#include "vjolt_controller_player.h"
#include "vjolt_debugrender.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar vjolt_player_collision_tolerance( "vjolt_player_collision_tolerance", "0.05" );
static ConVar vjolt_player_character_padding( "vjolt_player_character_padding", "0.02" );

static ConVar vjolt_player_debug( "vjolt_player_debug", "0" );
static ConVar vjolt_player_disable_limit( "vjolt_player_disable_limit", "0.1", 0, "The min speed^2 before we just go where physics wants to take us" );

// Player-vs-player separation is game movement's job, not vphysics'. With
// mutual collision on, overlapped player hulls (spawn rushes, crowded cells)
// force Jolt's EPA penetration-depth solver on every mutual pair every
// substep, which melts high-population servers. Default off: player hulls go
// in the MOVING_PLAYER layer, which collides with everything MOVING does
// except other player hulls. Toggles live (layer is refreshed every presim).
static ConVar vjolt_player_self_collision( "vjolt_player_self_collision", "0", FCVAR_NONE,
	"Whether player character hulls collide with each other in the physics world." );

// Character bodies that leave the playable coordinate range free-fall forever -- there is
// nothing outside the world to land on -- so their coordinates run away, degrade the
// broadphase tree for every query and eventually go non-finite, which then spreads to other
// characters through ground-detection queries (observed live: players parked in the void by
// AFK systems poisoned dozens of in-world players within minutes). Freeze the character at
// the bound instead; it recovers automatically on the next in-range target or teleport.
static ConVar vjolt_character_world_bound( "vjolt_character_world_bound", "20480", FCVAR_NONE,
	"Freeze player character bodies whose position leaves +/- this many units (0 = off)." );

// Cap on the effective velocity derived from character movement each step. Penetration
// correction (and historically, teleports) can move the character much further in one step
// than any legitimate motion; the derived spike otherwise feeds shadow collision events
// (phantom crush damage for players standing against props) and the impulse-based speed
// fallback. sv_maxvelocity defaults to 3500; raise this on extreme-velocity (surf) servers.
static ConVar vjolt_character_max_effective_velocity( "vjolt_character_max_effective_velocity", "8000", FCVAR_NONE,
	"Cap (u/s) on the per-step effective velocity derived from player character movement." );

static uint8 GetPlayerObjectLayer()
{
	return vjolt_player_self_collision.GetBool() ? Layers::MOVING : Layers::MOVING_PLAYER;
}

//-------------------------------------------------------------------------------------------------

// Component-wise Vector clamp
static Vector ClampVector( const Vector &x, const Vector &min, const Vector &max )
{
	return Vector(
		Clamp( x.x, min.x, max.x ),
		Clamp( x.y, min.y, max.y ),
		Clamp( x.z, min.z, max.z ) );
}

static void ComputePlayerController( Vector &vCurrentSpeed, const Vector &vDelta, const Vector &vMaxSpeed, float flScaleDelta, float flDamping, Vector *pOutImpulse )
{
	if ( vCurrentSpeed.LengthSqr() < 1e-6f )
	{
		vCurrentSpeed = vec3_origin;
	}

	Vector vDampAccel = vCurrentSpeed * -flDamping;
	Vector vDeltaAccel = vDelta * flScaleDelta;

	Vector vAcceleration = ClampVector( vDeltaAccel + vDampAccel, -vMaxSpeed, vMaxSpeed );

	vCurrentSpeed += vAcceleration;
	if ( pOutImpulse )
		*pOutImpulse = vAcceleration;
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsPlayerController::JoltPhysicsPlayerController( JoltPhysicsObject *pObject )
{
	SetObjectInternal( pObject );
}

JoltPhysicsPlayerController::~JoltPhysicsPlayerController()
{
	SetObjectInternal( nullptr );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground )
{
	// Reject poisoned game input: a non-finite target would persist in m_vTargetPosition and
	// re-poison the character every presim, defeating any body-side cleanup (observed live:
	// bodies healed in place were re-poisoned until the entity itself teleported, which
	// routes a fresh sane position through here).
	if ( !IsSaneVector( position, kMaxSaneCoordSource ) ||
		 !IsSaneVector( velocity, kMaxSaneVelocitySource ) ||
		 !std::isfinite( secondsToArrival ) )
	{
		if ( m_SanityLogThrottle.ShouldLog() )
			Log_Warning( LOG_VJolt, "Player controller %p: ignoring non-finite/runaway update (pos %g %g %g, vel %g %g %g)\n",
				this, position.x, position.y, position.z, velocity.x, velocity.y, velocity.z );
		return;
	}

	m_bUpdatedSinceLast = true;

	// Mirror the game-driven velocity onto the object for collision-event reporting
	// (see GetPlayerDrivenVelocity): impact damage must see what the game is driving
	// the player at, not the controller's correction velocity. Deliberately above the
	// early-out so the object's copy can never go stale relative to m_vCurrentSpeed.
	if ( m_pObject )
		m_pObject->SetPlayerDrivenVelocity( velocity );

	if ( ( velocity - m_vCurrentSpeed ).LengthSqr() < 1e-6f && ( position - m_vTargetPosition ).LengthSqr() < 1e-6f )
		return;

	m_vTargetPosition = position;
	m_flSecondsToArrival = secondsToArrival < 0 ? 0 : secondsToArrival;

	m_vCurrentSpeed = velocity;
	m_pCharacter->Activate();

	// Source passes CMoveData::m_outWishVel here, which is the velocity applied by this
	// command, not the player's desired or achieved velocity. It can legitimately be zero
	// once the player reaches speed. On moveable physics ground Source does not replace that
	// zero with its usual max-speed fallback, even though the swept position target continues
	// to advance. Disabling solely from velocity therefore makes the shadow advance only on
	// acceleration ticks (the visible zero/advance sticking cadence).
	Vector vObjectPosition;
	m_pObject->GetPosition( &vObjectPosition, nullptr );
	const Vector vPositionError = position - vObjectPosition;
	const float flPositionTolerance = Max( vjolt_player_collision_tolerance.GetFloat(), 1e-3f );
	const bool bHasVelocityDrive = velocity.LengthSqr() > vjolt_player_disable_limit.GetFloat();
	const bool bHasPositionDrive = vPositionError.LengthSqr() > flPositionTolerance * flPositionTolerance;
	m_bEnable = bHasVelocityDrive || bHasPositionDrive;

	if ( bHasVelocityDrive )
	{
		MaxSpeed( velocity );
	}
	else if ( bHasPositionDrive )
	{
		// Source still supplied a future position but no newly-applied velocity. Derive the
		// correction authority from that already-swept target and its requested arrival time.
		// This feeds the normal controller/simulation path; it is not a post-step teleport.
		const float flArrivalTime = Max( m_flSecondsToArrival, 1e-4f );
		m_vMaxSpeed = Abs( vPositionError / flArrivalTime );
	}

	// We ignore the given ground here, we use the Jolt player controller's ground.
}

void JoltPhysicsPlayerController::SetEventHandler( IPhysicsPlayerControllerEvent *handler )
{
	m_pHandler = handler;
}

bool JoltPhysicsPlayerController::IsInContact()
{
	uint32 nState = GetContactState( 0 );
	return !!( nState & PLAYER_CONTACT_PHYSICS );
}

void JoltPhysicsPlayerController::MaxSpeed( const Vector &velocity )
{
	Vector vCurrentVelocity;
	m_pObject->GetVelocity( &vCurrentVelocity, nullptr );

	Vector vDirection = velocity;
	float flLength = VectorNormalize( vDirection ); // Normalizes in place.

	float flDot = DotProduct( vDirection, vCurrentVelocity );
	if ( flDot > 0 )
	{
		m_vMaxSpeed = Abs( velocity - ( vDirection * flDot * flLength ) );
	}
	else
	{
		m_vMaxSpeed = Abs( velocity );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::SetObject( IPhysicsObject *pObject )
{
	SetObjectInternal( static_cast<JoltPhysicsObject *>( pObject ) );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsPlayerController::GetShadowPosition( Vector *position, QAngle *angles )
{
	return m_pObject->GetShadowPosition( position, angles );
}

void JoltPhysicsPlayerController::StepUp( float height )
{
	if ( height == 0.0f )
		return;

	Vector vPos;
	QAngle qAngles;
	m_pObject->GetPosition( &vPos, &qAngles );
	vPos.z += height;
	// Teleport, do not influence implicit velocity.
	m_pObject->SetPosition( vPos, qAngles, true );
}

void JoltPhysicsPlayerController::Jump()
{
	// This does nothing in VPhysics.
}

void JoltPhysicsPlayerController::GetShadowVelocity( Vector *velocity )
{
	if ( !velocity )
		return;

	m_pObject->GetVelocity( velocity, nullptr );

	Vector vBaseVelocity = JoltToSource::Distance( m_pCharacter->GetGroundVelocity() );
	*velocity -= vBaseVelocity;
}

IPhysicsObject *JoltPhysicsPlayerController::GetObject()
{
	return m_pObject;
}

void JoltPhysicsPlayerController::GetLastImpulse( Vector *pOut )
{
	if ( pOut )
		*pOut = m_vLastImpulse;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::SetPushMassLimit( float maxPushMass )
{
	m_flPushableMassLimit = maxPushMass;
}

void JoltPhysicsPlayerController::SetPushSpeedLimit( float maxPushSpeed )
{
	m_flPushableSpeedLimit = maxPushSpeed;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsPlayerController::GetPushMassLimit()
{
	return m_flPushableMassLimit;
}

float JoltPhysicsPlayerController::GetPushSpeedLimit()
{
	return m_flPushableSpeedLimit;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsPlayerController::WasFrozen()
{
	// I think here the code is referring to IVP freezing objects after inactivity (sleeping),
	// our objects are forced to never sleep, so we don't need to care?
	return false;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsPlayerController::OnContactValidate( const JPH::CharacterVirtual* inCharacter, const JPH::BodyID& inBodyID2, const JPH::SubShapeID& inSubShapeID2 )
{
	JPH::Body *pOtherBody = m_pObject->GetJoltEnvironment()->GetPhysicsSystem()->GetBodyLockInterfaceNoLock().TryGetBody( inBodyID2 );
	JoltPhysicsObject* pOtherObject = reinterpret_cast< JoltPhysicsObject* >( pOtherBody->GetUserData() );
	JoltPhysicsContactListener *pListener = m_pObject->GetJoltEnvironment()->GetContactListener();
	return pListener->ShouldCollide( m_pObject, pOtherObject );
}

void JoltPhysicsPlayerController::OnContactAdded( const JPH::CharacterVirtual* inCharacter, const JPH::BodyID& inBodyID2, const JPH::SubShapeID& inSubShapeID2, JPH::RVec3Arg inContactPosition, JPH::Vec3Arg inContactNormal, JPH::CharacterContactSettings& ioSettings )
{
	JoltPhysicsContactListener *pListener = m_pObject->GetJoltEnvironment()->GetContactListener();
	( void )pListener;
}

//-------------------------------------------------------------------------------------------------

static void CheckCollision( JoltPhysicsObject *pObject, JPH::CollideShapeCollector &ioCollector, JPH::BodyFilter &ioFilter )
{
	if ( !pObject->IsCollisionEnabled() ) 
	    return;
	
	JPH::PhysicsSystem *pSystem = pObject->GetJoltEnvironment()->GetPhysicsSystem();

	if ( !pObject->IsCollisionEnabled() )
		return;

	// Create query broadphase layer filter (mirrors the character body's own
	// layer so player-vs-player contacts match what the simulation generates)
	JPH::DefaultBroadPhaseLayerFilter broadphase_layer_filter = pSystem->GetDefaultBroadPhaseLayerFilter( GetPlayerObjectLayer() );

	// Create query object layer filter
	JPH::DefaultObjectLayerFilter object_layer_filter = pSystem->GetDefaultLayerFilter( GetPlayerObjectLayer() );

	// Determine position to test
	JPH::Vec3 position;
	JPH::Quat rotation;
	JPH::BodyInterface &bi = pSystem->GetBodyInterfaceNoLock();
	bi.GetPositionAndRotation( pObject->GetBodyID(), position, rotation );
	JPH::Mat44 query_transform = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pObject->GetBody()->GetShape()->GetCenterOfMass() );

	// Settings for collide shape
	JPH::CollideShapeSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = bi.GetLinearVelocity( pObject->GetBodyID() );
	settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
	settings.mMaxSeparationDistance = vjolt_player_character_padding.GetFloat();

	pSystem->GetNarrowPhaseQueryNoLock().CollideShape( pObject->GetBody()->GetShape(), JPH::Vec3::sReplicate( 1.0f ), query_transform, settings, JPH::Vec3::sZero(), ioCollector, broadphase_layer_filter, object_layer_filter, ioFilter );
}


template <bool MoveablesOnly>
class SourceHitFilter : public JPH::BodyFilter
{
public:
	SourceHitFilter( JPH::PhysicsSystem* pPhysicsSystem, JoltPhysicsObject* pSelfObject )
		: m_pPhysicsSystem( pPhysicsSystem )
		, m_pSelfObject( pSelfObject )
	{
	}

	bool ShouldCollideLocked( const JPH::Body &inBody ) const override
	{
		JoltPhysicsObject* pObject = reinterpret_cast<JoltPhysicsObject*>( inBody.GetUserData() );

		// Ignore self if specified. This can be nullptr if you don't want this.
		if ( pObject == m_pSelfObject )
			return false;

		if constexpr ( MoveablesOnly )
		{
			if ( pObject->IsTrigger() || !pObject->IsMoveable() )
				return false;
		}

		if ( !pObject->GetJoltEnvironment()->GetContactListener()->ShouldCollide( m_pSelfObject, pObject ) )
			return false;

		return true;
	}

private:
	JPH::PhysicsSystem	*m_pPhysicsSystem;
	JoltPhysicsObject	*m_pSelfObject;
};

uint32 JoltPhysicsPlayerController::GetContactState( uint16 nGameFlags )
{
	// This does not seem to affect much, we should aspire to have our physics be as 1:1 to brush collisions as possible anyway
	// Raphael: I was getting stuck at the slightest touch with this enabled on 64x Gmod.
#if defined( GAME_PORTAL2_OR_NEWER ) && !defined( GAME_GMOD )
	if ( !m_pObject->IsCollisionEnabled() )
		return 0;

	// Collector that finds the hit with the normal that is the most 'up'
	class ContactStateCollector : public JPH::CollideShapeCollector
	{
	public:
		ContactStateCollector( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pPlayerObject, uint16 nGameFlags )
			: m_pPhysicsSystem( pPhysicsSystem )
			, m_pPlayerObject( pPlayerObject )
			, m_nGameFlags( nGameFlags )
		{
		}

		void Reset() override
		{
			JPH::CollideShapeCollector::Reset();

			m_nFlagsOut = 0;
		}

		void AddHit( const JPH::CollideShapeResult &inResult ) override
		{
			JPH::BodyLockRead lock( m_pPhysicsSystem->GetBodyLockInterfaceNoLock(), inResult.mBodyID2 );
			const JPH::Body &body = lock.GetBody();

			JoltPhysicsObject *pObject = reinterpret_cast<JoltPhysicsObject *>( body.GetUserData() );

			if ( !pObject->IsControlledByGame() )
				m_nFlagsOut |= PLAYER_CONTACT_PHYSICS;

			if ( pObject->GetGameFlags() & m_nGameFlags )
				m_nFlagsOut |= PLAYER_CONTACT_GAMEOBJECT;
		}

		uint32					m_nFlagsOut = 0;

	private:
		JPH::PhysicsSystem		*m_pPhysicsSystem;
		JoltPhysicsObject		*m_pPlayerObject;
		uint16					m_nGameFlags;
	};

	JPH::PhysicsSystem *pSystem = m_pObject->GetEnvironment()->GetPhysicsSystem();
	ContactStateCollector collector( pSystem, m_pObject, nGameFlags );
	SourceHitFilter<true> filter( pSystem, m_pObject );
	CheckCollision( m_pObject, collector, filter );

	return collector.m_nFlagsOut;
#else
	return 0;
#endif
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsPlayerController::TryTeleportObject()
{
	if ( m_pHandler )
	{
		if ( !m_pHandler->ShouldMoveTo( m_pObject, m_vTargetPosition ) )
			return 0;
	}

	QAngle qCurrentAngles;
	m_pObject->GetPosition( nullptr, &qCurrentAngles );
	m_pObject->SetPosition( m_vTargetPosition, qCurrentAngles, true );
	m_pCharacter->SetPosition( SourceToJolt::Distance( m_vTargetPosition ) );

	// The teleport moved the character; without this, OnPostSimulate derives an effective
	// velocity from (new - stale old) / dt across the whole teleport distance and hands the
	// game a huge impulse spike (phantom impact damage / camera shake), which also runs away
	// through the m_vLastImpulse-based speed fallback.
	m_vOldPosition = m_vTargetPosition;
	m_vLastImpulse = vec3_origin;
	return 1;
}

void JoltPhysicsPlayerController::OnPreSimulate( float flDeltaTime )
{
	// The object and the character copy positions into each other every frame (object ->
	// character here, character -> object in OnPostSimulate), so one non-finite transform
	// self-sustains and re-poisons whichever side gets cleaned first. Sanitize the pair
	// before mirroring.
	Vector vObjectPosition;
	QAngle qObjectAngle;
	m_pObject->GetPosition( &vObjectPosition, &qObjectAngle );

	if ( !IsSaneVector( vObjectPosition, kMaxSaneCoordSource ) || !IsSaneQAngle( qObjectAngle ) )
	{
		// Body poisoned: recover to the game's requested target if sane, else the last good
		// simulated position. vec3_origin only as the absolute last resort -- the game heals
		// anything left there with its next real teleport.
		Vector vRecover = IsSaneVector( m_vTargetPosition, kMaxSaneCoordSource ) ? m_vTargetPosition : m_vOldPosition;
		if ( !IsSaneVector( vRecover, kMaxSaneCoordSource ) )
			vRecover = vec3_origin;

		if ( m_SanityLogThrottle.ShouldLog() )
			Log_Warning( LOG_VJolt, "Player controller %p: non-finite body position (%g %g %g), recovering to (%g %g %g)\n",
				this, vObjectPosition.x, vObjectPosition.y, vObjectPosition.z, vRecover.x, vRecover.y, vRecover.z );

		vObjectPosition = vRecover;
		qObjectAngle = vec3_angle;
		m_pObject->SetPosition( vObjectPosition, qObjectAngle, true );
		m_pObject->SetVelocity( &vec3_origin, &vec3_origin );
		m_vCurrentSpeed = vec3_origin;
		m_pObject->SetPlayerDrivenVelocity( vec3_origin );
		m_vLastImpulse = vec3_origin;
		m_bEnable = false; // Wait for a fresh game update before driving again.
	}

	// Characters outside the playable coordinate range free-fall forever and run their
	// coordinates away (see vjolt_character_world_bound). Snap back to the game's target if
	// it is in range, otherwise freeze at the bound until the game moves the player.
	const float flWorldBound = vjolt_character_world_bound.GetFloat();
	if ( flWorldBound > 0.0f && !IsSaneVector( vObjectPosition, flWorldBound ) )
	{
		if ( IsSaneVector( m_vTargetPosition, flWorldBound ) && TryTeleportObject() )
		{
			// Restore the proper layer before returning so the recovery frame does not
			// simulate one step in NO_COLLIDE (visible as a small floor-sink pop).
			m_pCharacter->SetLayer( m_pObject->IsCollisionEnabled() ? GetPlayerObjectLayer() : Layers::NO_COLLIDE );
			if ( m_bCharOutOfWorld )
			{
				m_bCharOutOfWorld = false;
				Log_Msg( LOG_VJolt, "Player controller %p: character recovered to in-range target (%.0f %.0f %.0f)\n",
					this, m_vTargetPosition.x, m_vTargetPosition.y, m_vTargetPosition.z );
			}
			return;
		}

		const Vector vClamped(
			Clamp( vObjectPosition.x, -flWorldBound, flWorldBound ),
			Clamp( vObjectPosition.y, -flWorldBound, flWorldBound ),
			Clamp( vObjectPosition.z, -flWorldBound, flWorldBound ) );

		if ( !m_bCharOutOfWorld )
		{
			m_bCharOutOfWorld = true;
			Log_Warning( LOG_VJolt, "Player controller %p: character left the world at (%.0f %.0f %.0f), freezing at the bound (recovers on next in-range update)\n",
				this, vObjectPosition.x, vObjectPosition.y, vObjectPosition.z );
		}

		m_pObject->SetPosition( vClamped, qObjectAngle, true );
		m_pObject->SetVelocity( &vec3_origin, &vec3_origin );
		m_pCharacter->SetPositionAndRotation( SourceToJolt::Distance( vClamped ), SourceToJolt::Angle( qObjectAngle ), JPH::EActivation::DontActivate );
		m_pCharacter->SetLinearVelocity( JPH::Vec3::sZero() );
		m_pCharacter->SetLayer( Layers::NO_COLLIDE );

		// Put the body to sleep so gravity stops re-accelerating it into the clamp every
		// frame. Update() / TryTeleportObject reactivate it on recovery.
		m_pObject->GetJoltEnvironment()->GetPhysicsSystem()->GetBodyInterfaceNoLock().DeactivateBody( m_pCharacter->GetBodyID() );

		m_vOldPosition = vClamped;
		m_vLastImpulse = vec3_origin;
		return;
	}

	if ( m_bCharOutOfWorld )
	{
		m_bCharOutOfWorld = false;
		Log_Msg( LOG_VJolt, "Player controller %p: character back in range at (%.0f %.0f %.0f)\n",
			this, vObjectPosition.x, vObjectPosition.y, vObjectPosition.z );
	}

	m_pCharacter->SetLayer( m_pObject->IsCollisionEnabled() ? GetPlayerObjectLayer() : Layers::NO_COLLIDE );

	// Update position from dummy object.
	m_pCharacter->SetPositionAndRotation( SourceToJolt::Distance( vObjectPosition ), SourceToJolt::Angle( qObjectAngle ), JPH::EActivation::DontActivate );

	Vector vOldPosition = JoltToSource::Distance( m_pCharacter->GetPosition() );
	Vector vOldVelocity = JoltToSource::Distance( m_pCharacter->GetLinearVelocity() );

	// The character's carried velocity can hold solver artifacts; a non-finite or runaway
	// value here would flow through ComputePlayerController into the next step.
	if ( !IsSaneVector( vOldVelocity, kMaxSaneVelocitySource ) )
	{
		vOldVelocity = vec3_origin;
		m_pCharacter->SetLinearVelocity( JPH::Vec3::sZero() );
	}

	Vector vDeltaPos = m_vTargetPosition - vOldPosition;

	if ( m_bEnable )
	{
		// Totally bogus! Measure error using last known estimate not current position.
		if ( vDeltaPos.LengthSqr() > JPH::Square( m_flMaxDeltaPosition ) )
		{
			if ( TryTeleportObject() )
				return;
		}
	}

	float flFraction = Min( m_flSecondsToArrival > 0.0f ? flDeltaTime / m_flSecondsToArrival : 1.0f, 1.0f );

	// XXX TODO Set Mass
	//m_pCharacter->GetBodyID()->SetMass(m_pObject->GetMass() * vjolt_player_mass_scale.GetFloat());
	m_pCharacter->SetPosition( SourceToJolt::Distance( vOldPosition ) );

	if ( m_bEnable )
	{
		Vector vGroundVelocity = JoltToSource::Distance( m_pCharacter->GetGroundVelocity() );

		Vector vControllerVelocity = vOldVelocity;

		vControllerVelocity -= vGroundVelocity;
		if ( !m_bUpdatedSinceLast )
		{
			float flLen = m_vLastImpulse.Length();
			Vector vTempMaxSpeed = Vector( flLen, flLen, flLen );
			ComputePlayerController( vControllerVelocity, vDeltaPos, vTempMaxSpeed, flFraction / flDeltaTime, m_flDampFactor, nullptr );
		}
		else
		{
			ComputePlayerController( vControllerVelocity, vDeltaPos, m_vMaxSpeed, flFraction / flDeltaTime, m_flDampFactor, &m_vLastImpulse );
		}
		vControllerVelocity += vGroundVelocity;

		// Final gate before the velocity enters the simulation: ground velocity comes from
		// an arbitrary body and the controller math divides by dt, so keep this finite.
		if ( !IsSaneVector( vControllerVelocity, kMaxSaneVelocitySource ) )
		{
			if ( m_SanityLogThrottle.ShouldLog() )
				Log_Warning( LOG_VJolt, "Player controller %p: sanitized non-finite controller velocity\n", this );
			vControllerVelocity = vec3_origin;
		}

		/* Way too experimental
		if (m_pCharacter->IsSupported())
		{
			auto groundBodyID = m_pCharacter->GetGroundBodyID();
			if (!groundBodyID.IsInvalid())
			{
				const JPH::BodyLockInterfaceNoLock &bodyLockInterface = m_pObject->GetEnvironment()->GetPhysicsSystem()->GetBodyLockInterfaceNoLock();
				JPH::Body *body = bodyLockInterface.TryGetBody(groundBodyID);

				if (body && body->IsDynamic())
				{
					// RaphaelIT7: We remove any velocity below this because else often you cannot jump off moving props.
					constexpr float kVelocityEpsilon = 0.025f;
					if (fabsf(vControllerVelocity.x) < kVelocityEpsilon)
						vControllerVelocity.x = 0.0f;

					if (fabsf(vControllerVelocity.y) < kVelocityEpsilon)
						vControllerVelocity.y = 0.0f;

					if (fabsf(vControllerVelocity.z) < kVelocityEpsilon)
						vControllerVelocity.z = 0.0f;
				}
			}
		}*/

		m_pCharacter->SetLinearVelocity( SourceToJolt::Distance( vControllerVelocity ) );
	}
	else
	{
		// IVP parity: an idle controller still holds the shadow against solver noise. If the
		// carried velocity is left untouched here, contact jitter from awake bodies underfoot
		// integrates tick over tick into a slow slide that the game's shadow-follow then turns
		// into real player motion (observed live: players standing on any unfrozen physics
		// object crept until their feet found brushes; zero drift on brushes or frozen props,
		// whose bodies do not jitter). Damp the carried velocity down to the ground's own.
		// Real shoves are unaffected: their displacement happens inside the simulation step
		// and re-enters as game-driven velocity, which re-enables the controller.
		Vector vGroundVelocity = JoltToSource::Distance( m_pCharacter->GetGroundVelocity() );
		Vector vIdleVelocity = ( vOldVelocity - vGroundVelocity ) * Clamp( 1.0f - m_flDampFactor, 0.0f, 1.0f ) + vGroundVelocity;
		if ( !IsSaneVector( vIdleVelocity, kMaxSaneVelocitySource ) )
			vIdleVelocity = vec3_origin;
		m_pCharacter->SetLinearVelocity( SourceToJolt::Distance( vIdleVelocity ) );
	}

	m_vOldPosition = vOldPosition;
}

void JoltPhysicsPlayerController::OnPostSimulate( float flDeltaTime )
{
	m_pCharacter->PostSimulation( vjolt_player_collision_tolerance.GetFloat() );

	// Calculate effective velocity
	Vector vNewPosition = JoltToSource::Distance( m_pCharacter->GetPosition() );

	// The character stepped into non-finite territory (broadphase-poisoning contagion, e.g.
	// through a poisoned ground body): restore the last sane transform and freeze instead of
	// propagating the poison into the game-visible object.
	if ( !IsSaneVector( vNewPosition, kMaxSaneCoordSource ) )
	{
		if ( m_SanityLogThrottle.ShouldLog() )
			Log_Warning( LOG_VJolt, "Player controller %p: character went non-finite post-step, restoring (%g %g %g)\n",
				this, m_vOldPosition.x, m_vOldPosition.y, m_vOldPosition.z );

		vNewPosition = IsSaneVector( m_vOldPosition, kMaxSaneCoordSource ) ? m_vOldPosition : vec3_origin;
		m_pCharacter->SetPosition( SourceToJolt::Distance( vNewPosition ), JPH::EActivation::DontActivate );
		m_pCharacter->SetLinearVelocity( JPH::Vec3::sZero() );
		m_pObject->SetPosition( vNewPosition, vec3_angle, false );
		m_pObject->SetVelocity( &vec3_origin, &vec3_origin );
		m_vCurrentSpeed = vec3_origin;
		m_pObject->SetPlayerDrivenVelocity( vec3_origin );
		m_vLastImpulse = vec3_origin;
		m_bEnable = false; // Wait for a fresh game update before driving again.
		return;
	}

	Vector vNewVelocity = ( vNewPosition - m_vOldPosition ) / flDeltaTime;

	// Cap the effective velocity: penetration-correction snaps can move the character much
	// further in one step than legitimate motion ever does, and this derived value feeds
	// both the game's shadow collision events (impact damage) and the speed fallback above.
	const float flEffectiveSpeed = vNewVelocity.Length();
	const float flMaxEffectiveSpeed = vjolt_character_max_effective_velocity.GetFloat();
	if ( !std::isfinite( flEffectiveSpeed ) )
		vNewVelocity = vec3_origin;
	else if ( flMaxEffectiveSpeed > 0.0f && flEffectiveSpeed > flMaxEffectiveSpeed )
		vNewVelocity *= flMaxEffectiveSpeed / flEffectiveSpeed;

	AngularImpulse vAngularImpulse = vec3_origin;

	m_pObject->SetPosition( vNewPosition, vec3_angle, false );
	m_pObject->SetVelocity( &vNewVelocity, &vAngularImpulse );

	m_vLastImpulse = vNewVelocity;

	if ( vjolt_player_debug.GetBool() )
	{
		JoltPhysicsDebugRenderer& debugRenderer = JoltPhysicsDebugRenderer::GetInstance();

		// Draw last impulse as a blue line.
		debugRenderer.DrawLine(
			SourceToJolt::Distance( vNewPosition ),
			SourceToJolt::Distance( vNewPosition + m_vLastImpulse ),
			JPH::Color( 0, 0, 255, 255 ) );

		// Draw new player velocity as a purple line.
		debugRenderer.DrawLine(
			SourceToJolt::Distance( vNewPosition ),
			SourceToJolt::Distance( vNewPosition + vNewVelocity ),
			JPH::Color( 255, 0, 255, 255 ) );

		Vector vecMins, vecMaxs;
		JPH::Mat44 matComTransform = m_pCharacter->GetWorldTransform().PreTranslated( m_pCharacter->GetShape()->GetCenterOfMass() );
		JoltToSource::AABBBounds( m_pCharacter->GetShape()->GetWorldSpaceBounds( matComTransform, JPH::Vec3{ 1.0f, 1.0f, 1.0f } ), vecMins, vecMaxs );
		debugRenderer.GetDebugOverlay()->AddBoxOverlay( vec3_origin, vecMins, vecMaxs, QAngle(), m_bEnable ? 0 : 255, m_bEnable ? 255 : 0, 0, 100, 0.0f );

#if 0
		Log_Msg( LOG_VJolt,
			"Player State:\n"
			"  vOldPosition: %g %g %g\n"
			"  vNewPosition: %g %g %g\n"
			"  vNewVelocity: %g %g %g\n"
			"  m_vLastImpulse: %g %g %g\n",
			m_vOldPosition.x, m_vOldPosition.x, m_vOldPosition.z,
			vNewPosition.x, vNewPosition.x, vNewPosition.z,
			vNewVelocity.x, vNewVelocity.x, vNewVelocity.z,
			m_vLastImpulse.x, m_vLastImpulse.x, m_vLastImpulse.z
		);
#endif
	}

	if ( m_bEnable )
	{
		m_flSecondsToArrival = Max( m_flSecondsToArrival - flDeltaTime, 0.0f );
	}
}

void JoltPhysicsPlayerController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( pObject == m_pObject )
	{
		SetObjectInternal( nullptr );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::SetObjectInternal( JoltPhysicsObject *pObject )
{
	if ( m_pObject == pObject )
		return;

	// Reset the last object
	if ( m_pObject )
	{
		m_pObject->RemoveDestroyedListener( this );
		m_pObject->RemoveCallbackFlags( CALLBACK_IS_PLAYER_CONTROLLER );
		m_pObject->SetPlayerDrivenVelocity( vec3_origin );
		m_pObject->UpdateLayer();

		m_pCharacter->RemoveFromPhysicsSystem();
		m_pCharacter = nullptr;
	}

	// Set our new object
	m_pObject = pObject;

	// Controllers are reused across respawns: the game detaches the dying player's object
	// and attaches the fresh one, often same-frame during mass respawn waves. Carrying
	// kinematic state across bodies drove the first presim from the PREVIOUS body's target
	// and impulse -- including its poison if it died non-finite (observed live: NaN
	// mass-poisoning onset correlated with simultaneous job-change respawns). Start neutral;
	// the game's first Update() re-arms the controller.
	m_vMaxSpeed = vec3_origin;
	m_vCurrentSpeed = vec3_origin;
	m_vLastImpulse = vec3_origin;
	m_flSecondsToArrival = 0.0f;
	m_bEnable = false;
	m_bUpdatedSinceLast = false;
	m_bCharOutOfWorld = false;

	// Adjust the new object
	if ( m_pObject )
	{
		// Set kinematic
		m_pObject->GetBody()->SetMotionType( JPH::EMotionType::Kinematic );
		m_pObject->AddDestroyedListener( this );
		m_pObject->AddCallbackFlags( CALLBACK_IS_PLAYER_CONTROLLER );
		// Objects can be recycled across controller attachments; never inherit a stale
		// game-driven velocity into collision-event reporting.
		m_pObject->SetPlayerDrivenVelocity( vec3_origin );
		m_pObject->UpdateLayer();

		static constexpr float k_flNormalSurfaceFriction = 0.8f; // Default surface friction.
		// We can't always get external convars in VPhysics Jolt sadly...
		// At least to my knowledge.
		// Assume a friction of "8" (the default) for now
		//ConVarRef sv_friction( "sv_friction" );
		static constexpr float sv_friction = 8;

		JPH::Ref<JPH::CharacterSettings> settings = new JPH::CharacterSettings();
		settings->mMass                        = m_pObject->GetMass();
		settings->mLayer                       = GetPlayerObjectLayer();
		settings->mUp                          = JPH::Vec3::sAxisZ();
		// IVP parity: the player shadow object does NOT gravity-fall on its own -- game
		// movement owns player gravity and feeds the resulting velocity through Update()
		// (which enables the controller whenever it is nonzero). With Jolt's default
		// gravity the character free-falls whenever the controller is idle: a player
		// standing still floats-in-place under IVP, but here a player parked outside the
		// world sank forever, ran its coordinates away and seeded the NaN contagion this
		// branch contains -- and driven characters had gravity applied on top of the
		// game-supplied velocity that already includes it.
		settings->mGravityFactor               = 0.0f;
		settings->mFriction                    = sv_friction * k_flNormalSurfaceFriction * ( 1.0f / 64.0f ); // Account for Source's friction being tick based.
		settings->mShape                       = m_pObject->GetBody()->GetShape();
		settings->mMaxSlopeAngle               = JPH::DegreesToRadians( 45.573 );
		settings->mEnhancedInternalEdgeRemoval = true;

		// Seed the controller's positional state from the new body, sanitized: a character
		// born at a non-finite position poisons the broadphase on its first step.
		Vector vStartPosition;
		m_pObject->GetPosition( &vStartPosition, nullptr );
		if ( !IsSaneVector( vStartPosition, kMaxSaneCoordSource ) )
		{
			if ( m_SanityLogThrottle.ShouldLog() )
				Log_Warning( LOG_VJolt, "Player controller %p: attached object has non-finite position (%g %g %g), seeding at origin\n",
					this, vStartPosition.x, vStartPosition.y, vStartPosition.z );
			vStartPosition = vec3_origin;
			m_pObject->SetPosition( vStartPosition, QAngle(), true );
		}
		m_vTargetPosition = vStartPosition;
		m_vOldPosition = vStartPosition;

		m_pCharacter = new JPH::Character( settings, SourceToJolt::Distance( vStartPosition ), JPH::Quat::sIdentity(), m_pObject->GetBody()->GetUserData(), m_pObject->GetJoltEnvironment()->GetPhysicsSystem() );
		m_pCharacter->AddToPhysicsSystem();
	}
}
