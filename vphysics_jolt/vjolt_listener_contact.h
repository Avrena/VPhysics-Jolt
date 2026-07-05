
#pragma once

#include "vjolt_controller_fluid.h"
#include "vjolt_surfaceprops.h"
#include "ankerl/unordered_dense.h"
#include <vector>

#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>

extern ConVar vjolt_contact_estimate;
extern ConVar vjolt_shadow_collision_min_speed;
extern ConVar vjolt_collision_event_validity_guard;

#if GAME_GMOD
// Defined in vjolt_object.cpp. Returns true iff pObject is still in the live-object registry
// (g_pObjects) -- a pointer-membership test only, never dereferences pObject, so it is safe on a
// stale/freed pointer (the wrapper is Unregistered at the top of ~JoltPhysicsObject).
extern bool IsValidPhyiscsObject( IPhysicsObject *pObject );
// Bumped on every object destruction; see the fast-path in FlushCallbacks.
extern uint32 g_JoltObjectDestroyGeneration;
// Soak instrumentation counters (temporary), defined in vjolt_object.cpp; bumped by the guard.
extern uint32 g_nCollisionGuardSkips;
extern uint32 g_nGuardActiveFlushes;
#endif

struct JoltPhysicsContactPair
{
	JoltPhysicsContactPair( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
		: pObject1(pObject1), pObject2(pObject2)
	{
	}

	JoltPhysicsObject *pObject1 = nullptr;
	JoltPhysicsObject *pObject2 = nullptr;
};

enum VPhysicsGameFlags : uint32
{
	FVPHYSICS_DMG_SLICE				= 0x0001,
	FVPHYSICS_CONSTRAINT_STATIC		= 0x0002,
	FVPHYSICS_PLAYER_HELD			= 0x0004,
	FVPHYSICS_PART_OF_RAGDOLL		= 0x0008,
	FVPHYSICS_MULTIOBJECT_ENTITY	= 0x0010,
	FVPHYSICS_HEAVY_OBJECT			= 0x0020,
	FVPHYSICS_PENETRATING			= 0x0040,
	FVPHYSICS_NO_PLAYER_PICKUP		= 0x0080,
	FVPHYSICS_WAS_THROWN			= 0x0100,
	FVPHYSICS_DMG_DISSOLVE			= 0x0200,
	FVPHYSICS_NO_IMPACT_DMG			= 0x0400,
	FVPHYSICS_NO_NPC_IMPACT_DMG		= 0x0800,
	FVPHYSICS_PUSH_PLAYER			= 0x1000,
	FVPHYSICS_NO_SELF_COLLISIONS	= 0x8000,
};

class JoltPhysicsContactListener final : public JPH::ContactListener
{
public:
	JoltPhysicsContactListener( JPH::PhysicsSystem &physicsSystem )
		: m_PhysicsSystem( physicsSystem )
	{
	}

	JPH::ValidateResult OnContactValidate( const JPH::Body &inBody1, const JPH::Body &inBody2, JPH::Vec3Arg inBaseOffset, const JPH::CollideShapeResult &inCollisionResult ) override
	{
		return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	void OnContactAdded( const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings ) override
	{	
		JoltPhysicsObject* pObject1 = reinterpret_cast<JoltPhysicsObject*>( inBody1.GetUserData() );
		JoltPhysicsObject* pObject2 = reinterpret_cast<JoltPhysicsObject*>( inBody2.GetUserData() );

		bool bShouldCollide = ShouldCollide( pObject1, pObject2 );
		// If the game says we shouldn't collide, we will treat this as a sensor
		// to satisfy the StartTouch/EndTouch events.
		ioSettings.mIsSensor = !bShouldCollide || ioSettings.mIsSensor;

		if ( !m_pGameListener )
			return;

		if ( pObject1->IsFluid() || pObject2->IsFluid() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsFluid() && ( pObject2->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidStartTouchEvents.EmplaceBack( uThreadId, pObject1, pObject2 );

			if ( pObject2->IsFluid() && ( pObject1->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidStartTouchEvents.EmplaceBack( uThreadId, pObject2, pObject1 );

			return;
		}

		if ( pObject1->IsTrigger() || pObject2->IsTrigger() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsTrigger() )
				m_EnterTriggerEvents.EmplaceBack( uThreadId, pObject1, pObject2 );
		
			if ( pObject2->IsTrigger() )
				m_EnterTriggerEvents.EmplaceBack( uThreadId, pObject2, pObject1 );
	
			return;
		}

		const bool bIsCollision			= bShouldCollide && JoltPhysicsCollisionEvent::IsCollision		( pObject1, pObject2 );
		const bool bIsShadowCollision	= bShouldCollide && JoltPhysicsCollisionEvent::IsShadowCollision( pObject1, pObject2 );

		if ( bIsCollision || bIsShadowCollision )
		{
			// Josh:
			// We know ahead of time what this is used for (playing sounds and such)
			// and it is not easily threadable
			// (unlike the StartTouch objects which we can get away as long as the objects themselves aren't concurrent it seems)
			// To avoid this causing locks and therefore lagging with many objects,
			// we can just know ahead of time what is going to cause a sound to play, which is
			// hardcoded at speed > 70.0f and deltaTime < 0.05 (the latter of which we don't track)
			// So we can just avoid sending these PreCollision in this case.

			const Vector vecCollideNormal = Vector( inManifold.mWorldSpaceNormal.GetX(), inManifold.mWorldSpaceNormal.GetY(), inManifold.mWorldSpaceNormal.GetZ() );
			const float flCollisionSpeed = JoltPhysicsCollisionEvent::GetCollisionSpeed( pObject1, pObject2, vecCollideNormal );

			// Skip impact reporting when either body is currently held by the player.
			// The physgun's shadow controller drives the held body into the contact
			// every tick; under IVP this would resolve to zero relative velocity at
			// the contact (run-to-convergence), but Jolt's fixed-iteration solver
			// leaves residual velocity that the engine reads as a "real" impact and
			// triggers camera shake / impact sounds for what is really player input.
			const bool bHeldByPlayer =
				( pObject1->GetGameFlags() & FVPHYSICS_PLAYER_HELD ) ||
				( pObject2->GetGameFlags() & FVPHYSICS_PLAYER_HELD );

			const bool bHasSound =
				flCollisionSpeed >= 70.0f &&
				!bHeldByPlayer &&
				pObject1->GetGameMaterialAllowsSounds() &&
				pObject2->GetGameMaterialAllowsSounds();

			const bool bSane = m_GlobalCollisionEventCount < MaxCollisionEvents;

			// Shadow-pair events (player vs prop) bypass the sound-speed gate and the event
			// cap because the game needs them for impact damage -- but Jolt re-adds
			// manifolds every few ticks for moving contacts, so a player standing against a
			// prop otherwise emits full impact events at rub speed. Gate on the pair's
			// approach speed: player impact damage thresholds start around ~300 u/s, so a
			// server can set the convar there and "player rubs prop" emits nothing while
			// "prop flies into player" still hurts. Default 0 preserves stock behavior.
			const bool bSendCollisionCallback = ( bHasSound && bSane ) ||
				( bIsShadowCollision && flCollisionSpeed >= vjolt_shadow_collision_min_speed.GetFloat() );

			if ( bSendCollisionCallback )
			{
				// IVP reports the time since this specific pair last collided, and the
				// game's impact damage code uses small deltas to discount repeated hits
				// from a sustained contact. The previous hardcoded 100.0 made every
				// manifold re-add look like a fresh full-energy impact.
				m_CollisionEvents.EmplaceBack( GetThreadId(),
					JoltPhysicsCollisionInfo( pObject1, pObject2, inManifold ),
					StampPairCollisionTime( pObject1, pObject2 ) );
				m_GlobalCollisionEventCount++;
			}
		}

		if ( ShouldTouchCallback( pObject1, pObject2 ) )
			m_StartTouchEvents.EmplaceBack( GetThreadId(), JoltPhysicsCollisionInfo( pObject1, pObject2, inManifold ) );
	}

	void OnContactPersisted( const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings ) override
	{
		JoltPhysicsObject* pObject1 = reinterpret_cast<JoltPhysicsObject*>( inBody1.GetUserData() );
		JoltPhysicsObject* pObject2 = reinterpret_cast<JoltPhysicsObject*>( inBody2.GetUserData() );

		bool bShouldCollide = ShouldCollide( pObject1, pObject2 );
		// If the game says we shouldn't collide, we will treat this as a sensor
		// to satisfy the StartTouch/EndTouch events.
		ioSettings.mIsSensor = !bShouldCollide || ioSettings.mIsSensor;

		// Sensors don't generate impulses or friction events.
		if ( ioSettings.mIsSensor )
			return;

		if ( !vjolt_contact_estimate.GetBool() )
			return;

		// Estimate the resolved contact and friction impulses for this manifold.
		// Jolt doesn't expose post-solve constraint impulses cheaply, so we run a few
		// iterations of the same impulse solver via EstimateCollisionResponse to get
		// per-tick estimates - used both for friction sound events and for
		// IPhysicsFrictionSnapshot::GetNormalForce / GetEnergyAbsorbed feedback.
		JPH::CollisionEstimationResult result;
		JPH::EstimateCollisionResponse( inBody1, inBody2, inManifold, result,
			ioSettings.mCombinedFriction, ioSettings.mCombinedRestitution,
			/*minVelocityForRestitution*/ 1.0f, /*numIterations*/ 4 );

		// Sum normal impulse across contact points and stash on each body for the
		// friction snapshot to read. IVP exposes get_vert_force per contact; this is
		// the closest equivalent we can produce in Jolt.
		float flNormalImpulseSum = 0.0f;
		for ( const JPH::CollisionEstimationResult::Impulse &imp : result.mImpulses )
			flNormalImpulseSum += imp.mContactImpulse;

		pObject1->AccumulateContactNormalImpulse( pObject2, flNormalImpulseSum );
		pObject2->AccumulateContactNormalImpulse( pObject1, flNormalImpulseSum );

		if ( !m_pGameListener )
			return;

		if ( !ShouldFrictionCallback( pObject1, pObject2 ) )
			return;

		// Sum friction impulse vectors across the manifold's contact points.
		JPH::Vec3 vFrictionImpulse = JPH::Vec3::sZero();
		for ( const JPH::CollisionEstimationResult::Impulse &imp : result.mImpulses )
			vFrictionImpulse += imp.mFrictionImpulse1 * result.mTangent1
			                  + imp.mFrictionImpulse2 * result.mTangent2;

		const float flFrictionImpulseMag = vFrictionImpulse.Length();
		if ( flFrictionImpulseMag < 1e-4f )
			return;

		// Relative tangent velocity at the contact (first point — sliding contacts
		// typically have one dominant manifold point).
		const JPH::Vec3 vWorldContact = inManifold.GetWorldSpaceContactPointOn1( 0 );
		const JPH::Vec3 vRel = inBody1.GetPointVelocity( vWorldContact ) - inBody2.GetPointVelocity( vWorldContact );
		const JPH::Vec3 vTangent = vRel - vRel.Dot( inManifold.mWorldSpaceNormal ) * inManifold.mWorldSpaceNormal;
		const float flTangentSpeed = vTangent.Length();

		// Skip near-stationary contacts to avoid event spam every tick.
		constexpr float kMinTangentSpeed = 0.5f; // ~50 cm/s
		if ( flTangentSpeed < kMinTangentSpeed )
			return;

		// IVP's friction event reports `eliminated_energy / dt / mass` (specific power, W/kg)
		// then converts to HL units with the same in²/m² factor we use here. We match that
		// so the engine's energy thresholds behave as they did under IVP. The friction
		// impulse from EstimateCollisionResponse is per-step; assume Source's typical
		// 60Hz physics step for the dt scaling.
		const float flMass1 = pObject1->IsStatic() ? 0.0f : pObject1->GetMass();
		const float flMass2 = pObject2->IsStatic() ? 0.0f : pObject2->GetMass();
		const float flMass = flMass1 > 0.0f ? flMass1 : flMass2;
		if ( flMass <= 0.0f )
			return;

		constexpr float kAssumedStepDt = 1.0f / 60.0f;
		const float flPowerPerMass = ( flFrictionImpulseMag * flTangentSpeed ) / kAssumedStepDt / flMass;
		const float flEnergy = JoltToSource::Energy( flPowerPerMass );

		// Snapshot also queries energy absorbed; track on both bodies.
		pObject1->AccumulateContactFrictionEnergy( pObject2, flEnergy );
		pObject2->AccumulateContactFrictionEnergy( pObject1, flEnergy );

		m_FrictionEvents.EmplaceBack( GetThreadId(),
			JoltPhysicsCollisionInfo( pObject1, pObject2, inManifold ),
			flEnergy );
	}

	void OnContactRemoved( const JPH::SubShapeIDPair &inSubShapePair )
	{
		if ( !m_pGameListener )
			return;

		// This is always called with all bodies locked.
		const JPH::BodyLockInterfaceNoLock &bodyInterface = m_PhysicsSystem.GetBodyLockInterfaceNoLock();

		JPH::Body *pBody1 = bodyInterface.TryGetBody( inSubShapePair.GetBody1ID() );
		JPH::Body *pBody2 = bodyInterface.TryGetBody( inSubShapePair.GetBody2ID() );

		// One of the bodies may have been deleted.
		// TODO(Josh): Handle calling end touch when we delete a body.
		if ( !pBody1 || !pBody2 )
			return;

		JoltPhysicsObject *pObject1 = reinterpret_cast< JoltPhysicsObject * >( pBody1->GetUserData() );
		JoltPhysicsObject *pObject2 = reinterpret_cast< JoltPhysicsObject * >( pBody2->GetUserData() );

		if ( pObject1->IsFluid() || pObject2->IsFluid() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsFluid() && ( pObject2->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidEndTouchEvents.EmplaceBack( uThreadId, pObject1, pObject2 );

			if ( pObject2->IsFluid() && ( pObject1->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidEndTouchEvents.EmplaceBack( uThreadId, pObject2, pObject1 );

			return;
		}

		if ( pObject1->IsTrigger() || pObject2->IsTrigger() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsTrigger() )
				m_LeaveTriggerEvents.EmplaceBack( uThreadId, pObject1, pObject2 );
		
			if ( pObject2->IsTrigger() )
				m_LeaveTriggerEvents.EmplaceBack( uThreadId, pObject2, pObject1 );
	
			return;
		}

		if ( !ShouldTouchCallback( pObject1, pObject2 ) )
			return;

		const uint32 uThreadId = GetThreadId();

		// Josh:
		// We don't have any collision data here
		// and caching it would be annoying and expensive.
		// 
		// Lucky for us though, the game simply just calls the stuff
		// to retrieve the contact point and normal, then just never uses it
		// so we can return anything we want and it will change *nothing*!
		m_EndTouchEvents.EmplaceBack( uThreadId, JoltPhysicsCollisionInfo( pObject1, pObject2 ) );
	}

	static inline uint64_t MakeCacheKey(JoltPhysicsObject *pObject0, JoltPhysicsObject *pObject1)
	{
		uint32_t keyA = pObject0->GetBodyID().GetIndexAndSequenceNumber();
		uint32_t keyB = pObject1->GetBodyID().GetIndexAndSequenceNumber();
		if (keyA > keyB)
			std::swap(keyA, keyB);

		return (uint64_t(keyA) << 32) | keyB;
	}

	// NCG: only drop cache entries that involve pObject0 rather than nuking the whole
	// map. A full clear forces every unrelated pair to re-run the locked (Lua-capable)
	// ShouldCollide on its next contact -- a stall storm on the collision-group changes
	// that are constant on a ragdoll-heavy server. The size cap bounds accumulation of
	// stale entries for destroyed bodies (recycled BodyIDs are never revisited).
	void InvalidShouldCollideCache( JoltPhysicsObject *pObject0 )
	{
		std::unique_lock<std::shared_mutex> mapLock( m_ShouldCollideCacheLock );

		if ( !pObject0 || m_ShouldCollideCache.size() > 100000 )
		{
			m_ShouldCollideCache.clear();
			return;
		}

		const uint32_t nKey = pObject0->GetBodyID().GetIndexAndSequenceNumber();
		std::vector<uint64_t> keysToErase;
		for ( const auto &kv : m_ShouldCollideCache )
		{
			const uint64_t k = kv.first;
			if ( uint32_t( k >> 32 ) == nKey || uint32_t( k & 0xFFFFFFFFu ) == nKey )
				keysToErase.push_back( k );
		}

		for ( const uint64_t k : keysToErase )
			m_ShouldCollideCache.erase( k );
	}

	bool ShouldCollide( JoltPhysicsObject *pObject0, JoltPhysicsObject *pObject1 )
	{
		VJoltAssert( pObject0 != pObject1 );

		if ( !pObject0 || !pObject1 )
			return false;

		if ( ( pObject0->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION ) && ( pObject1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
			return false;

		if ( ( pObject1->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION ) && ( pObject0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
			return false;

		if ( !m_pGameSolver )
			return true;

		// Josh:
		// Do some work the game does ahead of time to
		// avoid needless locking.
		if ( !PreEmptGameShouldCollide( pObject0, pObject1 ) )
			return false;

		// RaphaelIT7:
		// Let's check the cache
		// We cache it to prevent expensive locks as ShouldCollide is not thread safe (In GMod it can call back to lua)
		uint64_t nCacheKey = MakeCacheKey( pObject0, pObject1 );
		{
			std::shared_lock<std::shared_mutex> lock( m_ShouldCollideCacheLock );
			auto it = m_ShouldCollideCache.find( nCacheKey );
			if ( it != m_ShouldCollideCache.end() )
				return it->second;
		}

		// Actually ask the game now, locking both bodies so they cannot have
		// concurrent ShouldCollide calls.
		//JoltPhysicsObjectPairLock lock( pObject0->GetCollisionTestLock(), pObject1->GetCollisionTestLock() );
		std::unique_lock lock( m_ShouldCollideLock );

		// RaphaelIT7: We check again since a thread may have done the work for us
		{
			// We must lock due to InvalidShouldCollideCache (Though it probably should never be called from outside while Simulating?)
			std::shared_lock<std::shared_mutex> lock( m_ShouldCollideCacheLock );
			auto it = m_ShouldCollideCache.find( nCacheKey );
			if ( it != m_ShouldCollideCache.end() )
				return it->second;
		}

		bool bCollide = m_pGameSolver->ShouldCollide( pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData() );

		std::unique_lock<std::shared_mutex> mapLock( m_ShouldCollideCacheLock );
		m_ShouldCollideCache[ nCacheKey ] = bCollide;

		return bCollide;
	}

	bool PreEmptGameShouldCollide( JoltPhysicsObject *pObject0, JoltPhysicsObject *pObject1 )
	{
		// This function pre-empts the result of the
		// game's ShouldCollide implementation to avoid needless locking.

		// Check if the entities are the same and self-collisions are disabled.
		if ( pObject0->GetGameData() == pObject1->GetGameData() )
		{
			if ( ( pObject0->GetGameFlags() | pObject1->GetGameFlags() ) & FVPHYSICS_NO_SELF_COLLISIONS )
				return false;
		}

		// If both of these are constrained to the world, they shouldn't collide.
		if ( pObject0->GetGameFlags() & pObject1->GetGameFlags() & FVPHYSICS_CONSTRAINT_STATIC )
			return false;

		// We do wheels separately, the IS_VEHICLE_WHEEL is just to have some dummy object to return to the game.
		if ( ( pObject0->GetCallbackFlags() & CALLBACK_IS_VEHICLE_WHEEL ) || ( pObject1->GetCallbackFlags() & CALLBACK_IS_VEHICLE_WHEEL ) )
			return false;

		// Two shadow controlled objects should not collide.
		if ( pObject0->GetShadowController() && pObject1->GetShadowController() )
			return false;

		return true;
	}

	bool ShouldFrictionCallback( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
	{
		if ( !( pObject1->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
			return false;

		if ( !( pObject2->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
			return false;

		return true;
	}

	bool ShouldTouchCallback( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
	{
		uint32 uFlags = 0;
		uFlags |= pObject1->GetCallbackFlags();
		uFlags |= pObject2->GetCallbackFlags();

		if ( !( uFlags & CALLBACK_GLOBAL_TOUCH ) )
			return false;

		if ( !( uFlags & CALLBACK_GLOBAL_TOUCH_STATIC ) && ( pObject1->IsStatic() || pObject2->IsStatic() ) )
			return false;

		return true;
	}

	IPhysicsCollisionEvent *GetGameListener()
	{
		return m_pGameListener;
	}

	void SetGameListener( IPhysicsCollisionEvent *pListener )
	{
		m_pGameListener = pListener;
	}

	IPhysicsCollisionSolver *GetGameSolver()
	{
		return m_pGameSolver;
	}

	void SetGameSolver( IPhysicsCollisionSolver *pSolver )
	{
		m_pGameSolver = pSolver;
	}

	// Returns true (and records a skip) if either object was freed earlier in THIS flush by a
	// re-entrant game callback (see FlushCallbacks). Shared by every dispatch loop so the same
	// use-after-free class is closed on collision, friction, touch, trigger and fluid events -- not
	// just the collision Pre/Post pair. Cheap: one uint32 compare unless a mid-flush destroy actually
	// happened this flush (g_pObjects membership lookups only on that slow path). Always defined
	// (returns false when !GAME_GMOD) so the call sites need no #if.
	bool ShouldSkipFreedEvent( [[maybe_unused]] JoltPhysicsObject *pObj1, [[maybe_unused]] JoltPhysicsObject *pObj2 )
	{
	#if GAME_GMOD
		if ( m_bFlushValidityGuard && g_JoltObjectDestroyGeneration != m_nFlushDestroyGenStart &&
			 ( !IsValidPhyiscsObject( pObj1 ) || !IsValidPhyiscsObject( pObj2 ) ) )
		{
			++g_nCollisionGuardSkips;
			return true;
		}
	#endif
		return false;
	}

	void FlushCallbacks()
	{
		if ( !m_pGameListener )
			return;

	#if GAME_GMOD
		// Snapshot the guard state once per flush. The per-event guard below only does the
		// g_pObjects lookups when g_JoltObjectDestroyGeneration moves off this snapshot, i.e. a
		// game callback freed an object mid-flush -- otherwise it is a single integer compare.
		m_bFlushValidityGuard = vjolt_collision_event_validity_guard.GetBool();
		m_nFlushDestroyGenStart = g_JoltObjectDestroyGeneration;
	#endif

		// Collision events -- dispatched as matched Pre/Post PAIRS, one event at a time.
		//
		// Source's IVP contract (public/vphysics_interface.h: "PreCollision/PostCollision ALWAYS
		// come in matched pairs!!!") requires Pre and Post for one collision to arrive adjacently:
		// the engine's CCollisionEvent latches pObjects into a singleton (m_gameEvent) at Pre and
		// reuses it at Post without re-Init. The previous all-Pre-then-all-Post batching mis-paired
		// object<->entity for every Post but the last and, worse, could hand an object freed
		// mid-flush to a later Post -- which the engine forwards into CBoneFollower::VPhysicsCollision
		// -> CPhysicsSwapTemp ("bogus physics object" Sys_Error). Pairing Pre+Post per event keeps
		// the singleton latched to THIS event and bounds the free window to a single event.
		//
		// ORDERING NOTE: stock ran StartTouch/EnterTrigger/FluidStartTouch BETWEEN the all-Pre and
		// all-Post passes. Per-event pairing makes Pre and Post adjacent, so those three loops now run
		// AFTER every PostCollision (Friction/EndTouch/LeaveTrigger/FluidEndTouch were already after
		// Post in stock, unchanged). No Source contract orders StartTouch vs PostCollision (IVP
		// interleaves per contact), so this is benign -- but it moves a Post-callback free ahead of the
		// StartTouch family, which is why EVERY dispatch loop below carries the same validity guard.
		m_CollisionEvents.ForEach< true >( [ this ]( JoltPhysicsCollisionEvent& event )
		{
			JoltPhysicsObject *pObj1 = event.m_Data.GetPair().pObject1;
			JoltPhysicsObject *pObj2 = event.m_Data.GetPair().pObject2;

			// A game callback dispatched earlier in this same flush can UTIL_Remove an entity,
			// synchronously freeing its JoltPhysicsObject (re-entrant CleanupDeleteList). Such an
			// object is already absent from g_pObjects (Unregistered at the top of the dtor), so
			// skip the event rather than dereference -- or hand the engine -- a dangling pointer.
			// The same guard runs on every sibling loop below (touch/trigger/friction/fluid).
			if ( ShouldSkipFreedEvent( pObj1, pObj2 ) )
				return;

			// Compute collisionSpeed from the post-resolution body velocities (matches IVP's
			// contact->speed semantics: relative velocity at the contact surface after impulses).
			// Player-controlled sides substitute the CONTACT-TIME game-driven velocity stored in
			// the event (the live body velocity is the shadow-correction spike, and the controller
			// may already be detached/zeroed by an earlier callback in this same flush).
			Vector vel1, vel2, normal;
			if ( event.m_Data.IsObject1PlayerDriven() )
				vel1 = JoltToSource::Distance( event.m_Data.GetObject1PreCollisionVelocity() );
			else
				pObj1->GetVelocity( &vel1, nullptr );
			if ( event.m_Data.IsObject2PlayerDriven() )
				vel2 = JoltToSource::Distance( event.m_Data.GetObject2PreCollisionVelocity() );
			else
				pObj2->GetVelocity( &vel2, nullptr );
			event.m_Data.GetSurfaceNormal( normal );
			event.m_Event.collisionSpeed = fabsf( ( vel1 - vel2 ).Dot( normal ) );

			// Fake the velocities for both objects during PreCollision so the game gets a proper
			// Pre/Post delta for damage callbacks.
			JPH::Vec3 object1Vel = pObj1->FakeJoltLinearVelocity( event.m_Data.GetObject1PreCollisionVelocity() );
			JPH::Vec3 object2Vel = pObj2->FakeJoltLinearVelocity( event.m_Data.GetObject2PreCollisionVelocity() );
			m_pGameListener->PreCollision( &event.m_Event );
			pObj1->RestoreJoltLinearVelocity( object1Vel );
			pObj2->RestoreJoltLinearVelocity( object2Vel );

			// The game samples post-collision velocities during PostCollision (the deltaV term of
			// player crush damage). For player-controlled objects the body velocity is the
			// position-derived correction (a phantom), so fake it to the CONTACT-TIME game-driven
			// velocity stored in the event, mirroring the PreCollision faking: the player's own
			// deltaV then reads ~zero (IVP parity), and using the stored flag/velocity keeps
			// pre/post self-consistent even if the controller detached earlier in this flush.
			const bool bFake1 = event.m_Data.IsObject1PlayerDriven();
			const bool bFake2 = event.m_Data.IsObject2PlayerDriven();

			JPH::Vec3 realVel1 = JPH::Vec3::sZero();
			JPH::Vec3 realVel2 = JPH::Vec3::sZero();
			if ( bFake1 )
				realVel1 = pObj1->FakeJoltLinearVelocity( event.m_Data.GetObject1PreCollisionVelocity() );
			if ( bFake2 )
				realVel2 = pObj2->FakeJoltLinearVelocity( event.m_Data.GetObject2PreCollisionVelocity() );

			m_pGameListener->PostCollision( &event.m_Event );

			if ( bFake1 )
				pObj1->RestoreJoltLinearVelocity( realVel1 );
			if ( bFake2 )
				pObj2->RestoreJoltLinearVelocity( realVel2 );
		});

		// Send StartTouch events
		m_StartTouchEvents.ForEach< true >( [ this ]( JoltPhysicsCollisionData& event )
		{
			if ( ShouldSkipFreedEvent( event.GetPair().pObject1, event.GetPair().pObject2 ) )
				return;
			m_pGameListener->StartTouch( event.GetPair().pObject1, event.GetPair().pObject2, &event );
		});

		// Send EnterTrigger events
		m_EnterTriggerEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			if ( ShouldSkipFreedEvent( event.pObject1, event.pObject2 ) )
				return;
			m_pGameListener->ObjectEnterTrigger( event.pObject1, event.pObject2 );
		});

		// Send FluidStartTouch events
		m_FluidStartTouchEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			if ( ShouldSkipFreedEvent( event.pObject1, event.pObject2 ) )
				return;
			m_pGameListener->FluidStartTouch( event.pObject2, event.pObject1->GetFluidController() );
		});

		// Send Friction events. The game uses these to drive scrape sounds and
		// particle effects on sustained sliding contacts.
		m_FrictionEvents.ForEach< true >( [ this ]( JoltPhysicsFrictionEvent& event )
		{
			JoltPhysicsObject *pObj1 = event.m_Data.GetPair().pObject1;
			JoltPhysicsObject *pObj2 = event.m_Data.GetPair().pObject2;
			// Guard BEFORE the GetMaterialIndex() derefs below -- those read the wrapper directly, so a
			// freed pObj here is a read-after-free inside vphysics, not just a bad pointer to the engine.
			if ( ShouldSkipFreedEvent( pObj1, pObj2 ) )
				return;
			const int nMtl1 = pObj1->GetMaterialIndex();
			const int nMtl2 = pObj2->GetMaterialIndex();
		#if !defined( GAME_GMOD_64X )
			JoltPhysicsSurfaceProps &props = JoltPhysicsSurfaceProps::GetInstance();
			const int nMtl1Hit = props.RemapIVPMaterialIndex( nMtl2 );
			const int nMtl2Hit = props.RemapIVPMaterialIndex( nMtl1 );
		#else
			const int nMtl1Hit = nMtl2;
			const int nMtl2Hit = nMtl1;
		#endif
			m_pGameListener->Friction( pObj1, event.m_flEnergy, nMtl1, nMtl1Hit, &event.m_Data );
			m_pGameListener->Friction( pObj2, event.m_flEnergy, nMtl2, nMtl2Hit, &event.m_Data );
		});

		// Send EndTouch events
		m_EndTouchEvents.ForEach< true >( [ this ]( JoltPhysicsCollisionData& event )
		{
			if ( ShouldSkipFreedEvent( event.GetPair().pObject1, event.GetPair().pObject2 ) )
				return;
			m_pGameListener->EndTouch( event.GetPair().pObject1, event.GetPair().pObject2, &event );
		});

		// Send LeaveTrigger events
		m_LeaveTriggerEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			if ( ShouldSkipFreedEvent( event.pObject1, event.pObject2 ) )
				return;
			m_pGameListener->ObjectLeaveTrigger( event.pObject1, event.pObject2 );
		});

		// Send FluidEndTouch events
		m_FluidEndTouchEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			if ( ShouldSkipFreedEvent( event.pObject1, event.pObject2 ) )
				return;
			m_pGameListener->FluidEndTouch( event.pObject2, event.pObject1->GetFluidController() );
		});

		// Reset the collision event counter.
		m_GlobalCollisionEventCount = 0u;

	#if GAME_GMOD
		// Soak instrumentation: record whether any object was freed mid-dispatch this flush (the
		// monotonic destroy generation moved off the snapshot). Read via the vjolt_guard_stats concmd.
		if ( m_bFlushValidityGuard && g_JoltObjectDestroyGeneration != m_nFlushDestroyGenStart )
			++g_nGuardActiveFlushes;
	#endif
	}

	void PostSimulationFrame()
	{
		if ( m_pGameListener )
			m_pGameListener->PostSimulationFrame();
	}

	// Called once per Simulate from the owning environment; drives the per-pair
	// deltaCollisionTime reported on impact events. Integer microseconds: a float-seconds
	// accumulator stops advancing once its ULP exceeds the tick (~3 days of uptime), after
	// which every repeat impact would report delta 0 and the engine's repeat-collision
	// discount would suppress impact damage server-wide.
	void AdvanceSimulationTime( float flDeltaTime )
	{
		const uint64_t nDeltaUs = static_cast< uint64_t >( flDeltaTime * 1e6f + 0.5f );
		m_nSimulationTimeUs.store( m_nSimulationTimeUs.load( std::memory_order_relaxed ) + nDeltaUs, std::memory_order_relaxed );
	}

private:

	// Velocity as collision events should report it. For player-controlled objects the
	// body's velocity is the shadow controller's correction term ((target - pos) / dt),
	// which spikes on any blocked/penetrating contact and reads as a phantom high-speed
	// impact -- live kill feed showed players one-shot by TOUCHING resting items, because
	// the correction spike both passed the vjolt_shadow_collision_min_speed gate and
	// supplied the event's impact energy. IVP reported the game-driven velocity for the
	// player shadow object; do the same (fed via SetPlayerDrivenVelocity from the player
	// controller's Update).
	static Vector GetEffectiveVelocity( JoltPhysicsObject *pObject )
	{
		if ( pObject->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER )
			return pObject->GetPlayerDrivenVelocity();

		Vector vVelocity;
		pObject->GetVelocity( &vVelocity, nullptr );
		return vVelocity;
	}

	static JPH::Vec3 GetEffectiveJoltVelocity( JoltPhysicsObject *pObject )
	{
		if ( pObject->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER )
			return SourceToJolt::Distance( pObject->GetPlayerDrivenVelocity() );

		return pObject->GetBody()->GetLinearVelocity();
	}

	// Returns the time in seconds since this pair last produced a collision event (IVP
	// deltaCollisionTime semantics) and stamps 'now' for the next one. A pair's first event
	// reports "long ago" (100.0, matching the engine's out-of-range sentinel). Runs on Jolt
	// worker threads via OnContactAdded; emitted events are few (sound cap + shadow speed
	// gate), so the lock is quiet.
	float StampPairCollisionTime( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
	{
		const uint64_t nKey = MakeCacheKey( pObject1, pObject2 );
		const uint64_t nNowUs = m_nSimulationTimeUs.load( std::memory_order_relaxed );

		std::unique_lock lock( m_CollisionTimeLock );

		// Stale entries for destroyed bodies accumulate (recycled BodyIDs carry a new
		// sequence number and never revisit an old key); reset wholesale rather than
		// tracking body lifetimes. Pairs then briefly report "long ago" again.
		if ( m_LastCollisionTime.size() > 16384 )
			m_LastCollisionTime.clear();

		float flDelta = 100.0f;
		auto it = m_LastCollisionTime.find( nKey );
		if ( it != m_LastCollisionTime.end() )
		{
			flDelta = static_cast< float >( nNowUs - it->second ) * 1e-6f;
			it->second = nNowUs;
		}
		else
		{
			m_LastCollisionTime.emplace( nKey, nNowUs );
		}

		return flDelta;
	}

	static uint32 GetThreadId()
	{
		extern thread_local uint32 s_ThreadId;
		static std::atomic< uint32 > s_ThreadCtr = { 0u };
		if ( s_ThreadId == ~0u )
			s_ThreadId = s_ThreadCtr++;
		return s_ThreadId;
	}

	const JPH::PhysicsSystem &m_PhysicsSystem;

	IPhysicsCollisionEvent	*m_pGameListener = nullptr;
	IPhysicsCollisionSolver *m_pGameSolver = nullptr;

#if GAME_GMOD
	// Snapshotted at the top of FlushCallbacks and read by the dispatch lambdas, so the
	// mid-flush-free validity guard costs one int compare per event unless an object is actually
	// destroyed during the flush. m_bFlushValidityGuard also hoists the convar read out of the loop.
	uint32 m_nFlushDestroyGenStart = 0u;
	bool   m_bFlushValidityGuard = false;
#endif

	std::mutex m_ShouldCollideLock;
	std::shared_mutex m_ShouldCollideCacheLock;
	ankerl::unordered_dense::map< uint64_t, bool > m_ShouldCollideCache;

	// Per-pair last-collision-event stamps for deltaCollisionTime (see StampPairCollisionTime).
	std::mutex m_CollisionTimeLock;
	ankerl::unordered_dense::map< uint64_t, uint64_t > m_LastCollisionTime;
	std::atomic< uint64_t > m_nSimulationTimeUs = { 0ull };

	class JoltPhysicsCollisionInfo
	{
	public:
		JoltPhysicsCollisionInfo( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
			: m_CollisionPair{ pObject1, pObject2 }
		{
		}

		JoltPhysicsCollisionInfo( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2, const JPH::ContactManifold &inManifold )
			: m_CollisionPair{ pObject1, pObject2 }
			// Slart: Note this negated vector, it is important, Portal 2 bouncy paint needs it negated otherwise things fly into the surface they hit
			, m_SurfaceNormal( -Vector( inManifold.mWorldSpaceNormal.GetX(), inManifold.mWorldSpaceNormal.GetY(), inManifold.mWorldSpaceNormal.GetZ() ) )
			, m_ContactPoint( JoltToSource::Distance( inManifold.GetWorldSpaceContactPointOn1( 0 ) ) )
			// Unused...
			, m_ContactSpeed( vec3_origin )
			// Pre-collision velocities as the game will see them via PreCollision's
			// velocity faking: player-controlled sides report the game-driven velocity,
			// not the shadow-correction spike (see GetEffectiveJoltVelocity). The
			// player-driven decision is CAPTURED HERE, at contact time: the controller
			// can detach (death/disconnect from an earlier callback in the same flush)
			// or zero its state between contact and flush, and re-deriving from live
			// flags at flush time would reintroduce the phantom-velocity class of bug
			// on exactly those teardown ticks.
			, m_Velocity0( GetEffectiveJoltVelocity( pObject1 ) )
			, m_Velocity1( GetEffectiveJoltVelocity( pObject2 ) )
			, m_bPlayerDriven0( ( pObject1->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER ) != 0 )
			, m_bPlayerDriven1( ( pObject2->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER ) != 0 )
		{
		}

		JoltPhysicsContactPair m_CollisionPair;

		Vector m_SurfaceNormal = vec3_origin;
		Vector m_ContactPoint  = vec3_origin;
		Vector m_ContactSpeed  = vec3_origin;

		JPH::Vec3 m_Velocity0 = JPH::Vec3::sZero();
		JPH::Vec3 m_Velocity1 = JPH::Vec3::sZero();

		// Captured at contact time; see the manifold constructor.
		bool m_bPlayerDriven0 = false;
		bool m_bPlayerDriven1 = false;
	};

	class JoltPhysicsCollisionData final : public IPhysicsCollisionData
	{
	public:
		JoltPhysicsCollisionData( const JoltPhysicsCollisionInfo &info )
			: m_CollisionData{ info }
		{
		}

		void GetSurfaceNormal( Vector &out ) override
		{
			out = m_CollisionData.m_SurfaceNormal;
		}

		void GetContactPoint( Vector &out ) override
		{
			out = m_CollisionData.m_ContactPoint;
		}

		void GetContactSpeed( Vector &out ) override
		{
			out = m_CollisionData.m_ContactSpeed;
		}

		JoltPhysicsContactPair GetPair() const
		{
			return m_CollisionData.m_CollisionPair;
		}

		JPH::Vec3 GetObject1PreCollisionVelocity() const
		{
			return m_CollisionData.m_Velocity0;
		}

		JPH::Vec3 GetObject2PreCollisionVelocity() const
		{
			return m_CollisionData.m_Velocity1;
		}

		bool IsObject1PlayerDriven() const
		{
			return m_CollisionData.m_bPlayerDriven0;
		}

		bool IsObject2PlayerDriven() const
		{
			return m_CollisionData.m_bPlayerDriven1;
		}
	private:
		JoltPhysicsCollisionInfo m_CollisionData;
	};

	class JoltPhysicsCollisionEvent
	{
	public:
		JoltPhysicsCollisionEvent( const JoltPhysicsCollisionInfo &info, float flDeltaCollisionTime = 100.0f )
			: m_Data{ info }
		{
			JoltPhysicsObject *pObject1 = m_Data.GetPair().pObject1;
			JoltPhysicsObject* pObject2 = m_Data.GetPair().pObject2;

			m_Event.pObjects[0]			= pObject1;
			m_Event.pObjects[1]			= pObject2;
			m_Event.surfaceProps[0]		= pObject1->GetMaterialIndex();
			m_Event.surfaceProps[1]		= pObject2->GetMaterialIndex();
			m_Event.isCollision			= IsCollision( pObject1, pObject2 );
			m_Event.isShadowCollision	= IsShadowCollision( pObject1, pObject2 );
			m_Event.deltaCollisionTime	= flDeltaCollisionTime;
			m_Event.collisionSpeed		= 0.0f;
			m_Event.pInternalData		= &m_Data;
		}

		JoltPhysicsCollisionEvent( const JoltPhysicsCollisionEvent &other )
			: m_Event( other.m_Event )
			, m_Data ( other.m_Data )
		{
			// Re-target the event's internal data pointer to our own structure.
			m_Event.pInternalData = &m_Data;
		}

		JoltPhysicsCollisionEvent( JoltPhysicsCollisionEvent &&other )
			: m_Event( std::move( other.m_Event ) )
			, m_Data ( std::move( other.m_Data ) )
		{
			// Re-target the event's internal data pointer to our own structure.
			m_Event.pInternalData = &m_Data;
		}

		static bool IsCollision( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
		{
			bool bIsCollision = ( pObject1->GetCallbackFlags() & pObject2->GetCallbackFlags() ) & CALLBACK_GLOBAL_COLLISION;

			if ( pObject1->IsStatic() && !( pObject2->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
				bIsCollision = false;
			if ( pObject2->IsStatic() && !( pObject1->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
				bIsCollision = false;

			return bIsCollision;
		}

		static bool IsShadowCollision( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
		{
			return ( pObject1->GetCallbackFlags() ^ pObject2->GetCallbackFlags() ) & CALLBACK_SHADOW_COLLISION;
		}

		static float GetCollisionSpeed( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2, Vector vecNormal )
		{
			// Effective velocities: player-controlled objects report the game-driven
			// velocity, not the shadow-correction spike (see GetEffectiveVelocity).
			const Vector vecCollisionSpeed = GetEffectiveVelocity( pObject1 ) - GetEffectiveVelocity( pObject2 );
			return fabsf( vecCollisionSpeed.Dot( vecNormal ) );
		}

		vcollisionevent_t			m_Event = {};
		JoltPhysicsCollisionData	m_Data;
	};

	class JoltPhysicsFrictionEvent
	{
	public:
		JoltPhysicsFrictionEvent( const JoltPhysicsCollisionInfo &info, float flEnergy )
			: m_Data( info )
			, m_flEnergy( flEnergy )
		{
		}

		JoltPhysicsCollisionData	m_Data;
		float						m_flEnergy = 0.0f;
	};

	template < typename Data >
	struct JoltPhysicsEventTracker
	{
	public:
		template < typename... T >
		void EmplaceBack( uint32 uThreadId, T&&... val)
		{
			m_Mask |= 1ull << uThreadId;
			m_Events[ uThreadId ].emplace_back( std::forward< T >( val )... );
		}

		template < bool bClear, typename FuncType >
		void ForEach( FuncType func )
		{
			for ( uint32 thread = m_Mask; thread; thread &= thread - 1 )
			{
				const uint32 i = JPH::CountTrailingZeros( thread );
				for ( auto &event : m_Events[ i ] )
					func( event );

				if constexpr ( bClear )
					m_Events[ i ].clear();
			}

			if constexpr ( bClear )
				m_Mask = 0ull;
		}

	private:
		static constexpr uint32 kMaxThreads = 64;
		std::atomic< uint64_t >	m_Mask = { 0ull };
		std::vector< Data >		m_Events[ kMaxThreads ];
	};

	// The maximum number of sent collision events to send per-frame.
	// This is used to play stuff like sounds and physics fx.
	// This is quite expensive to do so, we rate-limit this quite aggressively.
	static constexpr uint32_t MaxCollisionEvents = 4;
	std::atomic< uint32 > m_GlobalCollisionEventCount = { 0u };

	JoltPhysicsEventTracker< JoltPhysicsCollisionEvent >	m_CollisionEvents;
	JoltPhysicsEventTracker< JoltPhysicsFrictionEvent >		m_FrictionEvents;

	JoltPhysicsEventTracker< JoltPhysicsCollisionData >		m_StartTouchEvents;
	JoltPhysicsEventTracker< JoltPhysicsCollisionData >		m_EndTouchEvents;

	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_EnterTriggerEvents;
	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_LeaveTriggerEvents;

	// For the fluid events:
	//   Object1 = the fluid
	//   Object2 = the object
	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_FluidStartTouchEvents;
	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_FluidEndTouchEvents;

};
