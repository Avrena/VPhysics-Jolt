
#include "cbase.h"

#include "vjolt_friction.h"
#include "vjolt_object.h"
#include "vjolt_environment.h"
#include "vjolt_surfaceprops.h"

#include "tier0/memdbgon.h"

// Friction snapshots are built from the contact listener's per-pair impulse
// accumulation for the most recent simulation step -- IVP parity: IVP reads its
// existing contact pairs. The previous implementation re-ran a full narrow-phase
// CollideShape query on EVERY snapshot creation; the engine's
// CalculateObjectStress (player crush damage, CBasePlayer::VPhysicsUpdate)
// creates snapshots for the player AND every touching object, per player, per
// tick, so on a crowded server with piled objects that went quadratic and
// melted the main thread (observed live: multi-second frames with ~all samples
// inside the query). The pair-map copy is O(contacts of self) with no collision
// work. NOTE: pair data is only populated while vjolt_contact_estimate is 1.
static ConVar vjolt_friction_snapshot( "vjolt_friction_snapshot", "1", FCVAR_NONE,
	"Populate friction snapshots from tracked contact pairs (CalculateObjectStress / PhysObj:GetStress / crush damage). 0 = empty snapshots." );

//-------------------------------------------------------------------------------------------------

JoltPhysicsFrictionSnapshot::JoltPhysicsFrictionSnapshot( JoltPhysicsObject *pObject )
	: m_pSelf( pObject )
{
	if ( !pObject || !pObject->IsCollisionEnabled() )
		return;

	if ( !vjolt_friction_snapshot.GetBool() )
		return;

	// Sleeping bodies get no contact callbacks, so their pair data is stale and
	// GetFreshContactPairs rejects it -- they report no contacts, matching the
	// rest of this module's event behavior (and avoiding stale pointers).
	std::vector< JoltPhysicsObject::ContactPairData > pairs;
	if ( !pObject->GetFreshContactPairs( pairs ) )
		return;

	// Impulses accumulate across all collision substeps of one Simulate call,
	// so the step time (not the substep time) converts them to force.
	float flDt = pObject->GetJoltEnvironment()->GetSimulationTimestep();
	if ( flDt <= 0.0f )
		flDt = 1.0f / 60.0f;

	m_contacts.reserve( pairs.size() );
	for ( const JoltPhysicsObject::ContactPairData &pair : pairs )
	{
		const float flImpulse = pair.vImpulse.Length(); // kg*m/s over the step
		if ( flImpulse <= 1e-6f )
			continue;

		Contact c;
		c.pOther = pair.pOther;
		c.vNormal = pair.vImpulse / flImpulse;
		c.vContactPoint = pair.vContactPoint;
		c.flPenetrationDepth = 0.0f;
		c.flNormalForce = JoltToSource::Distance( flImpulse / flDt );
		m_contacts.push_back( c );
	}
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsFrictionSnapshot::IsValid()
{
	return m_index < m_contacts.size();
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsFrictionSnapshot::GetObject( int index )
{
	if ( !IsValid() )
		return nullptr;
	return ( index == 0 ) ? static_cast< IPhysicsObject * >( m_pSelf )
	                      : static_cast< IPhysicsObject * >( m_contacts[ m_index ].pOther );
}

int JoltPhysicsFrictionSnapshot::GetMaterial( int index )
{
	if ( !IsValid() )
		return 0;
	return ( index == 0 ) ? m_pSelf->GetMaterialIndex()
	                      : m_contacts[ m_index ].pOther->GetMaterialIndex();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::GetContactPoint( Vector &out )
{
	if ( !IsValid() )
	{
		out.Zero();
		return;
	}
	out = m_contacts[ m_index ].vContactPoint;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::GetSurfaceNormal( Vector &out )
{
	if ( !IsValid() )
	{
		out.Zero();
		return;
	}
	out = m_contacts[ m_index ].vNormal;
}

float JoltPhysicsFrictionSnapshot::GetNormalForce()
{
	if ( !IsValid() )
		return 0.0f;

	// Computed at snapshot build from the pair's accumulated impulse vector;
	// GetSurfaceNormal() * GetNormalForce() reproduces sum(normal_i * force_i)
	// over the pair's contacts exactly, which is the only aggregate
	// CalculateObjectStress consumes.
	return m_contacts[ m_index ].flNormalForce;
}

float JoltPhysicsFrictionSnapshot::GetEnergyAbsorbed()
{
	if ( !IsValid() )
		return 0.0f;

	JoltPhysicsObject *pOther = m_contacts[ m_index ].pOther;
	if ( !pOther )
		return 0.0f;

	// Already in HL energy units - the contact listener fed JoltToSource::Energy()
	// when it accumulated this value (matches IVP's ConvertEnergyToHL of get_eliminated_energy).
	return m_pSelf->GetLastContactFrictionEnergy( pOther );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::RecomputeFriction()
{
	// IVP calls ContactPoint::recompute_friction() which forces the contact's friction
	// state to be re-derived from current materials. Jolt resolves friction every tick
	// from current state already; the closest action is to wake the body so the next
	// tick fully re-solves contacts. Wake is a no-op if already awake.
	if ( m_pSelf )
		m_pSelf->Wake();
}

void JoltPhysicsFrictionSnapshot::ClearFrictionForce()
{
	// IVP's set_friction_to_neutral resets the friction lambda accumulator on the
	// contact. Jolt's friction lambda is rebuilt each tick from the contact's current
	// state, so the equivalent is to drop our cached impulse for this pair and wake.
	if ( IsValid() && m_pSelf )
	{
		JoltPhysicsObject *pOther = m_contacts[ m_index ].pOther;
		if ( pOther )
		{
			// Erase BOTH sides: every map-erase path must stay symmetric or the
			// pairing invariant the destructor's partner scrub relies on breaks
			// (an asymmetric fresh entry could later surface a dangling pointer).
			m_pSelf->ClearContactImpulsesFor( pOther );
			pOther->ClearContactImpulsesFor( m_pSelf );
		}
		m_pSelf->Wake();
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::MarkContactForDelete()
{
	if ( !IsValid() )
		return;

	JoltPhysicsObject *pOther = m_contacts[ m_index ].pOther;
	if ( !pOther || pOther == m_pSelf )
		return;

	// Match IVP's deferred-delete pattern: collect the other-body pointers, act on
	// them in DeleteAllMarkedContacts.
	m_DeleteList.push_back( pOther );
}

void JoltPhysicsFrictionSnapshot::DeleteAllMarkedContacts( bool wakeObjects )
{
	if ( m_DeleteList.empty() )
		return;

	// IVP's CFrictionSnapshot::DeleteAllMarkedContacts calls DeleteAllFrictionPairs
	// (= unlink_contact_points_for_object) per other-body. Jolt manages the contact
	// manifold cache internally and doesn't expose a per-pair invalidation, but for
	// the use case this matters for - the physgun grabbing an object and wanting to
	// release pre-existing contacts so the held object can move freely - clearing
	// the cached contact impulse breaks the constraint feedback loop seen by the
	// grab controller, and any subsequent body motion (which the physgun applies
	// immediately via its motion controller) invalidates Jolt's body-pair cache so
	// contacts re-detect cleanly next tick.
	JPH::BodyInterface *pBodyInterface = nullptr;
	if ( wakeObjects && m_pSelf )
	{
		pBodyInterface = &m_pSelf->GetJoltEnvironment()->GetPhysicsSystem()->GetBodyInterfaceNoLock();
	}

	for ( JoltPhysicsObject *pOther : m_DeleteList )
	{
		if ( !pOther )
			continue;

		m_pSelf->ClearContactImpulsesFor( pOther );
		pOther->ClearContactImpulsesFor( m_pSelf );

		if ( pBodyInterface && !pOther->IsStatic() )
			pBodyInterface->ActivateBody( pOther->GetBodyID() );
	}

	if ( wakeObjects && m_pSelf )
		m_pSelf->Wake();

	m_DeleteList.clear();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::NextFrictionData()
{
	if ( m_index < m_contacts.size() )
		m_index++;
}

float JoltPhysicsFrictionSnapshot::GetFrictionCoefficient()
{
	if ( !IsValid() )
		return 0.0f;

	int matSelf = m_pSelf->GetMaterialIndex();
	int matOther = m_contacts[ m_index ].pOther->GetMaterialIndex();

	surfacedata_t *pSelf = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( matSelf );
	surfacedata_t *pOther = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( matOther );
	if ( !pSelf || !pOther )
		return 0.0f;

	return pSelf->physics.friction * pOther->physics.friction;
}
