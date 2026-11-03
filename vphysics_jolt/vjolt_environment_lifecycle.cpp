// Re-entrant deletion hardening adapted from schweiziske/GMod-Jolt-Physics
// a31992de4e801fdcc3ba8aacae5de668129f4283. See LICENSES/GMod-Jolt-Physics-MIT.txt.
// Keep this fork's deferred callbacks, controller ownership and solver unchanged.
#include "cbase.h"
#include "vjolt_environment.h"
#include "vjolt_collide.h"
#include "vjolt_controller_drag.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

namespace
{
template <typename T, typename Delete>
void DrainDeleteQueue( std::vector<T *> &queue, const Delete &destroy )
{
	// Callbacks may append and reallocate this vector. Clear the consumed slot
	// before calling game code, and retain no reference/iterator across it.
	for ( size_t i = 0; i < queue.size(); ++i )
	{
		T *entry = queue[i];
		queue[i] = nullptr;
		if ( entry )
			destroy( entry );
	}
	queue.clear();
}
}

JoltPhysicsEnvironment::~JoltPhysicsEnvironment()
{
	DestroyEnvironmentObjects();

	m_PhysicsSystem.RemoveStepListener( &m_ContactListener );
	m_PhysicsSystem.RemoveStepListener( m_pDragController );
	delete m_pDragController;
	m_pDragController = nullptr;
}

void JoltPhysicsEnvironment::DestroyObject( IPhysicsObject *pObject )
{
	if ( !pObject )
		return;

	auto *object = static_cast<JoltPhysicsObject *>( pObject );
	// The wrapper remains alive until the outer callback/deletion pass finishes.
	// Repeated requests in that interval must not append or free it twice.
	if ( object->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE )
		return;

	object->AddCallbackFlags( CALLBACK_MARKED_FOR_DELETE );
	m_pDeadObjects.push_back( object );
	if ( !m_bSimulating && !m_bEnableDeleteQueue && !m_bDeletingDeadObjects )
		DeleteDeadObjects();
}

void JoltPhysicsEnvironment::DestroyConstraint( IPhysicsConstraint *pConstraint )
{
	if ( !pConstraint )
		return;

	auto *constraint = static_cast<JoltPhysicsConstraint *>( pConstraint );
	if ( !constraint->MarkForDelete() )
		return;

	if ( m_bWakeObjectsOnConstraintDeletion )
	{
		if ( IPhysicsObject *object = constraint->GetReferenceObject() )
			object->Wake();
		if ( IPhysicsObject *object = constraint->GetAttachedObject() )
			object->Wake();
	}

	// Preserve Source's quick-delete semantics and listener detachment: only
	// simulation defers constraints. During object cleanup a game callback may
	// delete another constraint, which must detach from the still-live object.
	if ( m_bSimulating )
	{
		constraint->Deactivate();
		m_pDeadConstraints.push_back( constraint );
	}
	else
		delete constraint;
}

void JoltPhysicsEnvironment::CleanupDeleteList()
{
	// Simulate owns wrapper pointers through the end of its deferred callback
	// and controller phases. Cleanup requests there must wait for that boundary.
	if ( !m_bSimulating )
		DeleteDeadObjects();
}

void JoltPhysicsEnvironment::EnableDeleteQueue( bool enable )
{
	m_bEnableDeleteQueue = enable;
}

void JoltPhysicsEnvironment::DestroyCollideOnDeadObjectFlush( CPhysCollide *pCollide )
{
	if ( !pCollide )
		return;

	bool defer = m_bDeletingDeadObjects || m_bDestroyingEnvironmentObjects;
	for ( JoltPhysicsObject *object : m_pDeadObjects )
	{
		if ( object && object->GetCollide() == pCollide )
		{
			defer = true;
			break;
		}
	}
	if ( defer )
	{
		if ( !VectorContains( m_pDeadObjectCollides, pCollide ) )
			m_pDeadObjectCollides.push_back( pCollide );
		return;
	}
	JoltPhysicsCollision::GetInstance().DestroyCollide( pCollide );
}

void JoltPhysicsEnvironment::RemoveBodyAndDeleteObject( JoltPhysicsObject *pObject )
{
	// Invalidate while the old BodyID is still valid. The object destructor also
	// clears again after callbacks, before the ID can be reused by Jolt.
	m_ContactListener.InvalidShouldCollideCache( pObject );
	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	bodyInterface.RemoveBody( pObject->GetBodyID() );
	m_bObjectListDirty = true;
	delete pObject;
}

void JoltPhysicsEnvironment::DeleteDeadObjects()
{
	if ( m_bSimulating || m_bDeletingDeadObjects )
		return;

	struct DeleteScope
	{
		bool &active;
		explicit DeleteScope( bool &value ) : active( value ) { active = true; }
		~DeleteScope() { active = false; }
	} scope( m_bDeletingDeadObjects );

	do
	{
		DrainDeleteQueue( m_pDeadConstraints,
			[]( JoltPhysicsConstraint *constraint ) { delete constraint; } );
		DrainDeleteQueue( m_pDeadObjects,
			[this]( JoltPhysicsObject *object ) { RemoveBodyAndDeleteObject( object ); } );
	}
	while ( !m_pDeadConstraints.empty() || !m_pDeadObjects.empty() );

	// Objects and their destruction callbacks may still read the collide data.
	// Environment teardown can discover newly created wrappers in its next batch,
	// so it retains collides until all of those batches have finished as well.
	if ( !m_bDestroyingEnvironmentObjects )
		DrainDeleteQueue( m_pDeadObjectCollides,
			[]( CPhysCollide *collide ) { JoltPhysicsCollision::GetInstance().DestroyCollide( collide ); } );
}

void JoltPhysicsEnvironment::DestroyEnvironmentObjects()
{
	m_bEnableDeleteQueue = true;
	m_bDestroyingEnvironmentObjects = true;

	JPH::BodyIDVector bodies;
	for ( ;; )
	{
		// GetObjectList uses m_CachedBodies. Own this snapshot so callbacks can
		// query the object list without invalidating teardown's traversal.
		m_PhysicsSystem.GetBodies( bodies );
		std::unordered_set<JoltPhysicsObject *> queued(
			m_pDeadObjects.begin(), m_pDeadObjects.end() );
		queued.reserve( queued.size() + bodies.size() );
		bool foundObject = false;
		for ( const JPH::BodyID &id : bodies )
		{
			const JPH::Body *body = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( id );
			if ( !body )
				continue;
			auto *object = reinterpret_cast<JoltPhysicsObject *>( body->GetUserData() );
			if ( !object )
				continue;
			foundObject = true;
			// Player controller bodies can share the wrapper's user data. Also
			// preserve already-queued objects and public deletion flags without
			// accidentally enqueueing either wrapper twice.
			if ( queued.insert( object ).second )
			{
				object->AddCallbackFlags( CALLBACK_MARKED_FOR_DELETE );
				m_pDeadObjects.push_back( object );
			}
		}
		DeleteDeadObjects();
		if ( !foundObject )
			break;
	}

	m_bDestroyingEnvironmentObjects = false;
	DeleteDeadObjects();
}
