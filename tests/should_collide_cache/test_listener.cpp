// Compile actual listener methods extracted by prepare.py. These shims expose
// only the engine/object interfaces and event storage used by those methods.
#include "vjolt_should_collide_cache.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <functional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#define GAME_GMOD 1
#define GAME_GMOD_64X 1
#define VJoltAssert(value) assert(value)
using uint32 = std::uint32_t;
using uint64_t = std::uint64_t;
template<class T> T Max( T a, T b ) { return std::max( a, b ); }
enum { CALLBACK_ENABLING_COLLISION = 1, CALLBACK_MARKED_FOR_DELETE = 2 };
namespace JPH
{
struct Vec3 { float x = 0; static Vec3 sZero() { return {}; } };
struct BodyID { uint32 key; uint32 GetIndexAndSequenceNumber() const { return key; } };
}
struct JoltPhysicsObject
{
	uint32 key, flags = 0;
	bool collidable = true, isStatic = false;
	JPH::Vec3 velocity;
	JPH::BodyID GetBodyID() const { return { key }; }
	uint32 GetCallbackFlags() const { return flags; }
	void *GetGameData() { return this; }
	bool IsStatic() const { return isStatic; }
	int GetMaterialIndex() const { return 0; }
	void *GetFluidController() { return this; }
	JPH::Vec3 FakeJoltLinearVelocity( JPH::Vec3 replacement )
	{ const auto old = velocity; velocity = replacement; return old; }
	void RestoreJoltLinearVelocity( JPH::Vec3 value ) { velocity = value; }
};
struct IPhysicsCollisionSolver
{
	virtual ~IPhysicsCollisionSolver() = default;
	virtual bool ShouldCollide( JoltPhysicsObject *, JoltPhysicsObject *, void *, void * ) = 0;
};
struct Solver final : IPhysicsCollisionSolver
{
	int calls = 0;
	bool result = true;
	std::function<void()> during;
	bool ShouldCollide( JoltPhysicsObject *, JoltPhysicsObject *, void *, void * ) override
	{
		++calls;
		const bool answer = result;
		if ( during ) during();
		return answer;
	}
};
static uint32 g_JoltObjectDestroyGeneration = 0;
static std::unordered_set<JoltPhysicsObject *> live;
static bool IsValidPhyiscsObject( JoltPhysicsObject *object ) { return live.count( object ) != 0; }
struct JoltPhysicsContactPair { JoltPhysicsObject *pObject1, *pObject2; };
struct JoltPhysicsCollisionData
{
	JoltPhysicsContactPair pair;
	JoltPhysicsContactPair GetPair() const { return pair; }
	JPH::Vec3 GetObject1PreCollisionVelocity() const { return {}; }
	JPH::Vec3 GetObject2PreCollisionVelocity() const { return {}; }
	bool IsObject1PlayerDriven() const { return false; }
	bool IsObject2PlayerDriven() const { return false; }
};
struct CollisionEvent { float deltaCollisionTime = 0; };
struct JoltPhysicsCollisionEvent
{
	JoltPhysicsCollisionData m_Data;
	CollisionEvent m_Event;
	int sequence;
	int SortKey() const { return sequence; }
};
struct JoltPhysicsFrictionEvent { JoltPhysicsCollisionData m_Data; float m_flEnergy = 0; };
template<class T> struct Events
{
	std::vector<T> values;
	void CollectValid( std::vector<T *> &out )
	{ out.clear(); for ( auto &value : values ) out.push_back( &value ); }
	template<bool clear, class F> void ForEach( F function )
	{ for ( auto &value : values ) function( value ); if ( clear ) values.clear(); }
};
struct GameListener
{
	std::vector<std::string> delivered;
	std::function<void(const char *)> during;
	void Event( const char *name ) { delivered.emplace_back( name ); if ( during ) during( name ); }
	void PreCollision( CollisionEvent * ) { Event( "pre" ); }
	void PostCollision( CollisionEvent * ) { Event( "post" ); }
	void StartTouch( JoltPhysicsObject *, JoltPhysicsObject *, JoltPhysicsCollisionData * ) { Event( "start" ); }
	void EndTouch( JoltPhysicsObject *, JoltPhysicsObject *, JoltPhysicsCollisionData * ) { Event( "end" ); }
	void ObjectEnterTrigger( JoltPhysicsObject *, JoltPhysicsObject * ) { Event( "enter" ); }
	void ObjectLeaveTrigger( JoltPhysicsObject *, JoltPhysicsObject * ) { Event( "leave" ); }
	void FluidStartTouch( JoltPhysicsObject *, void * ) { Event( "fluid-start" ); }
	void FluidEndTouch( JoltPhysicsObject *, void * ) { Event( "fluid-end" ); }
	void Friction( JoltPhysicsObject *, float, int, int, JoltPhysicsCollisionData * ) { Event( "friction" ); }
};
struct ConVar { int GetInt() const { return 4096; } } vjolt_collision_max_events;
struct DispatchStorage
{
	GameListener *m_pGameListener = nullptr;
	uint32 m_nCollisionSubstep = 0, m_nFlushDestroyGenStart = 0;
	std::vector<JoltPhysicsCollisionEvent *> m_OrderedCollisionEvents;
	std::unordered_map<uint32, uint32> m_CollisionObjectCounts;
	Events<JoltPhysicsCollisionEvent> m_CollisionEvents;
	Events<JoltPhysicsCollisionData> m_StartTouchEvents, m_EndTouchEvents;
	Events<JoltPhysicsContactPair> m_EnterTriggerEvents, m_LeaveTriggerEvents, m_FluidStartTouchEvents, m_FluidEndTouchEvents;
	Events<JoltPhysicsFrictionEvent> m_FrictionEvents;
	float StampPairCollisionTime( JoltPhysicsObject *, JoltPhysicsObject * ) { return 1; }
};
#include "listener.inc"

static int checks = 0, failures = 0;
static void Check( bool result, const char *label )
{
	++checks;
	if ( !result ) { ++failures; std::printf( "FAIL %s\n", label ); }
}

static void ListenerCacheTests()
{
	JoltPhysicsObject a{1}, b{2}, c{3}, d{4};
	ListenerCache listener;
	Solver solver, replacement;
	listener.SetGameSolver( &solver );
	solver.result = false;
	Check( !listener.ShouldCollide( &a, &b ) && !listener.ShouldCollide( &b, &a ) && solver.calls == 1,
		"production listener caches a false answer and symmetric hit" );
	solver.result = true;
	Check( listener.ShouldCollide( &c, &d ) && solver.calls == 2, "production listener caches unrelated true" );
	listener.InvalidShouldCollideCache( &a );
	Check( listener.ShouldCollide( &a, &b ) && listener.ShouldCollide( &c, &d ) && solver.calls == 3,
		"production invalidation preserves unrelated cached answer" );

	listener.InvalidShouldCollideCache( nullptr );
	solver.result = false;
	solver.during = [&]() { listener.InvalidShouldCollideCache( &a ); solver.result = true; };
	Check( !listener.ShouldCollide( &a, &b ), "callback-time invalidation preserves this call's answer" );
	solver.during = {};
	const int calls = solver.calls;
	Check( listener.ShouldCollide( &a, &b ) && solver.calls == calls + 1,
		"callback-time invalidation forces a fresh next call (base stale-cache regression)" );

	listener.InvalidShouldCollideCache( nullptr );
	solver.result = false;
	solver.during = [&]() { listener.SetGameSolver( &replacement ); };
	Check( !listener.ShouldCollide( &a, &b ), "re-entrant solver replacement preserves old invocation answer" );
	Check( listener.GetGameSolver() == &replacement && listener.ShouldCollide( &a, &b ) && replacement.calls == 1,
		"replacement cannot inherit in-flight old solver result" );
	listener.SetGameSolver( &solver );
	solver.during = {};
	Check( !listener.ShouldCollide( &a, &b ), "explicit solver replacement invalidates a cached hit" );
	listener.SetGameSolver( nullptr );
	Check( listener.ShouldCollide( &a, &b ), "null solver retains existing allow behaviour" );
}

static void QueueAll( ListenerDispatch &listener, JoltPhysicsObject &a, JoltPhysicsObject &b )
{
	const JoltPhysicsContactPair pair{ &a, &b };
	listener.m_CollisionEvents.values.push_back( {{pair}, {}, 0} );
	listener.m_CollisionEvents.values.push_back( {{pair}, {}, 1} );
	listener.m_StartTouchEvents.values.push_back( {pair} );
	listener.m_EndTouchEvents.values.push_back( {pair} );
	listener.m_EnterTriggerEvents.values.push_back( pair );
	listener.m_LeaveTriggerEvents.values.push_back( pair );
	listener.m_FluidStartTouchEvents.values.push_back( pair );
	listener.m_FluidEndTouchEvents.values.push_back( pair );
	listener.m_FrictionEvents.values.push_back( {{pair}, 1} );
}

static void DispatchTests()
{
	JoltPhysicsObject a{1}, b{2};
	live = { &a, &b };
	ListenerDispatch listener;
	GameListener game;
	listener.m_pGameListener = &game;
	QueueAll( listener, a, b );
	game.during = [&]( const char *event )
	{ if ( std::string( event ) == "pre" ) a.flags |= CALLBACK_MARKED_FOR_DELETE; };
	listener.FlushCallbacks( 100 );
	Check( game.delivered == std::vector<std::string>{ "pre", "post" },
		"production flush completes started Pre/Post then drops all queued marked-endpoint events" );
	Check( listener.m_CollisionEvents.values.empty() && listener.m_StartTouchEvents.values.empty()
		&& listener.m_FrictionEvents.values.empty(), "skipped events still consumed" );

	a.flags = 0; b.flags = CALLBACK_MARKED_FOR_DELETE; game.delivered.clear(); game.during = {};
	QueueAll( listener, a, b );
	listener.FlushCallbacks( 100 );
	Check( game.delivered.empty(), "already-marked second endpoint receives no callbacks" );

	b.flags = 0; game.delivered.clear();
	listener.m_FrictionEvents.values.push_back( {{{&a,&b}}, 1} );
	game.during = [&]( const char *event )
	{ if ( std::string( event ) == "friction" ) b.flags |= CALLBACK_MARKED_FOR_DELETE; };
	listener.FlushCallbacks( 100 );
	Check( game.delivered == std::vector<std::string>{ "friction" },
		"first friction callback can suppress the independent second callback" );

	// A truly freed endpoint must be rejected through pointer membership BEFORE
	// reading its flags. ASan verifies this ordering with genuinely freed memory.
	a.flags = b.flags = 0; game.delivered.clear();
	auto *freed = new JoltPhysicsObject{8};
	live.insert( freed );
	listener.m_CollisionEvents.values.push_back( {{{&a,&b}}, {}, 0} );
	listener.m_StartTouchEvents.values.push_back( {{&a,freed}} );
	game.during = [&]( const char *event )
	{
		if ( std::string( event ) == "post" )
		{
			live.erase( freed ); delete freed; ++g_JoltObjectDestroyGeneration;
		}
	};
	listener.FlushCallbacks( 100 );
	Check( game.delivered == std::vector<std::string>{ "pre", "post" },
		"freed pointer is filtered before flag dereference" );
}

int main()
{
	ListenerCacheTests(); DispatchTests();
	std::printf( "%d checks, %d failures\n", checks, failures );
	return failures != 0;
}
