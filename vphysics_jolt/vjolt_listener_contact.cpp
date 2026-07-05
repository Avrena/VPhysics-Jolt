#include "cbase.h"

#include "vjolt_environment.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// RaphaelIT7: This is needed for 64x due to else creating - relocation R_X86_64_TPOFF32 against hidden symbol
thread_local uint32 s_ThreadId = ~0u;

// Kill-switch for the per-manifold contact impulse estimation in
// OnContactPersisted - it is the dominant per-contact cost in dense piles.
ConVar vjolt_contact_estimate( "vjolt_contact_estimate", "1", FCVAR_NONE,
	"Run per-manifold contact impulse estimation each step (friction sounds + IPhysicsFrictionSnapshot data). 0 skips it to reclaim perf in dense contact scenes." );

// Minimum pair approach speed (Source units/s along the contact normal) for shadow-pair
// collision events (player vs prop impact damage path). 0 = emit on every manifold add, the
// historical behavior. Player impact damage tables start caring around ~300 u/s, so setting
// that here stops "player rubs against prop" from emitting damage events every few ticks
// while keeping real "prop flies into player" impacts.
ConVar vjolt_shadow_collision_min_speed( "vjolt_shadow_collision_min_speed", "0", FCVAR_NONE,
	"Min approach speed (u/s) for player-shadow collision events. 0 = emit all (stock behavior)." );

// Skip queued collision (Pre/Post), friction, touch, trigger and fluid dispatch events whose
// JoltPhysicsObject was destroyed earlier in the same FlushCallbacks (a game callback UTIL_Remove'd
// the entity, freeing the wrapper via a re-entrant delete). Handing the freed pointer to the engine
// reaches CBoneFollower::VPhysicsCollision -> CPhysicsSwapTemp and raises "bogus physics object"
// (a hard Sys_Error); the friction loop would additionally read-after-free the wrapper directly.
// Membership test against g_pObjects, never dereferences the pointer, and is gated by a per-flush
// destroy generation so it costs one int compare per event unless an object is freed mid-flush.
// 1 = guard (crash fix, default), 0 = stock unguarded behavior for A/B comparison.
ConVar vjolt_collision_event_validity_guard( "vjolt_collision_event_validity_guard", "1", FCVAR_NONE,
	"Skip collision/friction/touch/trigger/fluid events whose physics object was freed mid-flush (fixes CPhysicsSwapTemp crash). 0 = stock." );

#if GAME_GMOD
// Soak instrumentation (TEMPORARY — remove before final promotion). Read the validity-guard
// counters live to learn whether synchronous mid-flush frees actually occur on this server: if
// both stay 0 under the melt+ci_strike repro, the Pre/Post pairing merge (not the guard) is what
// carries the fix. Counters are defined in vjolt_object.cpp (plain uint32, main-thread only).
extern uint32 g_nCollisionGuardSkips;
extern uint32 g_nGuardActiveFlushes;
extern uint32 g_JoltObjectDestroyGeneration;
static void Vjolt_GuardStats_Cmd( const CCommand &cmd )
{
	(void)cmd;
	Msg( "[vjolt] collision-event validity guard: %u events skipped, %u flushes with a mid-flush free (destroy gen = %u)\n",
		g_nCollisionGuardSkips, g_nGuardActiveFlushes, g_JoltObjectDestroyGeneration );
}
static ConCommand vjolt_guard_stats_cmd( "vjolt_guard_stats", Vjolt_GuardStats_Cmd,
	"Print VJolt collision-event validity-guard soak counters (events skipped / flushes with a mid-flush free)." );
#endif
