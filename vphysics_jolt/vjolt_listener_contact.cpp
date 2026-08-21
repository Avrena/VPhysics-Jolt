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
// historical behavior. Player impact damage tables start caring around ~300 u/s, so the
// default stops "player rubs against prop" from emitting damage events every few ticks
// while keeping real "prop flies into player" impacts. The default is deliberately the safe
// value: this convar is boot-env-only, and a server that loses its env plumbing (fresh
// volume, srcds_run replaced by an update) must not silently fall back into phantom
// touch-damage territory.
ConVar vjolt_shadow_collision_min_speed( "vjolt_shadow_collision_min_speed", "150", FCVAR_NONE,
	"Min approach speed (u/s) for player-shadow collision events. 0 = emit all (stock behavior)." );

#if GAME_GMOD
// Garry's Mod exposes collision callbacks to Lua as Entity:PhysicsCollide. Unlike the
// base games, those callbacks can own gameplay rather than only impact sounds/damage.
// Keep a tiny non-zero floor so Jolt manifold re-adds for resting contacts do not spam
// Lua, and retain a configurable safety cap for pathological contact scenes.
ConVar vjolt_collision_callback_min_speed( "vjolt_collision_callback_min_speed", "0.01", FCVAR_NONE,
	"Min approach speed (u/s) for ordinary GMod PhysicsCollide callbacks. 0 = emit every manifold add.",
	true, 0.0f, false, 0.0f );
ConVar vjolt_collision_callback_max_events( "vjolt_collision_callback_max_events", "64", FCVAR_NONE,
	"Max ordinary GMod PhysicsCollide callbacks per simulation frame. 0 = unlimited.",
	true, 0.0f, true, 4096.0f );
#endif
