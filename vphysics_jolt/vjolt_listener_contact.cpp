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
