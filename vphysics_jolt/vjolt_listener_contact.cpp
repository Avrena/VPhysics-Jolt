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
