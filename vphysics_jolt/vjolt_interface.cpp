//=================================================================================================
//
// The base physics DLL interface
//
//=================================================================================================

#include "cbase.h"

#include "vjolt_environment.h"
#include "vjolt_collide.h"
#include "vjolt_surfaceprops.h"
#include "vjolt_objectpairhash.h"

#include "vjolt_interface.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

// Slart:
// Pre-allocate 64 megabytes for physics allocations.
// I don't think we've tuned this value. It's just a big number that we probably won't ever hit.
static constexpr uint kTempAllocSize = 64 * 1024 * 1024;

// Josh:
// We cannot support more than 64 threads doing physics work because
// of the code I wrote in vjolt_listener_contact to dispatch events.
// It uses a single uint64_t bitmask that is iterated on for the thread-local
// event vectors.
// This isn't an issue, the benefits of more threads tends to trail off between
// 8-16 threads anyway.
static constexpr uint kMaxPhysicsThreads = 64;

DEFINE_LOGGING_CHANNEL_NO_TAGS( LOG_VJolt, "VJolt", 0, LS_MESSAGE, Color( 205, 142, 212, 255 ) );
DEFINE_LOGGING_CHANNEL_NO_TAGS( LOG_JoltInternal, "Jolt" );

JoltPhysicsInterface JoltPhysicsInterface::s_PhysicsInterface;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( JoltPhysicsInterface, IPhysics, VPHYSICS_INTERFACE_VERSION, JoltPhysicsInterface::GetInstance() );

#if GAME_GMOD
typedef JoltPhysicsInterface JoltPhysicsInterfaceGMod; // To avoid g_Create[...]_reg collision
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( JoltPhysicsInterfaceGMod, IPhysics, VPHYSICS_INTERFACE_VERSION_GMOD, JoltPhysicsInterface::GetInstance() );
#endif

//-------------------------------------------------------------------------------------------------

// Slart:
// Instead of using Jolt's allocator override functionality, we disable it and just define the
// functions here, all of Jolt's memory allocation goes through here, besides new and delete
// which use the Valve overrides in memoverride.cpp.
// For Desolation we use mi-malloc rather than dlmalloc, that also gets built into the statically
// linked releases for gmod (along with all of tier0 and vstdlib).
// RaphaelIT7: It should always be kept in mind that inBlock for Free/AlignedFree can be NULL as per jolt docs! (though the engine already checks for null)
#ifndef JPH_DISABLE_CUSTOM_ALLOCATOR
// RaphaelIT7:
// We don't use JPH_DISABLE_CUSTOM_ALLOCATOR on release builds!
// This is because of Jolt doing very frequent allocations, which had caused 20% CPU time in new & another 20% CPU time just for delete.
namespace JPH
{
	JPH_EXPORT AllocateFunction Allocate;
	JPH_EXPORT ReallocateFunction Reallocate;
	JPH_EXPORT FreeFunction Free;
	JPH_EXPORT AlignedAllocateFunction AlignedAllocate;
	JPH_EXPORT AlignedFreeFunction AlignedFree;
}

#define JPH Jolt
#endif
namespace JPH {
#undef JPH

	void *Allocate( size_t inSize )
	{
		return MemAlloc_Alloc( inSize );
	}

	// RaphaelIT7:
	// Currently only the GMod SDK has MemAlloc_Realloc defined
	// (I was lazy & don't really see a point to have it for any other game currently)
#ifndef JPH_DISABLE_CUSTOM_ALLOCATOR
	void *Reallocate( void *inBlock, size_t inOldSize, size_t inNewSize )
	{
		return MemAlloc_Realloc( inBlock, inNewSize );
	}
#endif

	void Free( void *inBlock )
	{
		MemAlloc_Free( inBlock );
	}

	void *AlignedAllocate( size_t inSize, size_t inAlignment )
	{
		return MemAlloc_AllocAligned( inSize, inAlignment );
	}

	void AlignedFree( void *inBlock )
	{
		MemAlloc_FreeAligned( inBlock );
	}
}

//-------------------------------------------------------------------------------------------------

InitReturnVal_t JoltPhysicsInterface::Init()
{
	const InitReturnVal_t nRetVal = BaseClass::Init();
	if ( nRetVal != INIT_OK )
	{
		return nRetVal;
	}

	MathLib_Init();

#ifndef JPH_DISABLE_CUSTOM_ALLOCATOR
	JPH::Allocate = Jolt::Allocate;
	JPH::Reallocate = Jolt::Reallocate;
	JPH::Free = Jolt::Free;
	JPH::AlignedAllocate = Jolt::AlignedAllocate;
	JPH::AlignedFree = Jolt::AlignedFree;
#endif

	// Install callbacks
	JPH::Trace = JoltPhysicsInterface::OnTrace;
	JPH_IF_ENABLE_ASSERTS( JPH::AssertFailed = JoltPhysicsInterface::OnAssert; )

	// Create a factory
	JPH::Factory::sInstance = new JPH::Factory();

	// Register all Jolt physics types
	JPH::RegisterTypes();

	// Create an allocator for temporary allocations during physics simulations
	m_pTempAllocator = new JPH::TempAllocatorImpl( kTempAllocSize );

	// Josh:
	// We may want to replace this with a better heuristic, or add a launch arg for this in future.
	// Right now, this does what -1 does in Jolt, but limits it to 64 threads, as we cannot support
	// more than this (see above).
	const uint32 threadCount = Min( std::thread::hardware_concurrency() - 1, kMaxPhysicsThreads );
	m_pJobSystem = new JPH::JobSystemThreadPool( JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, threadCount );

	return INIT_OK;
}

void JoltPhysicsInterface::Shutdown()
{
	delete m_pJobSystem;
	delete m_pTempAllocator;
	delete JPH::Factory::sInstance;

	BaseClass::Shutdown();
}

void *JoltPhysicsInterface::QueryInterface( const char *pInterfaceName )
{
	CreateInterfaceFn factory = Sys_GetFactoryThis();
	return factory( pInterfaceName, NULL );	
}

//-------------------------------------------------------------------------------------------------

static std::vector<JoltPhysicsEnvironment *> g_pPhysicsEnvironments;
IPhysicsEnvironment *JoltPhysicsInterface::CreateEnvironment()
{
	JoltPhysicsEnvironment *pEnvironment = new JoltPhysicsEnvironment();
	g_pPhysicsEnvironments.push_back(pEnvironment);
	return pEnvironment;
}

void JoltPhysicsInterface::DestroyEnvironment( IPhysicsEnvironment *pEnvironment )
{
	JoltPhysicsEnvironment *pJoltEnvironment = static_cast<JoltPhysicsEnvironment *>( pEnvironment );
	
	auto it = std::find(g_pPhysicsEnvironments.begin(), g_pPhysicsEnvironments.end(), pJoltEnvironment);
	if (it != g_pPhysicsEnvironments.end())
	{
		g_pPhysicsEnvironments.erase(it);
	}

	// delete pJoltEnvironment;
}

IPhysicsEnvironment *JoltPhysicsInterface::GetActiveEnvironmentByIndex( int index )
{
	if ( index < 0 || index >= (int)g_pPhysicsEnvironments.size() )
		return NULL;

	return g_pPhysicsEnvironments[index];
}

//-------------------------------------------------------------------------------------------------

IPhysicsObjectPairHash *JoltPhysicsInterface::CreateObjectPairHash()
{
	return new JoltPhysicsObjectPairHash;
}

void JoltPhysicsInterface::DestroyObjectPairHash( IPhysicsObjectPairHash *pHash )
{
	delete static_cast<JoltPhysicsObjectPairHash *>( pHash );
}

//-------------------------------------------------------------------------------------------------

IPhysicsCollisionSet *JoltPhysicsInterface::FindOrCreateCollisionSet( unsigned int id, int maxElementCount )
{
	if ( maxElementCount > 32 )
		return nullptr;

	if ( IPhysicsCollisionSet *pSet = FindCollisionSet( id ) )
		return pSet;

	auto result = m_CollisionSets.emplace( id, JoltPhysicsCollisionSet{} );
	return &result.first->second;
}

IPhysicsCollisionSet *JoltPhysicsInterface::FindCollisionSet( unsigned int id )
{
	auto iter = m_CollisionSets.find( id );
	if ( iter != m_CollisionSets.end() )
		return &iter->second;

	return nullptr;
}

void JoltPhysicsInterface::DestroyAllCollisionSets()
{
	m_CollisionSets.clear();
}

#if GAME_GMOD
extern bool IsValidPhyiscsObject( IPhysicsObject* pObject );
bool JoltPhysicsInterface::IsValidPhysicsObject( IPhysicsObject* pObject )
{
	return ::IsValidPhyiscsObject;
}
#endif

//-------------------------------------------------------------------------------------------------

void JoltPhysicsInterface::OnTrace( const char *fmt, ... )
{
	va_list args;
	char msg[MAX_LOGGING_MESSAGE_LENGTH];

	va_start( args, fmt );
	V_vsnprintf( msg, sizeof( msg ), fmt, args );
	va_end( args );

	Log_Msg( LOG_JoltInternal, "%s\n", msg );
}

bool JoltPhysicsInterface::OnAssert( const char *inExpression, const char *inMessage, const char *inFile, uint inLine )
{
	const char *message = inMessage ? inMessage : inExpression;
	(void) message;
	AssertMsg_Internal( false, inLine, inFile, message );
	return false;
}
