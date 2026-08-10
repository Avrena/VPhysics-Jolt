//=================================================================================================
//
// Constraints
//
//=================================================================================================

#include "cbase.h"

#include <array>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <optional>

#include "vjolt_environment.h"
#include "vjolt_layers.h"
#include "vjolt_object.h"

#include "vjolt_constraints.h"

#include "vjolt_layers.h"

#include "tier0/basetypes.h"
#include "mathlib/mathlib.h"

enum MatrixAxisType_t
{
	X_AXIS = 0,
	Y_AXIS = 1,
	Z_AXIS = 2,
};

//-------------------------------------------------------------------------------------------------

static ConVar vjolt_constraint_velocity_substeps( "vjolt_constraint_velocity_substeps", "0" );
static ConVar vjolt_constraint_position_substeps( "vjolt_constraint_position_substeps", "0" );

static ConVar vjolt_ragdoll_min_torque_friction( "vjolt_ragdoll_min_torque_friction", "0.05" );

static ConVar vjolt_onlyrot_recapture_ticks( "vjolt_onlyrot_recapture_ticks", "2", FCVAR_NONE,
	"Re-zero rotation-only (onlyAngularLimits) constraint frames to the bodies' current relative "
	"orientation this many simulation steps after creation (0 = keep the creation-time capture). "
	"Lua contraptions (LVS/simfphys) teleport wheels and anchors into their intended pose one tick "
	"AFTER constraining them, so the creation-time frames bake the spawn transient in as permanent "
	"joint error." );

static ConVar vjolt_onlyrot_tiny_axis_motor_frequency( "vjolt_onlyrot_tiny_axis_motor_frequency", "0", FCVAR_NONE,
	"Position-motor frequency (Hz) for canonical dynamic-to-static rotation-only Y/Z axes whose "
	"authored window is <= 1 degree. Applied only after the normal frame recapture; 0 disables it.",
	true, 0.0f, true, 10.0f );
static ConVar vjolt_onlyrot_tiny_axis_motor_damping( "vjolt_onlyrot_tiny_axis_motor_damping", "1", FCVAR_NONE,
	"Damping ratio for the default-off tiny-axis rotation-only position motor.",
	true, 0.0f, true, 4.0f );

// Diagnostic knob, default off: hardening mid-settle freezes whatever pose the spawn
// transient left (live trials: 30 ticks @ 8 Hz made LVS tank tilt WORSE, 6/6 vs 7/12
// baseline). The gentler vjolt_baumgarte_factor was an earlier M3A3 mitigation,
// not a proven general fix: the later 48-stock-BRDM sample still varied materially.
static ConVar vjolt_length_spring_warmup_ticks( "vjolt_length_spring_warmup_ticks", "0", FCVAR_NONE,
	"Give length (rope) constraints soft spring limits for this many simulation steps after "
	"creation, then harden to rigid. 0 disables the warmup (default)." );
static ConVar vjolt_length_spring_warmup_frequency( "vjolt_length_spring_warmup_frequency", "8", FCVAR_NONE,
	"Spring frequency (Hz) of length-constraint limits during the warmup window." );
static ConVar vjolt_length_spring_warmup_damping( "vjolt_length_spring_warmup_damping", "1.0", FCVAR_NONE,
	"Spring damping ratio of length-constraint limits during the warmup window." );
static ConVar vjolt_length_spring_frequency( "vjolt_length_spring_frequency", "0", FCVAR_NONE,
	"Steady-state spring frequency (Hz) of length-constraint limits after warmup. 0 = rigid "
	"(stock behavior)." );
static ConVar vjolt_length_spring_damping( "vjolt_length_spring_damping", "1.0", FCVAR_NONE,
	"Steady-state spring damping ratio of length-constraint limits after warmup." );

static constexpr float UNBREAKABLE_BREAK_LIMIT = 1e12f;

static ConVar vjolt_constraint_break_debug( "vjolt_constraint_break_debug", "0", FCVAR_NONE,
	"Log lambda + threshold values whenever a breakable constraint trips." );

//-------------------------------------------------------------------------------------------------

// Bounded, explicitly armed first-solve diagnostics for rotation-only SixDOF
// constraints. Physics paths only copy numeric state into these records; all
// text output is deferred to an operator-issued command after the measured
// updates have completed.
static constexpr uint32 ONLYROT_TRACE_MAX_RECORDS = 64;
static constexpr uint32 ONLYROT_TRACE_MAX_CONTACTS = 2;
static constexpr uint32 ONLYROT_TRACE_UPDATE_COUNT = 8;
static constexpr uint32 ONLYROT_TRACE_CONTACT_SAMPLE_COUNT = 4;

static int OnlyRotTraceContactSampleForUpdate( uint32 nUpdate )
{
	switch ( nUpdate )
	{
	case 2: return 0;
	case 3: return 1;
	case 4: return 2;
	case 7: return 3;
	default: return -1;
	}
}

enum OnlyRotTraceCompletion : uint8
{
	ONLYROT_TRACE_PENDING = 0,
	ONLYROT_TRACE_COMPLETE,
	ONLYROT_TRACE_DESTROYED,
};

struct OnlyRotTraceVec3
{
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
};

struct OnlyRotTraceQuat
{
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
	float w = 1.0f;
};

struct OnlyRotTraceBodyIdentity
{
	uint8 bValid = 0;
	uint8 bSourceStatic = 0;
	uint8 bSourceMotionEnabled = 0;
	uint8 bJoltActive = 0;
	uint8 nJoltMotionType = 0;
	uint16 nObjectLayer = 0;
	uint32 nBodyID = 0;
	uintp nGameData = 0;
};

struct OnlyRotTraceBodyState
{
	OnlyRotTraceBodyIdentity identity;
	OnlyRotTraceVec3 vPositionSource;
	OnlyRotTraceQuat qRotation;
	OnlyRotTraceVec3 vLinearVelocitySource;
	OnlyRotTraceVec3 vAngularVelocityDegrees;
};

struct OnlyRotTraceConstraintState
{
	uint8 bValid = 0;
	uint8 bEnabled = 0;
	uint8 bActive = 0;
	uint8 nSubType = 0;
	uint8 nRotationPositionMotorMask = 0;
	OnlyRotTraceQuat qRotationConstraintSpace;
};

struct OnlyRotTraceContact
{
	OnlyRotTraceBodyIdentity other;
	OnlyRotTraceVec3 vEstimatedNormalImpulse;
	OnlyRotTraceVec3 vContactPointSource;
};

struct OnlyRotTraceContactSummary
{
	uint8 bEstimateEnabled = 0;
	uint8 bFresh = 0;
	uint32 nStored = 0;
	uint32 nTotal = 0;
	std::array< OnlyRotTraceContact, ONLYROT_TRACE_MAX_CONTACTS > contacts{};
};

struct OnlyRotTraceUpdate
{
	uint8 bPreCaptured = 0;
	uint8 bPostCaptured = 0;
	uint8 bTickDiscontinuity = 0;
	uint32 nPreContactTick = 0;
	uint32 nPostContactTick = 0;
	std::array< OnlyRotTraceBodyState, 2 > preBodies{};
	std::array< OnlyRotTraceBodyState, 2 > postBodies{};
	OnlyRotTraceConstraintState preConstraint;
	OnlyRotTraceConstraintState postConstraint;
	OnlyRotTraceVec3 vPreviousLambdaPosition;
	OnlyRotTraceVec3 vPreviousLambdaRotation;
	OnlyRotTraceVec3 vPreviousLambdaMotorRotation;
	OnlyRotTraceVec3 vLambdaPosition;
	OnlyRotTraceVec3 vLambdaRotation;
	OnlyRotTraceVec3 vLambdaMotorRotation;
};

struct OnlyRotTraceContactSample
{
	uint8 bCaptured = 0;
	uint8 nUpdate = 0;
	std::array< OnlyRotTraceContactSummary, 2 > endpoints{};
};

struct OnlyRotTraceRecapture
{
	uint32 nCalls = 0;
	uint32 nLastTick = 0;
	int nLastCountdownBefore = 0;
	uint8 bRebuilt = 0;
	uint8 bOldEnabled = 0;
	uint8 bMotorCanonical = 0;
	uint8 nMotorRotationAxisMask = 0;
	float flMotorFrequency = 0.0f;
	float flMotorDamping = 0.0f;
	OnlyRotTraceQuat qReferenceToAttached;
	OnlyRotTraceVec3 vOldAxisX1;
	OnlyRotTraceVec3 vOldAxisY1;
	OnlyRotTraceVec3 vNewAxisX1;
	OnlyRotTraceVec3 vNewAxisY1;
};

struct OnlyRotTraceRecord
{
	uint32 nGeneration = 0;
	uint32 nTraceID = 0;
	uint32 nCreationContactTick = 0;
	uint32 nStartContactTick = 0;
	uint32 nHeldPreSimCalls = 0;
	uint8 nCompletion = ONLYROT_TRACE_PENDING;
	uint8 nPreUpdates = 0;
	uint8 nPostUpdates = 0;
	uint8 bExplicitStart = 0;
	uint8 bAnyTickDiscontinuity = 0;
	uint8 bHasGroup = 0;
	uint8 bSourceActive = 0;
	uint8 bUseClockwiseRotations = 0;
	uint8 nDegreesOfFreedomMask = 0;
	uint8 nFreeAxisMask = 0;
	uint8 nFixedAxisMask = 0;
	std::array< float, 3 > flLimitMinRadians{};
	std::array< float, 3 > flLimitMaxRadians{};
	std::array< float, 3 > flRotationMaxFriction{};
	OnlyRotTraceVec3 vPosition1Source;
	OnlyRotTraceVec3 vAxisX1;
	OnlyRotTraceVec3 vAxisY1;
	OnlyRotTraceVec3 vPosition2Source;
	OnlyRotTraceVec3 vAxisX2;
	OnlyRotTraceVec3 vAxisY2;
	std::array< OnlyRotTraceBodyState, 2 > creationBodies{};
	OnlyRotTraceConstraintState creationConstraint;
	std::array< OnlyRotTraceUpdate, ONLYROT_TRACE_UPDATE_COUNT > updates{};
	std::array< OnlyRotTraceContactSample, ONLYROT_TRACE_CONTACT_SAMPLE_COUNT > contactSamples{};
	OnlyRotTraceRecapture recapture;
};

struct OnlyRotTraceLive
{
	OnlyRotTraceRecord record;
	uint32 nHoldSerial = 0;
	uint8 nPreUpdates = 0;
	uint8 nPostUpdates = 0;
	uint8 bWaitingForStart = 0;
	uint8 bStarted = 0;
};

static std::array< OnlyRotTraceRecord, ONLYROT_TRACE_MAX_RECORDS > g_OnlyRotTraceCompleted{};
static uint32 g_nOnlyRotTraceCompleted = 0;
static uint32 g_nOnlyRotTraceDropped = 0;
static uint32 g_nOnlyRotTracePending = 0;
static uint32 g_nOnlyRotTraceLive = 0;
static uint32 g_nOnlyRotTraceHeldPending = 0;
static uint32 g_nOnlyRotTraceHeldLive = 0;
static uint32 g_nOnlyRotTraceArmBudget = 0;
static uint32 g_nOnlyRotTraceGeneration = 1;
static uint32 g_nOnlyRotTraceNextID = 1;
static uint32 g_nOnlyRotTraceStartSerial = 1;
static bool g_bOnlyRotTraceArmHeld = false;

static void ReleaseOnlyRotTraceHold( OnlyRotTraceLive &trace )
{
	if ( !trace.bWaitingForStart )
		return;

	Assert( g_nOnlyRotTraceHeldLive > 0 );
	if ( g_nOnlyRotTraceHeldLive > 0 )
		--g_nOnlyRotTraceHeldLive;
	if ( trace.record.nGeneration == g_nOnlyRotTraceGeneration )
	{
		Assert( g_nOnlyRotTraceHeldPending > 0 );
		if ( g_nOnlyRotTraceHeldPending > 0 )
			--g_nOnlyRotTraceHeldPending;
	}
	trace.bWaitingForStart = 0;
}

static void ReleaseOnlyRotTraceLive( std::unique_ptr< OnlyRotTraceLive > &pTrace )
{
	if ( !pTrace )
		return;

	ReleaseOnlyRotTraceHold( *pTrace );
	Assert( g_nOnlyRotTraceLive > 0 );
	if ( g_nOnlyRotTraceLive > 0 )
		--g_nOnlyRotTraceLive;
	pTrace.reset();
}

static OnlyRotTraceVec3 OnlyRotTraceUnitless( JPH::Vec3Arg value )
{
	return { value.GetX(), value.GetY(), value.GetZ() };
}

static OnlyRotTraceVec3 OnlyRotTraceSourceDistance( JPH::Vec3Arg value )
{
	return { JoltToSource::Distance( value.GetX() ), JoltToSource::Distance( value.GetY() ), JoltToSource::Distance( value.GetZ() ) };
}

static OnlyRotTraceVec3 OnlyRotTraceSourcePosition( JPH::RVec3Arg value )
{
	return {
		JoltToSource::Distance( static_cast< float >( value.GetX() ) ),
		JoltToSource::Distance( static_cast< float >( value.GetY() ) ),
		JoltToSource::Distance( static_cast< float >( value.GetZ() ) )
	};
}

static OnlyRotTraceVec3 OnlyRotTraceSourceVector( const Vector &value )
{
	return { value.x, value.y, value.z };
}

static OnlyRotTraceVec3 OnlyRotTraceDegrees( JPH::Vec3Arg value )
{
	return { RAD2DEG( value.GetX() ), RAD2DEG( value.GetY() ), RAD2DEG( value.GetZ() ) };
}

static OnlyRotTraceQuat OnlyRotTraceQuaternion( JPH::QuatArg value )
{
	return { value.GetX(), value.GetY(), value.GetZ(), value.GetW() };
}

static OnlyRotTraceBodyIdentity CaptureOnlyRotBodyIdentity( JoltPhysicsObject *pObject )
{
	OnlyRotTraceBodyIdentity out;
	if ( !pObject || !pObject->GetBody() )
		return out;

	JPH::Body *pBody = pObject->GetBody();
	out.bValid = 1;
	out.bSourceStatic = pObject->IsStatic();
	out.bSourceMotionEnabled = pObject->IsMotionEnabled();
	out.bJoltActive = pBody->IsActive();
	out.nJoltMotionType = static_cast< uint8 >( pBody->GetMotionType() );
	out.nObjectLayer = static_cast< uint16 >( pBody->GetObjectLayer() );
	out.nBodyID = pBody->GetID().GetIndexAndSequenceNumber();
	out.nGameData = reinterpret_cast< uintp >( pObject->GetGameData() );
	return out;
}

static OnlyRotTraceBodyState CaptureOnlyRotBodyState( JoltPhysicsObject *pObject )
{
	OnlyRotTraceBodyState out;
	out.identity = CaptureOnlyRotBodyIdentity( pObject );
	if ( !out.identity.bValid )
		return out;

	JPH::Body *pBody = pObject->GetBody();
	out.vPositionSource = OnlyRotTraceSourcePosition( pBody->GetPosition() );
	out.qRotation = OnlyRotTraceQuaternion( pBody->GetRotation() );
	out.vLinearVelocitySource = OnlyRotTraceSourceDistance( pBody->GetLinearVelocity() );
	out.vAngularVelocityDegrees = OnlyRotTraceDegrees( pBody->GetAngularVelocity() );
	return out;
}

static OnlyRotTraceConstraintState CaptureOnlyRotConstraintState( JPH::Constraint *pConstraint )
{
	OnlyRotTraceConstraintState out;
	if ( !pConstraint )
		return out;

	out.bValid = 1;
	out.bEnabled = pConstraint->GetEnabled();
	out.bActive = pConstraint->IsActive();
	out.nSubType = static_cast< uint8 >( pConstraint->GetSubType() );
	if ( pConstraint->GetSubType() == JPH::EConstraintSubType::SixDOF )
	{
		const JPH::SixDOFConstraint *pSixDOF = static_cast< const JPH::SixDOFConstraint * >( pConstraint );
		out.qRotationConstraintSpace = OnlyRotTraceQuaternion( pSixDOF->GetRotationInConstraintSpace() );
		for ( int i = 0; i < 3; ++i )
		{
			const JPH::SixDOFConstraint::EAxis eAxis = static_cast< JPH::SixDOFConstraint::EAxis >(
				JPH::SixDOFConstraint::EAxis::RotationX + i );
			if ( JPH::IsPositionMotor( pSixDOF->GetMotorState( eAxis ) ) )
				out.nRotationPositionMotorMask |= 1u << i;
		}
	}
	return out;
}

static void CaptureOnlyRotContacts( JoltPhysicsObject *pObject, OnlyRotTraceContactSummary &out )
{
	out.bEstimateEnabled = vjolt_contact_estimate.GetBool();
	if ( !pObject || !out.bEstimateEnabled )
		return;

	std::array< JoltPhysicsObject::ContactPairData, ONLYROT_TRACE_MAX_CONTACTS > pairs{};
	uint32 nStored = 0;
	uint32 nTotal = 0;
	out.bFresh = pObject->GetFreshContactPairs( pairs.data(), ONLYROT_TRACE_MAX_CONTACTS, nStored, nTotal );
	out.nStored = nStored;
	out.nTotal = nTotal;

	for ( uint32 i = 0; i < nStored; ++i )
	{
		OnlyRotTraceContact &contact = out.contacts[i];
		contact.other = CaptureOnlyRotBodyIdentity( pairs[i].pOther );
		contact.vEstimatedNormalImpulse = OnlyRotTraceSourceVector( pairs[i].vImpulse );
		contact.vContactPointSource = OnlyRotTraceSourceVector( pairs[i].vContactPoint );
	}
}

static const char *OnlyRotTraceCompletionName( uint8 nCompletion )
{
	switch ( nCompletion )
	{
	case ONLYROT_TRACE_COMPLETE: return "complete";
	case ONLYROT_TRACE_DESTROYED: return "destroyed_partial";
	default: return "pending";
	}
}

static const char *OnlyRotTraceMotionName( uint8 nMotionType )
{
	switch ( static_cast< JPH::EMotionType >( nMotionType ) )
	{
	case JPH::EMotionType::Static: return "static";
	case JPH::EMotionType::Kinematic: return "kinematic";
	case JPH::EMotionType::Dynamic: return "dynamic";
	default: return "unknown";
	}
}

static const char *OnlyRotTraceLayerName( uint16 nLayer )
{
	switch ( nLayer )
	{
	case Layers::NON_MOVING_WORLD: return "non_moving_world";
	case Layers::NON_MOVING_OBJECT: return "non_moving_object";
	case Layers::MOVING: return "moving";
	case Layers::NO_COLLIDE: return "no_collide";
	case Layers::DEBRIS: return "debris";
	case Layers::MOVING_PLAYER: return "moving_player";
	default: return "unknown";
	}
}

static void DumpOnlyRotBodyState( uint32 nTraceID, uint32 nUpdate, const char *pszPhase, const char *pszEndpoint, const OnlyRotTraceBodyState &body )
{
	Msg( "vjolt_onlyrot_trace id=%u update=%u phase=%s endpoint=%s valid=%u body=%u game=%p src_static=%u src_motion=%u jolt_active=%u motion=%s(%u) layer=%s(%u) "
		"pos_u=(%.6g %.6g %.6g) rot_q=(%.6g %.6g %.6g %.6g) vel_u_s=(%.6g %.6g %.6g) ang_deg_s=(%.6g %.6g %.6g)\n",
		nTraceID, nUpdate, pszPhase, pszEndpoint, body.identity.bValid, body.identity.nBodyID,
		reinterpret_cast< void * >( body.identity.nGameData ), body.identity.bSourceStatic, body.identity.bSourceMotionEnabled,
		body.identity.bJoltActive, OnlyRotTraceMotionName( body.identity.nJoltMotionType ), body.identity.nJoltMotionType,
		OnlyRotTraceLayerName( body.identity.nObjectLayer ), body.identity.nObjectLayer,
		body.vPositionSource.x, body.vPositionSource.y, body.vPositionSource.z,
		body.qRotation.x, body.qRotation.y, body.qRotation.z, body.qRotation.w,
		body.vLinearVelocitySource.x, body.vLinearVelocitySource.y, body.vLinearVelocitySource.z,
		body.vAngularVelocityDegrees.x, body.vAngularVelocityDegrees.y, body.vAngularVelocityDegrees.z );
}

static void DumpOnlyRotConstraintState( uint32 nTraceID, uint32 nUpdate, const char *pszPhase, const OnlyRotTraceConstraintState &constraint )
{
	Msg( "vjolt_onlyrot_trace id=%u update=%u phase=%s constraint_valid=%u enabled=%u active=%u subtype=%u position_motor_rotation_mask=0x%02x rotation_cs_q=(%.6g %.6g %.6g %.6g)\n",
		nTraceID, nUpdate, pszPhase, constraint.bValid, constraint.bEnabled, constraint.bActive, constraint.nSubType,
		constraint.nRotationPositionMotorMask,
		constraint.qRotationConstraintSpace.x, constraint.qRotationConstraintSpace.y,
		constraint.qRotationConstraintSpace.z, constraint.qRotationConstraintSpace.w );
}

static void DumpOnlyRotRecord( const OnlyRotTraceRecord &record )
{
	Msg( "vjolt_onlyrot_trace id=%u generation=%u completion=%s pre=%u post=%u create_tick=%u start_tick=%u explicit_start=%u held_presim=%u tick_discontinuity=%u group=%u source_active=%u clockwise=%u dof_mask=0x%02x free_mask=0x%02x fixed_mask=0x%02x\n",
		record.nTraceID, record.nGeneration, OnlyRotTraceCompletionName( record.nCompletion ), record.nPreUpdates,
		record.nPostUpdates, record.nCreationContactTick, record.nStartContactTick, record.bExplicitStart,
		record.nHeldPreSimCalls, record.bAnyTickDiscontinuity, record.bHasGroup,
		record.bSourceActive, record.bUseClockwiseRotations, record.nDegreesOfFreedomMask,
		record.nFreeAxisMask, record.nFixedAxisMask );
	Msg( "vjolt_onlyrot_trace id=%u creation limits_rad=((%.6g %.6g) (%.6g %.6g) (%.6g %.6g)) rotation_friction=(%.6g %.6g %.6g) "
		"frame1_pos_u=(%.6g %.6g %.6g) frame1_x=(%.6g %.6g %.6g) frame1_y=(%.6g %.6g %.6g) "
		"frame2_pos_u=(%.6g %.6g %.6g) frame2_x=(%.6g %.6g %.6g) frame2_y=(%.6g %.6g %.6g)\n",
		record.nTraceID,
		record.flLimitMinRadians[0], record.flLimitMaxRadians[0], record.flLimitMinRadians[1], record.flLimitMaxRadians[1],
		record.flLimitMinRadians[2], record.flLimitMaxRadians[2], record.flRotationMaxFriction[0],
		record.flRotationMaxFriction[1], record.flRotationMaxFriction[2],
		record.vPosition1Source.x, record.vPosition1Source.y, record.vPosition1Source.z,
		record.vAxisX1.x, record.vAxisX1.y, record.vAxisX1.z, record.vAxisY1.x, record.vAxisY1.y, record.vAxisY1.z,
		record.vPosition2Source.x, record.vPosition2Source.y, record.vPosition2Source.z,
		record.vAxisX2.x, record.vAxisX2.y, record.vAxisX2.z, record.vAxisY2.x, record.vAxisY2.y, record.vAxisY2.z );

	DumpOnlyRotBodyState( record.nTraceID, 0, "creation", "reference", record.creationBodies[0] );
	DumpOnlyRotBodyState( record.nTraceID, 0, "creation", "attached", record.creationBodies[1] );
	DumpOnlyRotConstraintState( record.nTraceID, 0, "creation", record.creationConstraint );

	for ( uint32 nUpdate = 0; nUpdate < ONLYROT_TRACE_UPDATE_COUNT; ++nUpdate )
	{
		const OnlyRotTraceUpdate &update = record.updates[nUpdate];
		if ( !update.bPreCaptured && !update.bPostCaptured )
			continue;
		const int nContactSampleIndex = OnlyRotTraceContactSampleForUpdate( nUpdate );
		const OnlyRotTraceContactSample *pContactSample =
			nContactSampleIndex >= 0 && record.contactSamples[nContactSampleIndex].bCaptured
				? &record.contactSamples[nContactSampleIndex]
				: nullptr;

		Msg( "vjolt_onlyrot_trace id=%u update=%u pre_tick=%u post_tick=%u pre_captured=%u post_captured=%u tick_discontinuity=%u contact_sampled=%u "
			"previous_lambda_position_jolt=(%.6g %.6g %.6g) previous_lambda_rotation_jolt=(%.6g %.6g %.6g) "
			"previous_lambda_motor_rotation_jolt=(%.6g %.6g %.6g) lambda_position_jolt=(%.6g %.6g %.6g) "
			"lambda_rotation_jolt=(%.6g %.6g %.6g) lambda_motor_rotation_jolt=(%.6g %.6g %.6g)\n",
			record.nTraceID, nUpdate + 1, update.nPreContactTick, update.nPostContactTick,
			update.bPreCaptured, update.bPostCaptured, update.bTickDiscontinuity, pContactSample != nullptr,
			update.vPreviousLambdaPosition.x, update.vPreviousLambdaPosition.y, update.vPreviousLambdaPosition.z,
			update.vPreviousLambdaRotation.x, update.vPreviousLambdaRotation.y, update.vPreviousLambdaRotation.z,
			update.vPreviousLambdaMotorRotation.x, update.vPreviousLambdaMotorRotation.y, update.vPreviousLambdaMotorRotation.z,
			update.vLambdaPosition.x, update.vLambdaPosition.y, update.vLambdaPosition.z,
			update.vLambdaRotation.x, update.vLambdaRotation.y, update.vLambdaRotation.z,
			update.vLambdaMotorRotation.x, update.vLambdaMotorRotation.y, update.vLambdaMotorRotation.z );

		if ( update.bPreCaptured )
		{
			DumpOnlyRotBodyState( record.nTraceID, nUpdate + 1, "pre", "reference", update.preBodies[0] );
			DumpOnlyRotBodyState( record.nTraceID, nUpdate + 1, "pre", "attached", update.preBodies[1] );
			DumpOnlyRotConstraintState( record.nTraceID, nUpdate + 1, "pre", update.preConstraint );
		}
		if ( !update.bPostCaptured )
			continue;

		DumpOnlyRotBodyState( record.nTraceID, nUpdate + 1, "post", "reference", update.postBodies[0] );
		DumpOnlyRotBodyState( record.nTraceID, nUpdate + 1, "post", "attached", update.postBodies[1] );
		DumpOnlyRotConstraintState( record.nTraceID, nUpdate + 1, "post", update.postConstraint );
		if ( !pContactSample )
			continue;

		for ( uint32 nEndpoint = 0; nEndpoint < 2; ++nEndpoint )
		{
			const char *pszEndpoint = nEndpoint == 0 ? "reference" : "attached";
			const OnlyRotTraceContactSummary &summary = pContactSample->endpoints[nEndpoint];
			Msg( "vjolt_onlyrot_trace id=%u update=%u endpoint=%s contact_estimate_enabled=%u fresh=%u stored=%u total=%u truncated=%u\n",
				record.nTraceID, nUpdate + 1, pszEndpoint, summary.bEstimateEnabled, summary.bFresh,
				summary.nStored, summary.nTotal, summary.nTotal > summary.nStored );
			for ( uint32 i = 0; i < summary.nStored; ++i )
			{
				const OnlyRotTraceContact &contact = summary.contacts[i];
				Msg( "vjolt_onlyrot_trace id=%u update=%u endpoint=%s contact=%u other_valid=%u other_body=%u other_game=%p "
					"other_src_static=%u other_src_motion=%u other_active=%u other_motion=%s(%u) other_layer=%s(%u) "
					"estimated_normal_impulse_kg_m_s=(%.6g %.6g %.6g) point_u=(%.6g %.6g %.6g)\n",
					record.nTraceID, nUpdate + 1, pszEndpoint, i, contact.other.bValid, contact.other.nBodyID,
					reinterpret_cast< void * >( contact.other.nGameData ), contact.other.bSourceStatic,
					contact.other.bSourceMotionEnabled, contact.other.bJoltActive,
					OnlyRotTraceMotionName( contact.other.nJoltMotionType ), contact.other.nJoltMotionType,
					OnlyRotTraceLayerName( contact.other.nObjectLayer ), contact.other.nObjectLayer,
					contact.vEstimatedNormalImpulse.x, contact.vEstimatedNormalImpulse.y, contact.vEstimatedNormalImpulse.z,
					contact.vContactPointSource.x, contact.vContactPointSource.y, contact.vContactPointSource.z );
			}
		}
	}

	Msg( "vjolt_onlyrot_trace id=%u recapture_calls=%u last_tick=%u last_countdown_before=%d rebuilt=%u old_enabled=%u "
		"motor_canonical=%u motor_rotation_mask=0x%02x motor_frequency_hz=%.6g motor_damping=%.6g "
		"relative_q=(%.6g %.6g %.6g %.6g) old_x1=(%.6g %.6g %.6g) old_y1=(%.6g %.6g %.6g) "
		"new_x1=(%.6g %.6g %.6g) new_y1=(%.6g %.6g %.6g)\n",
		record.nTraceID, record.recapture.nCalls, record.recapture.nLastTick, record.recapture.nLastCountdownBefore,
		record.recapture.bRebuilt, record.recapture.bOldEnabled,
		record.recapture.bMotorCanonical, record.recapture.nMotorRotationAxisMask,
		record.recapture.flMotorFrequency, record.recapture.flMotorDamping,
		record.recapture.qReferenceToAttached.x, record.recapture.qReferenceToAttached.y,
		record.recapture.qReferenceToAttached.z, record.recapture.qReferenceToAttached.w,
		record.recapture.vOldAxisX1.x, record.recapture.vOldAxisX1.y, record.recapture.vOldAxisX1.z,
		record.recapture.vOldAxisY1.x, record.recapture.vOldAxisY1.y, record.recapture.vOldAxisY1.z,
		record.recapture.vNewAxisX1.x, record.recapture.vNewAxisX1.y, record.recapture.vNewAxisX1.z,
		record.recapture.vNewAxisY1.x, record.recapture.vNewAxisY1.y, record.recapture.vNewAxisY1.z );
}

static void PrintOnlyRotTraceStatus()
{
	Msg( "vjolt_onlyrot_trace_status generation=%u armed=%u arm_mode=%s pending=%u live=%u held_pending=%u held_live=%u start_serial=%u completed=%u dropped=%u capacity=%u updates=%u "
		"contact_updates=3,4,5,8 contacts_per_endpoint=%u record_bytes=%u\n",
		g_nOnlyRotTraceGeneration, g_nOnlyRotTraceArmBudget, g_bOnlyRotTraceArmHeld ? "held" : "immediate",
		g_nOnlyRotTracePending, g_nOnlyRotTraceLive, g_nOnlyRotTraceHeldPending, g_nOnlyRotTraceHeldLive,
		g_nOnlyRotTraceStartSerial,
		g_nOnlyRotTraceCompleted, g_nOnlyRotTraceDropped, ONLYROT_TRACE_MAX_RECORDS, ONLYROT_TRACE_UPDATE_COUNT,
		ONLYROT_TRACE_MAX_CONTACTS, static_cast< uint32 >( sizeof( OnlyRotTraceRecord ) ) );
}

static bool ParseOnlyRotTraceArmCount( const char *pszValue, uint32 &nOut )
{
	errno = 0;
	char *pEnd = nullptr;
	const long nValue = std::strtol( pszValue, &pEnd, 10 );
	if ( errno == ERANGE || pEnd == pszValue || *pEnd != '\0'
		|| nValue < 0 || nValue > static_cast< long >( ONLYROT_TRACE_MAX_RECORDS ) )
		return false;

	nOut = static_cast< uint32 >( nValue );
	return true;
}

CON_COMMAND( vjolt_onlyrot_trace_next, "Arm bounded first-eight-update diagnostics (contact detail at updates 3,4,5,8) for the next N rotation-only constraints (0-64)." )
{
	if ( args.ArgC() != 2 )
	{
		Msg( "usage: vjolt_onlyrot_trace_next <0-%u>\n", ONLYROT_TRACE_MAX_RECORDS );
		PrintOnlyRotTraceStatus();
		return;
	}

	uint32 nRequested = 0;
	if ( !ParseOnlyRotTraceArmCount( args.Arg( 1 ), nRequested ) )
	{
		Msg( "vjolt_onlyrot_trace_next rejected: count must be an integer from 0 through %u\n", ONLYROT_TRACE_MAX_RECORDS );
		PrintOnlyRotTraceStatus();
		return;
	}
	const uint32 nReserved = Min( ONLYROT_TRACE_MAX_RECORDS, g_nOnlyRotTraceLive + g_nOnlyRotTraceCompleted );
	const uint32 nAvailable = ONLYROT_TRACE_MAX_RECORDS - nReserved;
	g_nOnlyRotTraceArmBudget = Min( nRequested, nAvailable );
	g_bOnlyRotTraceArmHeld = false;
	PrintOnlyRotTraceStatus();
}

CON_COMMAND( vjolt_onlyrot_trace_hold_next, "Attach bounded diagnostics to the next N rotation-only constraints, but wait for vjolt_onlyrot_trace_start before sampling." )
{
	if ( args.ArgC() != 2 )
	{
		Msg( "usage: vjolt_onlyrot_trace_hold_next <0-%u>\n", ONLYROT_TRACE_MAX_RECORDS );
		PrintOnlyRotTraceStatus();
		return;
	}

	uint32 nRequested = 0;
	if ( !ParseOnlyRotTraceArmCount( args.Arg( 1 ), nRequested ) )
	{
		Msg( "vjolt_onlyrot_trace_hold_next rejected: count must be an integer from 0 through %u\n", ONLYROT_TRACE_MAX_RECORDS );
		PrintOnlyRotTraceStatus();
		return;
	}
	const uint32 nReserved = Min( ONLYROT_TRACE_MAX_RECORDS, g_nOnlyRotTraceLive + g_nOnlyRotTraceCompleted );
	const uint32 nAvailable = ONLYROT_TRACE_MAX_RECORDS - nReserved;
	g_nOnlyRotTraceArmBudget = Min( nRequested, nAvailable );
	g_bOnlyRotTraceArmHeld = g_nOnlyRotTraceArmBudget > 0;
	PrintOnlyRotTraceStatus();
}

CON_COMMAND( vjolt_onlyrot_trace_start, "Release all current-generation held rotation-only traces together; sampling begins at the next physics update." )
{
	if ( args.ArgC() != 1 )
	{
		Msg( "usage: vjolt_onlyrot_trace_start\n" );
		PrintOnlyRotTraceStatus();
		return;
	}
	if ( g_nOnlyRotTraceArmBudget != 0 )
	{
		Msg( "vjolt_onlyrot_trace_start rejected: armed=%u constraints have not attached yet\n", g_nOnlyRotTraceArmBudget );
		PrintOnlyRotTraceStatus();
		return;
	}
	if ( g_nOnlyRotTraceHeldPending == 0 || g_nOnlyRotTraceHeldPending != g_nOnlyRotTracePending )
	{
		Msg( "vjolt_onlyrot_trace_start rejected: held_pending=%u pending=%u\n",
			g_nOnlyRotTraceHeldPending, g_nOnlyRotTracePending );
		PrintOnlyRotTraceStatus();
		return;
	}

	if ( ++g_nOnlyRotTraceStartSerial == 0 )
		++g_nOnlyRotTraceStartSerial;
	g_bOnlyRotTraceArmHeld = false;
	Msg( "vjolt_onlyrot_trace_start release_issued=1 generation=%u held=%u start_serial=%u\n",
		g_nOnlyRotTraceGeneration, g_nOnlyRotTraceHeldPending, g_nOnlyRotTraceStartSerial );
	PrintOnlyRotTraceStatus();
}

CON_COMMAND( vjolt_onlyrot_trace_status, "Report bounded rotation-only trace counters without dumping records." )
{
	PrintOnlyRotTraceStatus();
}

CON_COMMAND( vjolt_onlyrot_trace_dump, "Dump one completed bounded rotation-only trace by zero-based record index, after physics settles." )
{
	PrintOnlyRotTraceStatus();
	if ( args.ArgC() != 2 )
	{
		Msg( "usage: vjolt_onlyrot_trace_dump <record_index>; valid range is 0..%d\n",
			g_nOnlyRotTraceCompleted > 0 ? static_cast< int >( g_nOnlyRotTraceCompleted - 1 ) : -1 );
		return;
	}

	const int nRecord = atoi( args.Arg( 1 ) );
	if ( nRecord < 0 || static_cast< uint32 >( nRecord ) >= g_nOnlyRotTraceCompleted )
	{
		Msg( "vjolt_onlyrot_trace_dump invalid record_index=%d; valid range is 0..%d\n", nRecord,
			g_nOnlyRotTraceCompleted > 0 ? static_cast< int >( g_nOnlyRotTraceCompleted - 1 ) : -1 );
		return;
	}

	Msg( "vjolt_onlyrot_trace_dump record_index=%d generation=%u trace_id=%u completed=%u dropped=%u pending=%u live=%u\n",
		nRecord, g_OnlyRotTraceCompleted[nRecord].nGeneration, g_OnlyRotTraceCompleted[nRecord].nTraceID,
		g_nOnlyRotTraceCompleted, g_nOnlyRotTraceDropped, g_nOnlyRotTracePending, g_nOnlyRotTraceLive );
	DumpOnlyRotRecord( g_OnlyRotTraceCompleted[nRecord] );
}

CON_COMMAND( vjolt_onlyrot_trace_clear, "Disarm and clear rotation-only trace diagnostics; in-flight older-generation traces self-discard." )
{
	g_nOnlyRotTraceArmBudget = 0;
	g_bOnlyRotTraceArmHeld = false;
	g_nOnlyRotTracePending = 0;
	g_nOnlyRotTraceHeldPending = 0;
	g_nOnlyRotTraceCompleted = 0;
	g_nOnlyRotTraceDropped = 0;
	if ( ++g_nOnlyRotTraceGeneration == 0 )
		++g_nOnlyRotTraceGeneration;
	PrintOnlyRotTraceStatus();
}

//-------------------------------------------------------------------------------------------------

static JPH::Vec3 HingePerpendicularVector( JPH::Vec3Arg dir )
{
	return fabsf( dir.GetX() ) < 0.57f
		? JPH::Vec3::sAxisX().Cross( dir ).Normalized()
		: JPH::Vec3::sAxisY().Cross( dir ).Normalized();
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsConstraintGroup::JoltPhysicsConstraintGroup()
{
}

JoltPhysicsConstraintGroup::~JoltPhysicsConstraintGroup()
{
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraintGroup::Activate()
{
	for ( JoltPhysicsConstraint *pConstraint : m_pConstraints )
		pConstraint->Activate();
}

bool JoltPhysicsConstraintGroup::IsInErrorState()
{
	return false;
}

void JoltPhysicsConstraintGroup::ClearErrorState()
{
}

void JoltPhysicsConstraintGroup::GetErrorParams( constraint_groupparams_t *pParams )
{
	if ( pParams )
		*pParams = m_ErrorParams;
}

void JoltPhysicsConstraintGroup::SetErrorParams( const constraint_groupparams_t &params )
{
	m_ErrorParams = params;
}

void JoltPhysicsConstraintGroup::SolvePenetration( IPhysicsObject *pObj0, IPhysicsObject *pObj1 )
{
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraintGroup::AddConstraint( JoltPhysicsConstraint *pConstraint )
{
	m_pConstraints.push_back( pConstraint );
}

void JoltPhysicsConstraintGroup::RemoveConstraint( JoltPhysicsConstraint *pConstraint )
{
	Erase( m_pConstraints, pConstraint );
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsConstraint::JoltPhysicsConstraint( JoltPhysicsEnvironment *pPhysicsEnvironment, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, constraintType_t Type, JPH::Constraint* pConstraint, void *pGameData )
	: m_pPhysicsEnvironment( pPhysicsEnvironment )
	, m_pPhysicsSystem( pPhysicsEnvironment->GetPhysicsSystem() )
	, m_pObjReference( static_cast<JoltPhysicsObject*>( pReferenceObject ) )
	, m_pObjAttached( static_cast<JoltPhysicsObject*>( pAttachedObject ) )
	, m_ConstraintType( Type )
	, m_pConstraint( pConstraint )
	, m_pGameData( pGameData )
{
	m_pObjReference->AddDestroyedListener( this );
	m_pObjAttached->AddDestroyedListener( this );
	m_pPhysicsEnvironment->RegisterConstraint( this );
}

JoltPhysicsConstraint::~JoltPhysicsConstraint()
{
	if ( m_pGroup )
	{
		m_pGroup->RemoveConstraint( this );
		m_pGroup = nullptr;
	}

	m_pPhysicsEnvironment->UnregisterConstraint( this );

	DestroyConstraint();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::Activate()
{
	if ( m_pConstraint )
		m_pConstraint->SetEnabled( true );
}

void JoltPhysicsConstraint::Deactivate()
{
	if ( m_pConstraint )
		m_pConstraint->SetEnabled( false );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetGameData( void *gameData )
{
	m_pGameData = gameData;
}

void *JoltPhysicsConstraint::GetGameData() const
{
	return m_pGameData;
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsConstraint::GetReferenceObject() const
{
	return m_pObjReference;
}

IPhysicsObject *JoltPhysicsConstraint::GetAttachedObject() const
{
	return m_pObjAttached;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetLinearMotor( float speed, float maxLinearImpulse )
{
	if ( !m_pConstraint )
		return;

	speed = SourceToJolt::Distance( speed );
	maxLinearImpulse = SourceToJolt::Distance( maxLinearImpulse );

	switch ( m_ConstraintType )
	{
		case CONSTRAINT_SLIDING:
		{
			JPH::SliderConstraint *pConstraint = static_cast<JPH::SliderConstraint *>( m_pConstraint.GetPtr() );
			pConstraint->SetMotorState( speed ? JPH::EMotorState::Velocity : JPH::EMotorState::Off );
			pConstraint->SetTargetVelocity( speed );

			JPH::MotorSettings &motorSettings = pConstraint->GetMotorSettings();
			motorSettings.SetForceLimits( -maxLinearImpulse, maxLinearImpulse );

			break;
		}
	}
}

void JoltPhysicsConstraint::SetAngularMotor( float rotSpeed, float maxAngularImpulse )
{
	if ( !m_pConstraint )
		return;

	// rotSpeed is in deg/s -> rad/s
	rotSpeed = DEG2RAD( rotSpeed );
	// maxAngularImpulse is a torque/impulse limit in Source units (kg*in^2/s^2 -> N*m = kg*m^2/s^2)
	// NOT an angle, so DEG2RAD is wrong here. Use squared distance factor.
	maxAngularImpulse = SourceToJolt::Torque( maxAngularImpulse );

	switch ( m_ConstraintType )
	{
		case CONSTRAINT_RAGDOLL:
		{
			// Josh:
			// If you change the hinge optimization stuff, remember to
			// check this! m_ConstraintType is CONSTRAINT_HINGE for that! (same with normal vphysics)
			//
			// Something else to note is... does the below code for friction vs angular impulse work on
			// ragdolls -> hinges correctly? This happens in Source, but this may not necessarily be correct.
			// :/

			// InitialiseRagdoll creates a Fixed, Hinge or SwingTwist constraint depending on the
			// number of free axes -- or a SixDOF for onlyAngularLimits (rotation-only) joints --
			// so dispatch on the actual Jolt subtype. Blind-casting here (as this used to)
			// corrupts memory on $animatedfriction models. SixDOF is deliberately left
			// unmotored below: engine animated-friction never combines with rotation-only
			// joints, and a motor would re-apply the min-torque-friction floor to what is
			// typically a free-spinning mechanical axis (vehicle wheels).
			switch ( m_pConstraint->GetSubType() )
			{
				case JPH::EConstraintSubType::SwingTwist:
				{
					JPH::SwingTwistConstraint *pConstraint = static_cast<JPH::SwingTwistConstraint *>( m_pConstraint.GetPtr() );
					const JPH::EMotorState eMotorState = rotSpeed ? JPH::EMotorState::Velocity : JPH::EMotorState::Off;
					pConstraint->SetSwingMotorState( eMotorState );
					pConstraint->SetTwistMotorState( eMotorState );
					pConstraint->SetTargetAngularVelocityCS( JPH::Vec3::sReplicate( rotSpeed ) );
					pConstraint->SetMaxFrictionTorque( Max( vjolt_ragdoll_min_torque_friction.GetFloat(), fabsf( maxAngularImpulse ) ) );
					break;
				}

				case JPH::EConstraintSubType::Hinge:
				{
					JPH::HingeConstraint *pConstraint = static_cast<JPH::HingeConstraint *>( m_pConstraint.GetPtr() );
					pConstraint->SetMotorState( rotSpeed ? JPH::EMotorState::Velocity : JPH::EMotorState::Off );
					pConstraint->SetTargetAngularVelocity( rotSpeed );
					pConstraint->SetMaxFrictionTorque( Max( vjolt_ragdoll_min_torque_friction.GetFloat(), fabsf( maxAngularImpulse ) ) );
					break;
				}

				default:
					// Fixed joint (no free axes) -- nothing to motor.
					break;
			}
			break;
		}

		case CONSTRAINT_HINGE:
		{
			JPH::HingeConstraint *pConstraint = static_cast<JPH::HingeConstraint *>( m_pConstraint.GetPtr() );
			pConstraint->SetMotorState( rotSpeed ? JPH::EMotorState::Velocity : JPH::EMotorState::Off );
			pConstraint->SetTargetAngularVelocity( rotSpeed );

			JPH::MotorSettings &motorSettings = pConstraint->GetMotorSettings();
			motorSettings.SetForceLimits( -fabsf( maxAngularImpulse ), fabsf( maxAngularImpulse ) );

			break;
		}
	}
}

//-------------------------------------------------------------------------------------------------

// Slart: This is never called anywhere in our codebase
void JoltPhysicsConstraint::UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached )
{
}

// Slart: This is only used for visual debugging, which we don't *really* need since we have Jolt's debugger
bool JoltPhysicsConstraint::GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const
{
	if ( m_pObjReference && pConstraintToReference )
		m_pObjReference->GetPositionMatrix( pConstraintToReference );
	if ( m_pObjAttached && pConstraintToAttached )
		m_pObjAttached->GetPositionMatrix( pConstraintToAttached );
	return true;
}

bool JoltPhysicsConstraint::GetConstraintParams( constraint_breakableparams_t *pParams ) const
{
	if ( !pParams )
		return false;

	pParams->forceLimit = m_SourceForceLimit;
	pParams->torqueLimit = m_SourceTorqueLimit;
	pParams->bodyMassScale[0] = m_BodyMassScale[0];
	pParams->bodyMassScale[1] = m_BodyMassScale[1];
	pParams->strength = m_BreakStrength;
	pParams->isActive = m_pConstraint ? m_pConstraint->GetEnabled() : false;
	return true;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetBreakableParams( const constraint_breakableparams_t &params )
{
	m_SourceForceLimit = params.forceLimit;
	m_SourceTorqueLimit = params.torqueLimit;
	m_BreakStrength = params.strength;
	m_BodyMassScale[0] = params.bodyMassScale[0];
	m_BodyMassScale[1] = params.bodyMassScale[1];

	const bool bBreakLinear = params.forceLimit > 0.0f && params.forceLimit < UNBREAKABLE_BREAK_LIMIT;
	const bool bBreakAngular = params.torqueLimit > 0.0f && params.torqueLimit < UNBREAKABLE_BREAK_LIMIT;

	m_LinearBreakImpulse = bBreakLinear ? SourceToJolt::Distance( params.forceLimit ) : 0.0f;
	// torqueLimit is an angular impulse (kg*in^2/s), not an angle -- convert with the
	// squared distance factor to match the Jolt lambdas compared in CheckBroken.
	m_AngularBreakImpulse = bBreakAngular ? SourceToJolt::Torque( params.torqueLimit ) : 0.0f;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::OutputDebugInfo()
{

}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	DestroyConstraint();

	// Normal VPhysics calls ConstraintBroken when an object being killed destroys the constraint.
	m_pPhysicsEnvironment->NotifyConstraintDisabled( this );
}

//-------------------------------------------------------------------------------------------------
// Ragdoll
//-------------------------------------------------------------------------------------------------

static std::optional<JoltMatrixAxes> DOFBitToAxis( uint32 uDOFMask )
{
	if ( uDOFMask & 0b001 )
		return MatrixAxis::X;
	else if ( uDOFMask & 0b010 )
		return MatrixAxis::Y;
	else if ( uDOFMask & 0b100 )
		return MatrixAxis::Z;
	else
		return std::nullopt;
}

struct RagdollLimits_t
{
	struct Limit_t
	{
		float Min;
		float Max;

		float GetRange() const
		{
			return Max - Min;
		}
	};

	RagdollLimits_t( const constraint_ragdollparams_t &ragdoll )
	{
		for ( int i = 0; i < 3; i++ )
		{
			if ( ragdoll.useClockwiseRotations )
			{
				lAxisLimitsRad[i].Min = DEG2RAD( -ragdoll.axes[i].maxRotation );
				lAxisLimitsRad[i].Max = DEG2RAD( -ragdoll.axes[i].minRotation );
			}
			else
			{
				lAxisLimitsRad[i].Min = DEG2RAD( ragdoll.axes[i].minRotation );
				lAxisLimitsRad[i].Max = DEG2RAD( ragdoll.axes[i].maxRotation );
			}
		}
	}

	uint32 GetDegreesOfFreedomMask() const
	{
		uint32 uDOFMask = 0;

		for ( int i = 0; i < 3; i++ )
		{
			if ( lAxisLimitsRad[i].GetRange() > DEG2RAD( 5.0f ) )
				uDOFMask |= 1u << i;
		}

		return uDOFMask;
	}

	Limit_t lAxisLimitsRad[3]{};
};

void JoltPhysicsConstraint::InitialiseRagdoll( IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_RAGDOLL;
	SetBreakableParams( ragdoll.constraint );

	JPH::Mat44 constraintToReference = SourceToJolt::Matrix( ragdoll.constraintToReference );
	JPH::Mat44 constraintToAttached = SourceToJolt::Matrix( ragdoll.constraintToAttached );

	RagdollLimits_t limits = RagdollLimits_t( ragdoll );
	
	const uint32 uDOFMask = limits.GetDegreesOfFreedomMask();
	const uint32 uDOFCount = JPH::CountBits( uDOFMask );

	JPH::Body *pRefBody = m_pObjReference->GetBody();
	JPH::Body *pAttBody = m_pObjAttached->GetBody();

	matrix3x4_t refObjToWorld;
	m_pObjReference->GetPositionMatrix( &refObjToWorld );

	matrix3x4_t constraintToWorld;
	ConcatTransforms( refObjToWorld, ragdoll.constraintToReference, constraintToWorld );

	const float flMinTorqueFriction = vjolt_ragdoll_min_torque_friction.GetFloat();

	JPH::Constraint *pConstraint = nullptr;
	bool bArmOnlyRotTrace = false;
	JPH::Ref< JPH::SixDOFConstraintSettings > pOnlyRotTraceSettings;

	if ( ragdoll.onlyAngularLimits )
	{
		bArmOnlyRotTrace = g_nOnlyRotTraceArmBudget > 0;
		// "Constrain rotation only" (GMod AdvBallsocket onlyrotation): the relative
		// POSITION must stay free -- contraptions hang position off ropes/elastics
		// (or nothing) and constrain only the relative orientation. LVS/simfphys
		// suspend every wheel on such a constraint to a motion-disabled steer
		// anchor, so locking translation here (what the Fixed/Hinge/SwingTwist
		// mappings below all do) welds the whole vehicle to the anchor's spawn
		// position: wheels spin, vehicle never moves. Jolt's SixDOF constraint
		// expresses rotation-only directly: leave all translation axes free (the
		// default) and map each rotation axis to fixed/free/limited.
		JPH::Ref< JPH::SixDOFConstraintSettings > settings = new JPH::SixDOFConstraintSettings;
		settings->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;

		settings->mPosition1 = constraintToReference.GetTranslation() - pRefBody->GetShape()->GetCenterOfMass();
		settings->mAxisX1 = constraintToReference.GetAxisX();
		settings->mAxisY1 = constraintToReference.GetAxisY();

		settings->mPosition2 = constraintToAttached.GetTranslation() - pAttBody->GetShape()->GetCenterOfMass();
		settings->mAxisX2 = constraintToAttached.GetAxisX();
		settings->mAxisY2 = constraintToAttached.GetAxisY();

		// Pyramid keeps the Y/Z swing limits independent per axis, matching the
		// ragdoll parameter layout (cone would couple them).
		settings->mSwingType = JPH::ESwingType::Pyramid;

		for ( int i = 0; i < 3; i++ )
		{
			const JPH::SixDOFConstraintSettings::EAxis eAxis =
				static_cast< JPH::SixDOFConstraintSettings::EAxis >( JPH::SixDOFConstraintSettings::EAxis::RotationX + i );

			// Inverted windows (min > max) are emitted systematically by Lua
			// contraptions -- LVS mirrors every wheel's spin socket with a
			// min/max-swapped twin -- and IVP tolerated them. Left alone their
			// negative range would fall into the tiny-window branch below and
			// hard-weld an axis the author meant to leave (nearly) free, so
			// normalize by swap and classify the sane window instead.
			const float flMin = Min( limits.lAxisLimitsRad[i].Min, limits.lAxisLimitsRad[i].Max );
			const float flMax = Max( limits.lAxisLimitsRad[i].Min, limits.lAxisLimitsRad[i].Max );
			const float flRange = flMax - flMin;

			// Per-axis friction torque as specified (kg*in^2/s^2 -> N*m). No
			// vjolt_ragdoll_min_torque_friction floor here: that floor steadies
			// ragdoll joints, but on the free axis of a mechanical joint (a
			// vehicle wheel's spin axis) it would act as a permanent brake.
			settings->mMaxFriction[ eAxis ] = SourceToJolt::Torque( ragdoll.axes[i].torque );

			if ( flRange <= DEG2RAD( 1.0f ) )
			{
				// Near-zero windows are IVP's "hold this alignment" idiom
				// (LVS locks wheel yaw/roll with +/-0.0001deg). IVP's limits are
				// compliant and let ground contact pull a transient-crooked
				// capture straight; MakeFixedAxis is an infinitely stiff weld
				// that locks the capture error in forever. Keep the window but
				// floor it at +/-0.5deg around its midpoint, with a small
				// friction floor so the slack doesn't rattle.
				const float flCenter = 0.5f * ( flMin + flMax );
				const float flHalfRange = Max( 0.5f * flRange, DEG2RAD( 0.5f ) );
				settings->SetLimitedAxis( eAxis,
					Max( flCenter - flHalfRange, -JPH::JPH_PI ),
					Min( flCenter + flHalfRange, JPH::JPH_PI ) );
				settings->mMaxFriction[ eAxis ] = Max( settings->mMaxFriction[ eAxis ], flMinTorqueFriction );
				if ( i > 0 )
					m_nRotOnlyTinySwingAxisMask |= 1u << i;
			}
			else if ( flRange >= DEG2RAD( 359.0f ) )
			{
				settings->MakeFreeAxis( eAxis );
			}
			else
			{
				// Jolt's swing-twist part accepts the full [-pi, pi] on every
				// rotation axis; clamp to keep SetLimitedAxis inputs sane.
				const float flCap = DEG2RAD( 180.0f );
				settings->SetLimitedAxis( eAxis,
					Max( flMin, -flCap ),
					Min( flMax, flCap ) );
			}
		}

		pConstraint = settings->Create( *pRefBody, *pAttBody );
		if ( bArmOnlyRotTrace )
			pOnlyRotTraceSettings = settings;

		// The frames above came from Source matrices captured at the Lua call.
		// LVS/simfphys teleport wheels and steer anchors into their intended
		// orientation one tick AFTER constraining them (and a 10-wheel tank
		// stages this over many ticks), so that capture routinely encodes a
		// mid-transient pose. Schedule a one-shot re-zero of the frames onto
		// whatever relative orientation the bodies actually hold a few steps
		// from now.
		const int nRecaptureTicks = vjolt_onlyrot_recapture_ticks.GetInt();
		if ( nRecaptureTicks > 0 )
		{
			m_pRotOnlySettings = settings;
			m_nRotOnlyRecaptureTicks = nRecaptureTicks;
		}
	}
	else if ( uDOFCount == 0 )
	{
		JPH::FixedConstraintSettings settings;
		settings.mAutoDetectPoint = true;

		pConstraint = settings.Create( *pRefBody, *pAttBody );
	}
	else if ( uDOFCount == 1 )
	{
		JoltMatrixAxes eAxis = *DOFBitToAxis( uDOFMask );

		JPH::HingeConstraintSettings settings;
		settings.mPoint1 = SourceToJolt::Distance( GetColumn( constraintToWorld, MatrixAxis::Origin ) );
		settings.mPoint2 = SourceToJolt::Distance( GetColumn( constraintToWorld, MatrixAxis::Origin ) );
		settings.mHingeAxis1 = SourceToJolt::Unitless( GetColumn( constraintToWorld, eAxis ) );
		settings.mHingeAxis2 = SourceToJolt::Unitless( GetColumn( constraintToWorld, eAxis ) );
		settings.mNormalAxis1 = HingePerpendicularVector( settings.mHingeAxis1 );
		settings.mNormalAxis2 = HingePerpendicularVector( settings.mHingeAxis2 );
		settings.mLimitsMin = limits.lAxisLimitsRad[ eAxis ].Min;
		settings.mLimitsMax = limits.lAxisLimitsRad[ eAxis ].Max;
		settings.mMaxFrictionTorque = Max( flMinTorqueFriction, SourceToJolt::Torque( ragdoll.axes[ eAxis ].torque ) );
		
		pConstraint = settings.Create( *pRefBody, *pAttBody );
	}
	else
	{
		JPH::SwingTwistConstraintSettings settings;
		// Allow ~1deg either side to avoid joints glitching out.
		settings.mTwistMinAngle = Min( limits.lAxisLimitsRad[0].Min, DEG2RAD( -1.0f ) );
		settings.mTwistMaxAngle = Max( limits.lAxisLimitsRad[0].Max, DEG2RAD(  1.0f ) );
		settings.mNormalHalfConeAngle = Max( 0.5f * ( limits.lAxisLimitsRad[1].GetRange() ), DEG2RAD( 1.0f ) );
		settings.mPlaneHalfConeAngle = Max( 0.5f * ( limits.lAxisLimitsRad[2].GetRange() ), DEG2RAD( 1.0f ) );

		settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;

		settings.mPosition1 = constraintToReference.GetTranslation() - pRefBody->GetShape()->GetCenterOfMass();
		settings.mTwistAxis1 = constraintToReference.GetAxisX();
		settings.mPlaneAxis1 = constraintToReference.GetAxisY();

		settings.mPosition2 = constraintToAttached.GetTranslation() - pAttBody->GetShape()->GetCenterOfMass();
		settings.mTwistAxis2 = constraintToAttached.GetAxisX();
		settings.mPlaneAxis2 = constraintToAttached.GetAxisY();

		settings.mMaxFrictionTorque = Max( flMinTorqueFriction, SourceToJolt::Torque( ( ragdoll.axes[0].torque + ragdoll.axes[1].torque + ragdoll.axes[2].torque ) / 3.0f ) );

		pConstraint = settings.Create( *pRefBody, *pAttBody );
	}

	const bool bActive = !m_pGroup && ragdoll.constraint.isActive;

	m_pConstraint = pConstraint;
	m_pConstraint->SetEnabled( bActive );
	m_pPhysicsSystem->AddConstraint( m_pConstraint );

	if ( pOnlyRotTraceSettings )
	{
		--g_nOnlyRotTraceArmBudget;
		m_pOnlyRotTrace = std::make_unique< OnlyRotTraceLive >();
		++g_nOnlyRotTraceLive;
		OnlyRotTraceRecord &record = m_pOnlyRotTrace->record;
		record.nGeneration = g_nOnlyRotTraceGeneration;
		record.nTraceID = g_nOnlyRotTraceNextID++;
		if ( g_nOnlyRotTraceNextID == 0 )
			g_nOnlyRotTraceNextID = 1;
		record.nCreationContactTick = m_pPhysicsEnvironment->GetContactDataTick();
		if ( g_bOnlyRotTraceArmHeld )
		{
			record.bExplicitStart = 1;
			m_pOnlyRotTrace->nHoldSerial = g_nOnlyRotTraceStartSerial;
			m_pOnlyRotTrace->bWaitingForStart = 1;
			++g_nOnlyRotTraceHeldPending;
			++g_nOnlyRotTraceHeldLive;
		}
		record.bHasGroup = m_pGroup != nullptr;
		record.bSourceActive = ragdoll.constraint.isActive;
		record.bUseClockwiseRotations = ragdoll.useClockwiseRotations;
		record.nDegreesOfFreedomMask = static_cast< uint8 >( uDOFMask );

		for ( int i = 0; i < 3; ++i )
		{
			record.flLimitMinRadians[i] = limits.lAxisLimitsRad[i].Min;
			record.flLimitMaxRadians[i] = limits.lAxisLimitsRad[i].Max;
			const JPH::SixDOFConstraintSettings::EAxis eAxis =
				static_cast< JPH::SixDOFConstraintSettings::EAxis >( JPH::SixDOFConstraintSettings::EAxis::RotationX + i );
			record.flRotationMaxFriction[i] = pOnlyRotTraceSettings->mMaxFriction[eAxis];
		}

		for ( int i = 0; i < static_cast< int >( JPH::SixDOFConstraintSettings::EAxis::Num ); ++i )
		{
			const JPH::SixDOFConstraintSettings::EAxis eAxis = static_cast< JPH::SixDOFConstraintSettings::EAxis >( i );
			if ( pOnlyRotTraceSettings->IsFreeAxis( eAxis ) )
				record.nFreeAxisMask |= 1u << i;
			if ( pOnlyRotTraceSettings->IsFixedAxis( eAxis ) )
				record.nFixedAxisMask |= 1u << i;
		}

		record.vPosition1Source = OnlyRotTraceSourcePosition( pOnlyRotTraceSettings->mPosition1 );
		record.vAxisX1 = OnlyRotTraceUnitless( pOnlyRotTraceSettings->mAxisX1 );
		record.vAxisY1 = OnlyRotTraceUnitless( pOnlyRotTraceSettings->mAxisY1 );
		record.vPosition2Source = OnlyRotTraceSourcePosition( pOnlyRotTraceSettings->mPosition2 );
		record.vAxisX2 = OnlyRotTraceUnitless( pOnlyRotTraceSettings->mAxisX2 );
		record.vAxisY2 = OnlyRotTraceUnitless( pOnlyRotTraceSettings->mAxisY2 );
		record.creationBodies[0] = CaptureOnlyRotBodyState( m_pObjReference );
		record.creationBodies[1] = CaptureOnlyRotBodyState( m_pObjAttached );
		record.creationConstraint = CaptureOnlyRotConstraintState( m_pConstraint );

		++g_nOnlyRotTracePending;
		m_pPhysicsEnvironment->RegisterOnlyRotTraceConstraint( this );
	}
}

//-------------------------------------------------------------------------------------------------
// Hinge
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseHinge( IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_HINGE;
	SetBreakableParams( hinge.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::HingeConstraintSettings settings;
	settings.mPoint1 = SourceToJolt::Distance( hinge.worldPosition );
	settings.mPoint2 = SourceToJolt::Distance( hinge.worldPosition );

	settings.mHingeAxis1 = JPH::Vec3( hinge.worldAxisDirection.x, hinge.worldAxisDirection.y, hinge.worldAxisDirection.z );
	settings.mHingeAxis2 = JPH::Vec3( hinge.worldAxisDirection.x, hinge.worldAxisDirection.y, hinge.worldAxisDirection.z );

	settings.mNormalAxis1 = HingePerpendicularVector( settings.mHingeAxis1 );
	settings.mNormalAxis2 = HingePerpendicularVector( settings.mHingeAxis2 );

	if ( hinge.hingeAxis.minRotation != hinge.hingeAxis.maxRotation )
	{
		settings.mLimitsMin = DEG2RAD( -hinge.hingeAxis.maxRotation );
		settings.mLimitsMax = DEG2RAD( -hinge.hingeAxis.minRotation );
	}

	// Source torque is kg*in^2/s^2; Jolt expects N*m = kg*m^2/s^2. Convert with squared distance factor.
	settings.mMaxFrictionTorque = SourceToJolt::Torque( hinge.hingeAxis.torque );

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && hinge.constraint.isActive );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Sliding
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseSliding( IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_SLIDING;
	SetBreakableParams( sliding.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::SliderConstraintSettings settings;
	settings.mAutoDetectPoint = true;
	settings.SetSliderAxis( JPH::Vec3( sliding.slideAxisRef.x, sliding.slideAxisRef.y, sliding.slideAxisRef.z ) );

	if ( sliding.limitMin != sliding.limitMax )
	{
		settings.mLimitsMin = SourceToJolt::Distance( sliding.limitMin );
		settings.mLimitsMax = SourceToJolt::Distance( sliding.limitMax );
	}

	settings.mMaxFrictionForce = sliding.friction;

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && sliding.constraint.isActive );

	if ( sliding.velocity )
	{
		JPH::SliderConstraint *pConstraint = static_cast<JPH::SliderConstraint *>( m_pConstraint.GetPtr() );
		pConstraint->SetMotorState( JPH::EMotorState::Velocity );
		pConstraint->SetTargetVelocity( SourceToJolt::Distance( sliding.velocity ) );
	}

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Ballsocket
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseBallsocket( IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_BALLSOCKET;
	SetBreakableParams( ballsocket.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::PointConstraintSettings settings;
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	settings.mPoint1 = SourceToJolt::Distance( ballsocket.constraintPosition[0] ) - refBody->GetShape()->GetCenterOfMass();
	settings.mPoint2 = SourceToJolt::Distance( ballsocket.constraintPosition[1] ) - attBody->GetShape()->GetCenterOfMass();

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && ballsocket.constraint.isActive );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Fixed
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseFixed( IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_FIXED;
	SetBreakableParams( fixed.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::FixedConstraintSettings settings;
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
	settings.mAutoDetectPoint = true;

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && fixed.constraint.isActive );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Length
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseLength( IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_LENGTH;
	SetBreakableParams( length.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::DistanceConstraintSettings settings;
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	settings.mPoint1 = SourceToJolt::Distance( length.objectPosition[0] ) - refBody->GetShape()->GetCenterOfMass();
	settings.mPoint2 = SourceToJolt::Distance( length.objectPosition[1] ) - attBody->GetShape()->GetCenterOfMass();

	settings.mMinDistance = SourceToJolt::Distance( length.minLength );
	settings.mMaxDistance = SourceToJolt::Distance( length.totalLength );

	// Josh: UNDONE! Nothing seems to use strength on length ever
	// after analysing the codebase.
	//
	//settings.mFrequency = 1.0f - length.constraint.strength;
	//if ( settings.mFrequency )
	//	settings.mDamping = 1.0f;

	// Optional compliance for the distance limits (IVP length constraints are
	// springs at heart). Both paths default to rigid limits -- these are
	// diagnostic knobs for contraptions that misbehave under hard ropes; the
	// warmup variant hardens in PostSimulate.
	const int nWarmupTicks = vjolt_length_spring_warmup_ticks.GetInt();
	const float flWarmupFrequency = vjolt_length_spring_warmup_frequency.GetFloat();
	if ( nWarmupTicks > 0 && flWarmupFrequency > 0.0f )
	{
		settings.mLimitsSpringSettings.mFrequency = flWarmupFrequency;
		settings.mLimitsSpringSettings.mDamping = vjolt_length_spring_warmup_damping.GetFloat();
		m_nLengthSpringWarmupTicks = nWarmupTicks;
	}
	else
	{
		settings.mLimitsSpringSettings.mFrequency = vjolt_length_spring_frequency.GetFloat();
		settings.mLimitsSpringSettings.mDamping = vjolt_length_spring_damping.GetFloat();
	}

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && length.constraint.isActive );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Pulley
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialisePulley( IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_PULLEY;
	SetBreakableParams( pulley.constraint );

	// Get our bodies
	JPH::Body* refBody = m_pObjReference->GetBody();
	JPH::Body* attBody = m_pObjAttached->GetBody();

	JPH::PulleyConstraintSettings settings;
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	settings.mBodyPoint1 = SourceToJolt::Distance( pulley.objectPosition[0] ) - refBody->GetShape()->GetCenterOfMass();
	settings.mBodyPoint2 = SourceToJolt::Distance( pulley.objectPosition[1] ) - attBody->GetShape()->GetCenterOfMass();

	settings.mFixedPoint1 = SourceToJolt::Distance( pulley.pulleyPosition[0] );
	settings.mFixedPoint2 = SourceToJolt::Distance( pulley.pulleyPosition[1] );

	settings.mRatio = pulley.gearRatio;

	settings.mMaxLength = SourceToJolt::Distance( pulley.totalLength ); // PiMoN: from my testing, it is the same value as Jolt would calculate automatically

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && pulley.constraint.isActive );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------

static void GetConstraintImpulses( const JPH::Constraint *pConstraint, float &outLinear, float &outAngular )
{
	outLinear = 0.0f;
	outAngular = 0.0f;

	switch ( pConstraint->GetSubType() )
	{
		case JPH::EConstraintSubType::Fixed:
		{
			auto *p = static_cast< const JPH::FixedConstraint * >( pConstraint );
			outLinear = p->GetTotalLambdaPosition().Length();
			outAngular = p->GetTotalLambdaRotation().Length();
			break;
		}
		case JPH::EConstraintSubType::Point:
		{
			auto *p = static_cast< const JPH::PointConstraint * >( pConstraint );
			outLinear = p->GetTotalLambdaPosition().Length();
			break;
		}
		case JPH::EConstraintSubType::Hinge:
		{
			auto *p = static_cast< const JPH::HingeConstraint * >( pConstraint );
			outLinear = p->GetTotalLambdaPosition().Length();
			outAngular = Max( p->GetTotalLambdaRotation().Length(), fabsf( p->GetTotalLambdaRotationLimits() ) );
			break;
		}
		case JPH::EConstraintSubType::Slider:
		{
			auto *p = static_cast< const JPH::SliderConstraint * >( pConstraint );
			outLinear = Max( p->GetTotalLambdaPosition().Length(), fabsf( p->GetTotalLambdaPositionLimits() ) );
			outAngular = p->GetTotalLambdaRotation().Length();
			break;
		}
		case JPH::EConstraintSubType::Distance:
		{
			auto *p = static_cast< const JPH::DistanceConstraint * >( pConstraint );
			outLinear = fabsf( p->GetTotalLambdaPosition() );
			break;
		}
		case JPH::EConstraintSubType::SwingTwist:
		{
			auto *p = static_cast< const JPH::SwingTwistConstraint * >( pConstraint );
			outLinear = p->GetTotalLambdaPosition().Length();
			const float flTwist  = p->GetTotalLambdaTwist();
			const float flSwingY = p->GetTotalLambdaSwingY();
			const float flSwingZ = p->GetTotalLambdaSwingZ();
			outAngular = sqrtf( flTwist * flTwist + flSwingY * flSwingY + flSwingZ * flSwingZ );
			break;
		}
		case JPH::EConstraintSubType::Pulley:
		{
			auto *p = static_cast< const JPH::PulleyConstraint * >( pConstraint );
			outLinear = fabsf( p->GetTotalLambdaPosition() );
			break;
		}
		case JPH::EConstraintSubType::SixDOF:
		{
			// Rotation-only ragdoll constraints (onlyAngularLimits). Free axes
			// have inactive constraint parts and report zero lambda, so the
			// (free) translation part correctly contributes nothing.
			auto *p = static_cast< const JPH::SixDOFConstraint * >( pConstraint );
			outLinear = p->GetTotalLambdaPosition().Length();
			outAngular = p->GetTotalLambdaRotation().Length();
			break;
		}
		default:
			break;
	}
}

static float MaxInverseMass( JoltPhysicsObject *pA, JoltPhysicsObject *pB )
{
	auto invMassOf = []( JoltPhysicsObject *pObj ) -> float
	{
		if ( !pObj )
			return 0.0f;
		JPH::Body *pBody = pObj->GetBody();
		if ( !pBody || pBody->IsStatic() )
			return 0.0f;
		JPH::MotionProperties *pMP = pBody->GetMotionProperties();
		return pMP ? pMP->GetInverseMass() : 0.0f;
	};
	return Max( invMassOf( pA ), invMassOf( pB ) );
}

void JoltPhysicsConstraint::DiscardStaleOnlyRotTrace()
{
	if ( m_pOnlyRotTrace && m_pOnlyRotTrace->record.nGeneration != g_nOnlyRotTraceGeneration )
		ReleaseOnlyRotTraceLive( m_pOnlyRotTrace );
}

void JoltPhysicsConstraint::FinishOnlyRotTrace( uint8 nCompletion )
{
	if ( !m_pOnlyRotTrace )
		return;

	if ( m_pOnlyRotTrace->record.nGeneration != g_nOnlyRotTraceGeneration )
	{
		ReleaseOnlyRotTraceLive( m_pOnlyRotTrace );
		return;
	}

	OnlyRotTraceRecord &record = m_pOnlyRotTrace->record;
	record.nCompletion = nCompletion;
	record.nPreUpdates = m_pOnlyRotTrace->nPreUpdates;
	record.nPostUpdates = m_pOnlyRotTrace->nPostUpdates;

	if ( g_nOnlyRotTraceCompleted < ONLYROT_TRACE_MAX_RECORDS )
		g_OnlyRotTraceCompleted[g_nOnlyRotTraceCompleted++] = record;
	else
		++g_nOnlyRotTraceDropped;

	if ( g_nOnlyRotTracePending > 0 )
		--g_nOnlyRotTracePending;
	ReleaseOnlyRotTraceLive( m_pOnlyRotTrace );
}

bool JoltPhysicsConstraint::TraceOnlyRotPreSimulate()
{
	DiscardStaleOnlyRotTrace();
	if ( !m_pOnlyRotTrace )
		return false;

	if ( !m_pConstraint || !m_pObjReference || !m_pObjAttached )
	{
		FinishOnlyRotTrace( ONLYROT_TRACE_DESTROYED );
		return false;
	}
	if ( m_pOnlyRotTrace->bWaitingForStart )
	{
		if ( m_pOnlyRotTrace->nHoldSerial == g_nOnlyRotTraceStartSerial )
		{
			++m_pOnlyRotTrace->record.nHeldPreSimCalls;
			return true;
		}

		ReleaseOnlyRotTraceHold( *m_pOnlyRotTrace );
	}
	if ( !m_pOnlyRotTrace->bStarted )
	{
		m_pOnlyRotTrace->record.nStartContactTick = m_pPhysicsEnvironment->GetContactDataTick();
		m_pOnlyRotTrace->bStarted = 1;
	}

	if ( m_pOnlyRotTrace->nPreUpdates >= ONLYROT_TRACE_UPDATE_COUNT )
		return false;

	OnlyRotTraceUpdate &update = m_pOnlyRotTrace->record.updates[m_pOnlyRotTrace->nPreUpdates];
	update.bPreCaptured = 1;
	update.nPreContactTick = m_pPhysicsEnvironment->GetContactDataTick();
	update.preBodies[0] = CaptureOnlyRotBodyState( m_pObjReference );
	update.preBodies[1] = CaptureOnlyRotBodyState( m_pObjAttached );
	update.preConstraint = CaptureOnlyRotConstraintState( m_pConstraint );
	if ( m_pConstraint->GetSubType() == JPH::EConstraintSubType::SixDOF )
	{
		const JPH::SixDOFConstraint *pSixDOF = static_cast< const JPH::SixDOFConstraint * >( m_pConstraint.GetPtr() );
		update.vPreviousLambdaPosition = OnlyRotTraceUnitless( pSixDOF->GetTotalLambdaPosition() );
		update.vPreviousLambdaRotation = OnlyRotTraceUnitless( pSixDOF->GetTotalLambdaRotation() );
		update.vPreviousLambdaMotorRotation = OnlyRotTraceUnitless( pSixDOF->GetTotalLambdaMotorRotation() );
	}
	++m_pOnlyRotTrace->nPreUpdates;
	return m_pOnlyRotTrace->nPreUpdates < ONLYROT_TRACE_UPDATE_COUNT;
}

void JoltPhysicsConstraint::TraceOnlyRotPostBegin()
{
	DiscardStaleOnlyRotTrace();
	if ( !m_pOnlyRotTrace )
		return;
	if ( m_pOnlyRotTrace->bWaitingForStart )
		return;

	if ( !m_pConstraint || !m_pObjReference || !m_pObjAttached )
	{
		FinishOnlyRotTrace( ONLYROT_TRACE_DESTROYED );
		return;
	}

	if ( m_pOnlyRotTrace->nPostUpdates >= ONLYROT_TRACE_UPDATE_COUNT )
		return;

	const uint32 nUpdate = m_pOnlyRotTrace->nPostUpdates;
	OnlyRotTraceUpdate &update = m_pOnlyRotTrace->record.updates[nUpdate];
	update.bPostCaptured = 1;
	update.nPostContactTick = m_pPhysicsEnvironment->GetContactDataTick();
	if ( !update.bPreCaptured || update.nPostContactTick - update.nPreContactTick != 1 )
	{
		update.bTickDiscontinuity = 1;
		m_pOnlyRotTrace->record.bAnyTickDiscontinuity = 1;
	}

	update.postBodies[0] = CaptureOnlyRotBodyState( m_pObjReference );
	update.postBodies[1] = CaptureOnlyRotBodyState( m_pObjAttached );
	update.postConstraint = CaptureOnlyRotConstraintState( m_pConstraint );
	const int nContactSampleIndex = OnlyRotTraceContactSampleForUpdate( nUpdate );
	if ( nContactSampleIndex >= 0 )
	{
		OnlyRotTraceContactSample &sample = m_pOnlyRotTrace->record.contactSamples[nContactSampleIndex];
		sample.bCaptured = 1;
		sample.nUpdate = static_cast< uint8 >( nUpdate + 1 );
		CaptureOnlyRotContacts( m_pObjReference, sample.endpoints[0] );
		CaptureOnlyRotContacts( m_pObjAttached, sample.endpoints[1] );
	}

	if ( m_pConstraint->GetSubType() == JPH::EConstraintSubType::SixDOF )
	{
		const JPH::SixDOFConstraint *pSixDOF = static_cast< const JPH::SixDOFConstraint * >( m_pConstraint.GetPtr() );
		update.vLambdaPosition = OnlyRotTraceUnitless( pSixDOF->GetTotalLambdaPosition() );
		update.vLambdaRotation = OnlyRotTraceUnitless( pSixDOF->GetTotalLambdaRotation() );
		update.vLambdaMotorRotation = OnlyRotTraceUnitless( pSixDOF->GetTotalLambdaMotorRotation() );
	}

	++m_pOnlyRotTrace->nPostUpdates;
}

void JoltPhysicsConstraint::TraceOnlyRotPostEnd()
{
	if ( m_pOnlyRotTrace && !m_pOnlyRotTrace->bWaitingForStart
		&& m_pOnlyRotTrace->nPostUpdates >= ONLYROT_TRACE_UPDATE_COUNT )
		FinishOnlyRotTrace( ONLYROT_TRACE_COMPLETE );
}

void JoltPhysicsConstraint::PostSimulate()
{
	// Keep the normal path to one predictable null branch. For an armed trace,
	// Snapshot each update before the legacy c356 recapture can replace the
	// second-update constraint with a new, not-yet-solved SixDOF. Run post-end
	// only after any rebuild event has been recorded into the eventual record.
	if ( m_pOnlyRotTrace )
	{
		TraceOnlyRotPostBegin();
		RecaptureRotOnlyFrames();
		TraceOnlyRotPostEnd();
	}
	else
	{
		RecaptureRotOnlyFrames();
	}
	HardenLengthSpring();
	CheckBroken();
}

void JoltPhysicsConstraint::HardenLengthSpring()
{
	if ( m_nLengthSpringWarmupTicks <= 0 )
		return;

	if ( --m_nLengthSpringWarmupTicks > 0 )
		return;

	if ( !m_pConstraint || m_ConstraintType != CONSTRAINT_LENGTH )
		return;

	JPH::SpringSettings steady;
	steady.mFrequency = vjolt_length_spring_frequency.GetFloat();
	steady.mDamping = vjolt_length_spring_damping.GetFloat();

	static_cast< JPH::DistanceConstraint * >( m_pConstraint.GetPtr() )->SetLimitsSpringSettings( steady );
}

void JoltPhysicsConstraint::RecaptureRotOnlyFrames()
{
	if ( m_nRotOnlyRecaptureTicks <= 0 )
		return;

	OnlyRotTraceRecapture *pTraceRecapture = nullptr;
	if ( m_pOnlyRotTrace && m_pOnlyRotTrace->record.nGeneration == g_nOnlyRotTraceGeneration )
	{
		pTraceRecapture = &m_pOnlyRotTrace->record.recapture;
		++pTraceRecapture->nCalls;
		pTraceRecapture->nLastTick = m_pPhysicsEnvironment->GetContactDataTick();
		pTraceRecapture->nLastCountdownBefore = m_nRotOnlyRecaptureTicks;
	}

	if ( --m_nRotOnlyRecaptureTicks > 0 )
		return;

	JPH::Ref< JPH::SixDOFConstraintSettings > settings = std::move( m_pRotOnlySettings );

	if ( !settings || !m_pConstraint || !m_pObjReference || !m_pObjAttached )
		return;

	JPH::Body *pRefBody = m_pObjReference->GetBody();
	JPH::Body *pAttBody = m_pObjAttached->GetBody();

	// Re-express the attached body's constraint frame in reference-body local
	// space at the orientations the bodies hold RIGHT NOW, so the pose they have
	// actually settled into (after LVS's constrain-then-teleport init) becomes
	// the joint's rest pose instead of whatever the mid-transient capture was.
	const JPH::Quat qRefToAtt = pRefBody->GetRotation().Conjugated() * pAttBody->GetRotation();
	if ( pTraceRecapture )
	{
		pTraceRecapture->bRebuilt = 1;
		pTraceRecapture->qReferenceToAttached = OnlyRotTraceQuaternion( qRefToAtt );
		pTraceRecapture->vOldAxisX1 = OnlyRotTraceUnitless( settings->mAxisX1 );
		pTraceRecapture->vOldAxisY1 = OnlyRotTraceUnitless( settings->mAxisY1 );
	}
	settings->mAxisX1 = qRefToAtt * settings->mAxisX2;
	settings->mAxisY1 = qRefToAtt * settings->mAxisY2;

	const bool bMotorCanonical = pRefBody->GetMotionType() == JPH::EMotionType::Dynamic
		&& pAttBody->GetMotionType() == JPH::EMotionType::Static;
	const float flMotorFrequency = vjolt_onlyrot_tiny_axis_motor_frequency.GetFloat();
	const float flMotorDamping = vjolt_onlyrot_tiny_axis_motor_damping.GetFloat();
	uint8 nMotorRotationAxisMask = 0;
	if ( bMotorCanonical && m_nRotOnlyTinySwingAxisMask != 0
		&& std::isfinite( flMotorFrequency ) && flMotorFrequency > 0.0f
		&& std::isfinite( flMotorDamping ) && flMotorDamping >= 0.0f )
	{
		nMotorRotationAxisMask = m_nRotOnlyTinySwingAxisMask;
		for ( int i = 0; i < 3; ++i )
		{
			if ( ( nMotorRotationAxisMask & ( 1u << i ) ) == 0 )
				continue;
			const JPH::SixDOFConstraintSettings::EAxis eAxis =
				static_cast< JPH::SixDOFConstraintSettings::EAxis >( JPH::SixDOFConstraintSettings::EAxis::RotationX + i );
			settings->mMotorSettings[ eAxis ] = JPH::MotorSettings( flMotorFrequency, flMotorDamping );
		}
	}
	if ( pTraceRecapture )
	{
		pTraceRecapture->vNewAxisX1 = OnlyRotTraceUnitless( settings->mAxisX1 );
		pTraceRecapture->vNewAxisY1 = OnlyRotTraceUnitless( settings->mAxisY1 );
		pTraceRecapture->bMotorCanonical = bMotorCanonical;
		pTraceRecapture->nMotorRotationAxisMask = nMotorRotationAxisMask;
		pTraceRecapture->flMotorFrequency = flMotorFrequency;
		pTraceRecapture->flMotorDamping = flMotorDamping;
	}

	const bool bEnabled = m_pConstraint->GetEnabled();
	if ( pTraceRecapture )
		pTraceRecapture->bOldEnabled = bEnabled;
	m_pPhysicsSystem->RemoveConstraint( m_pConstraint );
	m_pConstraint = settings->Create( *pRefBody, *pAttBody );
	m_pConstraint->SetEnabled( bEnabled );
	if ( nMotorRotationAxisMask != 0 )
	{
		JPH::SixDOFConstraint *pSixDOF = static_cast< JPH::SixDOFConstraint * >( m_pConstraint.GetPtr() );
		pSixDOF->SetTargetOrientationCS( JPH::Quat::sIdentity() );
		for ( int i = 0; i < 3; ++i )
		{
			if ( ( nMotorRotationAxisMask & ( 1u << i ) ) == 0 )
				continue;
			const JPH::SixDOFConstraint::EAxis eAxis = static_cast< JPH::SixDOFConstraint::EAxis >(
				JPH::SixDOFConstraint::EAxis::RotationX + i );
			pSixDOF->SetMotorState( eAxis, JPH::EMotorState::Position );
		}
	}
	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

bool JoltPhysicsConstraint::CheckBroken()
{
	if ( !m_pConstraint || !m_pConstraint->GetEnabled() )
		return false;

	if ( m_LinearBreakImpulse <= 0.0f && m_AngularBreakImpulse <= 0.0f )
		return false;

	float flLinear = 0.0f;
	float flAngular = 0.0f;
	GetConstraintImpulses( m_pConstraint.GetPtr(), flLinear, flAngular );

	const int nIterations = Max( 1u, m_pPhysicsSystem->GetPhysicsSettings().mNumVelocitySteps );
	const float flLinearLimit = m_LinearBreakImpulse * float( nIterations );
	const float flAngularLimit = m_AngularBreakImpulse * float( nIterations );

	bool bLinearBreak = false;
	if ( m_LinearBreakImpulse > 0.0f && flLinear > 0.0f )
	{
		const float flInvMassMax = MaxInverseMass( m_pObjReference, m_pObjAttached );
		bLinearBreak = flLinear * flLinear * flInvMassMax > flLinearLimit * flLinearLimit;
	}

	const bool bAngularBreak = m_AngularBreakImpulse > 0.0f && flAngular > flAngularLimit;

	if ( !bLinearBreak && !bAngularBreak )
		return false;

	if ( vjolt_constraint_break_debug.GetBool() )
	{
		const float flInvMassMax = MaxInverseMass( m_pObjReference, m_pObjAttached );
		Log_Msg( LOG_VJolt,
			"Constraint break: type=%d sub=%d linear=%.3f (limit=%.3f, src=%.3f) angular=%.3f (limit=%.3f, src=%.3f) iters=%d invMassMax=%.5f reasons=%s%s\n",
			int( m_ConstraintType ), int( m_pConstraint->GetSubType() ),
			flLinear, flLinearLimit, m_SourceForceLimit,
			flAngular, flAngularLimit, m_SourceTorqueLimit,
			nIterations, flInvMassMax,
			bLinearBreak ? "linear " : "", bAngularBreak ? "angular" : "" );
	}

	m_pConstraint->SetEnabled( false );
	m_pPhysicsEnvironment->NotifyConstraintDisabled( this );
	return true;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SaveConstraintSettings( JPH::StateRecorder &recorder )
{
	recorder.Write( m_ConstraintType );
	auto settings = m_pConstraint->GetConstraintSettings();
	settings->SaveBinaryState( recorder );
	m_pConstraint->SaveState( recorder );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetGroup( IPhysicsConstraintGroup *pGroup )
{
	if ( m_pGroup )
		m_pGroup->RemoveConstraint( this );
	m_pGroup = static_cast< JoltPhysicsConstraintGroup * >( pGroup );
	if ( m_pGroup )
		m_pGroup->AddConstraint( this );
}

void JoltPhysicsConstraint::DestroyConstraint()
{
	if ( m_pOnlyRotTrace )
		FinishOnlyRotTrace( ONLYROT_TRACE_DESTROYED );

	if ( m_pObjAttached )
	{
		m_pObjAttached->RemoveDestroyedListener( this );
		m_pObjAttached = nullptr;
	}
	if ( m_pObjReference )
	{
		m_pObjReference->RemoveDestroyedListener( this );
		m_pObjReference = nullptr;
	}

	m_pRotOnlySettings = nullptr;
	m_nRotOnlyRecaptureTicks = 0;
	m_nRotOnlyTinySwingAxisMask = 0;
	m_nLengthSpringWarmupTicks = 0;

	if ( m_pConstraint )
	{
		m_pPhysicsSystem->RemoveConstraint( m_pConstraint );
		m_pConstraint = nullptr;
	}
}
