//=================================================================================================
//
// Constraints
//
//=================================================================================================

#include "cbase.h"

#include <cmath>
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

static ConVar vjolt_onlyrot_tiny_axis_motor_frequency( "vjolt_onlyrot_tiny_axis_motor_frequency", "10", FCVAR_NONE,
	"Position-motor frequency (Hz) for canonical dynamic-to-static rotation-only wheel joints with "
	"a free spin axis and near-zero Y/Z windows. 0 disables the motor.",
	true, 0.0f, true, 20.0f );
static ConVar vjolt_onlyrot_tiny_axis_motor_damping( "vjolt_onlyrot_tiny_axis_motor_damping", "1", FCVAR_NONE,
	"Damping ratio for the canonical rotation-only wheel alignment motor.",
	true, 0.0f, true, 4.0f );
static ConVar vjolt_onlyrot_tiny_axis_position_steps( "vjolt_onlyrot_tiny_axis_position_steps", "4", FCVAR_NONE,
	"Position iterations requested by canonical rotation-only wheel constraints. Jolt applies the "
	"maximum constraint override to the connected island, so this also stabilizes the wheel's "
	"suspension graph without raising the global position-step count. 0 disables the override.",
	true, 0.0f, true, 64.0f );

// Diagnostic knob, default off: hardening mid-settle freezes whatever pose the spawn
// transient left (live trials: 30 ticks @ 8 Hz made LVS tank tilt WORSE, 6/6 vs 7/12
// baseline). The actual fix for transient-captured contraptions is the gentler
// vjolt_baumgarte_factor default; see vjolt_environment.cpp.
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

// Contact depenetration needs to remain gentle for newly spawned multi-body contraptions, but
// vjolt_baumgarte_factor is also Jolt's correction rate for authored joints. With the production
// 0.01 factor, a sleeping hard rope can retain several Source units of drive-induced error. Raise
// only the affected island's position iterations until the rope is back inside its authored limit.
static ConVar vjolt_hard_distance_recovery_tolerance( "vjolt_hard_distance_recovery_tolerance", "1", FCVAR_NONE,
	"Hard distance-constraint error (Source units) that activates a temporary per-island position "
	"solver override. 0 disables recovery.",
	true, 0.0f, true, 64.0f );
static ConVar vjolt_hard_distance_recovery_position_steps( "vjolt_hard_distance_recovery_position_steps", "20", FCVAR_NONE,
	"Position iterations used while a hard distance constraint is outside its authored limit. "
	"The override is removed as soon as the constraint recovers. 0 disables recovery.",
	true, 0.0f, true, 64.0f );

static constexpr float UNBREAKABLE_BREAK_LIMIT = 1e12f;

static ConVar vjolt_constraint_break_debug( "vjolt_constraint_break_debug", "0", FCVAR_NONE,
	"Log lambda + threshold values whenever a breakable constraint trips." );

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
	m_pObjReference->AddConstraint( this );
	m_pObjAttached->AddConstraint( this );
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

	if ( ragdoll.onlyAngularLimits )
	{
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
		bool bFreeSpinAxis = false;
		uint8 nTinySwingAxisMask = 0;

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
					nTinySwingAxisMask |= 1u << i;
			}
			else if ( flRange >= DEG2RAD( 359.0f ) )
			{
				settings->MakeFreeAxis( eAxis );
				if ( i == 0 )
					bFreeSpinAxis = true;
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

		constexpr uint8 nBothSwingAxes = ( 1u << 1 ) | ( 1u << 2 );
		const bool bCanonicalWheelConstraint = bFreeSpinAxis && nTinySwingAxisMask == nBothSwingAxes;
		if ( bCanonicalWheelConstraint )
		{
			// Two global position iterations are not consistently enough for the
			// coupled wheel/socket/suspension graph. When that island fails to
			// converge, the Lua state is already correct (unlocked, restored mass,
			// grounded) but the wheel either spins without translating the chassis
			// or remains bound. Put the override on the narrowly identified wheel
			// socket; Jolt propagates the maximum to its connected island.
			const uint nPositionSteps = static_cast< uint >( vjolt_onlyrot_tiny_axis_position_steps.GetInt() );
			settings->mNumPositionStepsOverride = Max( settings->mNumPositionStepsOverride, nPositionSteps );
		}

		pConstraint = settings->Create( *pRefBody, *pAttBody );

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

			// Remember the characteristic wheel layout here. LVS initially pins
			// both bodies and enables the wheel one tick later, so body ordering
			// can only be classified reliably when the deferred recapture runs.
			if ( bCanonicalWheelConstraint )
				m_nRotOnlyTinySwingAxisMask = nTinySwingAxisMask;
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

void JoltPhysicsConstraint::PostSimulate()
{
	RecaptureRotOnlyFrames();
	HardenLengthSpring();
	UpdateHardDistanceRecovery();
	CheckBroken();
}

bool JoltPhysicsConstraint::HasHardDistanceErrorGreaterThan( float flTolerance ) const
{
	if ( !m_pConstraint || !m_pConstraint->GetEnabled()
		|| m_pConstraint->GetSubType() != JPH::EConstraintSubType::Distance )
		return false;

	const JPH::DistanceConstraint *pDistance =
		static_cast< const JPH::DistanceConstraint * >( m_pConstraint.GetPtr() );
	if ( pDistance->GetLimitsSpringSettings().HasStiffness() )
		return false;

	const JPH::Body *pBody1 = pDistance->GetBody1();
	const JPH::Body *pBody2 = pDistance->GetBody2();
	if ( !pBody1 || !pBody2 )
		return false;

	const JPH::Vec3 vLocal1 = pDistance->GetConstraintToBody1Matrix().GetTranslation();
	const JPH::Vec3 vLocal2 = pDistance->GetConstraintToBody2Matrix().GetTranslation();
	const JPH::RVec3 vWorld1 = pBody1->GetCenterOfMassPosition() + pBody1->GetRotation() * vLocal1;
	const JPH::RVec3 vWorld2 = pBody2->GetCenterOfMassPosition() + pBody2->GetRotation() * vLocal2;
	const float flDistance = JPH::Vec3( vWorld2 - vWorld1 ).Length();

	return flDistance < pDistance->GetMinDistance() - flTolerance
		|| flDistance > pDistance->GetMaxDistance() + flTolerance;
}

void JoltPhysicsConstraint::RequestSolverBoost( const JoltPhysicsObject *pRequester, uint nVelocitySteps, uint nPositionSteps )
{
	if ( !m_pConstraint || !pRequester )
		return;

	SolverBoostRequest *pRequest = nullptr;
	for ( SolverBoostRequest &request : m_SolverBoostRequests )
	{
		if ( request.m_pRequester == pRequester )
		{
			pRequest = &request;
			break;
		}
	}

	if ( !pRequest )
	{
		SaveSolverBoostBase();
		m_SolverBoostRequests.emplace_back();
		pRequest = &m_SolverBoostRequests.back();
		pRequest->m_pRequester = pRequester;
	}

	pRequest->m_nVelocitySteps = static_cast< uint8 >( Min( nVelocitySteps, 255u ) );
	pRequest->m_nPositionSteps = static_cast< uint8 >( Min( nPositionSteps, 255u ) );
	ApplySolverBoostRequests();
}

void JoltPhysicsConstraint::ReleaseSolverBoost( const JoltPhysicsObject *pRequester )
{
	for ( auto it = m_SolverBoostRequests.begin(); it != m_SolverBoostRequests.end(); ++it )
	{
		if ( it->m_pRequester != pRequester )
			continue;

		m_SolverBoostRequests.erase( it );
		ApplySolverBoostRequests();
		return;
	}
}

void JoltPhysicsConstraint::ApplySolverBoostRequests()
{
	if ( !m_pConstraint || !m_bSolverBoostBaseSaved )
		return;

	uint nVelocitySteps = m_nSavedVelocityStepsOverride;
	uint nPositionSteps = m_nSavedPositionStepsOverride;
	for ( const SolverBoostRequest &request : m_SolverBoostRequests )
	{
		nVelocitySteps = Max( nVelocitySteps, static_cast< uint >( request.m_nVelocitySteps ) );
		nPositionSteps = Max( nPositionSteps, static_cast< uint >( request.m_nPositionSteps ) );
	}
	if ( m_bHardDistanceRecoveryActive )
	{
		nPositionSteps = Max( nPositionSteps,
			static_cast< uint >( vjolt_hard_distance_recovery_position_steps.GetInt() ) );
	}

	m_pConstraint->SetNumVelocityStepsOverride( nVelocitySteps );
	m_pConstraint->SetNumPositionStepsOverride( nPositionSteps );

	if ( m_SolverBoostRequests.empty() && !m_bHardDistanceRecoveryActive )
		m_bSolverBoostBaseSaved = false;
}

void JoltPhysicsConstraint::SaveSolverBoostBase()
{
	if ( !m_pConstraint || m_bSolverBoostBaseSaved )
		return;

	m_nSavedVelocityStepsOverride = static_cast< uint8 >( m_pConstraint->GetNumVelocityStepsOverride() );
	m_nSavedPositionStepsOverride = static_cast< uint8 >( m_pConstraint->GetNumPositionStepsOverride() );
	m_bSolverBoostBaseSaved = true;
}

void JoltPhysicsConstraint::UpdateHardDistanceRecovery()
{
	const float flToleranceSource = vjolt_hard_distance_recovery_tolerance.GetFloat();
	const int nPositionSteps = vjolt_hard_distance_recovery_position_steps.GetInt();
	const bool bNeedsRecovery = flToleranceSource > 0.0f && nPositionSteps > 0
		&& HasHardDistanceErrorGreaterThan( SourceToJolt::Distance( flToleranceSource ) );

	if ( bNeedsRecovery != m_bHardDistanceRecoveryActive )
	{
		if ( bNeedsRecovery )
			SaveSolverBoostBase();

		m_bHardDistanceRecoveryActive = bNeedsRecovery;
		ApplySolverBoostRequests();
	}

	if ( !m_bHardDistanceRecoveryActive )
		return;

	// An out-of-tolerance island may already have gone to sleep, in which case a
	// higher iteration count alone is inert. Keep only its dynamic endpoints awake
	// until the authored hard limit is satisfied again.
	if ( m_pObjReference && m_pObjReference->IsMoveable() )
		m_pObjReference->Wake();
	if ( m_pObjAttached && m_pObjAttached->IsMoveable() )
		m_pObjAttached->Wake();
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
	settings->mAxisX1 = qRefToAtt * settings->mAxisX2;
	settings->mAxisY1 = qRefToAtt * settings->mAxisY2;

	const float flMotorFrequency = vjolt_onlyrot_tiny_axis_motor_frequency.GetFloat();
	const float flMotorDamping = vjolt_onlyrot_tiny_axis_motor_damping.GetFloat();
	// LVS mirrors each socket in both body orders. Select only the now-dynamic
	// wheel -> still-static master half so one side actively follows the target
	// while the reverse mirror remains a passive limit.
	const bool bEnableTinyAxisMotor = m_nRotOnlyTinySwingAxisMask != 0
		&& pRefBody->GetMotionType() == JPH::EMotionType::Dynamic
		&& pAttBody->GetMotionType() == JPH::EMotionType::Static
		&& std::isfinite( flMotorFrequency ) && flMotorFrequency > 0.0f
		&& std::isfinite( flMotorDamping ) && flMotorDamping >= 0.0f;
	if ( bEnableTinyAxisMotor )
	{
		for ( int i = 0; i < 3; ++i )
		{
			if ( ( m_nRotOnlyTinySwingAxisMask & ( 1u << i ) ) == 0 )
				continue;

			const JPH::SixDOFConstraintSettings::EAxis eAxis =
				static_cast< JPH::SixDOFConstraintSettings::EAxis >( JPH::SixDOFConstraintSettings::EAxis::RotationX + i );
			settings->mMotorSettings[ eAxis ] = JPH::MotorSettings( flMotorFrequency, flMotorDamping );
		}
	}

	const bool bEnabled = m_pConstraint->GetEnabled();
	m_pPhysicsSystem->RemoveConstraint( m_pConstraint );
	m_pConstraint = settings->Create( *pRefBody, *pAttBody );
	m_pConstraint->SetEnabled( bEnabled );
	// Creating from settings already restores the constraint's configured base
	// overrides. Only layer the transient boost state back on when a requester is
	// actually active; applying an empty request set here would replace that base
	// with the zero-initialized saved values.
	if ( m_bSolverBoostBaseSaved )
		ApplySolverBoostRequests();
	if ( bEnableTinyAxisMotor )
	{
		JPH::SixDOFConstraint *pSixDOF = static_cast< JPH::SixDOFConstraint * >( m_pConstraint.GetPtr() );
		pSixDOF->SetTargetOrientationCS( JPH::Quat::sIdentity() );
		for ( int i = 0; i < 3; ++i )
		{
			if ( ( m_nRotOnlyTinySwingAxisMask & ( 1u << i ) ) == 0 )
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
	if ( m_pObjAttached )
	{
		m_pObjAttached->RemoveDestroyedListener( this );
		m_pObjAttached->RemoveConstraint( this );
		m_pObjAttached = nullptr;
	}
	if ( m_pObjReference )
	{
		m_pObjReference->RemoveDestroyedListener( this );
		m_pObjReference->RemoveConstraint( this );
		m_pObjReference = nullptr;
	}

	m_pRotOnlySettings = nullptr;
	m_nRotOnlyRecaptureTicks = 0;
	m_nRotOnlyTinySwingAxisMask = 0;
	m_SolverBoostRequests.clear();
	m_bSolverBoostBaseSaved = false;
	m_bHardDistanceRecoveryActive = false;
	m_nSavedVelocityStepsOverride = 0;
	m_nSavedPositionStepsOverride = 0;
	m_nLengthSpringWarmupTicks = 0;

	if ( m_pConstraint )
	{
		m_pPhysicsSystem->RemoveConstraint( m_pConstraint );
		m_pConstraint = nullptr;
	}
}
