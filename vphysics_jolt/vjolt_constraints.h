//=================================================================================================
//
// Constraints
//
//=================================================================================================

#pragma once

#include "vjolt_internal_listeners.h"

enum constraintType_t
{
	CONSTRAINT_UNKNOWN = 0,
	CONSTRAINT_RAGDOLL,
	CONSTRAINT_HINGE,
	CONSTRAINT_FIXED,
	CONSTRAINT_BALLSOCKET,
	CONSTRAINT_SLIDING,
	CONSTRAINT_PULLEY,
	CONSTRAINT_LENGTH,
};

class JoltPhysicsConstraint;
class JoltPhysicsEnvironment;

class JoltPhysicsConstraintGroup final : public IPhysicsConstraintGroup
{
public:
	JoltPhysicsConstraintGroup();
	~JoltPhysicsConstraintGroup() override;

	void Activate() override;
	bool IsInErrorState() override;
	void ClearErrorState() override;
	void GetErrorParams( constraint_groupparams_t *pParams ) override;
	void SetErrorParams( const constraint_groupparams_t &params ) override;
	void SolvePenetration( IPhysicsObject *pObj0, IPhysicsObject *pObj1 ) override;

	void AddConstraint( JoltPhysicsConstraint *pConstraint );
	void RemoveConstraint( JoltPhysicsConstraint *pConstraint );

private:
	std::vector< JoltPhysicsConstraint * >	m_pConstraints;
	constraint_groupparams_t				m_ErrorParams = {};
};

class JoltPhysicsConstraint final : public IPhysicsConstraint, public IJoltObjectDestroyedListener
{
public:
	JoltPhysicsConstraint( JoltPhysicsEnvironment *pPhysicsEnvironment, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, constraintType_t Type = CONSTRAINT_UNKNOWN, JPH::Constraint* pConstraint = nullptr, void *pGameData = nullptr );
	~JoltPhysicsConstraint() override;

	void			Activate() override;
	void			Deactivate() override;

	void			SetGameData( void *gameData ) override;
	void *			GetGameData() const override;

	IPhysicsObject *GetReferenceObject() const override;
	IPhysicsObject *GetAttachedObject() const override;

	void			SetLinearMotor( float speed, float maxLinearImpulse ) override;
	void			SetAngularMotor( float rotSpeed, float maxAngularImpulse ) override;

	void			UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached ) override;
	bool			GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const override;
	bool			GetConstraintParams( constraint_breakableparams_t *pParams ) const override;

	void			OutputDebugInfo() override;

	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;

public:
	bool InitialiseHingeFromRagdoll( IPhysicsConstraintGroup* pGroup, const constraint_ragdollparams_t& ragdoll );
	void InitialiseRagdoll( IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll );
	void InitialiseHinge( IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge );
	void InitialiseSliding( IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding );
	void InitialiseBallsocket( IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket );
	void InitialiseFixed( IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed );
	void InitialiseLength( IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length );
	void InitialisePulley( IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley );

	void SaveConstraintSettings( JPH::StateRecorder &recorder );

	// Once per environment simulate step, after the Jolt update.
	void PostSimulate();

	// Shadow-driven constrained assemblies need extra solver work only while a
	// hard distance joint is actually displaced. The owning object uses this to
	// retire its temporary per-island solver override as soon as the assembly is
	// back inside the authored rope limits.
	bool HasHardDistanceErrorGreaterThan( float flTolerance ) const;

	// Jolt only considers a body's solver-step override when that body owns a
	// contact. Constraint overrides are always included in the island's step
	// calculation, which makes them the reliable owner for suspended assemblies.
	void RequestSolverBoost( const JoltPhysicsObject *pRequester, uint nVelocitySteps, uint nPositionSteps );
	void ReleaseSolverBoost( const JoltPhysicsObject *pRequester );

	bool CheckBroken();

private:

	void RecaptureRotOnlyFrames();
	void HardenLengthSpring();
	void UpdateHardDistanceRecovery();

	void SetGroup( IPhysicsConstraintGroup *pGroup );

	void DestroyConstraint();

	void SetBreakableParams( const constraint_breakableparams_t &params );
	void SaveSolverBoostBase();
	void ApplySolverBoostRequests();

	struct SolverBoostRequest
	{
		const JoltPhysicsObject *m_pRequester = nullptr;
		uint8 m_nVelocitySteps = 0;
		uint8 m_nPositionSteps = 0;
	};

	JoltPhysicsObject			*m_pObjReference = nullptr;
	JoltPhysicsObject			*m_pObjAttached = nullptr;
	JPH::Ref< JPH::Constraint > m_pConstraint;
	constraintType_t			m_ConstraintType = CONSTRAINT_UNKNOWN;

	// Rotation-only (onlyAngularLimits) ragdoll joints: settings kept alive for a
	// one-shot frame re-capture N steps after creation (see vjolt_onlyrot_recapture_ticks).
	JPH::Ref< JPH::SixDOFConstraintSettings >	m_pRotOnlySettings;
	int							m_nRotOnlyRecaptureTicks = 0;
	uint8						m_nRotOnlyTinySwingAxisMask = 0;

	std::vector< SolverBoostRequest >	m_SolverBoostRequests;
	bool						m_bSolverBoostBaseSaved = false;
	bool						m_bHardDistanceRecoveryActive = false;
	uint8						m_nSavedVelocityStepsOverride = 0;
	uint8						m_nSavedPositionStepsOverride = 0;

	// Length (rope) constraints: countdown until the soft warmup limits harden
	// (see vjolt_length_spring_warmup_ticks).
	int							m_nLengthSpringWarmupTicks = 0;

	JoltPhysicsConstraintGroup	*m_pGroup = nullptr;

	void						*m_pGameData = nullptr;
	JoltPhysicsEnvironment		*m_pPhysicsEnvironment = nullptr;
	JPH::PhysicsSystem			*m_pPhysicsSystem = nullptr;

	float						m_LinearBreakImpulse = 0.0f;
	float						m_AngularBreakImpulse = 0.0f;
	float						m_SourceForceLimit = 0.0f;
	float						m_SourceTorqueLimit = 0.0f;
	float						m_BreakStrength = 1.0f;
	float						m_BodyMassScale[2] = { 1.0f, 1.0f };
};
