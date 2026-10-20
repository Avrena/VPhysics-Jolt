#pragma once

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/PhysicsSettings.h>

// Eligibility is supplied by the Source wrapper: a free dynamic object against
// genuinely static geometry, with no constraint, spring or game controller.
// Added and Persisted must both use this policy; the settings are not cached.
inline void ConfigureContactRecovery( bool bEligible,
	const JPH::PhysicsSettings &physicsSettings, JPH::ContactSettings &settings )
{
	if ( !bEligible || settings.mIsSensor )
		return;

	settings.mBaumgarteOverride = physicsSettings.mBaumgarte > 0.2f ? physicsSettings.mBaumgarte : 0.2f;
	settings.mKeepAwakeForPenetration = physicsSettings.mPenetrationSlop > 0.0f;
}
