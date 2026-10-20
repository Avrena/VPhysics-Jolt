#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include "vjolt_contact_recovery.h"

static constexpr float Unit = 0.0254f;
static constexpr float Dt = 1.0f / 22.0f;

class BroadPhase final : public JPH::BroadPhaseLayerInterface
{
public:
	JPH::uint GetNumBroadPhaseLayers() const override { return 1; }
	JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer) const override { return JPH::BroadPhaseLayer(0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "test"; }
#endif
};
class ObjectBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
{
	bool ShouldCollide(JPH::ObjectLayer, JPH::BroadPhaseLayer) const override { return true; }
};
class ObjectPairs final : public JPH::ObjectLayerPairFilter
{
	bool ShouldCollide(JPH::ObjectLayer, JPH::ObjectLayer) const override { return true; }
};

struct World
{
	struct Listener final : JPH::ContactListener
	{
		World &world;
		bool recovery;
		bool inheritCorrectionFactor = false;
		float lastDepth = 0;
		int recoveryContacts = 0;
		explicit Listener(World &w, bool enabled) : world(w), recovery(enabled) {}
		void OnContactAdded(const JPH::Body &, const JPH::Body &, const JPH::ContactManifold &manifold, JPH::ContactSettings &settings) override
		{
			ConfigureContactRecovery(recovery, world.system.GetPhysicsSettings(), settings);
			if (inheritCorrectionFactor)
			{
				settings.mBaumgarteOverride = -1.0f;
			}
			lastDepth = manifold.mPenetrationDepth / Unit;
			recoveryContacts += settings.mKeepAwakeForPenetration;
		}
		void OnContactPersisted(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m, JPH::ContactSettings &s) override
		{
			OnContactAdded(a, b, m, s);
		}
	};
	BroadPhase broad;
	ObjectBroadPhase objectBroad;
	ObjectPairs pairs;
	JPH::TempAllocatorImpl allocator{4 * 1024 * 1024};
	JPH::JobSystemSingleThreaded jobs{2048};
	JPH::PhysicsSystem system;
	JPH::Array<JPH::Vec3> vertices;
	Listener listener;
	JPH::RVec3 offset = JPH::RVec3::sZero();

	explicit World(float baumgarte, bool recovery = false, int shapeKind = 0) : listener(*this, recovery)
	{
		system.Init(16, 0, 64, 64, broad, objectBroad, pairs);
		system.SetContactListener(&listener);
		system.SetGravity(JPH::Vec3(0, 0, -600 * Unit));
		auto settings = system.GetPhysicsSettings();
		settings.mNumVelocitySteps = 10;
		settings.mNumPositionSteps = 2;
		settings.mBaumgarte = baumgarte;
		settings.mPenetrationSlop = 0.01f * Unit;
		settings.mLinearCastThreshold = 0.0f;
		settings.mLinearCastMaxPenetration = 0.25f;
		system.SetPhysicsSettings(settings);

		if (shapeKind) offset = JPH::RVec3(1000, 2000, -500) * Unit;
		JPH::BodyCreationSettings floor(new JPH::BoxShape(JPH::Vec3(100, 100, 1)),
			offset + JPH::RVec3(0, 0, -1), JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0);
		floor.mFriction = 0.8f;
		system.GetBodyInterface().CreateAndAddBody(floor, JPH::EActivation::DontActivate);
		const float halfX = shapeKind ? 8.15f : 12.0f;
		const float halfY = shapeKind ? 5.92f : 12.0f;
		const float bottom = shapeKind ? 0.009f : -0.8f;
		const float top = shapeKind ? 0.031f : 19.5f;
		for (float x : {-halfX, halfX})
			for (float y : {-halfY, halfY})
				for (float z : {bottom, top})
					if (shapeKind != 2 || z == (x * y > 0 ? bottom : top))
						vertices.push_back(JPH::Vec3(x, y, z) * Unit);
	}
	~World()
	{
		JPH::BodyIDVector bodies;
		system.GetBodies(bodies);
		system.GetBodyInterface().RemoveBodies(bodies.data(), int(bodies.size()));
		system.GetBodyInterface().DestroyBodies(bodies.data(), int(bodies.size()));
	}
	JPH::Body &Hull(float height = 0.0f, float yaw = 0.0f, float mass = 12.0f)
	{
		JPH::ConvexHullShapeSettings hull(vertices, 0.0625f * Unit);
		hull.mHullTolerance = 0.0f;
		auto shape = hull.Create();
		if (shape.HasError()) { std::fprintf(stderr, "%s\n", shape.GetError().c_str()); std::exit(2); }
		JPH::BodyCreationSettings settings(shape.Get(), offset + JPH::RVec3(0, 0, height * Unit),
			JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), JPH::DegreesToRadians(yaw)), JPH::EMotionType::Dynamic, 0);
		settings.mMotionQuality = JPH::EMotionQuality::LinearCast;
		settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
		settings.mMassPropertiesOverride.mMass = mass;
		settings.mFriction = 0.8f;
		settings.mRestitution = 0.0f;
		settings.mLinearDamping = settings.mAngularDamping = 0.1f;
		auto &bi = system.GetBodyInterface();
		JPH::Body *body = bi.CreateBody(settings);
		bi.AddBody(body->GetID(), JPH::EActivation::Activate);
		return *body;
	}
	float Bottom(const JPH::Body &body) const
	{
		float low = 1.0e20f;
		for (auto vertex : vertices)
			low = std::min(low, ((body.GetWorldTransform() * vertex).GetZ() - offset.GetZ()) / Unit);
		return low;
	}
	float RoundedBottom(const JPH::Body &body) const
	{
		JPH::ConvexShape::SupportBuffer buffer;
		const auto *shape = static_cast<const JPH::ConvexShape *>(body.GetShape());
		const auto *support = shape->GetSupportFunction(JPH::ConvexShape::ESupportMode::ExcludeConvexRadius, buffer, JPH::Vec3::sOne());
		auto direction = body.GetRotation().Conjugated() * JPH::Vec3(0, 0, -1);
		auto point = body.GetCenterOfMassTransform() * support->GetSupport(direction);
		return (point.GetZ() - support->GetConvexRadius() - offset.GetZ()) / Unit;
	}
	void Step(int substeps = 2) { system.Update(Dt, substeps, &allocator, &jobs); }
};

static int checks = 0, failures = 0;
static void Check(bool condition, const char *message)
{
	++checks;
	if (!condition) { ++failures; std::printf("FAIL: %s\n", message); }
}

static int Settle(World &world, JPH::Body &body)
{
	int ticks = 0;
	for (; ticks < 220 && body.IsActive(); ++ticks) world.Step();
	return ticks;
}

static void Tests()
{
	World baseline(0.01f);
	auto &original = baseline.Hull();
	const int oldTicks = Settle(baseline, original);
	std::printf("baseline: bottom %.6f u, sleep %.6f s\n", baseline.Bottom(original), oldTicks * Dt);
	Check(!original.IsActive() && baseline.Bottom(original) < -0.3f, "production settings reproduce sleep with deep penetration");

	World corrected(0.01f, true);
	auto &body = corrected.Hull();
	const int newTicks = Settle(corrected, body);
	std::printf("recovery: bottom %.6f u, sleep %.6f s\n", corrected.Bottom(body), newTicks * Dt);
	Check(!body.IsActive(), "recovery permits sleep after separation");
	Check(corrected.Bottom(body) >= -0.01f, "recovery reaches the configured slop");
	Check(newTicks * Dt <= 1.5f, "embedded hull settles within 1.5 seconds");
	Check(corrected.system.GetPhysicsSettings().mBaumgarte == 0.01f, "global Baumgarte is unchanged");
	Check(corrected.system.GetPhysicsSettings().mTimeBeforeSleep == baseline.system.GetPhysicsSettings().mTimeBeforeSleep
		&& corrected.system.GetPhysicsSettings().mPointVelocitySleepThreshold == baseline.system.GetPhysicsSettings().mPointVelocitySleepThreshold,
		"global sleep thresholds are unchanged");

	World lowFactor(0.01f, true);
	lowFactor.listener.inheritCorrectionFactor = true;
	auto &slow = lowFactor.Hull();
	const int slowTicks = Settle(lowFactor, slow);
	std::printf("recovery at original factor: bottom %.6f u, sleep %.6f s\n", lowFactor.Bottom(slow), slowTicks * Dt);
	Check(slowTicks * Dt > 1.5f, "recovery is too slow at the original correction factor");
	Check(!slow.IsActive() && lowFactor.Bottom(slow) >= -0.01f && slowTicks * Dt <= 3.0f,
		"the final translation fallback resolves residual depth at the sleep boundary");

	World global(0.2f);
	auto &globalBody = global.Hull();
	Settle(global, globalBody);
	Check(global.Bottom(globalBody) >= -0.0101f, "global correction control confirms the cause");

	// Withholding eligibility leaves the entire trajectory and sleep time unchanged.
	World excluded(0.01f, false);
	auto &excludedBody = excluded.Hull();
	const int excludedTicks = Settle(excluded, excludedBody);
	Check(excludedTicks == oldTicks && excludedBody.GetPosition() == original.GetPosition()
		&& excludedBody.GetRotation() == original.GetRotation(), "excluded bodies retain the original result exactly");

	// Speculative contacts with no penetration must not move a body or delay sleep.
	World separated(0.01f, true);
	separated.system.SetGravity(JPH::Vec3::sZero());
	auto &separatedBody = separated.Hull(0.9f, 37.0f);
	const auto initialPosition = separatedBody.GetPosition();
	const auto initialRotation = separatedBody.GetRotation();
	const int separatedTicks = Settle(separated, separatedBody);
	Check(separatedBody.GetPosition() == initialPosition && separatedBody.GetRotation() == initialRotation,
		"nonpenetrating speculative contact leaves position and orientation unchanged");
	Check(separatedTicks == oldTicks, "nonpenetrating contact leaves sleep timing unchanged");

	World tilted(0.01f, true);
	tilted.system.SetGravity(JPH::Vec3::sZero());
	auto &tiltedBody = tilted.Hull();
	const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), JPH::DegreesToRadians(1.0f));
	tilted.system.GetBodyInterface().SetRotation(tiltedBody.GetID(), rotation, JPH::EActivation::Activate);
	Settle(tilted, tiltedBody);
	Check(!tiltedBody.IsActive() && tilted.Bottom(tiltedBody) >= -0.01f,
		"an embedded tilted hull recovers before sleep");
	World strong(0.8f, true);
	strong.system.SetGravity(JPH::Vec3::sZero());
	auto &strongBody = strong.Hull();
	strong.Step();
	Check(strong.Bottom(strongBody) <= 0.00001f && strong.Bottom(strongBody) >= -0.01f,
		"multiple manifold points do not overshoot at a high correction factor");

	World corner(0.01f, true);
	auto &cornerBody = corner.Hull();
	constexpr float wallX = 11.2f * Unit;
	JPH::BodyCreationSettings wall(new JPH::BoxShape(JPH::Vec3(1, 100, 100)),
		JPH::RVec3(wallX + 1, 0, 0), JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0);
	corner.system.GetBodyInterface().CreateAndAddBody(wall, JPH::EActivation::DontActivate);
	Settle(corner, cornerBody);
	float maxX = -1.0e20f;
	for (auto vertex : corner.vertices)
		maxX = std::max(maxX, (cornerBody.GetWorldTransform() * vertex).GetX());
	Check(!cornerBody.IsActive() && corner.Bottom(cornerBody) >= -0.01f
		&& maxX <= wallX + 0.01f * Unit, "floor and wall contacts both recover before sleep");

	// Reapply policy after restoring cached contacts; no new settings are serialized.
	World restored(0.01f, true);
	auto &restoredBody = restored.Hull();
	for (int i = 0; i < 3; ++i) restored.Step();
	JPH::StateRecorderImpl saved;
	restored.system.SaveState(saved);
	const auto savedPosition = restoredBody.GetPosition();
	restored.Step();
	saved.Rewind();
	Check(restored.system.RestoreState(saved), "physics state with cached contacts restores");
	Check(restoredBody.GetPosition() == savedPosition, "restore retains the saved position");
	Settle(restored, restoredBody);
	Check(!restoredBody.IsActive() && restored.Bottom(restoredBody) >= -0.01f,
		"restored contact callbacks reapply recovery");

	corrected.system.GetBodyInterface().SetPosition(body.GetID(), JPH::RVec3::sZero(), JPH::EActivation::Activate);
	Settle(corrected, body);
	Check(!body.IsActive() && corrected.Bottom(body) >= -0.01f, "a later embedded wake recovers again");

	// Opt-in must not change friction, restitution, sensors, or higher user correction.
	JPH::ContactSettings sensor;
	sensor.mCombinedFriction = 0.8f;
	sensor.mCombinedRestitution = 0.25f;
	sensor.mIsSensor = true;
	ConfigureContactRecovery(true, corrected.system.GetPhysicsSettings(), sensor);
	Check(sensor.mBaumgarteOverride == -1.0f && !sensor.mKeepAwakeForPenetration, "sensor contact does not opt in");
	sensor.mIsSensor = false;
	ConfigureContactRecovery(true, corrected.system.GetPhysicsSettings(), sensor);
	Check(sensor.mCombinedFriction == 0.8f && sensor.mCombinedRestitution == 0.25f, "material response is unchanged");
	auto settings = corrected.system.GetPhysicsSettings();
	settings.mBaumgarte = 0.5f;
	ConfigureContactRecovery(true, settings, sensor);
	Check(sensor.mBaumgarteOverride == 0.5f, "higher user correction remains effective");
	settings.mPenetrationSlop = 0.0f;
	ConfigureContactRecovery(true, settings, sensor);
	Check(!sensor.mKeepAwakeForPenetration, "zero slop does not require exact floating point convergence to sleep");
}

static void JamTests()
{
	struct Case { float penetration; bool combinedWalls; };
	const Case cases[] = {{-0.5f, false}, {0.25f, false}, {1.0f, false}, {4.0f, false}, {1.0f, true}};
	for (const Case &test : cases)
	{
		const float penetration = test.penetration;
		World world(0.01f, true);
		world.vertices.clear();
		for (float x : {-8.0f, 8.0f})
			for (float y : {-8.0f, 8.0f})
				for (float z : {-8.0f, 8.0f})
					world.vertices.push_back(JPH::Vec3(x, y, z) * Unit);

		auto &bi = world.system.GetBodyInterface();
		const float halfGap = (8.0f - penetration) * Unit;
		JPH::BodyID rightWall;
		JPH::StaticCompoundShapeSettings combined;
		for (float sign : {-1.0f, 1.0f})
		{
			const JPH::Vec3 position(sign * (halfGap + 8 * Unit), 0, 32 * Unit);
			auto *shape = new JPH::BoxShape(JPH::Vec3(8, 64, 64) * Unit);
			if (test.combinedWalls)
			{
				combined.AddShape(position, JPH::Quat::sIdentity(), shape);
				continue;
			}
			JPH::BodyCreationSettings wall(shape, JPH::RVec3(position),
				JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0);
			wall.mFriction = 0.8f;
			auto id = bi.CreateAndAddBody(wall, JPH::EActivation::DontActivate);
			if (sign > 0) rightWall = id;
		}
		if (test.combinedWalls)
		{
			auto shape = combined.Create();
			if (shape.HasError()) { std::fprintf(stderr, "%s\n", shape.GetError().c_str()); std::exit(2); }
			JPH::BodyCreationSettings walls(shape.Get(), JPH::RVec3::sZero(),
				JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0);
			walls.mFriction = 0.8f;
			bi.CreateAndAddBody(walls, JPH::EActivation::DontActivate);
		}
		auto &body = world.Hull(8.02f, 0.0f, 30.0f);
		float firstSleep = -1, maxStep = 0;
		int wakes = 0, moveTicks = 0;
		bool wasActive = true;
		auto previous = body.GetPosition();
		for (int tick = 1; tick <= 264; ++tick)
		{
			world.Step();
			const float time = tick * Dt;
			const float step = (body.GetPosition() - previous).Length() / Unit;
			previous = body.GetPosition();
			if (time >= 2.0f)
			{
				maxStep = std::max(maxStep, step);
				moveTicks += step > 0.01f;
			}
			if (!body.IsActive() && wasActive && firstSleep < 0) firstSleep = time;
			if (body.IsActive() && !wasActive) ++wakes;
			wasActive = body.IsActive();
		}
		std::printf("cube gap penetration %.2f u, combined_walls=%d: asleep=%d first_sleep=%.6f s wakes=%d move_ticks=%d max_step=%.6f u\n",
			penetration, test.combinedWalls, !body.IsActive(), firstSleep, wakes, moveTicks, maxStep);
		Check(!body.IsActive() && firstSleep >= 0 && firstSleep <= (penetration > 0? 4.0f : 1.5f),
			"cube between static walls settles and remains asleep");
		Check(wakes == 0 && moveTicks == 0 && maxStep <= 0.01f,
			"blocked recovery does not keep twitching");

		if (penetration == 1.0f && !test.combinedWalls)
		{
			// Classification is transient: removing a wall must restore useful
			// recovery when the body wakes, including persisted contact callbacks.
			bi.RemoveBody(rightWall);
			bi.DestroyBody(rightWall);
			bi.ActivateBody(body.GetID());
			const int ticks = Settle(world, body);
			float minX = 1.0e20f;
			for (auto vertex : world.vertices)
				minX = std::min(minX, (body.GetWorldTransform() * vertex).GetX());
			Check(!body.IsActive() && ticks * Dt <= 1.5f
				&& minX >= -halfGap - 0.01f * Unit && world.Bottom(body) >= -0.01f,
				"removing an opposing wall re-enables recovery after wake");
		}
	}
}

static void TaperTests()
{
	struct Case { float halfAngle; float penetration; bool closed; int substeps = 2; };
	const Case cases[] = {
		{0, 1, false}, {1, 1, false}, {2, 1, false}, {5, 1, false},
		{10, 1, false}, {2, 4, false}, {5, 4, false}, {2, 4, true}, {10, 1, true},
		{5, 4, true, 1}, {5, 4, true, 4}
	};
	for (const Case &test : cases)
	{
		World world(0.01f, true);
		world.vertices.clear();
		for (float x : {-8.0f, 8.0f})
			for (float y : {-8.0f, 8.0f})
				for (float z : {-8.0f, 8.0f})
					world.vertices.push_back(JPH::Vec3(x, y, z) * Unit);
		auto &bi = world.system.GetBodyInterface();
		const float angle = JPH::DegreesToRadians(test.halfAngle);
		JPH::BodyIDVector walls;
		for (float side : {-1.0f, 1.0f})
		{
			const JPH::Vec3 normal(-side * std::cos(angle), std::sin(angle), 0);
			const JPH::Vec3 along(side * std::sin(angle), std::cos(angle), 0);
			const JPH::Vec3 face(side * (8 - test.penetration), 0, 54.5f);
			JPH::BodyCreationSettings wall(new JPH::BoxShape(JPH::Vec3(4, 32, 55.5f) * Unit),
				JPH::RVec3((face - 4 * normal + 14 * along) * Unit),
				JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -side * angle), JPH::EMotionType::Static, 0);
			wall.mFriction = 0.8f;
			walls.push_back(bi.CreateAndAddBody(wall, JPH::EActivation::DontActivate));
		}
		if (test.closed)
		{
			JPH::BodyCreationSettings wall(new JPH::BoxShape(JPH::Vec3(32, 4, 55.5f) * Unit),
				JPH::RVec3(0, 14, 54.5f) * Unit, JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0);
			wall.mFriction = 0.8f;
			walls.push_back(bi.CreateAndAddBody(wall, JPH::EActivation::DontActivate));
		}
		auto &body = world.Hull(8.02f, 0, 30);
		float firstSleep = -1, maxStep = 0;
		int wakes = 0, moveTicks = 0;
		bool wasActive = true;
		auto previous = body.GetPosition();
		for (int tick = 1; tick <= 264; ++tick)
		{
			world.Step(test.substeps);
			const float time = tick * Dt;
			const float step = (body.GetPosition() - previous).Length() / Unit;
			previous = body.GetPosition();
			if (time >= 2)
			{
				maxStep = std::max(maxStep, step);
				moveTicks += step > 0.01f;
			}
			if (!body.IsActive() && wasActive && firstSleep < 0) firstSleep = time;
			if (body.IsActive() && !wasActive) ++wakes;
			wasActive = body.IsActive();
		}
		std::printf("taper %.0f deg, overlap %.1f u, closed=%d substeps=%d: asleep=%d first_sleep=%.6f s wakes=%d move_ticks=%d max_step=%.6f u\n",
			2 * test.halfAngle, test.penetration, test.closed, test.substeps, !body.IsActive(), firstSleep, wakes, moveTicks, maxStep);
		Check(!body.IsActive() && firstSleep >= 0 && firstSleep <= 3.0f,
			"tapered or closed gap settles within three seconds");
		Check(wakes == 0, "tapered or closed gap stays asleep");

		if (test.closed && test.halfAngle == 2 && test.substeps == 2)
		{
			auto *motion = body.GetMotionProperties();
			Check(!motion->CanRecoverContacts(), "deep recovery budget is exhausted");
			bi.ActivateBody(body.GetID());
			Check(motion->CanRecoverContacts(), "an actual wake renews the recovery budget");
			for (int tick = 0; tick < 24; ++tick) world.Step();
			Check(body.IsActive() && !motion->CanRecoverContacts(),
				"the next wake has a fresh but bounded recovery interval");
			bi.ActivateBody(body.GetID());
			Check(!motion->CanRecoverContacts(),
				"activating an already active body does not renew recovery");

			JPH::StateRecorderImpl saved;
			world.system.SaveState(saved);
			saved.Rewind();
			Check(world.system.RestoreState(saved) && motion->CanRecoverContacts(),
				"restoring an active body resets transient recovery state");
			for (auto id : walls)
			{
				bi.RemoveBody(id);
				bi.DestroyBody(id);
			}
			bi.SetPosition(body.GetID(), JPH::RVec3(0, 0, 2 * Unit), JPH::EActivation::DontActivate);
			bi.SetLinearAndAngularVelocity(body.GetID(), JPH::Vec3::sZero(), JPH::Vec3::sZero());
			const int ticks = Settle(world, body);
			Check(!body.IsActive() && ticks * Dt <= 1.5f && world.Bottom(body) >= -0.01f,
				"restored recovery can resolve a new embedded contact");
		}
	}
}

int main(int argc, char **argv)
{
	JPH::RegisterDefaultAllocator();
	JPH::Factory::sInstance = new JPH::Factory;
	JPH::RegisterTypes();
	int result;
	if (argc == 1)
	{
		Tests();
		JamTests();
		TaperTests();
		std::printf("%d checks, %d failures\n", checks, failures);
		result = failures != 0;
	}
	else
	{
		const float baumgarte = std::strtof(argv[1], nullptr);
		const bool recovery = argc > 2 && std::atoi(argv[2]) != 0;
		const int shapeKind = argc > 3 ? std::atoi(argv[3]) : 0;
		const float yaw = argc > 4 ? std::strtof(argv[4], nullptr) : 0;
		World world(baumgarte, recovery, shapeKind);
		JPH::Body &body = world.Hull(0.0f, yaw);
		const int ticks = Settle(world, body);
		std::printf("baumgarte=%.3f recovery=%d shape=%d yaw=%.1f bottom=%.6f u asleep=%d time=%.6f s ticks=%d lastDepth=%.6f recoveries=%d roundedBottom=%.6f\n",
			baumgarte, recovery, shapeKind, yaw, world.Bottom(body), !body.IsActive(), ticks * Dt, ticks,
			world.listener.lastDepth, world.listener.recoveryContacts, world.RoundedBottom(body));
		result = body.IsActive() || !std::isfinite(world.Bottom(body));
	}
	JPH::UnregisterTypes();
	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;
	return result;
}
