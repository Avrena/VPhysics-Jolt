#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/StateRecorderImpl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

using uint32 = JPH::uint32;
template <class T> static T Min(T a, T b) { return std::min(a, b); }
template <class T> static T Max(T a, T b) { return std::max(a, b); }
#define DEG2RAD(value) JPH::DegreesToRadians(value)

// Only the SDK inputs and engine services used by the extracted branch are
// shimmed. Limits, mapping, PostSimulate and baseline recapture are real code.
struct constraint_ragdollparams_t
{
    struct Axis { float minRotation, maxRotation, torque; } axes[3] = {
        {-180.0f, 180.0f, 0.0f}, {-0.0001f, 0.0001f, 0.0f}, {-0.0001f, 0.0001f, 0.0f}};
    bool useClockwiseRotations = false;
    bool onlyAngularLimits = true;
};
namespace SourceToJolt { static float Torque(float value) { return value * 0.0254f * 0.0254f; } }
static const struct { int GetInt() const { return 2; } } vjolt_onlyrot_recapture_ticks;
#include "ragdoll_limits.inl"
#include "rotation_correction.inl"

class JoltPhysicsConstraint
{
    struct Object { JPH::Body *body; JPH::Body *GetBody() const { return body; } } reference, attached;
    struct Group { void ApplySolverIterations(JoltPhysicsConstraint *) {} };
public:
    JoltPhysicsConstraint(JPH::PhysicsSystem &system, JPH::Body &ref, JPH::Body &att,
        const constraint_ragdollparams_t &ragdoll, JPH::Mat44 constraintToReference, JPH::Mat44 constraintToAttached)
        : reference{&ref}, attached{&att}, m_pPhysicsSystem(&system)
    {
        auto *pRefBody = &ref;
        auto *pAttBody = &att;
        RagdollLimits_t limits(ragdoll);
        [[maybe_unused]] const float flMinTorqueFriction = 0.05f;
        JPH::Constraint *pConstraint = nullptr;
#include "rotation_mapping.inl"
        m_pConstraint = pConstraint;
        system.AddConstraint(m_pConstraint);
    }
    ~JoltPhysicsConstraint() { m_pPhysicsSystem->RemoveConstraint(m_pConstraint); }
    void PostSimulate();
    void RecaptureRotOnlyFrames();
    void HardenLengthSpring() {} // Not a length constraint.
    bool CheckBroken() { return false; } // Test inputs are unbreakable.
    JPH::Ref<JPH::Constraint> m_pConstraint;
    JPH::Ref<JPH::SixDOFConstraintSettings> m_pRotOnlySettings;
    int m_nRotOnlyRecaptureTicks = 0;
    Object *m_pObjReference = &reference;
    Object *m_pObjAttached = &attached;
    Group *m_pGroup = nullptr;
    JPH::PhysicsSystem *m_pPhysicsSystem;
};
#include "post_simulate.inl"
#include "recapture.inl"

// Layer zero isolates joints. Layer one permits the optional ground-contact test.
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
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer) const override { return layer == 1; }
};
class ObjectPairs final : public JPH::ObjectLayerPairFilter
{
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override { return a == 1 && b == 1; }
};
struct World
{
    BroadPhase broad;
    ObjectBroadPhase objectBroad;
    ObjectPairs pairs;
    JPH::TempAllocatorImpl allocator{4 * 1024 * 1024};
    JPH::JobSystemSingleThreaded jobs{2048};
    JPH::PhysicsSystem system;
    bool contacts;
    explicit World(bool withContacts = false) : contacts(withContacts)
    {
        system.Init(16, 0, 64, 64, broad, objectBroad, pairs);
        system.SetGravity(contacts ? JPH::Vec3(0, -9.81f, 0) : JPH::Vec3::sZero());
        auto settings = system.GetPhysicsSettings();
        settings.mNumVelocitySteps = 10;
        settings.mNumPositionSteps = 2;
        settings.mBaumgarte = 0.01f;
        system.SetPhysicsSettings(settings);
    }
    ~World()
    {
        JPH::BodyIDVector bodies;
        system.GetBodies(bodies);
        system.GetBodyInterface().RemoveBodies(bodies.data(), int(bodies.size()));
        system.GetBodyInterface().DestroyBodies(bodies.data(), int(bodies.size()));
    }
    JPH::Body &Body(bool dynamic, JPH::Quat rotation = JPH::Quat::sIdentity())
    {
        JPH::BodyCreationSettings settings(new JPH::SphereShape(0.2f), contacts ? JPH::RVec3(0, 0.2f, 0) : JPH::RVec3::sZero(), rotation,
            dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static, contacts && dynamic ? 1 : 0);
        settings.mFriction = 0.8f;
        settings.mAllowSleeping = false;
        settings.mLinearDamping = settings.mAngularDamping = 0;
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = 100;
        auto &bi = system.GetBodyInterface();
        auto *body = bi.CreateBody(settings);
        bi.AddBody(body->GetID(), dynamic ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
        return *body;
    }
    JPH::BodyID Ground()
    {
        JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(100, 0.1f, 100)),
            JPH::RVec3(0, -0.1f, 0), JPH::Quat::sIdentity(), JPH::EMotionType::Static, 1);
        settings.mFriction = 0.8f;
        return system.GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::DontActivate);
    }
    void Step() { system.Update(1.0f / 22.0f, 2, &allocator, &jobs); }
};

static int checks = 0, failures = 0;
static void Check(bool condition, const char *message)
{
    ++checks;
    if (!condition) { ++failures; std::printf("FAIL: %s\n", message); }
}
static bool Near(float actual, float expected, float tolerance = 1.0e-7f)
{
    return std::isfinite(actual) && std::abs(actual - expected) <= tolerance;
}
static float AxisError(JPH::Vec3 actual, JPH::Vec3 expected)
{
    // atan2 remains useful at sub-degree angles where acos loses precision.
    return JPH::RadiansToDegrees(std::atan2(actual.Cross(expected).Length(), actual.Dot(expected)));
}
static JPH::Ref<JPH::SixDOFConstraintSettings> Settings(const JoltPhysicsConstraint &joint)
{
    return static_cast<JPH::SixDOFConstraintSettings *>(joint.m_pConstraint->GetConstraintSettings().GetPtr());
}

static void TestLimits()
{
    for (bool mirrored : {false, true})
    {
        World world;
        auto &wheel = world.Body(true);
        auto &master = world.Body(false);
        constraint_ragdollparams_t params;
        if (mirrored)
            for (int a = 1; a < 3; ++a) std::swap(params.axes[a].minRotation, params.axes[a].maxRotation);
        JoltPhysicsConstraint joint(world.system, wheel, master, params, JPH::Mat44::sIdentity(), JPH::Mat44::sIdentity());
        auto settings = Settings(joint);
        for (int a = 0; a < 3; ++a)
            Check(settings->IsFreeAxis(static_cast<JPH::SixDOFConstraintSettings::EAxis>(a)), "translation stays free");
        Check(settings->mLimitMin[3] <= -JPH::JPH_PI && settings->mLimitMax[3] >= JPH::JPH_PI, "wheel spin stays free");
        for (int a = 4; a < 6; ++a)
        {
            Check(Near(settings->mLimitMin[a], DEG2RAD(-0.0001f)), "authored near-zero minimum is not widened");
            Check(Near(settings->mLimitMax[a], DEG2RAD(0.0001f)), "authored near-zero maximum is not widened");
        }
        for (int a = 3; a < 6; ++a)
            Check(settings->mMaxFriction[a] == 0.0f, "zero authored torque adds no friction");
    }
    World world;
    auto &wheel = world.Body(true);
    auto &master = world.Body(false);
    constraint_ragdollparams_t params;
    params.axes[1] = {2.0f, 2.2f, 40.0f};
    params.useClockwiseRotations = true;
    JoltPhysicsConstraint joint(world.system, wheel, master, params, JPH::Mat44::sIdentity(), JPH::Mat44::sIdentity());
    auto settings = Settings(joint);
    Check(Near(settings->mLimitMin[4], DEG2RAD(-2.2f)), "clockwise asymmetric minimum is preserved");
    Check(Near(settings->mLimitMax[4], DEG2RAD(-2.0f)), "clockwise asymmetric maximum is preserved");
    Check(Near(settings->mMaxFriction[4], SourceToJolt::Torque(40.0f)), "authored torque alone is preserved");
}

static void TestRestFrameAndRecovery(float initialSkew)
{
    World world;
    auto &wheel = world.Body(true);
    auto &master = world.Body(false);
    constraint_ragdollparams_t params;
    JoltPhysicsConstraint joint(world.system, wheel, master, params, JPH::Mat44::sIdentity(), JPH::Mat44::sIdentity());
    std::swap(params.axes[1].minRotation, params.axes[1].maxRotation);
    std::swap(params.axes[2].minRotation, params.axes[2].maxRotation);
    JoltPhysicsConstraint mirror(world.system, master, wheel, params, JPH::Mat44::sIdentity(), JPH::Mat44::sIdentity());
    // A transient after creation must be corrected, never adopted as a new rest frame.
    world.system.GetBodyInterface().SetRotation(wheel.GetID(), JPH::Quat::sRotation(JPH::Vec3::sAxisY(), DEG2RAD(initialSkew)), JPH::EActivation::Activate);
    JPH::Ref<JPH::Constraint> original = joint.m_pConstraint;
    for (int i = 0; i < 2; ++i) { world.Step(); joint.PostSimulate(); mirror.PostSimulate(); }
    Check(joint.m_pConstraint == original, "PostSimulate does not replace the authored joint");
    auto settings = Settings(joint);
    Check(settings->mAxisX1.IsClose(JPH::Vec3::sAxisX(), 1.0e-12f), "transient tilt does not rewrite reference X");
    Check(settings->mAxisY1.IsClose(JPH::Vec3::sAxisY(), 1.0e-12f), "authored reference Y stays unchanged");
    for (int i = 0; i < 220; ++i) { world.Step(); joint.PostSimulate(); mirror.PostSimulate(); }
    const float error = AxisError(wheel.GetRotation() * JPH::Vec3::sAxisX(), JPH::Vec3::sAxisX());
    std::printf("initial %.2f deg -> recovered %.6f deg\n", initialSkew, error);
    Check(error < 0.01f, "wheel returns to authored axle alignment instead of a slack or recaptured pose");
    // Disturb the existing joint again, as during a later rotation/recovery.
    world.system.GetBodyInterface().SetRotation(wheel.GetID(), JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), DEG2RAD(-4.0f)), JPH::EActivation::Activate);
    for (int i = 0; i < 220; ++i) { world.Step(); joint.PostSimulate(); mirror.PostSimulate(); }
    Check(AxisError(wheel.GetRotation() * JPH::Vec3::sAxisX(), JPH::Vec3::sAxisX()) < 0.01f, "existing joint also recovers after a later disturbance");
}

static void TestCommonSpawnRotationAndFreeMotion()
{
    World world;
    const auto quarterTurn = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), DEG2RAD(90.0f));
    auto &wheel = world.Body(true);
    auto &master = world.Body(false, quarterTurn);
    constraint_ragdollparams_t params;
    const auto masterFrame = JPH::Mat44::sRotation(quarterTurn.Conjugated());
    JoltPhysicsConstraint joint(world.system, wheel, master, params, JPH::Mat44::sIdentity(), masterFrame);
    std::swap(params.axes[1].minRotation, params.axes[1].maxRotation);
    std::swap(params.axes[2].minRotation, params.axes[2].maxRotation);
    JoltPhysicsConstraint mirror(world.system, master, wheel, params, masterFrame, JPH::Mat44::sIdentity());
    // LVS starts wheel=0, master=90 yaw, then moves both by a common rotation.
    const auto chassis = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), DEG2RAD(31.0f)) * JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), DEG2RAD(57.0f));
    const auto common = chassis * quarterTurn.Conjugated();
    auto &bi = world.system.GetBodyInterface();
    bi.SetRotation(wheel.GetID(), common, JPH::EActivation::Activate);
    bi.SetRotation(master.GetID(), chassis, JPH::EActivation::DontActivate);
    const auto spinAxis = common * JPH::Vec3::sAxisX();
    const JPH::Vec3 velocity(3.0f, -2.0f, 1.0f);
    bi.SetLinearVelocity(wheel.GetID(), velocity);
    bi.SetAngularVelocity(wheel.GetID(), spinAxis * 8.0f);
    for (int i = 0; i < 22; ++i) { world.Step(); joint.PostSimulate(); mirror.PostSimulate(); }
    Check(AxisError(wheel.GetRotation() * JPH::Vec3::sAxisX(), spinAxis) < 0.01f, "common spawn rotation preserves the authored axle");
    Check(Near(wheel.GetAngularVelocity().Dot(spinAxis), 8.0f, 0.01f), "wheel remains freely spinning");
    Check((JPH::Vec3(wheel.GetPosition()) - velocity).Length() < 0.0001f, "wheel translation is not welded to the static steering master");
}

static void TestBrakeLockRelease()
{
    World world;
    auto &wheel = world.Body(true);
    auto &master = world.Body(false);
    constraint_ragdollparams_t params;
    JoltPhysicsConstraint axle(world.system, wheel, master, params, JPH::Mat44::sIdentity(), JPH::Mat44::sIdentity());
    auto &bi = world.system.GetBodyInterface();
    bi.SetRotation(wheel.GetID(), JPH::Quat::sRotation(JPH::Vec3::sAxisX(), DEG2RAD(37.0f)), JPH::EActivation::Activate);
    bi.SetAngularVelocity(wheel.GetID(), JPH::Vec3(8, 0, 0));
    const auto brakePhase = wheel.GetRotation() * JPH::Vec3::sAxisY();
    {
        // LVS adds an all-angular-axis brake socket at the current spin phase.
        for (auto &axis : params.axes) axis = {-0.1f, 0.1f, 0.0f};
        JoltPhysicsConstraint brake(world.system, wheel, master, params,
            JPH::Mat44::sIdentity(), JPH::Mat44::sRotation(wheel.GetRotation()));
        auto settings = Settings(brake);
        for (int axis = 3; axis < 6; ++axis)
        {
            Check(Near(settings->mLimitMin[axis], DEG2RAD(-0.1f)), "brake minimum is not widened");
            Check(Near(settings->mLimitMax[axis], DEG2RAD(0.1f)), "brake maximum is not widened");
        }
        for (int i = 0; i < 22; ++i) { world.Step(); axle.PostSimulate(); brake.PostSimulate(); }
        Check(wheel.GetAngularVelocity().Length() < 0.01f, "brake socket stops wheel spin");
        Check(AxisError(wheel.GetRotation() * JPH::Vec3::sAxisX(), JPH::Vec3::sAxisX()) < 0.01f, "brake does not change axle alignment");
        Check(AxisError(wheel.GetRotation() * JPH::Vec3::sAxisY(), brakePhase) < 0.01f, "brake preserves the captured spin phase");
    }
    bi.SetAngularVelocity(wheel.GetID(), JPH::Vec3(8, 0, 0));
    for (int i = 0; i < 22; ++i) { world.Step(); axle.PostSimulate(); }
    Check(Near(wheel.GetAngularVelocity().GetX(), 8.0f, 0.01f), "removing brake socket restores free spin");
}

// Characterize a moving steering target separately from the settled-pose tests.
// No contacts: this can establish angular lag, not prove live vehicle traction.
static void TestMovingSteeringMaster(unsigned positionSteps, bool inherit, bool contact = false)
{
    World world(contact);
    const JPH::BodyID ground = contact ? world.Ground() : JPH::BodyID();
    auto physicsSettings = world.system.GetPhysicsSettings();
    physicsSettings.mNumPositionSteps = positionSteps;
    world.system.SetPhysicsSettings(physicsSettings);
    auto &wheel = world.Body(true);
    auto &master = world.Body(false);
    constraint_ragdollparams_t params;
    JoltPhysicsConstraint axle(world.system, wheel, master, params, JPH::Mat44::sIdentity(), JPH::Mat44::sIdentity());
    std::swap(params.axes[1].minRotation, params.axes[1].maxRotation);
    std::swap(params.axes[2].minRotation, params.axes[2].maxRotation);
    JoltPhysicsConstraint mirror(world.system, master, wheel, params, JPH::Mat44::sIdentity(), JPH::Mat44::sIdentity());
    if (inherit)
    {
        // Negative control: reproduce the previous global-policy coupling.
        static_cast<JPH::SixDOFConstraint *>(axle.m_pConstraint.GetPtr())->SetRotationPositionCorrection(-1);
        static_cast<JPH::SixDOFConstraint *>(mirror.m_pConstraint.GetPtr())->SetRotationPositionCorrection(-1);
    }
    auto &bi = world.system.GetBodyInterface();
    bi.SetAngularVelocity(wheel.GetID(), JPH::Vec3(8, 0, 0));
    float worstError = 0.0f;
    float firstTurnError = 0.0f;
    float reversalError = 0.0f;
    unsigned contactSteps = 0;
    for (int tick = 0; tick < 66; ++tick)
    {
        // 0 -> 30 degrees over one third of a second, hold, then reverse.
        const float target = tick < 22 ? 30.0f * std::min((tick + 1) / 7.0f, 1.0f)
            : 30.0f - 60.0f * std::min((tick - 21) / 7.0f, 1.0f);
        bi.SetRotation(master.GetID(), JPH::Quat::sRotation(contact ? JPH::Vec3::sAxisY() : JPH::Vec3::sAxisZ(), DEG2RAD(target)), JPH::EActivation::DontActivate);
        if (contact)
            bi.SetAngularVelocity(wheel.GetID(), wheel.GetAngularVelocity() + (master.GetRotation() * JPH::Vec3::sAxisX()) * (16.0f / 22.0f));
        world.Step();
        axle.PostSimulate();
        mirror.PostSimulate();
        const float error = AxisError(wheel.GetRotation() * JPH::Vec3::sAxisX(), master.GetRotation() * JPH::Vec3::sAxisX());
        worstError = std::max(worstError, error);
        if (tick == 6) firstTurnError = error;
        if (tick == 28) reversalError = error;
        if (contact && world.system.WereBodiesInContact(wheel.GetID(), ground)) ++contactSteps;
    }
    const float finalError = AxisError(wheel.GetRotation() * JPH::Vec3::sAxisX(), master.GetRotation() * JPH::Vec3::sAxisX());
    std::printf("moving master %s / %u position steps / contact %d: turn %.4f deg, reversal %.4f deg, worst %.4f deg, settled %.4f deg, speed %.4f m/s, contact steps %u\n",
        inherit ? "global 0.01" : "scoped default", positionSteps, int(contact), firstTurnError, reversalError, worstError, finalError, wheel.GetLinearVelocity().Length(), contactSteps);
    Check(std::isfinite(worstError) && std::isfinite(finalError), "moving master characterization stays finite");
    if (!inherit)
    {
        Check(worstError < (positionSteps == 2 ? 2.0f : 0.25f), "moving steering master tracks without large angular lag");
        Check(finalError < 0.01f, "steering reversal settles back to the authored axle");
    }
    if (contact)
    {
        Check(contactSteps > 50, "steering test actually exercises sustained ground contact");
        Check(wheel.GetLinearVelocity().Length() > 0.5f, "driven wheel retains movement with ground friction");
    }
}

static void TestCorrectionScopeAndRecreation()
{
    World world;
    auto &wheel = world.Body(true);
    auto &master = world.Body(false);
    constraint_ragdollparams_t params;
    JoltPhysicsConstraint axle(world.system, wheel, master, params, JPH::Mat44::sIdentity(), JPH::Mat44::sIdentity());
    JPH::StateRecorderImpl saved;
    Settings(axle)->SaveBinaryState(saved);
    saved.Rewind();
    auto result = JPH::ConstraintSettings::sRestoreFromBinaryState(saved);
    Check(result.IsValid(), "existing settings format restores without a new serialized field");
    if (result.IsValid())
    {
        JPH::Ref<JPH::TwoBodyConstraint> restored = static_cast<JPH::TwoBodyConstraintSettings *>(result.Get().GetPtr())->Create(wheel, master);
        auto *six = static_cast<JPH::SixDOFConstraint *>(restored.GetPtr());
        Check(six->GetRotationPositionCorrection() == -1.0f, "fresh Jolt constraint inherits global correction by default");
        ConfigureRotationOnlyCorrection(six);
        Check(six->GetRotationPositionCorrection() == JPH::PhysicsSettings().mBaumgarte, "Source recreation helper reapplies the angular policy");
        JPH::StateRecorderImpl resaved;
        six->GetConstraintSettings()->SaveBinaryState(resaved);
        Check(saved.GetData() == resaved.GetData(), "runtime correction policy leaves serialized settings unchanged");
    }
    ConfigureRotationOnlyCorrection(nullptr);
    for (bool fullyLocked : {false, true})
    {
        float position[2], angle[2];
        for (int override = 0; override < 2; ++override)
        {
            World test;
            auto &body = test.Body(true);
            auto &anchor = test.Body(false);
            JPH::SixDOFConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.MakeFixedAxis(JPH::SixDOFConstraint::EAxis::TranslationX);
            settings.MakeFixedAxis(JPH::SixDOFConstraint::EAxis::RotationY);
            settings.MakeFixedAxis(JPH::SixDOFConstraint::EAxis::RotationZ);
            if (fullyLocked) settings.MakeFixedAxis(JPH::SixDOFConstraint::EAxis::RotationX);
            // Keep the translation basis on the static body, so correcting the
            // dynamic body's angle cannot rotate the axis being compared.
            JPH::Ref<JPH::TwoBodyConstraint> raw = settings.Create(anchor, body);
            auto *six = static_cast<JPH::SixDOFConstraint *>(raw.GetPtr());
            ConfigureRotationOnlyCorrection(six);
            Check(six->GetRotationPositionCorrection() == -1.0f, "Source helper leaves translation-constrained SixDOF untouched");
            if (override) six->SetRotationPositionCorrection(JPH::PhysicsSettings().mBaumgarte);
            test.system.GetBodyInterface().SetPositionAndRotation(body.GetID(), JPH::RVec3(1, 0, 0),
                JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), DEG2RAD(10.0f)), JPH::EActivation::Activate);
            six->SolvePositionConstraint(1.0f / 22.0f, 0.01f);
            position[override] = body.GetPosition().GetX();
            angle[override] = AxisError(body.GetRotation() * JPH::Vec3::sAxisX(), JPH::Vec3::sAxisX());
        }
        Check(Near(position[0], position[1]), "angular override does not alter translational correction");
        Check(angle[1] < angle[0] * 0.9f, "angular override reaches partial and fully locked rotation solves");
    }
}

int main()
{
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory;
    JPH::RegisterTypes();
    TestLimits();
    TestRestFrameAndRecovery(0.25f);
    TestRestFrameAndRecovery(2.0f);
    TestCommonSpawnRotationAndFreeMotion();
    TestBrakeLockRelease();
    for (unsigned positionSteps : {2u, 6u})
    {
        TestMovingSteeringMaster(positionSteps, true);
        TestMovingSteeringMaster(positionSteps, false);
    }
    TestMovingSteeringMaster(6, true, true);
    TestMovingSteeringMaster(6, false, true);
    TestCorrectionScopeAndRecreation();
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
