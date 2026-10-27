#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <tuple>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <vector>

using uint32 = uint32_t;
template<class T> T Min(T a, T b) { return std::min(a, b); }
template<class T> T Max(T a, T b) { return std::max(a, b); }
struct Vector
{
    float x, y, z;
    Vector(float a=0, float b=0, float c=0) : x(a), y(b), z(c) {}
    Vector operator-() const { return {-x,-y,-z}; }
    Vector operator-(Vector b) const { return {x-b.x,y-b.y,z-b.z}; }
    float Dot(Vector b) const { return x*b.x+y*b.y+z*b.z; }
};
const Vector vec3_origin;
namespace SourceToJolt { JPH::Vec3 Distance(Vector v) { return JPH::Vec3(v.x,v.y,v.z)*0.0254f; } }
namespace JoltToSource { Vector Distance(JPH::Vec3 v) { return {v.GetX()/0.0254f,v.GetY()/0.0254f,v.GetZ()/0.0254f}; } }
enum { CALLBACK_IS_PLAYER_CONTROLLER=1, CALLBACK_GLOBAL_COLLISION=2, CALLBACK_GLOBAL_COLLIDE_STATIC=4, CALLBACK_SHADOW_COLLISION=8 };
struct JoltPhysicsObject
{
    JPH::Body *body;
    int flags=CALLBACK_GLOBAL_COLLISION|CALLBACK_GLOBAL_COLLIDE_STATIC;
    Vector playerVelocity;
    JPH::BodyID GetBodyID() const { return body->GetID(); }
    JPH::Body *GetBody() const { return body; }
    int GetCallbackFlags() const { return flags; }
    bool IsStatic() const { return body->IsStatic(); }
    int GetMaterialIndex() const { return 0; }
    Vector GetPlayerDrivenVelocity() const { return playerVelocity; }
    void GetVelocity(Vector *out, void *) const { *out=JoltToSource::Distance(body->GetLinearVelocity()); }
};
struct IPhysicsCollisionData
{
    virtual void GetSurfaceNormal(Vector &)=0;
    virtual void GetContactPoint(Vector &)=0;
    virtual void GetContactSpeed(Vector &)=0;
};
struct vcollisionevent_t
{
    JoltPhysicsObject *pObjects[2]; int surfaceProps[2]; bool isCollision, isShadowCollision;
    float deltaCollisionTime, collisionSpeed; IPhysicsCollisionData *pInternalData;
};
#include "events.inc"

class BroadPhase final : public JPH::BroadPhaseLayerInterface
{
    JPH::uint GetNumBroadPhaseLayers() const override { return 1; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer) const override { return JPH::BroadPhaseLayer(0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "test"; }
#endif
};
class ObjectBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
{ bool ShouldCollide(JPH::ObjectLayer,JPH::BroadPhaseLayer) const override { return true; } };
class ObjectPairs final : public JPH::ObjectLayerPairFilter
{ bool ShouldCollide(JPH::ObjectLayer,JPH::ObjectLayer) const override { return true; } };

int main()
{
    JPH::RegisterDefaultAllocator(); JPH::Factory::sInstance=new JPH::Factory; JPH::RegisterTypes();
    int checks=0, failed=0;
    auto check=[&](bool ok,const char *name) { ++checks; if(!ok) { ++failed; std::printf("FAIL %s\n",name); } };
    auto near=[](float a,float b) { return std::fabs(a-b)<0.002f; };
    {
        BroadPhase broad; ObjectBroadPhase ob; ObjectPairs pairs; JPH::PhysicsSystem system;
        system.Init(16,0,64,64,broad,ob,pairs);
        auto &bi=system.GetBodyInterface();
        auto make=[&](JPH::EMotionType type) {
            JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(1,1,1)),JPH::RVec3::sZero(),JPH::Quat::sIdentity(),type,0);
            return bi.CreateBody(settings);
        };
        JoltPhysicsObject a{make(JPH::EMotionType::Dynamic)}, b{make(JPH::EMotionType::Static)};
        JPH::ContactManifold m;
        m.mBaseOffset=JPH::RVec3::sZero(); m.mWorldSpaceNormal=JPH::Vec3(0,0,-1);
        m.mRelativeContactPointsOn1.push_back(JPH::Vec3::sZero());
        m.mRelativeContactPointsOn2.push_back(JPH::Vec3::sZero());
        a.body->SetLinearVelocity(SourceToJolt::Distance({600,0,-900}));
        Contract::JoltPhysicsCollisionInfo info(&a,&b,m);
        Contract::JoltPhysicsCollisionEvent event(info,1);
        a.body->SetLinearVelocity(JPH::Vec3::sZero());
        Vector hit; event.m_Data.GetContactSpeed(hit);
        check(near(event.m_Event.collisionSpeed,900),"normal approach survives solver velocity changes");
        check(near(std::sqrt(hit.Dot(hit)),std::sqrt(600.0f*600+900.0f*900)),"oblique contact velocity retains tangent");
        check(near(JoltToSource::Distance(event.m_Data.GetObject1PreCollisionVelocity()).z,-900),"pre-collision COM snapshot");
        auto copy=event; auto moved=std::move(copy);
        check(moved.m_Event.pInternalData==&moved.m_Data,"copy/move owns internal callback data");
        a.body->SetAngularVelocity(JPH::Vec3(0,1,0));
        m.mRelativeContactPointsOn1[0]=JPH::Vec3(1,0,0);
        Contract::JoltPhysicsCollisionEvent angular({&a,&b,m},1);
        check(near(angular.m_Event.collisionSpeed,1/0.0254f),"angular point velocity contributes to approach");
        m.mRelativeContactPointsOn1[0]=JPH::Vec3(-1,0,0);
        m.mRelativeContactPointsOn1.push_back(JPH::Vec3(1,0,0));
        m.mRelativeContactPointsOn2.push_back(JPH::Vec3(1,0,0));
        Contract::JoltPhysicsCollisionInfo touch(&a,&b,m);
        check(touch.m_ContactPoint.x<0,"touch and friction retain the first manifold point");
        Contract::JoltPhysicsCollisionEvent edge({&a,&b,m,true},1);
        Vector point; edge.m_Data.GetContactPoint(point);
        check(near(edge.m_Event.collisionSpeed,1/0.0254f) && point.x>0,"approaching edge selected over separating first point");
        m.mRelativeContactPointsOn1.pop_back(); m.mRelativeContactPointsOn2.pop_back();
        a.body->SetLinearVelocity(SourceToJolt::Distance({0,0,900}));
        a.body->SetAngularVelocity(JPH::Vec3::sZero());
        Contract::JoltPhysicsCollisionEvent separating({&a,&b,m},1);
        check(separating.m_Event.collisionSpeed==0,"separating contact has no approach energy");
        a.flags|=CALLBACK_IS_PLAYER_CONTROLLER; a.playerVelocity={0,0,-50};
        a.body->SetLinearVelocity(SourceToJolt::Distance({0,0,-5000}));
        Contract::JoltPhysicsCollisionEvent player({&a,&b,m},1);
        a.flags=0; a.playerVelocity={};
        check(near(player.m_Event.collisionSpeed,50),"player correction spike excluded at contact time");
        check(player.m_Data.IsObject1PlayerDriven(),"player flag survives controller detach");
        check(near(JoltToSource::Distance(player.m_Data.GetObject1PreCollisionVelocity()).z,-50),"player velocity survives controller reset");
        Contract contract;
        check(contract.StampPairCollisionTime(&a,&b)==1,"first pair event delta is one");
        check(contract.StampPairCollisionTime(&b,&a)==0,"canonical pair repeated in same frame");
        contract.AdvanceSimulationTime(0.25f);
        check(near(contract.StampPairCollisionTime(&a,&b),0.25f),"repeat event delta");
        contract.AdvanceSimulationTime(10);
        check(contract.StampPairCollisionTime(&a,&b)==1,"delta capped at one second");
        contract.m_nSimulationTimeUs=uint64_t(10)*365*86400*1000000;
        contract.StampPairCollisionTime(&a,&b); contract.AdvanceSimulationTime(1/22.0f);
        check(near(contract.StampPairCollisionTime(&a,&b),1/22.0f),"long uptime retains tick precision");
        Contract::JoltPhysicsEventTracker<int> tracker;
        for(uint32 i : {0u,31u,32u,47u,63u}) tracker.EmplaceBack(i,int(i));
        std::vector<int> seen;
        tracker.ForEach<true>([&](int value){seen.push_back(value);});
        check(seen==std::vector<int>({0,31,32,47,63}),"all 64 event queue slots drain exactly once");
        seen.clear(); tracker.ForEach<false>([&](int value){seen.push_back(value);});
        check(seen.empty(),"drained tracker is empty");
        Contract::JoltPhysicsCollisionEvent later(info,2);
        check(event.SortKey()<later.SortKey(),"substep ordering precedes worker ordering");
        check(!Contract::IsOrdinaryImpact(0,0) && !Contract::IsOrdinaryImpact(-100,0),"zero floor rejects stationary and separating contacts");
        check(!Contract::IsOrdinaryImpact(0.005f,0) && Contract::IsOrdinaryImpact(0.02f,0),"stationary epsilon separates numerical noise from approach");
        check(!Contract::IsOrdinaryImpact(49,50) && Contract::IsOrdinaryImpact(50,50),"configured floor remains an additional bound");
        check(!Contract::IsOrdinaryImpact(13.7f,20) && Contract::IsOrdinaryImpact(20,20),"configured floor rejects typical substep gravity approach");
        const float nan=std::numeric_limits<float>::quiet_NaN(), inf=std::numeric_limits<float>::infinity();
        Contract::JoltPhysicsEventTracker<Contract::JoltPhysicsCollisionEvent> invalidQueue;
        invalidQueue.EmplaceBack(0,info,1);
        for(float bad : {nan,inf,-inf})
        {
            a.body->SetLinearVelocity(JPH::Vec3(bad,0,0));
            Contract::JoltPhysicsCollisionInfo invalid(&a,&b,m);
            invalidQueue.EmplaceBack(63,invalid,1);
            check(!invalid.IsFinite(),"non-finite body velocity rejected at snapshot");
            check(!Contract::IsOrdinaryImpact(bad,0),"non-finite approach cannot pass gate");
        }
        std::vector<Contract::JoltPhysicsCollisionEvent*> admitted;
        invalidQueue.CollectValid(admitted);
        std::sort(admitted.begin(),admitted.end(),[](auto *x,auto *y){return x->SortKey()<y->SortKey();});
        int dispatched=0;
        for(auto *e:admitted) { check(e->IsValidForDispatch(),"dispatch receives only finite event data"); ++dispatched; }
        check(dispatched==1,"queue filtering drops NaN and infinity before dispatch");
        std::vector<uint32_t> totalKeys;
        for(float f : {nan,inf,-inf,-0.0f,0.0f,-100.0f,100.0f}) totalKeys.push_back(Contract::FloatOrderKey(f));
        std::sort(totalKeys.begin(),totalKeys.end());
        check(std::is_sorted(totalKeys.begin(),totalKeys.end()),"integer float keys remain ordered with NaN and infinity");
        auto garbage=event;
        garbage.m_Event.collisionSpeed=nan;
        check(!garbage.IsValidForDispatch(),"corrupt event speed rejected before sorting");
        bi.DestroyBody(a.GetBodyID()); bi.DestroyBody(b.GetBodyID());
    }
    JPH::UnregisterTypes(); delete JPH::Factory::sInstance; JPH::Factory::sInstance=nullptr;
    std::printf("collision events: %d checks, %d failures\n",checks,failed);
    return failed!=0;
}
