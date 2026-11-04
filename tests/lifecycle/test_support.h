#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define GAME_GMOD 1
#define VJoltAssertMsg(condition, message) do { if (!(condition)) ++testAssertions; } while (false)
constexpr unsigned CALLBACK_MARKED_FOR_DELETE = 0x20;
using uint64 = std::uint64_t;
inline int testAssertions = 0;
inline int testFailures = 0;
inline int testChecks = 0;
inline int nextObjectID = 0;
inline int nextConstraintID = 0;
inline std::unordered_map<int, int> objectDestructions;
inline std::unordered_map<int, int> constraintDestructions;
inline std::vector<std::string> events;

inline void Check(bool condition, const char *message)
{
    ++testChecks;
    if (!condition) { ++testFailures; std::fprintf(stderr, "FAIL: %s\n", message); }
}

template<typename T, typename V> bool VectorContains(const T &values, const V &value)
{ return std::find(values.begin(), values.end(), value) != values.end(); }
template<typename T, typename V> void Erase(T &values, const V &value)
{ values.erase(std::remove(values.begin(), values.end(), value), values.end()); }

template<typename T> struct CUtlVector
{
    std::vector<T> values;
    int Count() const { return int(values.size()); }
    T &operator[](int index) { return values.at(std::size_t(index)); }
    void AddToTail(T value) { values.push_back(value); }
    void Remove(int index) { values.erase(values.begin() + index); }
    void FindAndRemove(T value) { auto it=std::find(values.begin(),values.end(),value); if(it!=values.end()) values.erase(it); }
};

class JoltPhysicsObject;
class JoltPhysicsConstraint;
class JoltPhysicsEnvironment;
struct CPhysCollide { bool alive = true; int destroys = 0; };
struct IPhysicsObject
{
    int testID = ++nextObjectID;
    virtual ~IPhysicsObject() { ++objectDestructions[testID]; events.push_back("object:"+std::to_string(testID)); }
    virtual void Wake() {}
};
struct IPhysicsConstraint
{
    int testID = ++nextConstraintID;
    virtual ~IPhysicsConstraint() { ++constraintDestructions[testID]; events.push_back("constraint:"+std::to_string(testID)); }
};
struct IPhysicsEnvironment { virtual ~IPhysicsEnvironment() = default; };
struct IJoltObjectDestroyedListener
{
    virtual ~IJoltObjectDestroyedListener() = default;
    virtual void OnJoltPhysicsObjectDestroyed(JoltPhysicsObject *) = 0;
};
struct IGModPhysicsObjectEvent
{
    std::function<void(JoltPhysicsObject *)> callback;
    void ObjectDestroyed(JoltPhysicsObject *object) { if(callback) callback(object); }
};
struct IPhysicsConstraintEvent
{
    std::function<void(JoltPhysicsConstraint *)> callback;
    void ConstraintBroken(JoltPhysicsConstraint *constraint) { if(callback) callback(constraint); }
};
struct JoltPhysicsDragController {};
struct JoltPhysicsConstraintGroup { void RemoveConstraint(JoltPhysicsConstraint *) {} };

namespace JPH
{
struct BodyID
{
    unsigned value = 0;
    bool operator==(const BodyID &other) const { return value == other.value; }
    bool IsInvalid() const { return value == 0; }
};
using BodyIDVector = std::vector<BodyID>;
struct Body
{
    BodyID id;
    uint64 data;
    bool added = true;
    uint64 GetUserData() const { return data; }
    BodyID GetID() const { return id; }
};
inline unsigned nextBodyID = 0;
inline std::map<unsigned, Body *> allBodies;
enum class EActivation { Activate };
struct Constraint { bool enabled = true; void SetEnabled(bool value) { enabled = value; } };
class PhysicsSystem;
class BodyInterface
{
public:
    explicit BodyInterface(PhysicsSystem *system) : owner(system) {}
    PhysicsSystem *owner;
    void RemoveBody(BodyID id);
    void DestroyBody(BodyID id);
    void AddBody(BodyID id, EActivation);
};
class BodyLockInterface
{
public:
    explicit BodyLockInterface(PhysicsSystem *system) : owner(system) {}
    PhysicsSystem *owner;
    Body *TryGetBody(BodyID id) const;
};
class PhysicsSystem
{
public:
    std::map<unsigned, Body *> bodies;
    BodyInterface bodyInterface{this};
    BodyLockInterface lockInterface{this};
    Body *CreateBody(void *object)
    {
        auto *body = new Body{{++nextBodyID}, reinterpret_cast<uint64>(object)};
        bodies.emplace(body->id.value,body); allBodies.emplace(body->id.value,body); return body;
    }
    BodyInterface &GetBodyInterfaceNoLock() { return bodyInterface; }
    const BodyLockInterface &GetBodyLockInterfaceNoLock() const { return lockInterface; }
    void GetBodies(BodyIDVector &out) const { out.clear(); for(auto entry:bodies) out.push_back(entry.second->id); }
    unsigned GetNumBodies() const { return unsigned(bodies.size()); }
    template<typename T> void RemoveStepListener(T *) {}
    void RemoveConstraint(Constraint *) {}
};
inline Body *BodyLockInterface::TryGetBody(BodyID id) const
{ auto it=owner->bodies.find(id.value); return it==owner->bodies.end()?nullptr:it->second; }
inline void BodyInterface::RemoveBody(BodyID id)
{
    Body *body=owner->lockInterface.TryGetBody(id);
    if (!body) throw std::runtime_error("remove missing body");
    body->added=false; events.push_back("remove:"+std::to_string(id.value));
}
inline void BodyInterface::DestroyBody(BodyID id)
{
    Body *body=owner->lockInterface.TryGetBody(id);
    if (!body) throw std::runtime_error("destroy missing body");
    owner->bodies.erase(id.value); allBodies.erase(id.value);
    events.push_back("body:"+std::to_string(id.value)); delete body;
}
inline void BodyInterface::AddBody(BodyID id, EActivation)
{ auto *body=allBodies.at(id.value); body->added=true; owner->bodies.emplace(id.value,body); }
}

struct JoltPhysicsContactListener
{
    std::unordered_set<JoltPhysicsObject *> cachedObjects;
    void InvalidShouldCollideCache(JoltPhysicsObject *object);
};
struct JoltPhysicsCollision
{
    static JoltPhysicsCollision &GetInstance() { static JoltPhysicsCollision value; return value; }
    void DestroyCollide(CPhysCollide *collide)
    {
        if (!collide) return;
        ++collide->destroys; collide->alive=false; events.push_back("collide");
        Check(collide->destroys == 1, "collision data is released exactly once");
    }
};

class JoltPhysicsEnvironment : public IPhysicsEnvironment
{
public:
    JoltPhysicsEnvironment() : m_pDragController(new JoltPhysicsDragController) {}
    ~JoltPhysicsEnvironment() override;
    void DestroyObject(IPhysicsObject *);
    void DestroyConstraint(IPhysicsConstraint *);
    void CleanupDeleteList();
    void EnableDeleteQueue(bool);
    void DestroyCollideOnDeadObjectFlush(CPhysCollide *);
    void RemoveBodyAndDeleteObject(JoltPhysicsObject *);
    void DeleteDeadObjects();
    void DestroyEnvironmentObjects();
    const IPhysicsObject **GetObjectList(int *) const;
    bool TransferObject(IPhysicsObject *, IPhysicsEnvironment *);
    void ObjectTransferHandOver(JoltPhysicsObject *);
    void RegisterConstraint(JoltPhysicsConstraint *);
    void UnregisterConstraint(JoltPhysicsConstraint *);
    void NotifyConstraintDisabled(JoltPhysicsConstraint *);
    void RemoveDirtyStaticBody(const JPH::BodyID &);
    void MarkObjectListDirty() const { m_bObjectListDirty=true; }
    JPH::PhysicsSystem *GetPhysicsSystem() { return &m_PhysicsSystem; }
    JoltPhysicsContactListener *GetContactListener() { return &m_ContactListener; }
    IGModPhysicsObjectEvent *GetGModObjectEvents() { return m_pGModObjectEvents; }

    bool m_bSimulating=false, m_bDeletingDeadObjects=false, m_bDestroyingEnvironmentObjects=false;
    bool m_bEnableDeleteQueue=false, m_bWakeObjectsOnConstraintDeletion=false;
    JPH::PhysicsSystem m_PhysicsSystem;
    JoltPhysicsContactListener m_ContactListener;
    JoltPhysicsDragController *m_pDragController;
    IGModPhysicsObjectEvent *m_pGModObjectEvents=nullptr;
    IPhysicsConstraintEvent *m_pConstraintListener=nullptr;
    bool m_EnableConstraintNotify=false;
    mutable JPH::BodyIDVector m_CachedBodies, m_DirtyStaticBodies;
    mutable std::vector<const IPhysicsObject *> m_CachedObjects;
    mutable unsigned m_nCachedObjectListBodyCount=~0u;
    mutable bool m_bObjectListDirty=true;
    std::vector<JoltPhysicsObject *> m_pDeadObjects;
    std::vector<JoltPhysicsConstraint *> m_pDeadConstraints, m_pConstraints;
    std::vector<CPhysCollide *> m_pDeadObjectCollides;
};

inline std::unordered_set<JoltPhysicsObject *> registry;
inline void UnregisterPhysicsObject(JoltPhysicsObject *object) { registry.erase(object); }

class JoltPhysicsObject : public IPhysicsObject
{
public:
    explicit JoltPhysicsObject(JoltPhysicsEnvironment *environment, CPhysCollide *collide=nullptr)
        : m_pEnvironment(environment), m_pPhysicsSystem(environment->GetPhysicsSystem()), collideData(collide)
    { m_pBody=m_pPhysicsSystem->CreateBody(this); registry.insert(this); }
    ~JoltPhysicsObject() override;
    unsigned GetCallbackFlags() const { return callbackFlags; }
    void AddCallbackFlags(unsigned value) { callbackFlags|=value; }
    JPH::BodyID GetBodyID() const { return m_pBody->GetID(); }
    CPhysCollide *GetCollide() const { return collideData; }
    void AddDestroyedListener(IJoltObjectDestroyedListener *);
    void RemoveDestroyedListener(IJoltObjectDestroyedListener *);
    void ScrubAndClearContactPairs() {}
    void RemoveShadowController() {}
    void RemoveConstraint(JoltPhysicsConstraint *) {}
    void UpdateEnvironment(JoltPhysicsEnvironment *environment)
    { m_pPhysicsSystem->bodies.erase(GetBodyID().value); m_pEnvironment=environment; m_pPhysicsSystem=environment->GetPhysicsSystem(); }
    JoltPhysicsEnvironment *m_pEnvironment;
    JPH::PhysicsSystem *m_pPhysicsSystem;
    JPH::Body *m_pBody;
    unsigned callbackFlags=0;
    CPhysCollide *collideData;
    CUtlVector<IJoltObjectDestroyedListener *> m_destroyedListeners;
};

inline void JoltPhysicsContactListener::InvalidShouldCollideCache(JoltPhysicsObject *object)
{ cachedObjects.erase(object); events.push_back("invalidate:"+std::to_string(object->GetBodyID().value)); }

class JoltPhysicsConstraint : public IPhysicsConstraint, public IJoltObjectDestroyedListener
{
public:
    JoltPhysicsConstraint(JoltPhysicsEnvironment *environment,JoltPhysicsObject *first=nullptr,JoltPhysicsObject *second=nullptr)
        : m_pObjReference(first),m_pObjAttached(second),m_pPhysicsEnvironment(environment),m_pPhysicsSystem(environment->GetPhysicsSystem())
    {
        if(first) first->AddDestroyedListener(this);
        if(second) second->AddDestroyedListener(this);
        environment->RegisterConstraint(this);
    }
    ~JoltPhysicsConstraint() override;
    void DestroyConstraint();
    void OnJoltPhysicsObjectDestroyed(JoltPhysicsObject *) override;
    void Deactivate();
    IPhysicsObject *GetReferenceObject() const;
    IPhysicsObject *GetAttachedObject() const;
    bool MarkForDelete();
    JoltPhysicsObject *m_pObjReference, *m_pObjAttached;
    JoltPhysicsEnvironment *m_pPhysicsEnvironment;
    JPH::PhysicsSystem *m_pPhysicsSystem;
    JoltPhysicsConstraintGroup *m_pGroup=nullptr;
    JPH::Constraint *m_pConstraint=nullptr;
    int m_nLengthSpringWarmupTicks=0;
    bool m_bMarkedForDelete=false;
};
