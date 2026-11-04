#include "test_support.h"

struct Listener final : IJoltObjectDestroyedListener
{
    std::function<void(JoltPhysicsObject *)> callback;
    explicit Listener(std::function<void(JoltPhysicsObject *)> value) : callback(std::move(value)) {}
    void OnJoltPhysicsObjectDestroyed(JoltPhysicsObject *object) override { callback(object); }
};

static void Simulation()
{
    JoltPhysicsEnvironment env;
    auto *object=new JoltPhysicsObject(&env);
    const int id=object->testID;
    env.m_bSimulating=true;
    env.DestroyObject(object);
    env.CleanupDeleteList();
    Check(objectDestructions[id]==0,"CleanupDeleteList retains wrappers during simulation/callback dispatch");
    Check(env.m_pDeadObjects.size()==1,"marked wrapper remains queued until simulation ends");
    env.m_bSimulating=false;
    env.CleanupDeleteList();
    Check(objectDestructions[id]==1,"wrapper is deleted once at the safe boundary");
}

static void DeferredObjects()
{
    JoltPhysicsEnvironment env;
    env.EnableDeleteQueue(true);
    auto *object=new JoltPhysicsObject(&env);
    const int id=object->testID;
    env.DestroyObject(object);
    env.DestroyObject(object);
    Check(testAssertions==0,"repeated deletion of a still-live marked wrapper is a no-op");
    Check(env.m_pDeadObjects.size()==1,"duplicate object request does not append a second entry");
    Check(objectDestructions[id]==0,"enabled delete queue defers object destruction");
    env.CleanupDeleteList();
    Check(objectDestructions[id]==1,"queued object is destroyed exactly once");
}

static void QueueGrowth()
{
    JoltPhysicsEnvironment env;
    env.EnableDeleteQueue(true);
    auto *first=new JoltPhysicsObject(&env);
    auto *second=new JoltPhysicsObject(&env);
    IGModPhysicsObjectEvent handler;
    env.m_pGModObjectEvents=&handler;
    bool appended=false;
    handler.callback=[&](JoltPhysicsObject *) {
        if(appended) return;
        appended=true;
        // Two entries were already pending, so the original range iterator
        // reads freed storage after this forces vector reallocation.
        for(int i=0;i<512;++i) env.DestroyObject(new JoltPhysicsObject(&env));
    };
    env.DestroyObject(first); env.DestroyObject(second);
    env.CleanupDeleteList();
    Check(objectDestructions.size()==514,"one cleanup drains objects enqueued by destruction callbacks");
    Check(env.m_PhysicsSystem.GetNumBodies()==0,"queue growth leaves no live body behind");
    env.m_pGModObjectEvents=nullptr;
}

static void NestedCleanup()
{
    JoltPhysicsEnvironment env;
    auto *object=new JoltPhysicsObject(&env);
    const int id=object->testID;
    IGModPhysicsObjectEvent handler;
    env.m_pGModObjectEvents=&handler;
    bool nested=false;
    handler.callback=[&](JoltPhysicsObject *current) {
        if(nested) return;
        nested=true;
        env.DestroyObject(current);
        env.CleanupDeleteList();
        Check(objectDestructions[id]==0,"nested cleanup cannot finish an active object destructor");
        env.DestroyObject(new JoltPhysicsObject(&env));
    };
    env.EnableDeleteQueue(true);
    env.DestroyObject(object);
    env.CleanupDeleteList();
    Check(objectDestructions[id]==1,"recursive cleanup never double-destroys its current object");
    Check(env.m_PhysicsSystem.GetNumBodies()==0,"outer cleanup drains nested deletion requests");
    env.m_pGModObjectEvents=nullptr;
}

static void Constraints()
{
    JoltPhysicsEnvironment env;
    auto *constraint=new JoltPhysicsConstraint(&env);
    const int id=constraint->testID;
    env.m_bSimulating=true;
    env.DestroyConstraint(constraint); env.DestroyConstraint(constraint);
    Check(env.m_pDeadConstraints.size()==1,"repeated constraint request is coalesced");
    env.m_bSimulating=false;
    env.CleanupDeleteList();
    Check(constraintDestructions[id]==1,"deferred constraint is destroyed once");

    env.EnableDeleteQueue(true);
    auto *object=new JoltPhysicsObject(&env);
    const int objectID=object->testID;
    env.DestroyObject(object);
    auto *immediate=new JoltPhysicsConstraint(&env);
    const int immediateID=immediate->testID;
    env.DestroyConstraint(immediate);
    Check(constraintDestructions[immediateID]==1,"object queue alone does not defer constraint deletion");
    Check(objectDestructions[objectID]==0,"deleting a constraint does not flush the enabled object queue");
    env.CleanupDeleteList();
}

static void ListenerMutation()
{
    JoltPhysicsEnvironment env;
    auto *object=new JoltPhysicsObject(&env);
    int remainingCalls=0;
    Listener remaining([&](JoltPhysicsObject *) { ++remainingCalls; });
    object->AddDestroyedListener(&remaining);
    auto *first=new JoltPhysicsConstraint(&env,object);
    auto *second=new JoltPhysicsConstraint(&env,object);
    IPhysicsConstraintEvent handler;
    env.m_pConstraintListener=&handler; env.m_EnableConstraintNotify=true;
    int brokenCalls=0;
    handler.callback=[&](JoltPhysicsConstraint *constraint) {
        ++brokenCalls;
        // Real OnJoltPhysicsObjectDestroyed calls the game ConstraintBroken
        // handler. Removing another listener here invalidated the old index.
        if(constraint==second) {
            env.DestroyConstraint(first);
            env.DestroyConstraint(second);
        }
    };
    env.DestroyObject(object);
    Check(remainingCalls==1,"remaining listener is notified once after another listener is removed");
    Check(brokenCalls==1,"removed constraint listener is not called later");
    Check(env.m_pConstraints.empty(),"constraints deleted by the game callback are drained");
    env.m_pConstraintListener=nullptr;
}

static void CollideLifetime()
{
    JoltPhysicsEnvironment env;
    CPhysCollide collide;
    auto *object=new JoltPhysicsObject(&env,&collide);
    IGModPhysicsObjectEvent handler;
    env.m_pGModObjectEvents=&handler;
    handler.callback=[&](JoltPhysicsObject *) {
        Check(collide.alive,"collision metadata remains alive throughout object destruction callbacks");
        env.DestroyCollideOnDeadObjectFlush(&collide);
        Check(collide.alive,"destroy-collide request inside callback is deferred");
    };
    env.EnableDeleteQueue(true);
    env.DestroyObject(object);
    env.DestroyCollideOnDeadObjectFlush(&collide);
    env.DestroyCollideOnDeadObjectFlush(&collide);
    env.CleanupDeleteList();
    Check(collide.destroys==1,"duplicate collision release requests are coalesced");
    env.m_pGModObjectEvents=nullptr;
}

static void Teardown()
{
    CPhysCollide collide;
    IGModPhysicsObjectEvent handler;
    int callbacks=0, createdID=0;
    auto *env=new JoltPhysicsEnvironment;
    env->m_pGModObjectEvents=&handler;
    env->EnableDeleteQueue(true);
    auto *first=new JoltPhysicsObject(env,&collide);
    auto *helper=env->m_PhysicsSystem.CreateBody(first);
    // Model the player's extra Jolt body, sharing user data with its wrapper.
    Listener releaseHelper([&](JoltPhysicsObject *) {
        env->m_PhysicsSystem.GetBodyInterfaceNoLock().DestroyBody(helper->GetID());
    });
    first->AddDestroyedListener(&releaseHelper);
    for(int i=0;i<64;++i) env->DestroyObject(new JoltPhysicsObject(env,&collide));
    auto *publicMarked=new JoltPhysicsObject(env,&collide);
    publicMarked->AddCallbackFlags(CALLBACK_MARKED_FOR_DELETE);
    new JoltPhysicsObject(env,&collide);
    env->DestroyObject(first);
    env->DestroyCollideOnDeadObjectFlush(&collide);
    handler.callback=[&](JoltPhysicsObject *) {
        ++callbacks;
        int count=0;
        env->GetObjectList(&count);
        Check(count>0,"GetObjectList is safe from a destruction callback");
        Check(collide.alive,"teardown retains collide metadata through every body batch");
        if(callbacks==1) {
            auto *created=new JoltPhysicsObject(env,&collide);
            createdID=created->testID;
        }
    };
    delete env;
    Check(callbacks==68,"teardown deletes queued, live, publicly marked and newly created wrappers once");
    Check(objectDestructions[createdID]==1,"teardown discovers wrappers created by callbacks");
    Check(JPH::allBodies.empty(),"teardown removes wrapper and helper bodies");
    Check(collide.destroys==1,"teardown releases shared collide after all wrappers");
}

static void TeardownSnapshot()
{
    IGModPhysicsObjectEvent handler;
    auto *env=new JoltPhysicsEnvironment;
    env->m_pGModObjectEvents=&handler;
    for(int i=0;i<4;++i) new JoltPhysicsObject(env);
    int callbacks=0;
    handler.callback=[&](JoltPhysicsObject *) {
        ++callbacks;
        int count=0;
        env->GetObjectList(&count);
        Check(count>0,"teardown callback can refresh the public body cache");
    };
    delete env;
    Check(callbacks==4,"public cache refresh cannot skip or revisit teardown snapshot bodies");
    Check(JPH::allBodies.empty(),"teardown snapshot visits every body");
}

static void CacheInvalidation()
{
    JoltPhysicsEnvironment env;
    auto *object=new JoltPhysicsObject(&env);
    const unsigned id=object->GetBodyID().value;
    env.m_ContactListener.cachedObjects.insert(object);
    IGModPhysicsObjectEvent handler;
    env.m_pGModObjectEvents=&handler;
    handler.callback=[&](JoltPhysicsObject *current) {
        Check(!registry.count(current),"validity registry is invalidated before the game destruction callback");
        Check(!env.m_ContactListener.cachedObjects.count(current),"old collision decisions are gone before destruction callbacks");
        env.m_ContactListener.cachedObjects.insert(current);
    };
    env.DestroyObject(object);
    Check(env.m_ContactListener.cachedObjects.empty(),"callback-created cache entries are removed before BodyID reuse");
    auto first=std::find(events.begin(),events.end(),"invalidate:"+std::to_string(id));
    auto removed=std::find(events.begin(),events.end(),"remove:"+std::to_string(id));
    Check(first<removed,"initial cache invalidation precedes RemoveBody");
    auto destroyed=std::find(events.begin(),events.end(),"body:"+std::to_string(id));
    Check(destroyed!=events.begin() && *(destroyed-1)=="invalidate:"+std::to_string(id),"final invalidation immediately precedes DestroyBody");
    env.m_pGModObjectEvents=nullptr;
}

static void Transfer()
{
    JoltPhysicsEnvironment source,destination;
    auto *object=new JoltPhysicsObject(&source);
    source.m_ContactListener.cachedObjects.insert(object);
    Check(source.TransferObject(object,&destination),"transfer succeeds through the existing production path");
    Check(source.m_ContactListener.cachedObjects.empty(),"transfer invalidates source environment collision decisions");
    Check(object->m_pEnvironment==&destination,"transfer still changes object environment");
    destination.DestroyObject(object);
}

int main(int argc,char **argv)
{
    if(argc!=2) return 2;
    const std::map<std::string,void(*)()> scenarios={
        {"simulation",Simulation},{"deferred_objects",DeferredObjects},{"queue_growth",QueueGrowth},
        {"nested_cleanup",NestedCleanup},{"constraints",Constraints},{"listener_mutation",ListenerMutation},
        {"collide_lifetime",CollideLifetime},{"teardown",Teardown},{"teardown_snapshot",TeardownSnapshot},
        {"cache_invalidation",CacheInvalidation},{"transfer",Transfer}};
    auto it=scenarios.find(argv[1]); if(it==scenarios.end()) return 2;
    try { it->second(); }
    catch(const std::exception &error) { std::fprintf(stderr,"FAIL: %s\n",error.what()); return 1; }
    Check(registry.empty(),"scenario leaves no live physics wrapper");
    std::printf("%s: %d checks, %d failures\n",argv[1],testChecks,testFailures);
    return testFailures?1:0;
}
