#include <vector>
#include <cstdio>

// Only engine storage and services are shimmed. The eligibility method below is
// extracted verbatim from vjolt_object.cpp, including every exclusion.
struct IJoltObjectDestroyedListener { virtual ~IJoltObjectDestroyedListener() = default; };
struct StepListener { virtual ~StepListener() = default; };
struct JoltPhysicsDragController : StepListener, IJoltObjectDestroyedListener {};
struct Body { bool dynamic = true; bool IsDynamic() const { return dynamic; } };
struct Environment
{
	JoltPhysicsDragController drag;
	JoltPhysicsDragController *GetDragController() { return &drag; }
};
struct List
{
	std::vector<IJoltObjectDestroyedListener *> values;
	int Count() const { return int(values.size()); }
	auto operator[](int i) const { return values[i]; }
};
enum { FVPHYSICS_PLAYER_HELD = 4, FVPHYSICS_PART_OF_RAGDOLL = 8, FVPHYSICS_MULTIOBJECT_ENTITY = 16 };
struct JoltPhysicsObject
{
	Body *m_pBody;
	Environment *m_pEnvironment;
	void *m_pShadowController = nullptr;
	bool controlled = false;
	unsigned short m_gameFlags = 0;
	std::vector<int> m_pConstraints;
	List m_destroyedListeners;
	bool IsControlledByGame() const { return controlled; }
	bool CanRecoverContactPenetration() const;
};
#include "eligibility.inc"

int main()
{
	int checks = 0, failures = 0;
	auto check = [&](bool ok, const char *name) {
		++checks;
		if (!ok) { ++failures; std::printf("FAIL: %s\n", name); }
	};
	Body body;
	Environment environment;
	JoltPhysicsObject object{&body, &environment};
	check(object.CanRecoverContactPenetration(), "free dynamic object is eligible");
	object.m_destroyedListeners.values.push_back(&environment.drag);
	check(object.CanRecoverContactPenetration(), "passive drag remains eligible");
	IJoltObjectDestroyedListener attachment;
	object.m_destroyedListeners.values.push_back(&attachment);
	check(!object.CanRecoverContactPenetration(), "spring or controller attachment is excluded");
	object.m_destroyedListeners.values.pop_back();
	check(object.CanRecoverContactPenetration(), "removing the attachment restores eligibility");
	object.m_pConstraints.push_back(1);
	check(!object.CanRecoverContactPenetration(), "constraint is excluded");
	object.m_pConstraints.clear();
	check(object.CanRecoverContactPenetration(), "removing the constraint restores eligibility");
	body.dynamic = false;
	check(!object.CanRecoverContactPenetration(), "static and kinematic bodies are excluded");
	body.dynamic = true;
	object.controlled = true;
	check(!object.CanRecoverContactPenetration(), "game and player control are excluded");
	object.controlled = false;
	object.m_pShadowController = &body;
	check(!object.CanRecoverContactPenetration(), "physical and kinematic shadows are excluded");
	object.m_pShadowController = nullptr;
	for (unsigned short flag : {FVPHYSICS_PLAYER_HELD, FVPHYSICS_PART_OF_RAGDOLL, FVPHYSICS_MULTIOBJECT_ENTITY})
	{
		object.m_gameFlags = flag;
		check(!object.CanRecoverContactPenetration(), "held, ragdoll and multiobject flags are excluded");
	}
	object.m_gameFlags = 0;
	check(object.CanRecoverContactPenetration(), "clearing excluded flags restores eligibility");
	std::printf("%d checks, %d failures\n", checks, failures);
	return failures != 0;
}
