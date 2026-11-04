#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <dlfcn.h>
#include <initializer_list>
#include "vphysics_interface.h"

// Uses the pinned SDK's actual IPhysics interfaces and the actual loaded backend.
// No engine classes or physics methods are mocked. There is no game/Lua context.
static void Require(bool ok, const char *message)
{
    if (!ok) { std::fprintf(stderr, "%s\n", message); std::exit(1); }
}

int main(int argc, char **argv)
{
    Require(argc == 2, "usage: probe_lifecycle backend.so");
    void *module = dlopen(argv[1], RTLD_NOW | RTLD_GLOBAL);
    if (!module) { std::fprintf(stderr, "dlopen: %s\n", dlerror()); return 1; }
    auto factory = reinterpret_cast<CreateInterfaceFn>(dlsym(module, "CreateInterface"));
    Require(factory != nullptr, "missing CreateInterface");
    for (const char *name : {VPHYSICS_INTERFACE_VERSION, VPHYSICS_INTERFACE_VERSION_GMOD,
        VPHYSICS_COLLISION_INTERFACE_VERSION, VPHYSICS_SURFACEPROPS_INTERFACE_VERSION})
    {
        int status = -1;
        Require(factory(name, &status) != nullptr && status == 0, "interface factory failed");
        std::printf("%s factory OK\n", name);
    }
    auto *physics = static_cast<IPhysics *>(factory(VPHYSICS_INTERFACE_VERSION, nullptr));

    // Init explicitly supports absent g_pCVar; no synthetic engine factory is supplied.
    Require(physics->Init() == INIT_OK, "backend Init failed");
    std::puts("backend Init OK");
    for (int iteration = 0; iteration < 16; ++iteration)
    {
        IPhysicsEnvironment *environment = physics->CreateEnvironment();
        Require(environment != nullptr, "CreateEnvironment failed");
        environment->SetGravity(Vector(0, 0, -600));
        objectparams_t parameters{};
        parameters.mass = 10;
        parameters.inertia = 1;
        parameters.rotInertiaLimit = 0.05f;
        parameters.pName = "qualification sphere";
        parameters.enableCollisions = true;
        IPhysicsObject *bodies[32]{};
        for (int i = 0; i < 32; ++i)
        {
            bodies[i] = environment->CreateSphereObject(4, 0, Vector(i * 12, 0, 128),
                QAngle(0, 0, 0), &parameters, false);
            Require(bodies[i] && physics->IsValidPhysicsObject(bodies[i]), "sphere creation/registry failed");
            bodies[i]->Wake();
        }
        for (int step = 0; step < 4; ++step) environment->Simulate(1.0f / 66.0f);
        Vector position;
        bodies[0]->GetPosition(&position, nullptr);
        Require(std::isfinite(position.z) && position.z < 128, "sphere did not integrate finite gravity motion");
        for (auto *body : bodies) environment->DestroyObject(body);
        environment->CleanupDeleteList();
        for (auto *body : bodies)
            Require(!physics->IsValidPhysicsObject(body), "destroyed sphere remains registered");
        physics->DestroyEnvironment(environment);
        Require(physics->GetActiveEnvironmentByIndex(0) == nullptr, "environment registry not empty");
    }
    physics->Shutdown();
    std::puts("16 environments / 512 sphere lifecycles / 64 simulation steps / Shutdown OK");
    dlclose(module);
}
