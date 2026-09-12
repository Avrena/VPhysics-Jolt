#include "test_support.h"
#include "vjolt_objectpairhash.h"
#include <cstdio>
#include <vector>

static int checks = 0, failures = 0;
static void Check(bool condition, const char *message)
{
    ++checks;
    if (!condition)
    {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

int main()
{
    // Three distinct, live pointer values in the production hash's same
    // 1024-way bucket. Works with both identity and mixed std::hash<void*>.
    std::array<char, 4096> storage{};
    std::array<std::vector<void *>, 1024> buckets;
    std::vector<void *> colliding;
    for (char &token : storage)
    {
        auto &bucket = buckets[std::hash<void *>{}(&token) & 1023];
        bucket.push_back(&token);
        if (bucket.size() == 3)
        {
            colliding = bucket;
            break;
        }
    }
    Check(colliding.size() == 3, "fixture finds three colliding pointer keys");
    if (colliding.size() != 3) return 1;
    void *chassis = colliding[0], *unrelated = colliding[1], *absent = colliding[2];
    std::array<char, 4> wheels{};

    {
        JoltPhysicsObjectPairHash hash;
        hash.AddObjectPair(chassis, &wheels[0]);
        hash.AddObjectPair(unrelated, &wheels[1]);
        void *partners[8]{};
        int count = hash.GetPairListForObject(chassis, 8, partners);
        Check(count == 1, "partner enumeration excludes foreign bucket entries");
        Check(count == 1 && partners[0] == &wheels[0], "enumeration returns only the chassis wheel");
        Check(hash.GetPairListForObject(absent, 8, partners) == 0, "absent colliding key has no partners");
        hash.RemoveAllPairsForObject(absent);
        Check(hash.IsObjectPairInHash(chassis, &wheels[0]), "cleaning an absent colliding object preserves wheel NoCollide");
        Check(hash.IsObjectPairInHash(unrelated, &wheels[1]), "absent-key cleanup preserves all unrelated pairs");
    }
    {
        JoltPhysicsObjectPairHash hash;
        hash.AddObjectPair(chassis, &wheels[0]);
        hash.AddObjectPair(unrelated, &wheels[1]);
        hash.RemoveAllPairsForObject(unrelated);
        Check(!hash.IsObjectPairInHash(unrelated, &wheels[1]), "cleanup removes the requested object's pair");
        Check(hash.IsObjectPairInHash(chassis, &wheels[0]), "cleanup preserves another object's same-bucket pair");
        Check(hash.GetPairCountForObject(chassis) == 1, "foreign cleanup preserves chassis pair count");
    }
    {
        JoltPhysicsObjectPairHash hash;
        hash.AddObjectPair(chassis, &wheels[0]);
        hash.AddObjectPair(chassis, &wheels[1]);
        hash.AddObjectPair(&wheels[0], chassis); // Same unordered pair, not a new reference.
        Check(hash.GetPairCountForObject(chassis) == 2, "duplicate addition does not inflate count");
        hash.RemoveObjectPair(chassis, &wheels[0]);
        Check(hash.GetPairCountForObject(chassis) == 1, "removing one pair decrements only one reference");
        Check(hash.IsObjectInHash(chassis), "object membership survives while another pair remains");
        Check(hash.IsObjectPairInHash(chassis, &wheels[1]), "remaining pair survives single-pair removal");
        Check(!hash.IsObjectInHash(&wheels[0]), "removed sole partner is no longer a member");
        hash.RemoveObjectPair(chassis, &wheels[0]);
        Check(hash.GetPairCountForObject(chassis) == 1, "repeated removal is a no-op");
        void *partners[2] = {nullptr, &wheels[3]};
        Check(hash.GetPairListForObject(chassis, 0, partners) == 0 && !partners[0], "zero capacity does not write");
        Check(hash.GetPairListForObject(chassis, 1, partners) == 1 && partners[0] == &wheels[1]
            && partners[1] == &wheels[3], "bounded enumeration respects capacity");
        hash.RemoveAllPairsForObject(chassis);
        Check(!hash.IsObjectInHash(chassis) && !hash.IsObjectInHash(&wheels[1]), "final cleanup removes both memberships");
        Check(!hash.IsObjectPairInHash(chassis, &wheels[1]), "final cleanup removes pair");
    }
    {
        JoltPhysicsObjectPairHash hash;
        hash.AddObjectPair(chassis, unrelated); // Both ends share the object bucket.
        hash.AddObjectPair(chassis, &wheels[0]);
        hash.AddObjectPair(unrelated, &wheels[1]);
        hash.RemoveAllPairsForObject(chassis);
        Check(!hash.IsObjectPairInHash(chassis, unrelated), "same-bucket endpoint pair is removed safely");
        Check(!hash.IsObjectPairInHash(chassis, &wheels[0]), "all requested pairs are removed");
        Check(hash.IsObjectPairInHash(unrelated, &wheels[1]), "partner's other pair survives cleanup");
        Check(hash.GetPairCountForObject(unrelated) == 1, "partner's reference count remains consistent");
    }
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
