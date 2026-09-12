#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <unordered_set>
#include <utility>

// Interface-only stand-in for the SDK PCH. This test checks implementation
// behavior, not the engine ABI. Keep signatures identical to object_hash.h.
class IPhysicsObjectPairHash
{
public:
    virtual ~IPhysicsObjectPairHash() = default;
    virtual void AddObjectPair(void *, void *) = 0;
    virtual void RemoveObjectPair(void *, void *) = 0;
    virtual bool IsObjectPairInHash(void *, void *) = 0;
    virtual void RemoveAllPairsForObject(void *) = 0;
    virtual bool IsObjectInHash(void *) = 0;
    virtual int GetPairCountForObject(void *) = 0;
    virtual int GetPairListForObject(void *, int, void **) = 0;
};

template<typename T, typename Value>
constexpr bool Contains(const T &container, const Value &value)
{
    return container.find(value) != container.end();
}
