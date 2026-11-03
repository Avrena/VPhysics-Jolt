#pragma once

// Adapted from schweiziske/GMod-Jolt-Physics at a31992de4e801fdcc3ba8aacae5de668129f4283.
// See LICENSES/GMod-Jolt-Physics-MIT.txt for the upstream licence notice.
// Keep the mutation-epoch contract, with indexed endpoint removal and bounded
// admission so a shared world/high-degree body cannot make cleanup quadratic.
#include "ankerl/unordered_dense.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <utility>
#include <vector>

class VJoltShouldCollideCache final
{
public:
	struct Lookup
	{
		std::uint64_t pairKey = 0;
		std::uint64_t mutationEpoch = 0;
		bool found = false;
		bool shouldCollide = false;
	};

	// At capacity, still return the game's answer but do not cache a new pair.
	// Invalidating/deleting bodies makes space without evicting unrelated pairs.
	explicit VJoltShouldCollideCache( std::size_t maxEntries = 100000 )
		: m_MaxEntries( std::min( maxEntries, std::size_t( std::numeric_limits<std::uint32_t>::max() ) ) )
	{
	}

	Lookup Find( std::uint32_t bodyKey0, std::uint32_t bodyKey1 ) const
	{
		if ( bodyKey1 < bodyKey0 )
			std::swap( bodyKey0, bodyKey1 );
		Lookup result;
		result.pairKey = ( std::uint64_t( bodyKey0 ) << 32 ) | bodyKey1;
		std::shared_lock<std::shared_mutex> lock( m_Mutex );
		result.mutationEpoch = m_MutationEpoch;
		const auto entry = m_Entries.find( result.pairKey );
		if ( entry != m_Entries.end() )
		{
			result.found = true;
			result.shouldCollide = entry->second.shouldCollide;
		}
		return result;
	}

	bool StoreIfUnchanged( const Lookup &lookup, bool shouldCollide )
	{
		std::unique_lock<std::shared_mutex> lock( m_Mutex );
		// RecheckCollisionFilter can run inside the game callback, including for
		// a pair not yet cached. Never repopulate its result after invalidation.
		if ( lookup.mutationEpoch != m_MutationEpoch )
			return false;
		const auto existing = m_Entries.find( lookup.pairKey );
		if ( existing != m_Entries.end() )
		{
			existing->second.shouldCollide = shouldCollide;
			return true;
		}
		if ( m_Entries.size() >= m_MaxEntries )
			return false;

		auto entry = m_Entries.emplace( lookup.pairKey, Entry{} ).first;
		entry->second.shouldCollide = shouldCollide;
		const std::uint32_t bodyA = BodyA( lookup.pairKey ), bodyB = BodyB( lookup.pairKey );
		entry->second.slotA = AddEndpoint( bodyA, lookup.pairKey );
		entry->second.slotB = bodyA == bodyB ? entry->second.slotA : AddEndpoint( bodyB, lookup.pairKey );
		return true;
	}

	void Invalidate( std::uint32_t bodyKey )
	{
		std::unique_lock<std::shared_mutex> lock( m_Mutex );
		++m_MutationEpoch;
		auto object = m_ObjectPairs.find( bodyKey );
		if ( object == m_ObjectPairs.end() )
			return;

		// Own this list before erasing any neighbours from the dense map.
		const std::vector<std::uint64_t> pairs = std::move( object->second );
		m_ObjectPairs.erase( object );
		for ( const std::uint64_t pairKey : pairs )
		{
			const Entry entry = m_Entries.at( pairKey );
			const std::uint32_t bodyA = BodyA( pairKey ), bodyB = BodyB( pairKey );
			const std::uint32_t otherBody = bodyA == bodyKey ? bodyB : bodyA;
			if ( otherBody != bodyKey )
				RemoveEndpoint( otherBody, bodyA == bodyKey ? entry.slotB : entry.slotA );
			m_Entries.erase( pairKey );
		}
	}

	void Clear()
	{
		std::unique_lock<std::shared_mutex> lock( m_Mutex );
		++m_MutationEpoch;
		m_Entries.clear();
		m_ObjectPairs.clear();
	}

	std::size_t EntryCount() const
	{
		std::shared_lock<std::shared_mutex> lock( m_Mutex );
		return m_Entries.size();
	}

	std::size_t ObjectCount() const
	{
		std::shared_lock<std::shared_mutex> lock( m_Mutex );
		return m_ObjectPairs.size();
	}

	std::size_t PairCountForObject( std::uint32_t bodyKey ) const
	{
		std::shared_lock<std::shared_mutex> lock( m_Mutex );
		const auto object = m_ObjectPairs.find( bodyKey );
		return object == m_ObjectPairs.end() ? 0 : object->second.size();
	}

	std::size_t EndpointSlotCapacity() const
	{
		std::shared_lock<std::shared_mutex> lock( m_Mutex );
		std::size_t capacity = 0;
		for ( const auto &object : m_ObjectPairs ) capacity += object.second.capacity();
		return capacity;
	}

private:
	struct Entry
	{
		std::uint32_t slotA = 0;
		std::uint32_t slotB = 0;
		bool shouldCollide = false;
	};

	static std::uint32_t BodyA( std::uint64_t key ) { return std::uint32_t( key >> 32 ); }
	static std::uint32_t BodyB( std::uint64_t key ) { return std::uint32_t( key ); }

	std::uint32_t AddEndpoint( std::uint32_t bodyKey, std::uint64_t pairKey )
	{
		auto &pairs = m_ObjectPairs[bodyKey];
		const std::uint32_t slot = std::uint32_t( pairs.size() );
		pairs.push_back( pairKey );
		return slot;
	}

	void RemoveEndpoint( std::uint32_t bodyKey, std::uint32_t slot )
	{
		auto object = m_ObjectPairs.find( bodyKey );
		auto &pairs = object->second;
		const std::uint64_t movedPair = pairs.back();
		if ( slot + std::size_t( 1 ) != pairs.size() )
		{
			pairs[slot] = movedPair;
			Entry &movedEntry = m_Entries.at( movedPair );
			if ( BodyA( movedPair ) == bodyKey )
				movedEntry.slotA = slot;
			else
				movedEntry.slotB = slot;
		}
		pairs.pop_back();
		if ( pairs.empty() )
			m_ObjectPairs.erase( object );
		else if ( pairs.capacity() > 8 && pairs.size() <= pairs.capacity() / 4 )
		{
			// Keep a sequence of previously busy, still-live bodies from retaining
			// unbounded historical adjacency capacity. Geometric shrinking makes
			// total copying proportional to removed edges, not degree per erase.
			std::vector<std::uint64_t>( pairs.begin(), pairs.end() ).swap( pairs );
		}
	}

	const std::size_t m_MaxEntries;
	mutable std::shared_mutex m_Mutex;
	std::uint64_t m_MutationEpoch = 0;
	ankerl::unordered_dense::map<std::uint64_t, Entry> m_Entries;
	ankerl::unordered_dense::map<std::uint32_t, std::vector<std::uint64_t>> m_ObjectPairs;
};
