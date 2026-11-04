#include "vjolt_should_collide_cache.h"
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <random>
#include <thread>
#include <unordered_map>

static int checks = 0, failures = 0;
static void Check( bool result, const char *label )
{
	++checks;
	if ( !result ) { ++failures; std::printf( "FAIL %s\n", label ); }
}
static bool Put( VJoltShouldCollideCache &cache, std::uint32_t a, std::uint32_t b, bool value )
{
	return cache.StoreIfUnchanged( cache.Find( a, b ), value );
}

static void BasicAndEpochTests()
{
	VJoltShouldCollideCache cache;
	Check( Put( cache, 1, 2, false ) && Put( cache, 3, 4, true ), "cache true and false" );
	Check( cache.Find( 2, 1 ).found && !cache.Find( 2, 1 ).shouldCollide, "unordered pair identity" );
	Check( Put( cache, 2, 1, true ) && cache.PairCountForObject( 1 ) == 1, "duplicate updates do not duplicate adjacency" );
	const auto pending = cache.Find( 1, 2 );
	cache.Invalidate( 1 );
	Check( !cache.Find( 1, 2 ).found && cache.Find( 3, 4 ).shouldCollide, "only affected pair invalidated" );
	Check( !cache.StoreIfUnchanged( pending, false ), "in-flight result cannot undo invalidation" );
	Check( Put( cache, 1, 2, false ), "same IDs can be reused after invalidation" );
	Check( !cache.StoreIfUnchanged( pending, true ) && !cache.Find( 1, 2 ).shouldCollide,
		"old epoch cannot overwrite newer same-ID result (ABA)" );
	const auto absent = cache.Find( 7, 8 );
	cache.Invalidate( 7 );
	Check( !cache.StoreIfUnchanged( absent, true ), "uncached body invalidation rejects an in-flight miss" );
	const auto clearPending = cache.Find( 1, 2 );
	cache.Clear();
	Check( !cache.StoreIfUnchanged( clearPending, true ) && cache.EntryCount() == 0 && cache.ObjectCount() == 0,
		"full clear invalidates pending work and both indexes" );
	Check( Put( cache, 9, 9, true ) && cache.PairCountForObject( 9 ) == 1, "self pair owns one endpoint slot" );
	cache.Invalidate( 9 );
	Check( cache.EntryCount() == 0 && cache.ObjectCount() == 0, "self pair cleanup" );
}

static void CapacityTests()
{
	VJoltShouldCollideCache cache( 2 );
	Check( Put( cache, 1, 2, false ) && Put( cache, 3, 4, true ), "fill capacity" );
	Check( !Put( cache, 5, 6, false ) && cache.EntryCount() == 2, "capacity bypasses new pair admission" );
	Check( cache.Find( 1, 2 ).found && !cache.Find( 1, 2 ).shouldCollide && cache.Find( 3, 4 ).shouldCollide,
		"capacity preserves unrelated false and true entries" );
	Check( Put( cache, 1, 2, true ), "existing pair can update at capacity" );
	cache.Invalidate( 1 );
	Check( Put( cache, 5, 6, false ) && cache.EntryCount() == 2, "body cleanup permits admission again" );
	VJoltShouldCollideCache disabled( 0 );
	Check( !Put( disabled, 1, 2, true ) && disabled.ObjectCount() == 0, "zero-capacity cache has no reverse allocations" );
}

static void BacklinkAndModelTests()
{
	VJoltShouldCollideCache cache;
	// Every deletion moves a shared hub's last neighbour. This catches stale
	// slots in the remaining entries, including both sorted endpoint positions.
	for ( std::uint32_t i = 1; i <= 2000; ++i )
	{
		Put( cache, 0, i, ( i & 1 ) != 0 );
		Put( cache, i, 3000, ( i & 1 ) == 0 );
	}
	std::vector<std::uint32_t> order;
	for ( std::uint32_t i = 1; i <= 2000; ++i ) order.push_back( i );
	std::mt19937 random( 0x4a01 );
	std::shuffle( order.begin(), order.end(), random );
	std::size_t remaining = order.size();
	for ( std::uint32_t i : order )
	{
		cache.Invalidate( i );
		if ( --remaining == 1 )
			Check( cache.EndpointSlotCapacity() <= 24,
				"previous high-degree hubs release historical vector capacity" );
	}
	Check( cache.EntryCount() == 0 && cache.ObjectCount() == 0, "shuffled shared-hub deletion repairs all backlinks" );

	// Compare externally visible results against a simple independent map over
	// mixed updates, self pairs, invalidation, clearing and dense neighbour graphs.
	std::unordered_map<std::uint64_t, bool> model;
	bool matches = true;
	for ( int operation = 0; operation < 4000; ++operation )
	{
		std::uint32_t a = random() % 40, b = random() % 40;
		if ( a > b ) std::swap( a, b );
		const std::uint64_t key = ( std::uint64_t( a ) << 32 ) | b;
		if ( operation % 53 == 0 ) { cache.Clear(); model.clear(); }
		else if ( operation % 5 == 0 )
		{
			cache.Invalidate( a );
			for ( auto it = model.begin(); it != model.end(); )
				if ( std::uint32_t( it->first >> 32 ) == a || std::uint32_t( it->first ) == a ) it = model.erase( it );
				else ++it;
		}
		else { const bool value = ( random() & 1 ) != 0; Put( cache, a, b, value ); model[key] = value; }
		matches = matches && cache.EntryCount() == model.size();
		for ( std::uint32_t x = 0; x < 40; ++x )
		{
			std::size_t degree = 0;
			for ( std::uint32_t y = x; y < 40; ++y )
			{
				const auto value = cache.Find( x, y );
				const auto expected = model.find( ( std::uint64_t( x ) << 32 ) | y );
				matches = matches && value.found == ( expected != model.end() )
					&& ( !value.found || value.shouldCollide == expected->second );
			}
			for ( const auto &entry : model )
				if ( std::uint32_t( entry.first >> 32 ) == x || std::uint32_t( entry.first ) == x ) ++degree;
			matches = matches && cache.PairCountForObject( x ) == degree;
		}
	}
	Check( matches, "mixed dense graph operations agree with independent model" );
}

static void ConcurrentTests()
{
	VJoltShouldCollideCache cache;
	std::mutex mutex;
	std::condition_variable changed;
	bool lookedUp = false, invalidated = false, stored = true;
	std::thread callback( [&]()
	{
		const auto lookup = cache.Find( 1, 2 );
		{
			std::unique_lock<std::mutex> lock( mutex );
			lookedUp = true; changed.notify_all();
			changed.wait( lock, [&]() { return invalidated; } );
		}
		stored = cache.StoreIfUnchanged( lookup, true );
	} );
	{
		std::unique_lock<std::mutex> lock( mutex );
		changed.wait( lock, [&]() { return lookedUp; } );
		cache.Invalidate( 1 );
		Put( cache, 1, 2, false );
		invalidated = true; changed.notify_all();
	}
	callback.join();
	Check( !stored && cache.Find( 1, 2 ).found && !cache.Find( 1, 2 ).shouldCollide,
		"controlled cross-thread invalidate/recreate beats stale callback result" );

	cache.Clear();
	std::atomic<int> wrong{0};
	std::vector<std::thread> workers;
	for ( std::uint32_t worker = 0; worker < 4; ++worker )
		workers.emplace_back( [&, worker]()
		{
			std::mt19937 random( 93 + worker );
			for ( int i = 0; i < 12000; ++i )
			{
				const std::uint32_t a = random() % 200 + 1, b = random() % 3;
				const bool expected = ( ( a ^ b ) & 1 ) != 0;
				const auto lookup = cache.Find( a, b );
				if ( lookup.found && lookup.shouldCollide != expected ) ++wrong;
				if ( !lookup.found ) cache.StoreIfUnchanged( lookup, expected );
				if ( i % 7 == 0 ) cache.Invalidate( a );
			}
		} );
	for ( auto &worker : workers ) worker.join();
	for ( std::uint32_t a = 1; a <= 200; ++a ) cache.Invalidate( a );
	Check( wrong == 0 && cache.EntryCount() == 0 && cache.ObjectCount() == 0,
		"concurrent readers/stores/invalidation preserve answers and reclaim adjacency" );
}

int main()
{
	BasicAndEpochTests(); CapacityTests(); BacklinkAndModelTests(); ConcurrentTests();
	std::printf( "%d checks, %d failures\n", checks, failures );
	return failures != 0;
}
