// Synthetic microbenchmark, not a game-server/TPS benchmark. LegacyScan retains
// the old map+lock+scan algorithm for a controlled local cost comparison.
#include "vjolt_should_collide_cache.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#if defined(__GLIBC__)
#include <malloc.h>
#endif

class LegacyScan
{
public:
	struct Lookup { std::uint64_t pairKey; bool found, shouldCollide; };
	Lookup Find( std::uint32_t a, std::uint32_t b ) const
	{
		if ( b < a ) std::swap( a, b );
		const std::uint64_t key = ( std::uint64_t( a ) << 32 ) | b;
		std::shared_lock<std::shared_mutex> lock( mutex );
		const auto it = entries.find( key );
		return { key, it != entries.end(), it == entries.end() ? false : it->second };
	}
	bool StoreIfUnchanged( Lookup lookup, bool value )
	{
		std::unique_lock<std::shared_mutex> lock( mutex );
		entries[lookup.pairKey] = value;
		return true;
	}
	void Invalidate( std::uint32_t key )
	{
		std::unique_lock<std::shared_mutex> lock( mutex );
		if ( entries.size() > 100000 ) { entries.clear(); return; }
		std::vector<std::uint64_t> erase;
		for ( const auto &entry : entries )
			if ( std::uint32_t( entry.first >> 32 ) == key || std::uint32_t( entry.first ) == key ) erase.push_back( entry.first );
		for ( const auto pair : erase ) entries.erase( pair );
	}
private:
	mutable std::shared_mutex mutex;
	ankerl::unordered_dense::map<std::uint64_t, bool> entries;
};

using Clock = std::chrono::steady_clock;
static volatile std::uint64_t sink = 0;
static double Ms( Clock::time_point start ) { return std::chrono::duration<double,std::milli>( Clock::now() - start ).count(); }
static std::size_t HeapBytes()
{
#if defined(__GLIBC__)
	const auto info = mallinfo2();
	return info.uordblks + info.hblkhd;
#else
	return 0;
#endif
}

template<class Cache> static void Run( const char *name )
{
	constexpr std::uint32_t count = 50000, iterations = 1000000, removals = 1000;
	{
		Cache cache;
		constexpr std::uint32_t bodies = 25000;
		// A degree-four ring: two unique edges per body, within the backend's
		// 65,536-body limit. Unlike disjoint pairs, index memory is shared.
		const auto pair = []( std::uint32_t index )
		{
			const std::uint32_t a = index % bodies;
			return std::make_pair( a, ( a + index / bodies + 1 ) % bodies );
		};
		const auto before = HeapBytes();
		auto begin = Clock::now();
		for ( std::uint32_t i = 0; i < count; ++i )
		{
			const auto keys = pair( i );
			cache.StoreIfUnchanged( cache.Find( keys.first, keys.second ), ( i & 1 ) != 0 );
		}
		const double fill = Ms( begin );
		const auto after = HeapBytes();
		begin = Clock::now();
		for ( std::uint32_t i = 0; i < iterations; ++i )
		{
			const auto keys = pair( ( std::uint64_t( i ) * 317 ) % count );
			const auto value = cache.Find( keys.first, keys.second );
			sink += value.found + value.shouldCollide;
		}
		const double hits = Ms( begin );
		begin = Clock::now();
		for ( std::uint32_t i = 0; i < removals; ++i ) cache.Invalidate( ( i * 317 ) % bodies );
		const double invalidate = Ms( begin );
		std::printf( "%s sparse_%u_bodies: insert_%u_ms=%.3f hit_%u_ms=%.3f invalidate_%u_ms=%.3f heap_delta_bytes=%zu\n",
			name, bodies, count, fill, iterations, hits, removals, invalidate, after >= before ? after-before : 0 );
	}
	{
		Cache cache;
		constexpr std::uint32_t leaves = 50000;
		const auto before = HeapBytes();
		auto begin = Clock::now();
		for ( std::uint32_t i = 1; i <= leaves; ++i ) cache.StoreIfUnchanged( cache.Find( 0, i ), true );
		const double fill = Ms( begin );
		const auto after = HeapBytes();
		begin = Clock::now();
		for ( std::uint32_t i = 0; i < iterations; ++i )
		{
			const auto value = cache.Find( 0, ( std::uint64_t( i ) * 317 ) % leaves + 1 );
			sink += value.found + value.shouldCollide;
		}
		const double hits = Ms( begin );
		begin = Clock::now();
		for ( std::uint32_t i = 0; i < removals; ++i ) cache.Invalidate( ( i * 317 ) % leaves + 1 );
		std::printf( "%s hub_%u_bodies: insert_%u_ms=%.3f hit_%u_ms=%.3f invalidate_%u_ms=%.3f heap_delta_bytes=%zu\n",
			name, leaves+1, leaves, fill, iterations, hits, removals, Ms( begin ), after >= before ? after-before : 0 );
	}
}

int main( int argc, char **argv )
{
	if ( argc > 1 && std::string( argv[1] ) == "legacy" ) Run<LegacyScan>( "legacy" );
	else if ( argc > 1 && std::string( argv[1] ) == "indexed" ) Run<VJoltShouldCollideCache>( "indexed" );
	else { Run<LegacyScan>( "legacy" ); Run<VJoltShouldCollideCache>( "indexed" ); }
	return sink == 0;
}
