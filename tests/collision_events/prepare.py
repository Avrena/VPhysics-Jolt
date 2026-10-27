"""Extract the actual event implementation; only the Source object API is shimmed."""
from pathlib import Path
import sys

source = (Path(__file__).resolve().parents[2] / 'vphysics_jolt/vjolt_listener_contact.h').read_text(encoding='utf-8')

def block(marker, semicolon=False):
    start = source.index(marker)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + (';' if semicolon else '') + '\n'

parts = [block('struct JoltPhysicsContactPair', True), 'class Contract { public:\n']
for marker in ['static inline uint64_t MakeCacheKey', 'static Vector GetEffectiveVelocity',
               'static JPH::Vec3 GetEffectiveJoltVelocity', 'static JPH::Vec3 GetEffectivePointVelocity',
               'float StampPairCollisionTime', 'void AdvanceSimulationTime']:
    parts.append(block(marker))
for marker in ['class JoltPhysicsCollisionInfo', 'class JoltPhysicsCollisionData final',
               'class JoltPhysicsCollisionEvent', 'template < typename Data >']:
    parts.append(block(marker, True))
parts += ['std::unordered_map<uint64_t, uint64_t> m_LastCollisionTime;\n',
          'std::atomic<uint64_t> m_nSimulationTimeUs{0};\n};\n']
out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
(out / 'events.inc').write_text('\n'.join(parts), encoding='utf-8')
