"""Extract production listener methods. The fixture supplies only engine interfaces/storage."""
import argparse
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('output', type=Path)
parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[2] /
                    'vphysics_jolt/vjolt_listener_contact.h')
args = parser.parse_args()
source = args.source.read_text(encoding='utf-8')


def block(marker):
    start = source.index(marker)
    brace = source.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


parts = ['class ListenerCache { public:\n']
for marker in ['static inline uint64_t MakeCacheKey', 'void InvalidShouldCollideCache',
               'bool ShouldCollide(', 'IPhysicsCollisionSolver *GetGameSolver',
               'void SetGameSolver']:
    parts.append(block(marker))
parts.append('bool PreEmptGameShouldCollide(JoltPhysicsObject *a, JoltPhysicsObject *b) '
             '{ return a->collidable && b->collidable; }\nstd::mutex m_ShouldCollideLock;\n')
# Allow the same integration tests to compile the actual base revision as a
# negative control. No copy/model of ShouldCollide or its invalidation is used.
if 'm_ShouldCollideCacheLock;' in source:
    parts += ['IPhysicsCollisionSolver *m_pGameSolver = nullptr;\n',
              'std::shared_mutex m_ShouldCollideCacheLock;\n',
              'ankerl::unordered_dense::map<uint64_t, bool> m_ShouldCollideCache;\n']
else:
    parts += ['std::atomic<IPhysicsCollisionSolver *> m_pGameSolver{nullptr};\n',
              'VJoltShouldCollideCache m_ShouldCollideCache;\n']
parts.append('};\nclass ListenerDispatch : public DispatchStorage { public:\n')
for marker in ['bool ShouldSkipFreedEvent', 'void FlushCallbacks(']:
    parts.append(block(marker))
parts.append('};\n')
args.output.mkdir(parents=True, exist_ok=True)
(args.output / 'listener.inc').write_text('\n'.join(parts), encoding='utf-8')
