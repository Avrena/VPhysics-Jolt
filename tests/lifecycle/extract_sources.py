"""Compile production lifecycle code against SDK/Jolt interface stand-ins.

The optional baseline ref extracts the original methods from git, so the same
regressions can demonstrate the pre-fix failure without keeping a code copy.
"""
import argparse
from pathlib import Path
import re
import subprocess


def method(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    # Ignore braces in comments/strings when matching the method's real body.
    token = re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]')
    for match in token.finditer(source, opening):
        if match.group() == "{":
            depth += 1
        elif match.group() == "}":
            depth -= 1
            if depth == 0:
                return source[start:match.end()] + "\n"
    raise RuntimeError("Unclosed method: " + signature)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline-ref", default="")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    def read(name):
        path = "vphysics_jolt/" + name
        if args.baseline_ref:
            return subprocess.check_output(
                ["git", "-C", str(args.root), "show", args.baseline_ref + ":" + path], text=True)
        return (args.root / path).read_text()

    environment = read("vjolt_environment.cpp")
    objects = read("vjolt_object.cpp")
    constraints = read("vjolt_constraints.cpp")
    header = read("vjolt_constraints.h")
    preamble = '#include "test_support.h"\n'

    if args.baseline_ref:
        lifecycle = preamble + "\n".join(method(environment, signature) for signature in (
            "JoltPhysicsEnvironment::~JoltPhysicsEnvironment()",
            "void JoltPhysicsEnvironment::DestroyObject(",
            "void JoltPhysicsEnvironment::DestroyConstraint(",
            "void JoltPhysicsEnvironment::CleanupDeleteList()",
            "void JoltPhysicsEnvironment::EnableDeleteQueue(",
            "void JoltPhysicsEnvironment::DestroyCollideOnDeadObjectFlush(",
            "void JoltPhysicsEnvironment::RemoveBodyAndDeleteObject(",
            "void JoltPhysicsEnvironment::DeleteDeadObjects()"))
    else:
        lifecycle = read("vjolt_environment_lifecycle.cpp")
        # Replace includes only. The entire implementation is compiled as-is.
        lifecycle = preamble + re.sub(r'^#include[^\n]*\n', "", lifecycle, flags=re.MULTILINE)
    (args.output / "lifecycle.cpp").write_text(lifecycle)

    shared = preamble
    for signature in (
        "const IPhysicsObject **JoltPhysicsEnvironment::GetObjectList(",
        "bool JoltPhysicsEnvironment::TransferObject(",
        "void JoltPhysicsEnvironment::ObjectTransferHandOver(",
        "void JoltPhysicsEnvironment::RegisterConstraint(",
        "void JoltPhysicsEnvironment::UnregisterConstraint(",
        "void JoltPhysicsEnvironment::NotifyConstraintDisabled(",
        "void JoltPhysicsEnvironment::RemoveDirtyStaticBody("):
        shared += method(environment, signature)
    for signature in (
        "JoltPhysicsObject::~JoltPhysicsObject()",
        "void JoltPhysicsObject::AddDestroyedListener(",
        "void JoltPhysicsObject::RemoveDestroyedListener("):
        shared += method(objects, signature)
    for signature in (
        "JoltPhysicsConstraint::~JoltPhysicsConstraint()",
        "void JoltPhysicsConstraint::DestroyConstraint()",
        "void JoltPhysicsConstraint::OnJoltPhysicsObjectDestroyed(",
        "void JoltPhysicsConstraint::Deactivate()",
        "IPhysicsObject *JoltPhysicsConstraint::GetReferenceObject() const",
        "IPhysicsObject *JoltPhysicsConstraint::GetAttachedObject() const"):
        shared += method(constraints, signature)
    if not args.baseline_ref:
        shared += method(header, "bool MarkForDelete()").replace(
            "bool MarkForDelete()", "bool JoltPhysicsConstraint::MarkForDelete()", 1)
    (args.output / "shared_methods.cpp").write_text(shared)


if __name__ == "__main__":
    main()
