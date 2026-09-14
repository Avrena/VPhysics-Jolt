cmake_minimum_required(VERSION 3.20)

# Build-time extraction, not a second implementation of the native mapping.
# This keeps the focused fixture independent of the engine PCH/SDK runtime.
if(NOT DEFINED SOURCE_FILE)
    set(SOURCE_FILE "${CMAKE_CURRENT_LIST_DIR}/../../vphysics_jolt/vjolt_constraints.cpp")
endif()
if(NOT DEFINED OUTPUT_DIR)
    message(FATAL_ERROR "Pass -DOUTPUT_DIR=<generated include directory>")
endif()
file(READ "${SOURCE_FILE}" source)
string(REPLACE "\r\n" "\n" source "${source}")
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

function(extract_between begin_marker end_marker output_name)
    string(FIND "${source}" "${begin_marker}" begin)
    string(FIND "${source}" "${end_marker}" end)
    if(begin LESS 0 OR end LESS_EQUAL begin)
        message(FATAL_ERROR "Production source changed: cannot extract ${output_name}")
    endif()
    math(EXPR length "${end} - ${begin}")
    string(SUBSTRING "${source}" ${begin} ${length} extracted)
    file(WRITE "${OUTPUT_DIR}/${output_name}" "${extracted}\n")
endfunction()

extract_between("struct RagdollLimits_t" "void JoltPhysicsConstraint::InitialiseRagdoll("
    "ragdoll_limits.inl")
string(FIND "${source}" "static void ConfigureRotationOnlyCorrection(" correction)
if(correction GREATER_EQUAL 0)
    extract_between("static void ConfigureRotationOnlyCorrection("
        "JoltPhysicsConstraint::JoltPhysicsConstraint(" "rotation_correction.inl")
else()
    # Before this policy existed, recreation had no corresponding operation.
    file(WRITE "${OUTPUT_DIR}/rotation_correction.inl"
        "static void ConfigureRotationOnlyCorrection(JPH::Constraint *) {}\n")
endif()
extract_between("\tif ( ragdoll.onlyAngularLimits )" "\n\telse if ( uDOFCount == 0 )"
    "rotation_mapping.inl")
extract_between("void JoltPhysicsConstraint::PostSimulate()"
    "void JoltPhysicsConstraint::ApplyGroupSolverIterations(" "post_simulate.inl")

# The baseline has a delayed recapture; the fixed implementation does not.
string(FIND "${source}" "void JoltPhysicsConstraint::RecaptureRotOnlyFrames()" recapture)
if(recapture GREATER_EQUAL 0)
    extract_between("void JoltPhysicsConstraint::RecaptureRotOnlyFrames()"
        "bool JoltPhysicsConstraint::CheckBroken()" "recapture.inl")
else()
    file(WRITE "${OUTPUT_DIR}/recapture.inl" "")
endif()
