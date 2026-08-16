# Includes the gtest.cmake
include(gtest)

# Recursively collects all the test files in the test Directory
file(GLOB_RECURSE TEST_SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
        )

# Creates the Test Target
add_executable(${PROJECT_NAME}_test ${TEST_SRC})

# Links target breakupModelTest against gtest_main (entry point, so no own main method is required)
target_link_libraries(${PROJECT_NAME}_test
        GTest::gtest_main
        GTest::gtest
        GTest::gmock
        ${PROJECT_NAME}_lib
        )

# Copies the test resources into a directory next to the test executable
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/resources" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

##########################################################
# Generating the scaled Eros benchmark meshes via Python
##########################################################
# Prefers the project's local virtualenv (./.venv), since that is where open3d is expected
# to be installed; falls back to whatever Python3 CMake finds on the system otherwise.
find_program(EROS_MESH_SCALING_PYTHON_EXECUTABLE
        NAMES python3 python
        HINTS "${PROJECT_SOURCE_DIR}/.venv/bin" "${PROJECT_SOURCE_DIR}/.venv/Scripts"
        NO_DEFAULT_PATH
)
if(NOT EROS_MESH_SCALING_PYTHON_EXECUTABLE)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    set(EROS_MESH_SCALING_PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
endif()

if(EROS_MESH_SCALING_PYTHON_EXECUTABLE)
    execute_process(
            COMMAND ${EROS_MESH_SCALING_PYTHON_EXECUTABLE} -c "import open3d"
            RESULT_VARIABLE EROS_MESH_SCALING_OPEN3D_MISSING
            OUTPUT_QUIET ERROR_QUIET
    )
endif()

if(EROS_MESH_SCALING_PYTHON_EXECUTABLE AND EROS_MESH_SCALING_OPEN3D_MISSING EQUAL 0)
    message(STATUS "Python3 with open3d found at ${EROS_MESH_SCALING_PYTHON_EXECUTABLE}. Scaled Eros benchmark meshes will be generated via scale_mesh.py")

    set(EROS_MESH_SOURCE_DIR "${PROJECT_SOURCE_DIR}/script/mesh")
    set(EROS_SCALED_MESH_DIR "${CMAKE_CURRENT_BINARY_DIR}/resources")
    set(EROS_SCALE_SCRIPT "${PROJECT_SOURCE_DIR}/script/scale_mesh.py")

    # Matches the face amounts hardcoded in script/scale_mesh.py: round(1000 * sqrt(3)^k) for k in range(10)
    set(EROS_FACE_AMOUNTS 1000 1732 3000 5196 9000 15588 27000 46765 81000 140296)
    set(EROS_SCALED_MESH_FILES "")
    foreach(amount ${EROS_FACE_AMOUNTS})
        list(APPEND EROS_SCALED_MESH_FILES
                "${EROS_SCALED_MESH_DIR}/Eros_scaled-${amount}.node"
                "${EROS_SCALED_MESH_DIR}/Eros_scaled-${amount}.face"
        )
    endforeach()

    add_custom_command(
            OUTPUT ${EROS_SCALED_MESH_FILES}
            COMMAND ${CMAKE_COMMAND} -E make_directory "${EROS_SCALED_MESH_DIR}"
            COMMAND ${CMAKE_COMMAND} -E copy "${EROS_MESH_SOURCE_DIR}/Eros.node" "${EROS_MESH_SOURCE_DIR}/Eros.face" "${EROS_SCALED_MESH_DIR}"
            COMMAND ${EROS_MESH_SCALING_PYTHON_EXECUTABLE} "${EROS_SCALE_SCRIPT}" "Eros.node" "Eros.face"
            WORKING_DIRECTORY "${EROS_SCALED_MESH_DIR}"
            DEPENDS "${EROS_MESH_SOURCE_DIR}/Eros.node" "${EROS_MESH_SOURCE_DIR}/Eros.face" "${EROS_SCALE_SCRIPT}"
            COMMENT "Generating scaled Eros meshes for benchmarking via scale_mesh.py"
            VERBATIM
    )

    add_custom_target(generate_eros_scaled_meshes DEPENDS ${EROS_SCALED_MESH_FILES})
    add_dependencies(${PROJECT_NAME}_test generate_eros_scaled_meshes)
else()
    message(STATUS "Python3 with open3d not found (checked ${PROJECT_SOURCE_DIR}/.venv and the system Python3). Skipping generation of scaled Eros benchmark meshes (see script/scale_mesh.py).")
endif()

include(GoogleTest)

# Adds the Tests to CTest by querying the test target executable
gtest_discover_tests(${PROJECT_NAME}_test)