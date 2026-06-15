include(FetchContent)

set(KD_TREE_VERSION 0.0.1)

find_package(KDTree ${KD_TREE_VERSION} QUIET CONFIG)

if(KDTree_FOUND)
    message(STATUS "KD-Tree found at ${KDTree_DIR}. Using existing installation.")
else()
    message(STATUS "KD-Tree not found. Fetching from GitHub Release ${KD_TREE_VERSION}.")

    # Ensure fetched KD-Tree uses the same Thrust backend as this project.
    set(KD_TREE_PARALLELIZATION ${POLYHEDRAL_GRAVITY_PARALLELIZATION} CACHE STRING "KD-Tree parallelization backend" FORCE)
    set(BUILD_KD_TREE_EXECUTABLE OFF CACHE BOOL "Build KD-Tree Executable" FORCE)
    set(BUILD_KD_TREE_LIBRARY ON CACHE BOOL "Build KD-Tree Library" FORCE)

    FetchContent_Declare(
        KDTree
        GIT_REPOSITORY https://github.com/Blauben/kd-tree.git
        GIT_TAG v${KD_TREE_VERSION}
    )

    FetchContent_MakeAvailable(KDTree)
endif()