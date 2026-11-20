include(FetchContent)

# find_package only takes numeric versions, the git tag carries the pre-release suffix
set(KD_TREE_VERSION 0.0.2)
set(KD_TREE_GIT_TAG v0.0.2.alpha)

find_package(KDTree ${KD_TREE_VERSION} QUIET CONFIG)

if(KDTree_FOUND)
    message(STATUS "KD-Tree found at ${KDTree_DIR}. Using existing installation.")
else()
    message(STATUS "KD-Tree not found. Fetching from GitHub Release ${KD_TREE_GIT_TAG}.")

    # Ensure fetched KD-Tree uses the same Thrust backend as this project.
    # KD-Tree only knows CPP/OMP, so TBB falls back to serial (avoids nested threading anyway)
    if (${POLYHEDRAL_GRAVITY_PARALLELIZATION} STREQUAL "OMP")
        set(KD_TREE_PARALLELIZATION "OMP" CACHE STRING "KD-Tree parallelization backend" FORCE)
    else ()
        set(KD_TREE_PARALLELIZATION "CPP" CACHE STRING "KD-Tree parallelization backend" FORCE)
    endif ()
    set(BUILD_KD_TREE_EXECUTABLE OFF CACHE BOOL "Build KD-Tree Executable" FORCE)
    set(BUILD_KD_TREE_LIBRARY ON CACHE BOOL "Build KD-Tree Library" FORCE)

    FetchContent_Declare(
        KDTree
        GIT_REPOSITORY https://github.com/Blauben/kd-tree.git
        GIT_TAG ${KD_TREE_GIT_TAG}
    )

    FetchContent_MakeAvailable(KDTree)
endif()
