include(FetchContent)

message(STATUS "Setting up tetgen")

# IMPORTANT NOTE
# We do not use find_package here, as we modify the one source file slightly to suppress output via stdout!!!

# Fetches the version 1.6 for tetgen
FetchContent_Declare(tetgen
        GIT_REPOSITORY https://github.com/libigl/tetgen.git
        GIT_TAG e05aca7df74e3f531bc35733ed87d36d437266c5 # release 1.6
)

FetchContent_MakeAvailable(tetgen)

# Modify the tetgen library to suppress console output from the printf function
if(NOT EXISTS ${tetgen_SOURCE_DIR}/tetgen_mod.cxx)
    message(STATUS "Creating modified tetgen.cxx in order to prevent console output from library")
    file(READ ${tetgen_SOURCE_DIR}/tetgen.cxx TETGEN_CXX)

    string(REPLACE
            "#include \"tetgen.h\""
            "#include \"tetgen.h\"\n#define printf(fmt, ...) (0)\n"
            TETGEN_CXX "${TETGEN_CXX}")

    file(WRITE ${tetgen_SOURCE_DIR}/tetgen_mod.cxx
            "${TETGEN_CXX}"
    )
else()
    message(STATUS "A modified tetgen.cxx already exists! It is assumed that it is the correct one disabling output")
endif()

# Add the modified version of the tetgen library
add_library(tetgen_lib STATIC
        ${tetgen_SOURCE_DIR}/tetgen_mod.cxx
        ${tetgen_SOURCE_DIR}/predicates.cxx
)

# Define the TETLIBRARY macro for usage
target_compile_definitions(tetgen_lib PRIVATE -DTETLIBRARY)

# Include the tetgen source directory for the library
target_include_directories(tetgen_lib INTERFACE "${tetgen_SOURCE_DIR}")

# Disable warnings from the library target
target_compile_options(tetgen_lib PRIVATE -w)