cmake_minimum_required(VERSION 3.28)

# project
project(map_matching_2
        VERSION 1.0.10
        DESCRIPTION "Map Matching 2"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON CACHE BOOL "No developer warnings" FORCE)

# info
message(STATUS "${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "Using compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Using version: ${CMAKE_CXX_COMPILER_VERSION}")
if (DEFINED CMAKE_GENERATOR)
    message(STATUS "Using generator: ${CMAKE_GENERATOR}")
endif ()

include(CMakeDependentOption)

# options
cmake_dependent_option(TOP_LEVEL_PROJECT "Top level project" ON
        "CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR" OFF)
cmake_dependent_option(ENABLE_TESTS "Enable unit tests" ON "TOP_LEVEL_PROJECT" OFF)
# disable in GCC 13.2 and below due to bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=104634
# disable in MSVC due to decreased performance and due to triggering fatal error LNK1248
# see: https://learn.microsoft.com/en-us/cpp/error-messages/tool-errors/linker-tools-error-lnk1248?view=msvc-170
cmake_dependent_option(EXPLICIT_TEMPLATES "Enable explicit template instantiations" ON
        "NOT (CMAKE_CXX_COMPILER_ID STREQUAL \"GNU\" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13.3) AND NOT MSVC" OFF)
cmake_dependent_option(ENABLE_IPO "Enable interprocedural optimization" ON
        "NOT (CMAKE_BUILD_TYPE STREQUAL \"Debug\")" OFF)
cmake_dependent_option(DEBUG_BUILD "Enable debug build" ON
        "CMAKE_BUILD_TYPE STREQUAL \"Debug\"" OFF)
option(NATIVE_BUILD "Enable native build" OFF)
option(EXTERNAL_DEPENDENCIES "Configure external dependencies" ON)
option(REPORT_COMPILE_TIME "Report compile time" OFF)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    option(SET_OG_DEBUG_FLAG "Set Debug flag to -Og" OFF)
    option(COMPRESS_DEBUG_STRINGS "Compress strings in Debug builds" OFF)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        option(USE_GOLD_LINKER "Use gold linker instead of ld" ON)
        option(CREATE_GDB_INDEX "Create gdb-index" ON)
        option(DISABLE_TEMPLATE_BACKTRACE_LIMIT "Disable template backtrace limit" ON)
        option(ENABLE_EXTERNIS "Enable externis" OFF)
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        option(USE_LIBCPP "Use LLVM libc++ instead of GNU libstd++" ON)
        option(USE_LLD_LINKER "Use LLVM lld linker instead of GNU ld" ON)
        option(ENABLE_FTIME_TRACE "Enable ftime-trace" OFF)
    endif ()
elseif (MSVC)
    option(USE_STATIC_MSVC_RUNTIME_LIBRARY "Use static MSVC runtime library" ON)
endif ()

# rpmalloc crashes in MSVC DEBUG with static runtime
# see: https://github.com/mjansson/rpmalloc/issues/175
cmake_dependent_option(ENABLE_RPMALLOC "Enable rpmalloc" OFF
        "USE_STATIC_MSVC_RUNTIME_LIBRARY AND DEBUG_BUILD AND MSVC" ON)

# rpath override on for default install prefix
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(OVERRIDE_RPATH_CACHE ON CACHE BOOL "override rpath cache" FORCE)
endif ()
cmake_dependent_option(OVERRIDE_RPATH "Override rpath" ON
        "OVERRIDE_RPATH_CACHE" OFF)

# status
message(STATUS "Top level project: ${TOP_LEVEL_PROJECT}")
message(STATUS "Unit tests: ${ENABLE_TESTS}")
message(STATUS "Explicit templates: ${EXPLICIT_TEMPLATES}")
message(STATUS "Enable interprocedural optimization: ${ENABLE_IPO}")
message(STATUS "Debug build: ${DEBUG_BUILD}")
message(STATUS "Native build: ${NATIVE_BUILD}")
message(STATUS "Configure external dependencies: ${EXTERNAL_DEPENDENCIES}")
message(STATUS "Report compile time: ${REPORT_COMPILE_TIME}")

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(STATUS "Set Debug flag to -Og: ${SET_OG_DEBUG_FLAG}")
    message(STATUS "Compress Debug build strings: ${COMPRESS_DEBUG_STRINGS}")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        message(STATUS "Use gold linker instead of ld: ${USE_GOLD_LINKER}")
        message(STATUS "Create gdb-index, works with gold linker: ${CREATE_GDB_INDEX}")
        message(STATUS "Disable template backtrace limit: ${DISABLE_TEMPLATE_BACKTRACE_LIMIT}")
        message(STATUS "Enable externis: ${ENABLE_EXTERNIS}")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        message(STATUS "Use LLVM libc++ instead of GNU libstd++: ${USE_LIBCPP}")
        message(STATUS "Use LLVM lld linker instead of GNU ld: ${USE_LLD_LINKER}")
        message(STATUS "Enable ftime-trace: ${ENABLE_FTIME_TRACE}")
    endif ()
elseif (MSVC)
    message(STATUS "Use static MSVC runtime library: ${USE_STATIC_MSVC_RUNTIME_LIBRARY}")
endif ()

message(STATUS "Enable rpmalloc: ${ENABLE_RPMALLOC}")
message(STATUS "Override rpath: ${OVERRIDE_RPATH}")

# configuration
if (EXPLICIT_TEMPLATES)
    add_definitions(-DEXPLICIT_TEMPLATES)
endif ()

if (NATIVE_BUILD)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-march=native" _MARCH_NATIVE)
    if (_MARCH_NATIVE)
        add_compile_options(-march=native)
        message(STATUS "Enabled native build with: -march=native")
    endif ()
endif ()

if (DEBUG_BUILD)
    add_definitions(-DMM2_DEBUG_BUILD)
endif ()

if (ENABLE_IPO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT _IPO_RESULT OUTPUT _IPO_OUTPUT)
    if (_IPO_RESULT)
        message(STATUS "Enabled IPO: ${_IPO_RESULT}")
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    else ()
        message(WARNING "IPO is not supported: ${_IPO_OUTPUT}")
    endif ()
endif ()

if (REPORT_COMPILE_TIME)
    set(CMAKE_C_COMPILER_LAUNCHER "${CMAKE_COMMAND}" "-E" "time" CACHE STRING "compile time report" FORCE)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CMAKE_COMMAND}" "-E" "time" CACHE STRING "compile time report" FORCE)
endif ()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # both GCC and Clang
    if (SET_OG_DEBUG_FLAG)
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")
    endif ()
    if (COMPRESS_DEBUG_STRINGS)
        add_compile_options($<$<CONFIG:Debug>:-gz>)
    endif ()

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        # only GCC
        if (USE_GOLD_LINKER)
            add_link_options($<$<CONFIG:Debug>:-fuse-ld=gold>)
        endif ()
        if (CREATE_GDB_INDEX)
            add_link_options($<$<CONFIG:Debug>:-Wl,--gdb-index>)
        endif ()
        if (DISABLE_TEMPLATE_BACKTRACE_LIMIT)
            add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-ftemplate-backtrace-limit=0>)
        endif ()
        if (ENABLE_EXTERNIS)
            add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fplugin=externis>)
            add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fplugin-arg-externis-trace=trace.json>)
        endif ()
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        # only Clang
        if (USE_LIBCPP)
            add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-stdlib=libc++>)
            add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-stdlib=libc++>)
            add_link_options($<$<COMPILE_LANGUAGE:CXX>:-stdlib=libc++>)
        endif ()
        if (USE_LLD_LINKER)
            add_link_options(-fuse-ld=lld)
        endif ()
        if (ENABLE_FTIME_TRACE)
            add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-ftime-trace>)
        endif ()
    endif ()
elseif (MSVC)
    # fatal error C1128
    # see https://learn.microsoft.com/en-us/cpp/error-messages/compiler-errors-1/fatal-error-c1128?view=msvc-170
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/bigobj>)
    if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    endif ()
endif ()

if (ENABLE_RPMALLOC)
    add_definitions(-DMM2_ENABLE_RPMALLOC)
endif ()

# libraries
if (WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".a")
else ()
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
endif ()

if (EXTERNAL_DEPENDENCIES)
    add_subdirectory(external)
endif ()

# source
add_subdirectory(src)

# tests
if (ENABLE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif ()
