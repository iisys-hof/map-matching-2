# boost
set(boost_SOURCE_DIR "${FETCHCONTENT_BASE_DIR}/boost-src")
if (EXISTS "${boost_SOURCE_DIR}")
    set(boost_EXISTS ON)
    set(FETCHCONTENT_FULLY_DISCONNECTED ON)
endif ()

FetchContent_Declare(boost
        URL https://archives.boost.io/release/1.88.0/source/boost_1_88_0.tar.bz2
        URL_HASH SHA256=46d9d2c06637b219270877c9e16155cbd015b6dc84349af064c088e9b5b12f7b
        SOURCE_SUBDIR this-directory-does-not-exist)

FetchContent_MakeAvailable(boost)

if (NOT boost_EXISTS)
    if (WIN32)
        set(_boost_b2 "${boost_SOURCE_DIR}/b2.exe")
    else ()
        set(_boost_b2 "${boost_SOURCE_DIR}/b2")
    endif ()

    if (NOT EXISTS "${_boost_b2}")
        string(REGEX MATCH "^([0-9]+)" CXX_COMPILER_VERSION_MAJOR ${CMAKE_CXX_COMPILER_VERSION})

        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            set(_compiler "gcc")
            set(_compiler_version "${CXX_COMPILER_VERSION_MAJOR}")
        elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            set(_compiler "clang")
            set(_compiler_version "${CXX_COMPILER_VERSION_MAJOR}")
        elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
            set(_compiler "msvc")
            set(_compiler_version "${MSVC_TOOLSET_VERSION}")
        else ()
            message(FATAL_ERROR "Unsupported compiler: ${CMAKE_CXX_COMPILER_ID}")
        endif ()

        set(USER_CONFIG_FILE "${boost_SOURCE_DIR}/tools/build/src/user-config.jam")

        file(WRITE "${USER_CONFIG_FILE}"
                "using ${_compiler} : ${_compiler_version} : \"${CMAKE_CXX_COMPILER}\" ;")
        set(_toolset "toolset=${_compiler}")

        message(STATUS "Set boost toolset to ${_toolset}")

        if (WIN32)
            set(_bootstrap_path ${boost_SOURCE_DIR}/bootstrap.bat)
            set(_bootstrap_cxx "${_compiler}")
        else ()
            set(_bootstrap_path ${boost_SOURCE_DIR}/bootstrap.sh)
            set(_bootstrap_cxx "--with-${_toolset} --prefix=${boost_BINARY_DIR}")
        endif ()

        execute_process(COMMAND ${_bootstrap_path} ${_bootstrap_cxx}
                WORKING_DIRECTORY "${boost_SOURCE_DIR}")
    endif ()
endif ()

set(BOOST_ROOT "${boost_BINARY_DIR}" CACHE PATH "Boost root" FORCE)
set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL "Boost no system paths" FORCE)
set(Boost_USE_STATIC_LIBS ON CACHE BOOL "Boost static libs" FORCE)
if (MSVC)
    if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
        set(Boost_USE_STATIC_RUNTIME ON CACHE BOOL "Boost static runtime" FORCE)
    endif ()
endif ()

find_package(Boost QUIET CONFIG HINTS "${boost_BINARY_DIR}" NO_DEFAULT_PATH)

if (NOT Boost_FOUND)
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(_variant "variant=debug")
    else ()
        set(_variant "variant=release")
    endif ()
    message(STATUS "Set boost variant: ${_variant}")

    set(_cxxflags "")
    set(_linkflags "")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        set(_cxxflags "cxxflags=-stdlib=libc++")
        set(_linkflags "linkflags=-stdlib=libc++ -fuse-ld=lld")
    endif ()
    if (NOT _cxxflags STREQUAL "")
        message(STATUS "Set cxxflags: ${_cxxflags}")
    endif ()
    if (NOT _linkflags STREQUAL "")
        message(STATUS "Set linkflags: ${_linkflags}")
    endif ()

    if (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
        set(_address_model "address-model=64")
    elseif (${CMAKE_SIZEOF_VOID_P} EQUAL 4)
        set(_address_model "address-model=32")
    else ()
        message(WARNING "Unknown address model.")
    endif ()
    if (NOT _address_model STREQUAL "")
        message(STATUS "Set address-model: ${_address_model}")
    endif ()

    if (MSVC)
        if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
            set(_runtime_link "runtime-link=static")
        endif ()
        if (NOT _runtime_link STREQUAL "")
            message(STATUS "Set runtime-link: ${_runtime_link}")
        endif ()
    endif ()

    execute_process(COMMAND ./b2 install link=static,shared ${_runtime_link} threading=multi
            ${_variant} ${_toolset} ${_address_model} ${_cxxflags} ${_linkflags}
            --without-python --prefix=${boost_BINARY_DIR}
            WORKING_DIRECTORY "${boost_SOURCE_DIR}")
endif ()

find_package(Boost REQUIRED CONFIG HINTS "${boost_BINARY_DIR}" NO_DEFAULT_PATH)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${boost_SOURCE_DIR}/LICENSE_1_0.txt"
            DESTINATION doc/boost)
endif ()

unset(FETCHCONTENT_FULLY_DISCONNECTED)
