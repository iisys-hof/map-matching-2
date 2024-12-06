# protobuf
FetchContent_Declare(protobuf
        GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
        GIT_TAG v29.1
        SOURCE_SUBDIR cmake
        EXCLUDE_FROM_ALL
        FIND_PACKAGE_ARGS)

FetchContent_GetProperties(protobuf)
if (NOT protobuf_POPULATED)
    FetchContent_Populate(protobuf)

    # workaround for list functionality
    string(REPLACE ";" "\\;" ZLIB_ROOT_ESCAPED "${ZLIB_ROOT}")

    if (MSVC)
        if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
            set(PROTOBUF_MSVC_STATIC_RUNTIME "-Dprotobuf_MSVC_STATIC_RUNTIME=ON")
        endif ()
    endif ()

    execute_process(COMMAND ${CMAKE_COMMAND}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DCMAKE_GENERATOR=${CMAKE_GENERATOR}
            -DCMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}
            -DABSL_PROPAGATE_CXX_STD=ON
            -DZLIB_ROOT=${ZLIB_ROOT_ESCAPED}
            -DZLIB_USE_STATIC_LIBS=ON
            ${PROTOBUF_MSVC_STATIC_RUNTIME}
            -Dprotobuf_BUILD_TESTS=OFF
            -Dprotobuf_BUILD_EXAMPLES=OFF
            -Dprotobuf_BUILD_PROTOBUF_BINARIES=ON
            -Dprotobuf_BUILD_PROTOC_BINARIES=OFF
            -Dprotobuf_INSTALL=OFF
            -S "${protobuf_SOURCE_DIR}"
            -B "${protobuf_BINARY_DIR}"
    )
    execute_process(COMMAND ${CMAKE_COMMAND}
            --build "${protobuf_BINARY_DIR}")
endif ()

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${protobuf_SOURCE_DIR}/cmake" CACHE STRING "cmake module path" FORCE)
set(Protobuf_ROOT "${protobuf_SOURCE_DIR}/src;${protobuf_BINARY_DIR}" CACHE PATH "protobuf root path" FORCE)

find_package(Protobuf REQUIRED)

if (NOT BUILD_ONLY_LIBRARY)
    install(FILES
            "${protobuf_SOURCE_DIR}/LICENSE"
            DESTINATION doc/protobuf)
endif ()
