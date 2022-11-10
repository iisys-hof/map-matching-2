find_package(Boost REQUIRED)

add_library(learning INTERFACE)

target_precompile_headers(learning
        INTERFACE
        <absl/container/flat_hash_set.h>)

target_compile_definitions(learning
        INTERFACE
        BOOST_ALLOW_DEPRECATED_HEADERS)

target_include_directories(learning
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(learning
        INTERFACE
        environment
        matching
        absl::flat_hash_set
        Boost::boost)
