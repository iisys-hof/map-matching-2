set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

add_library(matching INTERFACE)

target_precompile_headers(matching
        INTERFACE
        <absl/container/flat_hash_map.h>
        <absl/container/btree_set.h>
        <absl/container/btree_map.h>
        <boost/thread.hpp>
        <boost/thread/executors/basic_thread_pool.hpp>)

target_compile_definitions(matching
        INTERFACE
        BOOST_ALLOW_DEPRECATED_HEADERS
        BOOST_THREAD_VERSION=5)

target_include_directories(matching
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(matching
        INTERFACE
        geometry
        learning
        util
        absl::flat_hash_map
        absl::btree
        Threads::Threads
        Boost::boost
        Boost::thread)
