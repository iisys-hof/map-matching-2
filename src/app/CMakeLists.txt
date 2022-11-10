find_package(Boost REQUIRED COMPONENTS program_options)

add_library(app INTERFACE)

target_precompile_headers(app
        INTERFACE
        <boost/program_options.hpp>)

target_compile_definitions(app
        INTERFACE
        BOOST_ALLOW_DEPRECATED_HEADERS
        BOOST_THREAD_VERSION=5)

target_include_directories(app
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(app
        INTERFACE
        Boost::boost
        Boost::program_options)
