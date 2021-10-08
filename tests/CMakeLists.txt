set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_executable(map_matching_2_tests)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-Wno-deprecated-declarations" _deprecated_declarations)
if (_deprecated_declarations)
    target_compile_options(map_matching_2_tests
            PRIVATE
            -Wno-deprecated-declarations)
    message(STATUS "Disabled deprecation warning for tests")
endif ()

target_sources(map_matching_2_tests
        PRIVATE
        library_tests.cpp
        route_tests.cpp
        compare_tests.cpp
        network_test.cpp
        matcher_test.cpp)

target_link_libraries(map_matching_2_tests
        PRIVATE
        util
        environment
        geometry
        io
        learning
        matching
        Boost::unit_test_framework)
