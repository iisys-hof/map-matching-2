set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost REQUIRED COMPONENTS serialization unit_test_framework)

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
        compare_tests.cpp
        library_tests.cpp
        matcher_test.cpp
        network_test.cpp
        route_tests.cpp
        serialization_tests.cpp)

target_compile_definitions(map_matching_2_tests
        PRIVATE
        BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL)

target_link_libraries(map_matching_2_tests
        PRIVATE
        util
        environment
        geometry
        io
        learning
        matching
        Boost::serialization
        Boost::unit_test_framework)
