set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
if (MSVC)
    if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
        set(Boost_USE_STATIC_RUNTIME ON)
    endif ()
endif ()

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
        compare_tests.cpp
        concept_inheritance.cpp
        graph_algorithm_tests.cpp
        graph_tests.cpp
        matcher_tests.cpp
        memory_mapped_index_tests.cpp
        memory_mapped_tests.cpp
        memory_mapped_wrapper_tests.cpp
        model_algorithm_tests.cpp
        model_tests.cpp
        network_tests.cpp
        route_tests.cpp
        serialization_tests.cpp
        util_tests.cpp
)

target_compile_definitions(map_matching_2_tests
        PRIVATE
        BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL)

target_link_libraries(map_matching_2_tests
        PRIVATE
        library
        Boost::unit_test_framework)

if (MSVC)
    add_custom_command(TARGET map_matching_2_tests POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:map_matching_2_tests> $<TARGET_RUNTIME_DLLS:map_matching_2_tests>
            COMMAND_EXPAND_LISTS)
endif ()
