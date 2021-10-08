find_package(Boost REQUIRED)

add_library(learning STATIC)

target_sources(learning
        PRIVATE
        algorithms/policy_iteration_performance.cpp
        algorithms/q_learning.cpp
        algorithms/q_learning_performance.cpp
        algorithms/value_iteration.cpp
        algorithms/value_iteration_performance.cpp
        algorithms/viterbi.cpp)

target_compile_definitions(learning
        PUBLIC
        BOOST_ALLOW_DEPRECATED_HEADERS)

target_include_directories(environment
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(learning
        PUBLIC
        environment
        matching
        Boost::boost)
