add_library(environment STATIC)

target_sources(environment
        PRIVATE
        environments/hmm.cpp
        environments/single.cpp)

target_include_directories(environment
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(environment
        PUBLIC
        matching)
