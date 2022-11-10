add_library(environment INTERFACE)

target_precompile_headers(environment
        INTERFACE
        <cstddef>
        <cstdint>
        <cmath>
        <limits>
        <numeric>
        <utility>
        <algorithm>
        <iterator>
        <string>
        <vector>
        <tuple>
        <absl/container/flat_hash_map.h>)

target_include_directories(environment
        INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(environment
        INTERFACE
        matching
        absl::flat_hash_map)
