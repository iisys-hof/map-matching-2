// Copyright (C) 2020-2021 Adrian Wöltche
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see https://www.gnu.org/licenses/.

#ifndef MAP_MATCHING_2_BENCHMARK_HPP
#define MAP_MATCHING_2_BENCHMARK_HPP

#include <functional>
#include <chrono>

namespace map_matching_2::util {

    double benchmark(const std::function<void()> &function) {
        using clock = std::chrono::high_resolution_clock;

        const auto start = clock::now();
        function();
        const auto end = clock::now();

        const std::chrono::duration<double> duration = end - start;
        return duration.count();
    }

}

#endif //MAP_MATCHING_2_BENCHMARK_HPP
