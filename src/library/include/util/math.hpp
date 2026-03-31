// Copyright (C) 2026 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_UTIL_MATH_HPP
#define MAP_MATCHING_2_UTIL_MATH_HPP

#include <cstddef>
#include <cmath>
#include <vector>
#include <stdexcept>

#include "concepts.hpp"

namespace map_matching_2::util {

    [[nodiscard]] double gaussian_weight(std::size_t curr, std::size_t center, std::size_t size,
            double min_weight, double max_weight, double sigma = 2.0);

    template<std::integral T, std::floating_point F>
    [[nodiscard]] T round(F value) {
        if constexpr (sizeof(T) <= sizeof(long)) {
            return static_cast<T>(std::lround(value));
        } else {
            return static_cast<T>(std::llround(value));
        }
    }

    template<numeric T>
    [[nodiscard]] std::vector<T> interpolate(const T low, const T high, const std::size_t count) {
        std::vector<T> result;

        if (count == 0) {
            return result;
        }

        if (count == 1) {
            result.push_back(low);
            return result;
        }

        result.reserve(count);

        if constexpr (std::is_floating_point_v<T>) {
            for (std::size_t i = 0; i < count; ++i) {
                const T d = static_cast<T>(i) / static_cast<T>(count - 1);
                T value = low + (high - low) * d;
                result.push_back(value);
            }
        } else if constexpr (std::is_integral_v<T>) {
            for (std::size_t i = 0; i < count; ++i) {
                const double d = static_cast<double>(i) / static_cast<double>(count - 1);
                double value = static_cast<double>(low) + (static_cast<double>(high) - static_cast<double>(low)) * d;
                result.push_back(util::round<T, decltype(value)>(value));
            }
        } else {
            static_assert(util::dependent_false_v<T>, "invalid numeric type");
            throw std::invalid_argument{"invalid numeric type"};
        }

        return result;
    }

    template<numeric T, std::floating_point D>
    [[nodiscard]] std::vector<T> interpolate_by_distance(const T low, const T high, const std::vector<D> &distances) {
        std::vector<T> result;

        if (distances.empty()) {
            return result;
        }

        result.reserve(distances.size() + 1);

        D total_distance{};
        for (const auto &dist : distances) {
            total_distance += dist;
        }

        D cumulative_distance{};
        for (std::size_t i = 0; i < distances.size() + 1; ++i) {
            const D t = total_distance > D{} ? cumulative_distance / total_distance : D{};

            if constexpr (std::is_floating_point_v<T>) {
                T value = low + (high - low) * static_cast<T>(t);
                result.push_back(value);
            } else if constexpr (std::is_integral_v<T>) {
                double value = static_cast<double>(low) + (static_cast<double>(high) - static_cast<double>(low))
                        * static_cast<double>(t);
                result.push_back(util::round<T, decltype(value)>(value));
            } else {
                static_assert(util::dependent_false_v<T>, "invalid numeric type");
                throw std::invalid_argument{"invalid numeric type"};
            }

            if (i < distances.size()) {
                cumulative_distance += distances[i];
            }
        }

        return result;
    }

}

#endif //MAP_MATCHING_2_UTIL_MATH_HPP
