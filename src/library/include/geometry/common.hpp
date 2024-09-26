// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_COMMON_HPP
#define MAP_MATCHING_2_GEOMETRY_COMMON_HPP

#include <cstdint>
#include <numbers>

namespace map_matching_2::geometry {

    template<typename Float>
    struct default_float_type {};

    template<>
    struct default_float_type<float> {
        static constexpr float v0 = 0.0f;
        static constexpr float v1 = 1.0f;
        static constexpr float v2 = 2.0f;
        static constexpr float v180 = 180.0f;
        static constexpr float v360 = 360.0f;
    };

    template<>
    struct default_float_type<double> {
        static constexpr double v0 = 0.0;
        static constexpr double v1 = 1.0;
        static constexpr double v2 = 2.0;
        static constexpr double v180 = 180.0;
        static constexpr double v360 = 360.0;
    };

    template<>
    struct default_float_type<long double> {
        static constexpr long double v0 = 0.0L;
        static constexpr long double v1 = 1.0L;
        static constexpr long double v2 = 2.0L;
        static constexpr long double v180 = 180.0L;
        static constexpr long double v360 = 360.0L;
    };

    [[nodiscard]] constexpr std::uint64_t next_pow2(std::uint64_t x) {
        x--;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        x |= x >> 32;
        x++;
        return x;
    }

    template<typename Float>
    [[nodiscard]] constexpr Float rad2deg(const Float rad) {
        return rad * (default_float_type<Float>::v180 / std::numbers::pi_v<Float>);
    }

    template<typename Float>
    [[nodiscard]] constexpr Float deg2rad(const Float deg) {
        return deg * (std::numbers::pi_v<Float> / default_float_type<Float>::v180);
    }

    template<typename Float>
    [[nodiscard]] constexpr Float angle_diff(const Float a, const Float b) {
        Float d = a - b;
        return (d >= default_float_type<Float>::v180
                ? d - default_float_type<Float>::v360
                : (d <= -default_float_type<Float>::v180
                   ? d + default_float_type<Float>::v360
                   : d));
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_COMMON_HPP
