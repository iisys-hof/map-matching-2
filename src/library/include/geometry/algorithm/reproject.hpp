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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_REPROJECT_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_REPROJECT_HPP

#include "types/geometry/reprojector.hpp"

namespace map_matching_2::geometry {

    [[nodiscard]] point_reprojector_variant create_point_reprojector(const srs_transform &srs_transform);

    template<typename InPoint, typename OutPoint>
    void reproject_point(const InPoint &in_point, OutPoint &out_point,
            const point_reprojector_variant &reprojector_variant) {
        std::visit([&]<typename PointReprojector>(PointReprojector &&reprojector) {
            using point_reprojector_type = std::remove_reference_t<PointReprojector>;
            using in_point_type = typename point_reprojector_type::in_point_type;
            using out_point_type = typename point_reprojector_type::out_point_type;

            if (reprojector.transform()) {
                if constexpr (std::assignable_from<OutPoint &, out_point_type> and
                    std::assignable_from<in_point_type &, InPoint>) {
                    out_point = reprojector(in_point);
                } else {
                    static_assert(not(std::assignable_from<OutPoint &, out_point_type> and
                                std::assignable_from<in_point_type &, InPoint>),
                            "reprojector not called because types mismatch");
                    throw std::invalid_argument{"reprojector not called because types mismatch"};
                }
            } else {
                if constexpr (std::assignable_from<OutPoint &, InPoint>) {
                    out_point = in_point;
                } else {
                    static_assert(not std::assignable_from<OutPoint &, InPoint>,
                            "no transform reprojector and types mismatch");
                    throw std::invalid_argument{"no transform reprojector and types mismatch"};
                }
            }
        }, reprojector_variant);
    }

    template<typename InLine, typename OutLine>
    void reproject_line(const InLine &in_line, OutLine &out_line,
            const point_reprojector_variant &reprojector_variant) {
        using in_point_type = geometry::point_type_t<InLine>;
        using out_point_type = geometry::point_type_t<OutLine>;

        out_line.reserve(in_line.size());
        for (const in_point_type &input_point : in_line) {
            out_point_type out_point;
            reproject_point(input_point, out_point, reprojector_variant);
            out_line.emplace_back(std::move(out_point));
        }
    }

    template<typename InMultiLine, typename OutMultiLine>
    void reproject_multi_line(const InMultiLine &in_multi_line, OutMultiLine &out_multi_line,
            const point_reprojector_variant &reprojector_variant) {
        using in_line_type = typename InMultiLine::line_type;
        using out_line_type = typename OutMultiLine::line_type;

        out_multi_line.reserve(in_multi_line.size());
        for (const in_line_type &input_line : in_multi_line) {
            out_line_type out_line;
            reproject_line(input_line, out_line, reprojector_variant);
            out_multi_line.emplace_back(std::move(out_line));
        }
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_REPROJECT_HPP
