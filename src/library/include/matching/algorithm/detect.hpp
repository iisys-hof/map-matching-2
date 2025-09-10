// Copyright (C) 2021-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_MATCHING_ALGORITHM_DETECT_HPP
#define MAP_MATCHING_2_MATCHING_ALGORITHM_DETECT_HPP

#include <vector>

#include <boost/unordered/unordered_flat_set.hpp>

#include "geometry/rich_type/concepts.hpp"

#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/azimuth.hpp"

#include "matching/defect.hpp"

namespace map_matching_2::matching {

    template<geometry::is_rich_line RichLine>
    class detector {

    public:
        using rich_line_type = RichLine;
        using set_type = boost::unordered_flat_set<defect>;

        [[nodiscard]] static std::vector<set_type> detect(const rich_line_type &rich_line,
                const bool detect_duplicates = true,
                const bool detect_warps = true, const double warp_speed = 1500.0) {
            std::vector<set_type> defects(rich_line.size(), set_type{});

            if (detect_duplicates) {
                for (std::size_t from = 0, to = 1; to < rich_line.size(); ++from, ++to) {
                    if (_equal(rich_line, from, to)) {
                        defects[to].emplace(defect::equal);
                    }
                }
            }

            if (detect_warps) {
                for (std::size_t from = 0, to = 1; to < rich_line.size(); ++from, ++to) {
                    if (_warp(rich_line, from, to, warp_speed)) {
                        // we detected a warp but we still have to check whether we remove the from or to point
                        double destination_distance = std::numeric_limits<double>::max();
                        double origin_distance = std::numeric_limits<double>::max();
                        if (to + 1 < rich_line.size()) {
                            // simulate removing the to point and calculate the new segment distance
                            destination_distance = geometry::point_distance(rich_line.at(from), rich_line.at(to + 1));
                        }
                        if (from >= 1) {
                            // simulate removing the from point and calculate the new segment distance
                            origin_distance = geometry::point_distance(rich_line.at(from - 1), rich_line.at(to));
                        }

                        if (destination_distance <= origin_distance) {
                            // the destination distance smaller (or equal), so removing the to point is advised
                            defects[to].emplace(defect::warp);
                        } else {
                            // the origin distance smaller, so removing the from point is advised
                            defects[from].emplace(defect::warp);
                        }
                    }
                }
            }

            for (auto &defect_set : defects) {
                if (defect_set.empty()) {
                    defect_set.emplace(defect::none);
                }
            }

            // debug output
            // for (std::size_t i = 0; i < defects.size(); ++i) {
            //     const auto &defect_set = defects.at(i);
            //     if (not defect_set.contains(defect::none)) {
            //         std::cout << "Defect at " << i << ": " << defect_set << std::endl;
            //     }
            // }

            return defects;
        }

        static void remove_defects(rich_line_type &rich_line, const std::vector<set_type> &defects) {
            std::vector<std::size_t> remove;
            for (std::size_t i = 0; i < defects.size(); ++i) {
                const auto &defect_set = defects[i];
                if (not defect_set.contains(defect::none)) {
                    remove.emplace_back(i);
                }
            }

            rich_line.erase(remove);
        }

    private:
        bool static _equal(const rich_line_type &rich_line, const std::size_t from, const std::size_t to) {
            if (from >= rich_line.size() or to >= rich_line.size()) {
                return false;
            }

            return geometry::equals_points(rich_line.at(from), rich_line.at(to));
        }

        bool static _warp(const rich_line_type &rich_line, const std::size_t from, const std::size_t to,
                const double warp_speed) {
            using point_type = typename geometry::rich_line_traits<rich_line_type>::point_type;

            if constexpr (geometry::is_time_point<point_type>) {
                const point_type &from_point = rich_line.at(from);
                const point_type &to_point = rich_line.at(to);

                if (geometry::equals_points(from_point, to_point)) {
                    // on equal position, we did not warp because we did not move at all
                    return false;
                }

                if (from_point.is_valid() and to_point.is_valid()) {
                    // only continue if both points contain valid timestamps
                    if (to_point.timestamp() == from_point.timestamp()) {
                        // on the exact same timestamp with different positions, we definitely warped
                        return true;
                    }

                    const double time_difference = to_point.timestamp() - from_point.timestamp();
                    const double way = geometry::point_distance(from_point, to_point);
                    // speed in m/s is way through time, multiplied by 3.6 is km/h
                    const double speed = (way / time_difference) * 3.6;

                    return speed >= warp_speed;
                }
            }

            return false;
        }

    };

}

#endif //MAP_MATCHING_2_MATCHING_ALGORITHM_DETECT_HPP
