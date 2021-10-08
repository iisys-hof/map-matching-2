// Copyright (C) 2021-2021 Adrian Wöltche
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

#include "detector.hpp"

#include <geometry/track/types.hpp>
#include <geometry/util.hpp>

namespace map_matching_2::matching {

    std::ostream &operator<<(std::ostream &out, const std::set<defect> &defect_set) {
        std::string defect_str;
        if (defect_set.contains(defect::none)) {
            defect_str.append("none, ");
        }
        if (defect_set.contains(defect::equal)) {
            defect_str.append("equal, ");
        }
        if (defect_set.contains(defect::forward_backward)) {
            defect_str.append("forward_backward, ");
        }
        if (defect_str.length() > 0 and defect_str.ends_with(", ")) {
            defect_str.erase(defect_str.length() - 2);
        }
        out << defect_str;
        return out;
    }

    template<typename Track>
    std::vector<std::set<defect>> detector<Track>::detect(
            const Track &track, const bool detect_duplicates, const bool detect_forward_backward) const {
        std::vector<std::set<defect>> defects;
        defects.reserve(track.measurements.size());
        for (std::size_t i = 0; i < track.measurements.size(); ++i) {
            defects.emplace_back(std::set<defect>{defect::none});
        }

        if (detect_duplicates) {
            for (std::int64_t from = 0, to = 1; to < track.measurements.size(); ++from, ++to) {
                if (_equal(track, from, to)) {
                    defects[to].emplace(defect::equal);
                }
            }
        }

        if (detect_forward_backward) {
            for (std::int64_t from = 0, to = 1; to < track.measurements.size(); ++from, ++to) {
                while (to < track.measurements.size() and
                       (defects[to].contains(defect::forward_backward) or
                        defects[to].contains(defect::equal))) {
                    ++to;
                }
                if (_forward_backward(track, from, to)) {
                    defects[to].emplace(defect::forward_backward);
                    --from;
                    --to;
                } else if (from + 1 != to) {
                    from = to - 1;
                }
            }
        }

        for (auto &defect_set: defects) {
            if (defect_set.size() > 1) {
                defect_set.erase(defect::none);
            }
        }

        // debug output
//            for (std::size_t i = 0; i < defects.size(); ++i) {
//                std::set<defect> defect_set = defects.at(i);
//                if (not defect_set.contains(defect::none)) {
//                    std::cout << "Defect at " << i << ": " << defect_set << std::endl;
//                }
//            }

        return defects;
    }

    template<typename Track>
    std::set<defect> detector<Track>::detect(
            const Track &track, const std::size_t from, const std::size_t to,
            const bool detect_forward_backward) const {
        std::set<defect> defect_set;

        if (detect_forward_backward and _forward_backward(track, from, to)) {
            defect_set.emplace(defect::forward_backward);
        }

        if (defect_set.empty()) {
            defect_set.emplace(defect::none);
        }

        return defect_set;
    }

    template<typename Track>
    bool detector<Track>::_equal(const Track &track, const std::size_t from, const std::size_t to) const {
        if (from >= track.measurements.size() or to >= track.measurements.size()) {
            return false;
        }

        return geometry::equals_points(track.line[from], track.line[to]);
    }

    template<typename Track>
    bool detector<Track>::_forward_backward(const Track &track, const std::size_t from, const std::size_t to,
                                            const double relative_difference) const {
        if (from >= track.measurements.size() or to >= track.measurements.size()) {
            return false;
        }

        const std::int64_t prev = ((std::int64_t) from) - 1;
        if (prev < 0) {
            return false;
        }

        bool adjacent_from = prev + 1 == from;
        const auto current_azimuth =
                adjacent_from ? track.rich_segments[prev].azimuth
                              : geometry::azimuth_deg(track.line[prev], track.line[from]);
        bool adjacent_to = from + 1 == to;
        const auto next_azimuth =
                adjacent_to ? track.rich_segments[from].azimuth
                            : geometry::azimuth_deg(track.line[from], track.line[to]);
        const auto diff_azimuth = geometry::angle_diff(current_azimuth, next_azimuth);

        const auto relative_probability = std::fabs(diff_azimuth) / 180.0;
        assert(relative_probability >= 0.0 and relative_probability <= 1.0);

        return relative_probability >= relative_difference;
    }

    template
    class detector<geometry::track::types_geographic::track_type>;

    template
    class detector<geometry::track::types_cartesian::track_type>;

}
