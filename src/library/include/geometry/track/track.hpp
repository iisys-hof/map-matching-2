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

#ifndef MAP_MATCHING_2_GEOMETRY_TRACK_TRACK_HPP
#define MAP_MATCHING_2_GEOMETRY_TRACK_TRACK_HPP

#include <boost/unordered/unordered_flat_set.hpp>

#include "geometry/rich_type/traits/rich_line.hpp"

#include "measurement.hpp"

namespace map_matching_2::geometry::track {

    template<typename RichLine>
    struct track {

        using rich_line_type = RichLine;
        using rich_segment_type = typename rich_line_traits<rich_line_type>::rich_segment_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using measurement_type = measurement<point_type>;

        std::string id;
        rich_line_type rich_line;

        constexpr track() = default;

        constexpr track(std::string id, line_type line)
            : id{std::move(id)}, rich_line{std::move(line)} {}

        template<typename InLine>
        track(std::string id, InLine input_line,
                const geometry::point_reprojector_variant &reprojector_variant) requires
            (geometry::is_line<InLine>)
            : id{std::move(id)} {
            line_type line;
            reproject_line(input_line, line, reprojector_variant);
            rich_line = rich_line_type{std::move(line)};
        }

        constexpr track(std::string id, rich_line_type rich_line)
            : id{std::move(id)}, rich_line{std::move(rich_line)} {}

        constexpr track(const track &other) = default;

        constexpr track(track &&other) noexcept = default;

        constexpr track &operator=(const track &other) = default;

        constexpr track &operator=(track &&other) noexcept = default;

        constexpr ~track() = default;

        [[nodiscard]] std::string wkt() const {
            return rich_line.wkt();
        }

        [[nodiscard]] static std::vector<std::string> header() {
            return {"id", "geometry"};
        }

        [[nodiscard]] std::vector<std::string> row() const {
            std::vector<std::string> row;
            row.reserve(2);
            row.emplace_back(id);
            row.emplace_back(wkt());
            return row;
        }

        [[nodiscard]] std::string str() const {
            const auto &fields = row();
            std::string str = "Track(";
            bool first = true;
            for (const auto &field : fields) {
                if (!first) {
                    str.append(",");
                }
                first = false;
                str.append(field);
            }
            str.append(")");
            return str;
        }

        [[nodiscard]] static track create_track(std::initializer_list<coordinate_type> coordinates) {
            line_type new_line;
            new_line.reserve(coordinates.size() / 2);
            for (auto it = coordinates.begin(); it != coordinates.end(); ++it) {
                const auto x = *it;
                const auto y = *(++it);
                new_line.emplace_back(point_type{x, y});
            }

            return track{"", new_line};
        }

        static void sort_tracks(std::vector<track> &tracks, bool ascending = true) {
            if (ascending) {
                std::sort(tracks.begin(), tracks.end(), [](const auto &left, const auto &right) {
                    return left.rich_line.size() < right.rich_line.size();
                });
            } else {
                std::sort(tracks.begin(), tracks.end(), [](const auto &left, const auto &right) {
                    return left.rich_line.size() > right.rich_line.size();
                });
            }
        }

        friend std::ostream &operator<<(std::ostream &out, const track &track) {
            return out << track.str();
        }

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_TRACK_TRACK_HPP
