// Copyright (C) 2021-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_TRACK_MULTI_TRACK_HPP
#define MAP_MATCHING_2_GEOMETRY_TRACK_MULTI_TRACK_HPP

#include "geometry/rich_type/traits/multi_rich_line.hpp"

#include "track.hpp"

namespace map_matching_2::geometry::track {

    template<typename MultiRichLine>
    struct multi_track {

        using multi_rich_line_type = MultiRichLine;
        using point_type = typename multi_rich_line_traits<multi_rich_line_type>::point_type;
        using line_type = typename multi_rich_line_traits<multi_rich_line_type>::line_type;
        using multi_line_type = typename multi_rich_line_traits<multi_rich_line_type>::multi_line_type;
        using rich_line_type = typename multi_rich_line_traits<multi_rich_line_type>::rich_line_type;
        using rich_segment_type = typename multi_rich_line_traits<multi_rich_line_type>::rich_segment_type;
        using distance_type = typename multi_rich_line_traits<multi_rich_line_type>::distance_type;
        using length_type = typename multi_rich_line_traits<multi_rich_line_type>::length_type;
        using angle_type = typename multi_rich_line_traits<multi_rich_line_type>::angle_type;
        using coordinate_type = typename multi_rich_line_traits<multi_rich_line_type>::coordinate_type;
        using track_type = track<rich_line_type>;

        std::string id;
        multi_rich_line_type multi_rich_line;

        constexpr multi_track() = default;

        constexpr multi_track(track_type track)
            : id{std::move(track.id)}, multi_rich_line{std::move(track.rich_line)} {}

        constexpr multi_track(std::string id, line_type line)
            : id{std::move(id)}, multi_rich_line{rich_line_type{std::move(line)}} {}

        template<typename InLine>
        multi_track(std::string id, InLine input_line,
                const geometry::point_reprojector_variant &reprojector_variant) requires
            (geometry::is_line<InLine>)
            : id{std::move(id)} {
            line_type line;
            reproject_line(input_line, line, reprojector_variant);
            multi_rich_line = multi_rich_line_type{rich_line_type{std::move(line)}};
        }

        constexpr multi_track(std::string id, multi_line_type multi_line)
            : id{std::move(id)}, multi_rich_line{std::move(multi_line)} {}

        template<typename InMultiLine>
        multi_track(std::string id, InMultiLine input_multi_line,
                const geometry::point_reprojector_variant &reprojector_variant) requires
            (geometry::is_multi_line<InMultiLine>)
            : id{std::move(id)} {
            multi_line_type multi_line;
            reproject_multi_line(input_multi_line, multi_line, reprojector_variant);
            multi_rich_line = multi_rich_line_type{std::move(multi_line)};
        }

        constexpr multi_track(std::string id, multi_rich_line_type multi_rich_line)
            : id{std::move(id)}, multi_rich_line{std::move(multi_rich_line)} {}

        constexpr multi_track(const multi_track &other) = default;

        constexpr multi_track(multi_track &&other) noexcept = default;

        constexpr multi_track &operator=(const multi_track &other) = default;

        constexpr multi_track &operator=(multi_track &&other) noexcept = default;

        constexpr ~multi_track() = default;

        [[nodiscard]] constexpr std::size_t size() const {
            return multi_rich_line.size();
        }

        [[nodiscard]] constexpr track_type get_track(std::size_t index) const {
            return track_type{id, multi_rich_line.at(index)};
        }

        [[nodiscard]] std::string wkt() const {
            return multi_rich_line.wkt();
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
            std::string str = "MultiTrack(";
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

        static void sort_tracks(std::vector<multi_track> &multi_tracks, bool ascending = true) {
            if (ascending) {
                std::sort(multi_tracks.begin(), multi_tracks.end(), [&](const auto &left, const auto &right) {
                    const auto sizes = std::pair{left.multi_rich_line.sizes(), right.multi_rich_line.sizes()};
                    return sizes.first < sizes.second;
                });
            } else {
                std::sort(multi_tracks.begin(), multi_tracks.end(), [&](const auto &left, const auto &right) {
                    const auto sizes = std::pair{left.multi_rich_line.sizes(), right.multi_rich_line.sizes()};
                    return sizes.first > sizes.second;
                });
            }
        }

        friend std::ostream &operator<<(std::ostream &out, const multi_track &multi_track) {
            return out << multi_track.str();
        }

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_TRACK_MULTI_TRACK_HPP
