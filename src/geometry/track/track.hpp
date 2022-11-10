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

#ifndef MAP_MATCHING_2_TRACK_HPP
#define MAP_MATCHING_2_TRACK_HPP

#include <absl/container/btree_set.h>

#include <boost/geometry/srs/epsg.hpp>
#include <boost/geometry/srs/projection.hpp>
#include <boost/geometry/srs/transformation.hpp>

#include "../rich_line.hpp"
#include "measurement.hpp"

namespace map_matching_2::geometry::track {

    template<typename RichLine>
    struct track : public RichLine {

        using rich_line_type = RichLine;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using measurement_type = measurement<point_type>;

        std::string id;

        track()
                : track{"", rich_line_type{}} {}

        track(std::string id, const line_type &line)
                : rich_line_type{line}, id{std::move(id)} {}

        track(std::string id, rich_line_type rich_line)
                : rich_line_type{std::move(rich_line)}, id{std::move(id)} {}

        ~track() = default;

        track(const track &other)
                : rich_line_type{static_cast<const rich_line_type &>(other)}, id{other.id} {}

        template<typename RichLineT>
        track(const track<RichLineT> &other)
                : rich_line_type{static_cast<const RichLineT &>(other)}, id{other.id} {}

        track(track &&other) noexcept
                : rich_line_type{std::move(static_cast<rich_line_type &&>(other))}, id{std::move(other.id)} {}

        template<typename RichLineT>
        track(track<RichLineT> &&other) noexcept
                : rich_line_type{std::move(static_cast<RichLineT &&>(other))}, id{std::move(other.id)} {}

        track &operator=(const track &other) {
            rich_line_type::operator=(static_cast<const rich_line_type &>(other));
            id = other.id;
            return *this;
        }

        template<typename RichLineT>
        track &operator=(const track<RichLineT> &other) {
            rich_line_type::operator=(static_cast<const RichLineT &>(other));
            id = other.id;
            return *this;
        }

        track &operator=(track &&other) noexcept {
            rich_line_type::operator=(std::move(static_cast<rich_line_type &&>(other)));
            id = std::move(other.id);
            return *this;
        }

        template<typename RichLineT>
        track &operator=(track<RichLineT> &&other) noexcept {
            rich_line_type::operator=(std::move(static_cast<RichLineT &&>(other)));
            id = std::move(other.id);
            return *this;
        }

        [[nodiscard]] track thin_out(const absl::btree_set<std::size_t> &indices_to_remove) const {
            line_type new_line;
            new_line.reserve(this->size() - indices_to_remove.size());

            line_type line = this->line();
            for (std::size_t i = 0; i < line.size(); ++i) {
                if (not indices_to_remove.contains(i)) {
                    new_line.emplace_back(line[i]);
                }
            }

            return track{id, std::move(new_line)};
        }

        template<typename TrackReprojected, typename SRSOriginal, typename SRSProjected>
        TrackReprojected reproject(SRSOriginal from, SRSProjected to) const {
            using original_line_type = line_type;
            using reprojected_line_type = typename TrackReprojected::line_type;

            boost::geometry::srs::transformation<> transformation{from, to};

            const original_line_type &line_original = this->line();
            reprojected_line_type line_reprojected;
            line_reprojected.reserve(line_original.size());

            transformation.forward(line_original, line_reprojected);

            return TrackReprojected{id, std::move(line_reprojected)};
        }

        [[nodiscard]] std::vector<std::string> header() const {
            return {"id", "geometry", "measurements"};
        }

        [[nodiscard]] std::vector<std::string> row() const {
            std::vector<std::string> row;
            row.reserve(2);
            row.emplace_back(this->id);
            row.emplace_back(this->wkt());
            return row;
        }

        [[nodiscard]] std::string str() const {
            const auto &fields = row();
            std::string str = "Track(";
            bool first = true;
            for (const auto &field: fields) {
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
                    return left.size() < right.size();
                });
            } else {
                std::sort(tracks.begin(), tracks.end(), [](const auto &left, const auto &right) {
                    return left.size() > right.size();
                });
            }
        }

        friend std::ostream &operator<<(std::ostream &out, const track &track) {
            return out << track.str();
        }

    };

    template<typename Point>
    using track_type = track<rich_line_type<Point>>;

    template<typename Point>
    using eager_track_type = track<eager_rich_line_type<Point>>;

    template<typename Point>
    using lazy_track_type = track<lazy_rich_line_type<Point>>;

}

#endif //MAP_MATCHING_2_TRACK_HPP
