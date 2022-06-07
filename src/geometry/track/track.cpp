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

#include "track.hpp"

#include "../network/types.hpp"
#include "../types.hpp"
#include "../util.hpp"

namespace map_matching_2::geometry::track {


    template<typename Measurement>
    track<Measurement>::track() : track{"", rich_line_type{}} {}

    template<typename Measurement>
    track<Measurement>::track(std::string id, line_type line)
            : rich_line_type{std::move(line)}, id{std::move(id)} {}

    template<typename Measurement>
    track<Measurement>::track(std::string id, rich_line_type rich_line)
            : rich_line_type{std::move(rich_line)}, id{std::move(id)} {}

    template<typename Measurement>
    track<Measurement>::track(std::string id, std::vector<measurement_type> measurements)
            : rich_line_type{Measurement::measurements2line(measurements)}, id{std::move(id)} {}

    template<typename Measurement>
    track<Measurement>
    track<Measurement>::thin_out(const absl::btree_set<std::size_t> &indices_to_remove) const {
        line_type new_line;
        new_line.reserve(this->line().size() - indices_to_remove.size());

        const line_type &line = this->line();
        for (std::size_t i = 0; i < line.size(); ++i) {
            if (not indices_to_remove.contains(i)) {
                new_line.emplace_back(line[i]);
            }
        }

        return track{id, std::move(new_line)};
    }

    template<typename Measurement>
    void
    track<Measurement>::simplify(const bool retain_reversals, const double tolerance, const double reverse_tolerance) {
        static_cast<rich_line_type *>(this)->simplify(retain_reversals, tolerance, reverse_tolerance);
    }

    template<typename Measurement>
    template<typename Network>
    void track<Measurement>::median_merge(const double tolerance, const bool adaptive, const Network *network) {
        static_cast<rich_line_type *>(this)->median_merge(tolerance, adaptive, network);
    }

    template<typename Measurement>
    std::vector<std::string> track<Measurement>::header() const {
        return {"id", "geometry", "measurements"};
    }

    template<typename Measurement>
    std::vector<std::string> track<Measurement>::row() const {
        std::vector<std::string> row;
        row.reserve(2);
        row.emplace_back(this->id);
        row.emplace_back(this->wkt());
        return row;
    }

    template<typename Measurement>
    std::string track<Measurement>::str() const {
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

    template<typename Measurement>
    track<Measurement>
    track<Measurement>::create_track(
            const std::initializer_list<typename boost::geometry::coordinate_type<point_type>::type> coordinates) {
        line_type new_line;
        new_line.reserve(coordinates.size() / 2);
        for (auto it = coordinates.begin(); it != coordinates.end(); ++it) {
            const auto x = *it;
            const auto y = *(++it);
            new_line.emplace_back(point_type{x, y});
        }

        return track{"", std::move(new_line)};
    }

    template<typename Measurement>
    void track<Measurement>::sort_tracks(std::vector<track> &tracks, const bool ascending) {
        if (ascending) {
            std::sort(tracks.begin(), tracks.end(), [](const auto &left, const auto &right) {
                return left.line().size() < right.line().size();
            });
        } else {
            std::sort(tracks.begin(), tracks.end(), [](const auto &left, const auto &right) {
                return left.line().size() > right.line().size();
            });
        }
    }

    template<typename Measurement>
    std::ostream &operator<<(std::ostream &out, const track<Measurement> &track) {
        out << track.str();
        return out;
    }

    template
    class track<measurement<geometry::types_geographic::point_type>>;

    template
    class track<measurement<geometry::types_cartesian::point_type>>;

    template
    void track<measurement<geometry::types_geographic::point_type>>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_geographic::network_static *network);

    template
    void track<measurement<geometry::types_cartesian::point_type>>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_cartesian::network_static *network);

    template
    std::ostream &
    operator<<(std::ostream &out, const track <measurement<geometry::types_geographic::point_type>> &track);

    template
    std::ostream &
    operator<<(std::ostream &out, const track <measurement<geometry::types_cartesian::point_type>> &track);

}
