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

#include "multi_track.hpp"

#include "../network/types.hpp"
#include "../types.hpp"
#include "../util.hpp"

namespace map_matching_2::geometry::track {


    template<typename Measurement>
    multi_track<Measurement>::multi_track() : multi_track{"", track_type{}} {}

    template<typename Measurement>
    multi_track<Measurement>::multi_track(std::string id, track_type track)
            : multi_track{std::move(id), std::vector{std::move(track)}} {}

    template<typename Measurement>
    multi_track<Measurement>::multi_track(std::string id, line_type line)
            : multi_track{id, track_type{std::move(id), std::move(line)}} {}

    template<typename Measurement>
    multi_track<Measurement>::multi_track(std::string id, rich_line_type rich_line)
            : multi_track{id, track_type{std::move(id), std::move(rich_line)}} {}

    template<typename Measurement>
    multi_track<Measurement>::multi_track(std::string id, std::vector<measurement_type> measurements)
            : multi_track{id, track_type{std::move(id), std::move(measurements)}} {}

    template<typename Measurement>
    multi_track<Measurement>::multi_track(std::string id, std::vector<line_type> lines)
            : multi_track{id, multi_rich_line_type{multi_rich_line_type::lines2multi_lines(lines)}} {}

    template<typename Measurement>
    multi_track<Measurement>::multi_track(std::string id, std::vector<rich_line_type> rich_lines)
            : multi_track{id, multi_rich_line_type{std::move(rich_lines)}} {}

    template<typename Measurement>
    multi_track<Measurement>::multi_track(std::string id, multi_rich_line_type multi_rich_line)
            : multi_rich_line_type{std::move(multi_rich_line)}, id{std::move(id)} {
        tracks.reserve(this->rich_lines.size());
        for (std::size_t i = 0; i < this->rich_lines.size(); ++i) {
            tracks.emplace_back(
                    track_type{id.append("_").append(std::to_string(i)), this->rich_lines[i]});
        }
    }

    template<typename Measurement>
    multi_track<Measurement>::multi_track(std::string id, std::vector<track_type> tracks)
            : id{std::move(id)}, tracks{std::move(tracks)} {
        this->rich_lines.reserve(this->tracks.size());
        for (const rich_line_type &rich_line: this->tracks) {
            this->rich_lines.emplace_back(rich_line);
        }
        this->multi_line.reserve(this->rich_lines.size());
        for (const rich_line_type &rich_line: this->rich_lines) {
            this->multi_line.emplace_back(rich_line.line);
        }
        this->_complete();
    }

    template<typename Measurement>
    void multi_track<Measurement>::simplify(const bool retain_reversals, const double tolerance,
                                            const double reverse_tolerance) {
        for (track_type &track: tracks) {
            track.simplify(retain_reversals, tolerance, reverse_tolerance);
        }
    }

    template<typename Measurement>
    template<typename Network>
    void multi_track<Measurement>::median_merge(const double tolerance, const bool adaptive, const Network *network) {
        for (track_type &track: tracks) {
            track.median_merge(tolerance, adaptive, network);
        }
    }

    template<typename Measurement>
    std::vector<std::string> multi_track<Measurement>::header() const {
        return {"id", "geometry"};
    }

    template<typename Measurement>
    std::vector<std::string> multi_track<Measurement>::row() const {
        std::vector<std::string> row;
        row.reserve(2);
        row.emplace_back(this->id);
        row.emplace_back(this->wkt());
        return row;
    }

    template<typename Measurement>
    std::string multi_track<Measurement>::str() const {
        const auto &fields = row();
        std::string str = "MultiTrack(";
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
    void multi_track<Measurement>::sort_tracks(std::vector<multi_track> &multi_tracks, const bool ascending) {
        const auto size = [&](const multi_track &left, const multi_track &right) {
            std::size_t left_size = 0;
            for (const track_type &track: left.tracks) {
                left_size += track.measurements.size();
            }
            std::size_t right_size = 0;
            for (const track_type &track: right.tracks) {
                right_size += track.measurements.size();
            }
            return std::pair{left_size, right_size};
        };

        if (ascending) {
            std::sort(multi_tracks.begin(), multi_tracks.end(), [&](const auto &left, const auto &right) {
                const auto sizes = size(left, right);
                return sizes.first < sizes.second;
            });
        } else {
            std::sort(multi_tracks.begin(), multi_tracks.end(), [&](const auto &left, const auto &right) {
                const auto sizes = size(left, right);
                return sizes.first > sizes.second;
            });
        }
    }

    template<typename Measurement>
    std::ostream &operator<<(std::ostream &out, const multi_track<Measurement> &multi_track) {
        out << multi_track.str();
        return out;
    }

    template
    class multi_track<measurement<geometry::types_geographic::point_type>>;

    template
    class multi_track<measurement<geometry::types_cartesian::point_type>>;

    template
    void multi_track<measurement<geometry::types_geographic::point_type>>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_geographic::network_static *network);

    template
    void multi_track<measurement<geometry::types_cartesian::point_type>>::median_merge(
            const double tolerance, const bool adaptive,
            const geometry::network::types_cartesian::network_static *network);

    template
    std::ostream &
    operator<<(std::ostream &out, const multi_track <measurement<geometry::types_geographic::point_type>> &multi_track);

    template
    std::ostream &
    operator<<(std::ostream &out, const multi_track <measurement<geometry::types_cartesian::point_type>> &multi_track);

}
