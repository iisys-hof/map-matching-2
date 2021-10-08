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

#ifndef MAP_MATCHING_2_MULTI_TRACK_HPP
#define MAP_MATCHING_2_MULTI_TRACK_HPP

#include "../multi_rich_line.hpp"
#include "track.hpp"

namespace map_matching_2::geometry::track {

    template<typename Measurement>
    struct multi_track : multi_rich_line<boost::geometry::model::linestring<typename Measurement::point_type>> {

        using measurement_type = Measurement;
        using point_type = typename Measurement::point_type;
        using line_type = boost::geometry::model::linestring<point_type>;
        using rich_line_type = rich_line<line_type>;
        using distance_type = typename rich_line_type::distance_type;
        using track_type = track<Measurement>;
        using multi_rich_line_type = multi_rich_line<line_type>;

        std::string id;
        std::vector<track_type> tracks;

        multi_track();

        multi_track(std::string id, track_type track);

        multi_track(std::string id, line_type line);

        multi_track(std::string id, rich_line_type rich_line);

        multi_track(std::string id, std::vector<measurement_type> measurements);

        multi_track(std::string id, multi_rich_line_type multi_rich_line);

        multi_track(std::string id, std::vector<track_type> tracks);

        ~multi_track() = default;

        multi_track(const multi_track &other) = default;

        multi_track(multi_track &&other) noexcept = default;

        multi_track &operator=(const multi_track &other) = default;

        multi_track &operator=(multi_track &&other) noexcept = default;

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3);

        template<typename Network>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr);

        template<typename TrackReprojected, typename SRSOriginal, typename SRSProjected>
        TrackReprojected reproject(SRSOriginal from, SRSProjected to) const {
            using reprojected_point_type = typename TrackReprojected::point_type;
            using reprojected_measurement = measurement<reprojected_point_type>;
            using track_reprojected_type = track<reprojected_measurement>;

            std::vector<track_reprojected_type> reprojected_tracks;
            reprojected_tracks.reserve(tracks.size());
            for (const track_type &track: tracks) {
                reprojected_tracks.emplace_back(
                        track.template reproject<track_reprojected_type, SRSOriginal, SRSProjected>(from, to));
            }

            return TrackReprojected{id, std::move(reprojected_tracks)};
        }

        [[nodiscard]] std::vector<std::string> header() const;

        [[nodiscard]] std::vector<std::string> row() const;

        [[nodiscard]] std::string str() const;

        static void sort_tracks(std::vector<multi_track> &multi_tracks, bool ascending = true);

        template<typename T>
        friend std::ostream &operator<<(std::ostream &out, const multi_track<T> &multi_track);

    };

}

#endif //MAP_MATCHING_2_MULTI_TRACK_HPP
