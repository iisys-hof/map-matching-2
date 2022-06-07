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

    template<typename Measurement>
    struct track : rich_line<boost::geometry::model::linestring<typename Measurement::point_type>> {

        using measurement_type = Measurement;
        using point_type = typename Measurement::point_type;
        using line_type = boost::geometry::model::linestring<point_type>;
        using rich_line_type = rich_line<line_type>;
        using distance_type = typename rich_line_type::distance_type;

        std::string id;

        track();

        track(std::string id, line_type line);

        track(std::string id, rich_line_type rich_line);

        track(std::string id, std::vector<measurement_type> measurements);

        ~track() = default;

        track(const track &other) = default;

        track(track &&other) noexcept = default;

        track &operator=(const track &other) = default;

        track &operator=(track &&other) noexcept = default;

        track thin_out(const absl::btree_set<std::size_t> &indices_to_remove) const;

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3);

        template<typename Network>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr);

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

        [[nodiscard]] std::vector<std::string> header() const;

        [[nodiscard]] std::vector<std::string> row() const;

        [[nodiscard]] std::string str() const;

        static track
        create_track(std::initializer_list<typename boost::geometry::coordinate_type<point_type>::type> coordinates);

        static void sort_tracks(std::vector<track> &tracks, bool ascending = true);

        template<typename T>
        friend std::ostream &operator<<(std::ostream &out, const track<T> &track);

    };

}

#endif //MAP_MATCHING_2_TRACK_HPP
