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

#include "input_importer.hpp"

#include <iostream>

#include <boost/algorithm/string.hpp>

#include <geometry/track/types.hpp>

namespace map_matching_2::io::track {

    template<typename MultiTrack>
    input_importer<MultiTrack>::input_importer(std::unordered_map<std::string, MultiTrack> &tracks)
            : _tracks{tracks} {}

    template<typename MultiTrack>
    void input_importer<MultiTrack>::read() {
        std::size_t id = 0;
        std::string wkt_string;
        while (std::getline(std::cin, wkt_string)) {
            boost::trim(wkt_string);
            if (wkt_string.empty()) {
                break;
            }

            try {
                std::string id_str = std::to_string(id++);
                boost::to_upper(wkt_string);
                if (wkt_string.starts_with("POINT")) {
                    std::vector<measurement_type> measurements;
                    std::uint64_t timestamp = 0;
                    boost::tokenizer tokenizer{wkt_string, separator};
                    for (auto wkt_point: tokenizer) {
                        boost::trim(wkt_point);
                        point_type point;
                        boost::geometry::read_wkt(wkt_point, point);
                        measurements.emplace_back(measurement_type{timestamp++, std::move(point)});
                    }
                    _tracks.emplace(id_str, MultiTrack{id_str, std::move(measurements)});
                } else if (wkt_string.starts_with("LINESTRING")) {
                    line_type line;
                    boost::geometry::read_wkt(wkt_string, line);
                    _tracks.emplace(id_str, MultiTrack{id_str, std::move(line)});
                } else {
                    throw std::exception{};
                }
            } catch (std::exception &e) {
                std::clog << "Could not parse the following track, skipping ...\n" << wkt_string << std::endl;
            }
        }
    }

    template
    class input_importer<geometry::track::types_geographic::multi_track_type>;

    template
    class input_importer<geometry::track::types_cartesian::multi_track_type>;

}
