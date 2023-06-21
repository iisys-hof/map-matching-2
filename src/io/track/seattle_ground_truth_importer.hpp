// Copyright (C) 2021-2023 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_SEATTLE_GROUND_TRUTH_IMPORTER_HPP
#define MAP_MATCHING_2_SEATTLE_GROUND_TRUTH_IMPORTER_HPP

#include <fstream>
#include <string>
#include <vector>

#include <absl/container/flat_hash_map.h>

#include <geometry/network/network.hpp>

#include "../importer.hpp"

namespace map_matching_2::io::track {

    template<typename MultiTrack>
    class seattle_ground_truth_importer : public importer {

    public:
        using measurement_type = typename MultiTrack::measurement_type;
        using point_type = typename MultiTrack::point_type;
        using line_type = typename MultiTrack::line_type;

        seattle_ground_truth_importer(std::string filename, std::string network_file,
                                      absl::flat_hash_map<std::string, MultiTrack> &tracks)
                : importer{std::move(filename)}, _network_file{std::move(network_file)}, _tracks{tracks} {}

        void read() override {
            std::ifstream seattle;
            seattle.open(_network_file);

            std::vector<std::string> parts;

            std::string line;
            // skip header
            std::getline(seattle, line);
            // read body
            while (std::getline(seattle, line)) {
                boost::split(parts, line, boost::is_any_of("\t"));

                osmium::object_id_type edge_id = std::stol(parts[0]);
                bool two_way = std::stoi(parts[3]) == 1;
                std::string wkt_line = parts[6];

                boost::trim(wkt_line);

                line_type edge_line;
                boost::geometry::read_wkt(wkt_line, edge_line);

                _lines.emplace(edge_id, std::pair{std::move(edge_line), two_way});
                parts.clear();
            }

            std::ifstream route;
            route.open(this->filename());

            line_type ground_truth;

            // skip header
            std::getline(route, line);
            // read body
            while (std::getline(route, line)) {
                boost::split(parts, line, boost::is_any_of("\t"));

                osmium::object_id_type edge_id = std::stol(parts[0]);
                bool from_to_to = std::stoi(parts[1]) == 1;

                const auto &edge_pair = _lines.at(edge_id);
                const line_type &edge = edge_pair.first;
                const bool two_way = edge_pair.second;

                if (from_to_to) {
                    std::copy(ground_truth.empty() ? edge.cbegin() : std::next(edge.cbegin()),
                              edge.cend(), std::back_inserter(ground_truth));
                } else if (two_way) {
                    std::copy(ground_truth.empty() ? edge.crbegin() : std::next(edge.crbegin()),
                              edge.crend(), std::back_inserter(ground_truth));
                } else {
                    throw std::runtime_error{"edge is traversed in reverse direction but edge is defined as one-way"};
                }

                parts.clear();
            }

            std::string id{"0"};
            _tracks.emplace(id, MultiTrack{id, ground_truth});
        }

    private:
        const std::string _network_file;
        absl::flat_hash_map<osmium::object_id_type, std::pair<line_type, bool>> _lines;
        absl::flat_hash_map<std::string, MultiTrack> &_tracks;

    };

}

#endif //MAP_MATCHING_2_SEATTLE_GROUND_TRUTH_IMPORTER_HPP
