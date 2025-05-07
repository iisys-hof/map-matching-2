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

#ifndef MAP_MATCHING_2_IO_TRACK_SEATTLE_GROUND_TRUTH_IMPORTER_HPP
#define MAP_MATCHING_2_IO_TRACK_SEATTLE_GROUND_TRUTH_IMPORTER_HPP

#include <fstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/geometry/io/wkt/read.hpp>
#include <boost/unordered/unordered_flat_map.hpp>

#include <osmium/osm/types.hpp>

#include "../importer.hpp"

namespace map_matching_2::io::track {

    template<typename Forwarder>
    class seattle_ground_truth_importer : public importer {

    public:
        using forwarder_type = Forwarder;

        seattle_ground_truth_importer(std::vector<std::string> filenames,
                forwarder_type &forwarder,
                const geometry::srs_transform &srs_transform,
                const geometry::point_reprojector_variant &reprojector_variant)
            : importer{std::move(filenames)}, _forwarder{forwarder}, _srs_transform{srs_transform},
            _reprojector_variant{reprojector_variant} {}

        void read() override {
            _parse_dispatch();
        }

    private:
        forwarder_type &_forwarder;
        const geometry::srs_transform &_srs_transform;
        const geometry::point_reprojector_variant &_reprojector_variant;

        void _parse_dispatch() {
            geometry::srs_dispatch(_srs_transform, [this]<typename InputCS, typename OutputCS>() {
                using point_type_in = geometry::point_type<InputCS>;
                using point_type_out = geometry::time_point_type<OutputCS>;

                using line_type = typename geometry::models<point_type_in>::template line_type<>;

                using multi_track_type = geometry::track::multi_track_type<point_type_out>;

                const auto &filenames = this->filenames();
                if (filenames.size() != 2) {
                    throw std::invalid_argument{
                            "both the road_network.txt and the ground_truth_route.txt need to be given as track-files."
                    };
                }

                for (std::size_t i = 0; i + 1 < filenames.size(); i += 2) {
                    if (not std::filesystem::exists(filenames[i])) {
                        std::cerr << std::format("file '{}' does not exist, skipping ...", filenames[i]) << std::endl;
                        continue;
                    }

                    std::ifstream seattle;
                    seattle.open(filenames[i]);

                    boost::unordered::unordered_flat_map<std::uint64_t, std::pair<line_type, bool>> lines;

                    std::vector<std::string> parts;

                    std::string line;
                    // skip header
                    std::getline(seattle, line);
                    // read body
                    while (std::getline(seattle, line)) {
                        boost::split(parts, line, boost::is_any_of("\t"));

                        std::uint64_t edge_id = std::stoul(parts.at(0));
                        bool two_way = std::stoi(parts.at(3)) == 1;
                        std::string wkt_line = parts.at(6);

                        boost::trim(wkt_line);

                        line_type edge_line;
                        boost::geometry::read_wkt(wkt_line, edge_line);

                        lines.emplace(edge_id, std::pair{std::move(edge_line), two_way});
                    }

                    if (not std::filesystem::exists(filenames[i + 1])) {
                        std::cerr << std::format("file '{}' does not exist, skipping ...", filenames[i + 1]) <<
                                std::endl;
                        continue;
                    }

                    std::ifstream route;
                    route.open(filenames[i + 1]);

                    line_type ground_truth;

                    // skip header
                    std::getline(route, line);
                    // read body
                    while (std::getline(route, line)) {
                        boost::split(parts, line, boost::is_any_of("\t"));

                        std::uint64_t edge_id = std::stoul(parts.at(0));
                        bool from_to_to = std::stoi(parts.at(1)) == 1;

                        const auto &edge_pair = lines.at(edge_id);
                        const line_type &edge = edge_pair.first;
                        const bool two_way = edge_pair.second;

                        if (from_to_to) {
                            std::copy(ground_truth.empty() ? edge.cbegin() : std::next(edge.cbegin()),
                                    edge.cend(), std::back_inserter(ground_truth));
                        } else if (two_way) {
                            std::copy(ground_truth.empty() ? edge.crbegin() : std::next(edge.crbegin()),
                                    edge.crend(), std::back_inserter(ground_truth));
                        } else {
                            throw std::runtime_error{
                                    "edge is traversed in reverse direction but edge is defined as one-way"
                            };
                        }
                    }

                    std::string id_str{std::to_string(i / 2)};
                    _forwarder.pass(multi_track_type{id_str, std::move(ground_truth), _reprojector_variant});
                }
            });
        }

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_SEATTLE_GROUND_TRUTH_IMPORTER_HPP
