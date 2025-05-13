// Copyright (C) 2021-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_TRACK_EDGES_LIST_IMPORTER_HPP
#define MAP_MATCHING_2_IO_TRACK_EDGES_LIST_IMPORTER_HPP

#include <fstream>
#include <string>
#include <format>
#include <vector>
#include <queue>

#include <boost/algorithm/string.hpp>

#include "types/geometry/track/multi_track.hpp"
#include "types/geometry/reprojector.hpp"

#include "geometry/algorithm/srs_dispatch.hpp"
#include "geometry/algorithm/equals.hpp"

#include "graph/traits/graph.hpp"

#include "util/checks.hpp"

#include "../importer.hpp"

namespace map_matching_2::io::track {

    template<typename Forwarder, typename Graph>
    class edges_list_importer : public importer {

    public:
        using forwarder_type = Forwarder;
        using graph_type = Graph;

        using multi_track_variant_type = typename forwarder_type::multi_track_variant_type;

        using edge_data_type = typename graph::graph_traits<Graph>::edge_data_type;
        using edge_descriptor = typename graph::graph_traits<Graph>::edge_descriptor;

        edges_list_importer(std::vector<std::string> filenames, forwarder_type &forwarder,
                const geometry::srs_transform &srs_transform,
                const geometry::point_reprojector_variant &reprojector_variant, const graph_type &graph,
                const std::string &delimiter, const std::vector<std::string> &columns, const std::size_t skip_lines)
            : importer{std::move(filenames)}, _forwarder{forwarder}, _srs_transform{srs_transform},
            _reprojector_variant{reprojector_variant}, _graph{graph}, _delimiters{parse_delimiter(delimiter)},
            _columns{columns}, _skip_lines{skip_lines} {}

        void read() override {
            const auto edges_v = _graph.edges_view();
            for (auto it = edges_v.begin(); it != edges_v.end(); it = edges_v.next(it)) {
                const edge_data_type &edge = edges_v[it].get();
                std::int64_t edge_id = edge.id;
                auto [iterator, inserted] = _edge_map.try_emplace(edge_id);
                auto &edges = iterator->second;
                edges.emplace_back(it);
            }

            _parse_dispatch();

            _edge_map.clear();
        }

    private:
        using edge_map_type = boost::unordered::unordered_flat_map<std::int64_t, std::vector<edge_descriptor>>;

        forwarder_type &_forwarder;
        const geometry::srs_transform &_srs_transform;
        const geometry::point_reprojector_variant &_reprojector_variant;
        const graph_type &_graph;
        const std::vector<char> _delimiters;
        const std::vector<std::string> _columns;
        const std::size_t _skip_lines;
        edge_map_type _edge_map;

        [[nodiscard]] bool _is_connected(const std::vector<edge_descriptor> &edges) const {
            for (std::size_t i = 0; i + 1 < edges.size(); ++i) {
                const edge_data_type &edge_from = _graph.get_edge_data(edges.at(i));
                const edge_data_type &edge_to = _graph.get_edge_data(edges.at(i + 1));
                if (edge_from.rich_line.empty() or edge_to.rich_line.empty() or
                    not geometry::equals_points(edge_from.rich_line.back(), edge_to.rich_line.front())) {
                    return false;
                }
            }
            return true;
        }

        void _parse_dispatch() {
            geometry::srs_dispatch(_srs_transform, [this]<typename InputCS, typename OutputCS>() {
                using point_type_out = geometry::time_point_type<OutputCS>;

                using edge_rich_line_type = typename edge_data_type::rich_line_type;
                using edge_point_type = typename edge_rich_line_type::point_type;
                using edge_compatible_rich_line_type = geometry::rich_line_type<edge_point_type>;
                using edge_compatible_multi_rich_line_type = geometry::multi_rich_line_type<edge_point_type>;

                using multi_track_type = geometry::track::multi_track_type<point_type_out>;

                const auto &filenames = this->filenames();
                for (std::size_t i = 0; i < filenames.size(); ++i) {
                    if (not std::filesystem::exists(filenames[i])) {
                        std::cerr << std::format("file '{}' does not exist, skipping ...", filenames[i]) << std::endl;
                        continue;
                    }

                    std::ifstream route;
                    route.open(filenames[i]);

                    bool error = false;
                    std::int64_t prev_edge_id = std::numeric_limits<std::int64_t>::min();
                    std::string line;
                    std::vector<std::string> parts;
                    std::vector<std::int64_t> edge_ids;
                    std::vector<std::vector<edge_descriptor>> edge_descriptor_list;
                    for (std::size_t j = 0; j < _skip_lines; ++j) {
                        std::getline(route, line);
                    }
                    while (std::getline(route, line)) {
                        if (not _delimiters.empty()) {
                            boost::split(parts, line, boost::is_any_of(_delimiters));
                        }

                        edge_ids.clear();
                        if (_delimiters.empty()) {
                            edge_ids.emplace_back(std::stol(line));
                        } else {
                            for (const auto &column : _columns) {
                                std::size_t col = 0;
                                if (util::is_number(column)) {
                                    col = std::stoul(column);
                                }
                                edge_ids.emplace_back(std::stol(parts.at(col)));
                            }
                        }

                        for (const auto edge_id : edge_ids) {
                            if (edge_id == prev_edge_id) {
                                // skip duplicate same edge ids
                                continue;
                            }
                            prev_edge_id = edge_id;

                            const auto &edge_descriptors = _edge_map.at(edge_id);
                            if (edge_descriptors.empty()) {
                                std::cerr << std::format(
                                        "file '{}' contains edge with id {} but no such edge exists in the network, skipping ...",
                                        filenames[i], edge_id) << std::endl;
                                error = true;
                                break;
                            }

                            edge_descriptor_list.emplace_back(edge_descriptors);
                        }
                    }

                    if (edge_descriptor_list.empty()) {
                        std::cerr << std::format(
                                "file '{}' did not contain anything to extract, skipping ...",
                                filenames[i]) << std::endl;
                        error = true;
                    }

                    if (error) {
                        // error happened, skip processing of this file
                        continue;
                    }

                    std::vector<edge_compatible_rich_line_type> edges;

                    std::size_t start_step = 0;
                    std::size_t end_step = edge_descriptor_list.size();
                    while (start_step < end_step) {
                        std::size_t step_count = end_step - start_step;
                        // backtracking
                        std::vector<edge_descriptor> path;
                        std::queue<std::pair<std::size_t, std::vector<edge_descriptor>>> queue;

                        queue.push({start_step, path});
                        while (not queue.empty()) {
                            auto [step, current_path] = queue.front();
                            queue.pop();

                            if (_is_connected(current_path)) {
                                // valid solution found
                                if (current_path.size() > path.size()) {
                                    // better than current solution
                                    path = current_path;
                                }
                                if (current_path.size() == step_count) {
                                    // reached the end
                                    start_step = end_step;
                                    break;
                                }
                            }

                            if (step < end_step) {
                                const auto &edge_descriptors = edge_descriptor_list.at(step);
                                for (const auto &edge_descriptor : edge_descriptors) {
                                    auto new_path = current_path;
                                    new_path.emplace_back(edge_descriptor);
                                    if (_is_connected(new_path)) {
                                        queue.push({step + 1, new_path});
                                    }
                                }
                            }
                        }

                        for (const edge_descriptor &edge_desc : path) {
                            const edge_data_type &edge_from = _graph.get_edge_data(edge_desc);
                            edges.emplace_back(edge_compatible_rich_line_type{edge_from.rich_line.line()});
                        }

                        start_step += path.empty() ? 1 : path.size();
                    }

                    // found paths through the network
                    const auto edge_multi_rich_line = edge_compatible_multi_rich_line_type{}.merge(std::move(edges));

                    std::string id_str{std::to_string(i)};
                    _forwarder.pass(multi_track_type{id_str, edge_multi_rich_line.multi_line(), _reprojector_variant});
                }
            });
        }

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_EDGES_LIST_IMPORTER_HPP
