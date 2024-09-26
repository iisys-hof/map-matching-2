// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_NETWORK_BASE_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_NETWORK_BASE_HPP

#include "io/csv_exporter.hpp"

#include "geometry/rich_type/traits/rich_line.hpp"

#include "graph/traits/graph.hpp"

namespace map_matching_2::geometry::network {

    template<typename GraphHelper>
    class network_base {

    public:
        using graph_helper_type = GraphHelper;
        using tag_helper_type = typename graph_helper_type::tag_helper_type;

        using graph_type = typename graph_helper_type::graph_type;
        using vertex_data_type = typename graph::graph_traits<graph_type>::vertex_data_type;
        using edge_data_type = typename graph::graph_traits<graph_type>::edge_data_type;
        using vertex_descriptor = typename graph::graph_traits<graph_type>::vertex_descriptor;
        using vertex_size_type = typename graph::graph_traits<graph_type>::vertex_size_type;
        using edge_descriptor = typename graph::graph_traits<graph_type>::edge_descriptor;
        using edge_size_type = typename graph::graph_traits<graph_type>::edge_size_type;

        using rich_line_type = typename edge_data_type::rich_line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using coordinate_type = typename rich_line_traits<rich_line_type>::coordinate_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;

        constexpr network_base(graph_helper_type graph_helper = {})
            : _graph_helper{std::move(graph_helper)} {}

        constexpr network_base(const network_base &other) = default;

        constexpr network_base(network_base &&other) noexcept = default;

        constexpr network_base &operator=(const network_base &other) = default;

        constexpr network_base &operator=(network_base &&other) noexcept = default;

        constexpr ~network_base() = default;

        [[nodiscard]] constexpr const graph_helper_type &graph_helper() const {
            return _graph_helper;
        }

        [[nodiscard]] constexpr graph_helper_type &graph_helper() {
            return _graph_helper;
        }

        [[nodiscard]] constexpr const tag_helper_type &tag_helper() const {
            return graph_helper().tag_helper();
        }

        [[nodiscard]] constexpr tag_helper_type &tag_helper() {
            return graph_helper().tag_helper();
        }

        [[nodiscard]] constexpr const graph_type &graph() const {
            return graph_helper().graph();
        }

        [[nodiscard]] constexpr graph_type &graph() {
            return graph_helper().graph();
        }

        [[nodiscard]] constexpr const vertex_data_type &vertex(const vertex_descriptor &vertex_desc) const {
            return graph().get_vertex_data(vertex_desc);
        }

        [[nodiscard]] constexpr vertex_data_type &vertex(const vertex_descriptor &vertex_desc) {
            return graph().get_vertex_data(vertex_desc);
        }

        [[nodiscard]] constexpr const edge_data_type &edge(const edge_descriptor &edge_desc) const {
            return graph().get_edge_data(edge_desc);
        }

        [[nodiscard]] constexpr edge_data_type &edge(const edge_descriptor &edge_desc) {
            return graph().get_edge_data(edge_desc);
        }

        [[nodiscard]] length_type length() const {
            length_type length = geometry::default_float_type<length_type>::v0;
            const auto edges_v = this->graph().edges_view();
            for (auto it = edges_v.begin(); it != edges_v.end(); it = edges_v.next(it)) {
                const edge_data_type &edge = edges_v[it].get();
                length += edge.rich_line.length();
            }
            return length;
        }

        void save_nodes_csv(std::string filename) const {
            io::csv_exporter<',', '"'> csv{std::move(filename)};
            bool first = true;
            const auto vertices_v = graph().vertices_view();
            for (auto it = vertices_v.begin(); it != vertices_v.end(); it = vertices_v.next(it)) {
                const vertex_data_type &vertex = vertices_v[it].get();
                if (first) {
                    csv << vertex.header();
                    first = false;
                }
                csv << vertex.row(tag_helper());
            }
        }

        void save_edges_csv(std::string filename) const {
            io::csv_exporter<',', '"'> csv{std::move(filename)};
            bool first = true;
            const auto edges_v = graph().edges_view();
            for (auto it = edges_v.begin(); it != edges_v.end(); it = edges_v.next(it)) {
                const edge_data_type &edge = edges_v[it].get();
                if (first) {
                    csv << edge.header();
                    first = false;
                }
                csv << edge.row(tag_helper());
            }
        }

        void save_csv(std::string nodes, std::string edges) const {
            save_nodes_csv(std::move(nodes));
            save_edges_csv(std::move(edges));
        }

    protected:
        graph_helper_type _graph_helper;

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_NETWORK_BASE_HPP
