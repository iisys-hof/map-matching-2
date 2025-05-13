// Copyright (C) 2023-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GRAPH_ALGORITHM_VISITOR_HPP
#define MAP_MATCHING_2_GRAPH_ALGORITHM_VISITOR_HPP

namespace map_matching_2::graph {

    class visitor {

    public:
        [[nodiscard]] constexpr bool stopped() const {
            return _stop;
        }

        void stop() {
            _stop = true;
        }

        template<typename Graph>
        void discover_vertex(const Graph &graph, const typename Graph::vertex_descriptor &vertex_desc) {}

        template<typename Graph>
        void examine_vertex(const Graph &graph, const typename Graph::vertex_descriptor &vertex_desc) {}

        template<typename Graph>
        void examine_edge(const Graph &graph, const typename Graph::edge_descriptor &edge_desc) {}

        template<typename Graph>
        void finish_vertex(const Graph &graph, const typename Graph::vertex_descriptor &vertex_desc) {}

    private:
        bool _stop{false};

    };

}

#endif //MAP_MATCHING_2_GRAPH_ALGORITHM_VISITOR_HPP
