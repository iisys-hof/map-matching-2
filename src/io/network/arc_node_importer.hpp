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

#ifndef MAP_MATCHING_2_ARC_NODE_IMPORTER_HPP
#define MAP_MATCHING_2_ARC_NODE_IMPORTER_HPP

#include <vector>
#include <unordered_map>

#include "../importer.hpp"

namespace map_matching_2::io::network {

    template<typename Network>
    class arc_node_importer : public importer {

    private:
        Network &_network;
        const std::string _nodes;

        std::vector<typename Network::node_type> _node_map;
        std::unordered_map<std::size_t, typename Network::vertex_descriptor> _vertex_map;

    public:
        arc_node_importer(std::string arcs, std::string nodes, Network &network);

        void read() override;

        [[nodiscard]] const std::string &arcs() const;

        [[nodiscard]] const std::string &nodes() const;

    };

}

#endif //MAP_MATCHING_2_ARC_NODE_IMPORTER_HPP
