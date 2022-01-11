// Copyright (C) 2022 Adrian Wöltche
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

#include "network_saver.hpp"

#include <fstream>

#include <boost/archive/binary_oarchive.hpp>

#include <geometry/network/types.hpp>

namespace map_matching_2::io::network {

    template<typename Network>
    network_saver<Network>::network_saver(std::string filename, const Network &network)
            : exporter{std::move(filename)}, _network{network} {}

    template<typename Network>
    void network_saver<Network>::write() {
        std::ofstream file;
        file.open(this->filename(), std::ios_base::binary | std::ios_base::trunc);

        if (file.good()) {
            boost::archive::binary_oarchive out{file};
            out << _network;
        }
    }

    template
    class network_saver<geometry::network::types_geographic::network_static>;

    template
    class network_saver<geometry::network::types_cartesian::network_static>;

}
