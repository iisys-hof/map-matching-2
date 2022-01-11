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

#ifndef MAP_MATCHING_2_NETWORK_SAVER_HPP
#define MAP_MATCHING_2_NETWORK_SAVER_HPP

#include "../exporter.hpp"

namespace map_matching_2::io::network {

    template<typename Network>
    class network_saver : public exporter {

    private:
        const Network &_network;

    public:
        network_saver(std::string filename, const Network &network);

        void write();

    };

}

#endif //MAP_MATCHING_2_NETWORK_SAVER_HPP
