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

#ifndef MAP_MATCHING_2_EDGES_LIST_IMPORTER_HPP
#define MAP_MATCHING_2_EDGES_LIST_IMPORTER_HPP

#include <vector>

#include <absl/container/flat_hash_map.h>

#include "../importer.hpp"

namespace map_matching_2::io::track {

    template<typename Network, typename MultiTrack>
    class edges_list_importer : public importer {

    public:
        using measurement_type = typename MultiTrack::measurement_type;
        using point_type = typename measurement_type::point_type;
        using line_type = typename MultiTrack::line_type;

        edges_list_importer(std::string filename, const Network &_network,
                            absl::flat_hash_map<std::string, MultiTrack> &tracks);

        void read() override;

    private:
        const Network &_network;
        absl::flat_hash_map<std::string, MultiTrack> &_tracks;

    };

}

#endif //MAP_MATCHING_2_EDGES_LIST_IMPORTER_HPP
