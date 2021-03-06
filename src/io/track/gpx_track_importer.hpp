// Copyright (C) 2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GPX_TRACK_IMPORTER_HPP
#define MAP_MATCHING_2_GPX_TRACK_IMPORTER_HPP

#include <string>

#include <absl/container/flat_hash_map.h>

#include "../importer.hpp"

namespace map_matching_2::io::track {

    template<typename MultiTrack>
    class gpx_track_importer : public importer {

    protected:
        absl::flat_hash_map<std::string, MultiTrack> &_tracks;

    public:
        using measurement_type = typename MultiTrack::measurement_type;
        using point_type = typename measurement_type::point_type;
        using line_type = typename MultiTrack::line_type;

        explicit gpx_track_importer(std::string filename, absl::flat_hash_map<std::string, MultiTrack> &tracks);

        void read() override;

    };

}

#endif //MAP_MATCHING_2_GPX_TRACK_IMPORTER_HPP
