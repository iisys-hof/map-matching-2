// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_CSV_TRACK_IMPORTER_HPP
#define MAP_MATCHING_2_CSV_TRACK_IMPORTER_HPP

#include "../csv_importer.hpp"

namespace map_matching_2::io::track {

    struct csv_track_importer_settings {

        char delimiter = ',';
        bool no_header = false;
        bool no_id = false;
        bool wkt = false;
        bool no_parse_time = false;
        std::vector<std::string> field_id = {"id"};
        std::string id_aggregator = "_";
        std::string field_x = "x";
        std::string field_y = "y";
        std::string field_time = "time";
        std::string time_format = "%FT%T%Oz";
        std::string field_geometry = "geometry";

    };

    template<typename MultiTrack>
    class csv_track_importer : public csv_importer {

    public:
        using measurement_type = typename MultiTrack::measurement_type;
        using point_type = typename measurement_type::point_type;
        using line_type = typename MultiTrack::line_type;
        using multi_line_type = typename MultiTrack::multi_line_type;

        csv_track_importer_settings settings;

        csv_track_importer(std::string filename, std::unordered_map<std::string, MultiTrack> &tracks);

    protected:
        std::unordered_map<std::string, MultiTrack> &_tracks;

        void configure_format(csv::CSVFormat &format) override;

        void process_row(std::size_t n, const csv::CSVRow &row) override;

        void finish_import() override;

    private:
        std::unordered_map<std::string, std::vector<measurement_type>> _measurements;

        [[nodiscard]] bool _is_number(const std::string &str);

    };

}

#endif //MAP_MATCHING_2_CSV_TRACK_IMPORTER_HPP
