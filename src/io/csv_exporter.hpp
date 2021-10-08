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

#ifndef MAP_MATCHING_2_CSV_EXPORTER_HPP
#define MAP_MATCHING_2_CSV_EXPORTER_HPP

#include <csv.hpp>

#include "exporter.hpp"

namespace map_matching_2::io {

    template<char Delimiter = ',', char Quote = '"', bool Flush = false>
    class csv_exporter : public exporter, public csv::DelimWriter<std::ofstream, Delimiter, Quote, Flush> {

    public:
        explicit csv_exporter(std::string filename);

    };

}

#endif //MAP_MATCHING_2_CSV_EXPORTER_HPP
