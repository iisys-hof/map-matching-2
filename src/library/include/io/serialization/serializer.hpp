// Copyright (C) 2022-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_SERIALIZATION_SERIALIZER_HPP
#define MAP_MATCHING_2_IO_SERIALIZATION_SERIALIZER_HPP

#include <fstream>

#include <boost/archive/binary_oarchive.hpp>

#include "io/exporter.hpp"

namespace map_matching_2::io::serialization {

    template<typename Data>
    class serializer : public exporter {

    public:
        constexpr serializer(std::string filename, const Data &data)
            : exporter{std::move(filename)}, _data{data} {}

        void write() const {
            std::ofstream file;
            file.open(this->filename(), std::ios_base::binary | std::ios_base::trunc);

            if (file.is_open() && file.good()) {
                boost::archive::binary_oarchive out{file};
                out << _data;
            }
        }

    private:
        const Data &_data;

    };

}

#endif //MAP_MATCHING_2_IO_SERIALIZATION_SERIALIZER_HPP
