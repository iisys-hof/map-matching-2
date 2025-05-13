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

#ifndef MAP_MATCHING_2_IO_SERIALIZATION_DESERIALIZER_HPP
#define MAP_MATCHING_2_IO_SERIALIZATION_DESERIALIZER_HPP

#include <fstream>
#include <filesystem>

#include <boost/archive/binary_iarchive.hpp>

#include "io/importer.hpp"

namespace map_matching_2::io::serialization {

    template<typename Data>
    class deserializer : public importer {

    public:
        constexpr deserializer(std::string filename, Data &data)
            : importer{std::move(filename)}, _data{data} {}

        void read() override {
            const std::string &filename = this->filenames().at(0);

            std::filesystem::path path{filename};
            if (not std::filesystem::exists(path)) {
                throw std::runtime_error("file " + filename + " does not exist.");
            }

            std::ifstream file;
            file.open(path, std::ios_base::binary);
            if (file.is_open() && file.good()) {
                boost::archive::binary_iarchive in{file};
                in >> _data;
            }
        }

    private:
        Data &_data;

    };

}

#endif //MAP_MATCHING_2_IO_SERIALIZATION_DESERIALIZER_HPP
