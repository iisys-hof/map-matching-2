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

#ifndef MAP_MATCHING_2_MEMORY_MAPPED_FILE_HPP
#define MAP_MATCHING_2_MEMORY_MAPPED_FILE_HPP

#include <boost/interprocess/managed_mapped_file.hpp>

namespace map_matching_2::io::memory_mapped {

    struct mmap_start_remover {
        const char *filename;

        mmap_start_remover(const char *filename)
                : filename{filename} {
            boost::interprocess::file_mapping::remove(this->filename);
        }
    };

    struct mmap_remover : public mmap_start_remover {
        const char *filename;

        mmap_remover(const char *filename)
                : mmap_start_remover{filename} {}

        ~mmap_remover() {
            boost::interprocess::file_mapping::remove(this->filename);
        }
    };

    template<typename MMapFile, typename Allocator, typename Data>
    void mmap_auto_grow(MMapFile *&mmap_file, Allocator *&allocator, Data *&data, const char *filename,
                        std::size_t &size, const char *data_identifier) {
        mmap_file->flush();

        delete allocator;
        delete mmap_file;

        boost::interprocess::managed_mapped_file::grow(filename, size);
        size *= 2;

        mmap_file = new boost::interprocess::managed_mapped_file{boost::interprocess::open_only, filename};
        allocator = new Allocator{mmap_file->get_segment_manager()};
        data = mmap_file->template find<Data>(data_identifier).first;
    }

    template<typename MMapFile, typename Allocator, typename Data, typename ...Values>
    typename Data::reference
    emplace_back_auto_grow(MMapFile *&mmap_file, Allocator *&allocator, Data *&data, const char *filename,
                           std::size_t &size, const char *data_identifier, Values &&...values) {
        std::size_t tries = 0;
        while (tries <= 1) {
            try {
                return data->emplace_back(std::forward<Values>(values)...);
            } catch (boost::interprocess::bad_alloc &ex) {
                tries++;
                std::size_t previous_size = data->size();
                mmap_auto_grow(mmap_file, allocator, data, filename, size, data_identifier);
                if (data->size() != previous_size) {
                    break;
                }
            }
        }

        throw boost::interprocess::bad_alloc{};
    }

}

#endif //MAP_MATCHING_2_MEMORY_MAPPED_FILE_HPP
