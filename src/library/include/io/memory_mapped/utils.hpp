// Copyright (C) 2022-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_UTILS_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_UTILS_HPP

#include <cstddef>
#include <functional>
#include <filesystem>

#include <boost/interprocess/managed_mapped_file.hpp>

#include "util/macros.hpp"

namespace map_matching_2::io::memory_mapped {

    template<typename T>
    concept has_grow = requires(T t) {
        { t.grow() };
    };

    struct mmap_start_remover {
        const char *filename;

        explicit mmap_start_remover(const char *filename)
            : filename{filename} {
            boost::interprocess::file_mapping::remove(this->filename);
        }
    };

    struct mmap_remover : public mmap_start_remover {
        explicit mmap_remover(const char *filename)
            : mmap_start_remover{filename} {}

        ~mmap_remover() {
            boost::interprocess::file_mapping::remove(this->filename);
        }
    };

    template<typename File, typename Allocator, typename Data>
    void mmap_auto_grow(File *&file, Allocator *&allocator, Data *&data, const char *filename,
            std::size_t &size, const char *data_identifier) {
        file->flush();

        if (allocator) {
            delete allocator;
        }
        if (file) {
            delete file;
        }

        File::grow(filename, size);
        size *= 2;

        file = new File{boost::interprocess::open_only, filename};
        allocator = new Allocator{file->get_segment_manager()};
        data = file->template find<Data>(data_identifier).first;
    }

    template<typename Try, typename Catch>
    auto retry_alloc(const Try &try_function, const Catch &catch_function, bool &critical,
            const std::size_t retries = 1) {
        if (critical) {
            return try_function();
        } else {
            std::size_t tries = 0;
            while (tries <= retries) {
                try {
                    critical = true;
                    if constexpr (std::is_void_v<std::invoke_result_t<Try>>) {
                        try_function();
                        critical = false;
                        return;
                    } else {
                        auto ret = try_function();
                        critical = false;
                        return ret;
                    }
                } catch (boost::interprocess::bad_alloc &ex) {
                    catch_function(ex);
                    if (++tries > retries) {
                        critical = false;
                        throw;
                    }
                }
            }
        }

        throw boost::interprocess::bad_alloc{};
    }

    template<typename Try, typename Catch>
    auto retry_alloc(const Try &try_function, const Catch &catch_function, const std::size_t retries = 1) {
        bool critical = false;
        return retry_alloc(try_function, catch_function, critical, retries);
    }

    template<typename Try>
    auto retry_alloc(const Try &try_function, bool &critical, const std::size_t retries = 1) {
        const auto &catch_function = [](auto &ex) constexpr -> void {};

        return retry_alloc(try_function, catch_function, critical, retries);
    }

    template<typename Try>
    auto retry_alloc(const Try &try_function, const std::size_t retries = 1) {
        bool critical = false;
        return retry_alloc(try_function, critical, retries);
    }

    [[nodiscard]] std::size_t get_total_memory();

    enum SPARSE_STATUS {
        NO_SPARSE_FILE,
        IS_SPARSE_FILE
    };

#if defined(MM2_WINDOWS)

    SPARSE_STATUS make_sparse_file(const std::string &file);

    void grow_file(const std::string &file, std::size_t size);

    bool flush_file(const std::string &file, void *addr, std::size_t size, bool async = false);

#elif defined(MM2_LINUX)

    bool flush_file(void *addr, std::size_t size, bool async = false);

#endif

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_UTILS_HPP
