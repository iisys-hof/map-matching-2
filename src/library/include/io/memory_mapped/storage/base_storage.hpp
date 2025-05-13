// Copyright (C) 2023-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_BASE_STORAGE_HPP
#define MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_BASE_STORAGE_HPP

#include <cmath>
#include <string>
#include <filesystem>
#include <format>

#include <boost/interprocess/containers/string.hpp>

#include "util/macros.hpp"
#include "util/compiler.hpp"
#include "util/version.hpp"

#include "../concepts.hpp"
#include "../utils.hpp"

namespace map_matching_2::io::memory_mapped::storage {

    enum open_or_create_tag {
        OPEN_READ_ONLY,
        DESTROY_AND_CREATE
    };

    enum destroy_on_close_tag {
        DO_NOT_DESTROY_ON_CLOSE,
        DESTROY_ON_CLOSE
    };

    enum shrink_on_close_tag {
        DO_NOT_SHRINK_ON_CLOSE,
        SHRINK_ON_CLOSE
    };

    template<has_mmap_type Types>
    class mmap_base_container {

    public:
        constexpr mmap_base_container(const mmap_base_container &other) = delete;

        constexpr mmap_base_container &operator=(const mmap_base_container &other) = delete;

    protected:
        using types = Types;
        using mmap_type = typename types::mmap_type;

        constexpr static const char *info_identifier = "info";

        class info {

        public:
            using allocator_type = typename types::template allocator_type<char>;
            using string_type = boost::interprocess::basic_string<char, std::char_traits<char>, allocator_type>;

            info(const std::string &compiler, const std::string &version, const allocator_type &allocator)
                : _compiler(std::cbegin(compiler), std::cend(compiler), allocator),
                _version(std::cbegin(version), std::cend(version), allocator) {}

            void assign(const std::string &compiler, const std::string &version) {
                _compiler.assign(std::cbegin(compiler), std::cend(compiler));
                _version.assign(std::cbegin(version), std::cend(version));
            }

            constexpr const string_type &compiler() const {
                return _compiler;
            }

            constexpr const string_type &version() const {
                return _version;
            }

            [[nodiscard]] std::string compiler_str() const {
                return {std::cbegin(_compiler), std::cend(_compiler)};
            }

            [[nodiscard]] std::string version_str() const {
                return {std::cbegin(_version), std::cend(_version)};
            }

        private:
            string_type _compiler;
            string_type _version;
        };

        constexpr mmap_base_container() = default;

        constexpr mmap_base_container(mmap_base_container &&other) noexcept
            : _storage{std::move(other._storage)} {
            other._storage = nullptr;
        }

        constexpr mmap_base_container &operator=(mmap_base_container &&other) noexcept {
            if (this != &other) {
                _storage = std::move(other._storage);
                other._storage = nullptr;
            }
            return *this;
        }

        ~mmap_base_container() {
            close();
        }

        void create_only(const std::string &name, std::size_t size) {
#if defined(MM2_WINDOWS)
            const std::size_t init = 512;
            _make_sparse_file(boost::interprocess::create_only, name, init);
            _grow_file(name, init, size);
            open(name);
#else
            _storage = new mmap_type{boost::interprocess::create_only, name.c_str(), size};
            process_info(boost::interprocess::create_only, *_storage, name);
#endif
        }

        void open_or_create(const std::string &name, std::size_t size) {
#if defined(MM2_WINDOWS)
            const std::size_t init = 512;
            _make_sparse_file(boost::interprocess::open_or_create, name, init);
            _grow_file(name, init, size);
            open(name);
#else
            _storage = new mmap_type{boost::interprocess::open_or_create, name.c_str(), size};
            process_info(boost::interprocess::open_or_create, *_storage, name);
#endif
        }

        void open_read_only(const std::string &name) {
            _storage = new mmap_type{boost::interprocess::open_read_only, name.c_str()};
            process_info(boost::interprocess::open_read_only, *_storage, name);
        }

        void open(const std::string &name) {
            _storage = new mmap_type{boost::interprocess::open_only, name.c_str()};
            process_info(boost::interprocess::open_only, *_storage, name);
        }

        void open_copy_on_write(const std::string &name) {
            _storage = new mmap_type{boost::interprocess::open_copy_on_write, name.c_str()};
            process_info(boost::interprocess::open_copy_on_write, *_storage, name);
        }

        [[nodiscard]] static auto *find_info(mmap_type &storage) {
            return storage.template find<info>(info_identifier).first;
        }

        static auto *construct_info(mmap_type &storage, const std::string &compiler, const std::string &version) {
            typename info::allocator_type allocator{storage.get_segment_manager()};
            return storage.template construct<info>(info_identifier)(compiler, version, allocator);
        }

        static auto *write_info(mmap_type &storage) {
            const auto compiler = util::get_compiler_info();
            const auto version = util::version();

            auto *saved_info = find_info(storage);
            if (saved_info) {
                saved_info->assign(compiler, version);
            } else {
                return construct_info(storage, compiler, version);
            }
            return saved_info;
        }

        static auto *write_and_compare_info(mmap_type &storage, const std::string &name) {
            write_info(storage);
            return compare_info(storage, name);
        }

        static auto *contains_info(mmap_type &storage, const std::string &name) {
            auto *saved_info = find_info(storage);

            if (not saved_info) {
                const auto compiler = util::get_compiler_info();
                const auto version = util::version();

                throw std::runtime_error{
                        std::format(
                                "storage '{}' does not contain compiler info '{}' and version number '{}', please rebuild storage",
                                name, compiler, version)
                };
            }

            return saved_info;
        }

        static auto *compare_info(mmap_type &storage, const std::string &name) {
            auto *saved_info = contains_info(storage, name);

            const auto compiler = util::get_compiler_info();
            const std::string converted_compiler = saved_info->compiler_str();
            if (compiler != converted_compiler) {
                throw std::runtime_error{
                        std::format(
                                "storage '{}' was built with compiler '{}' but current compiler is '{}', please rebuild storage",
                                name, converted_compiler, compiler)
                };
            }

            const auto version = util::version();
            const std::string converted_version = saved_info->version_str();
            if (version != converted_version) {
                throw std::runtime_error{
                        std::format(
                                "storage '{}' was built with software version '{}' but current version is '{}', please rebuild storage",
                                name, converted_version, version)
                };
            }

            return saved_info;
        }

        template<typename CreationTag>
        static auto *process_info(CreationTag creation_tag, mmap_type &storage, const std::string &name) {
            if constexpr (std::same_as<CreationTag, boost::interprocess::create_only_t>) {
                return write_and_compare_info(storage, name);
            } else if constexpr (std::same_as<CreationTag, boost::interprocess::open_only_t>) {
                return compare_info(storage, name);
            } else if constexpr (std::same_as<CreationTag, boost::interprocess::open_read_only_t>) {
                return compare_info(storage, name);
            } else if constexpr (std::same_as<CreationTag, boost::interprocess::open_read_private_t>) {
                return compare_info(storage, name);
            } else if constexpr (std::same_as<CreationTag, boost::interprocess::open_copy_on_write_t>) {
                return compare_info(storage, name);
            } else if constexpr (std::same_as<CreationTag, boost::interprocess::open_or_create_t>) {
                if (const auto *saved_info = find_info(storage); saved_info) {
                    return compare_info(storage, name);
                } else {
                    return write_and_compare_info(storage, name);
                }
            } else {
                throw std::invalid_argument{"unknown creation_tag"};
            }
        }

    public:
        [[nodiscard]] constexpr const mmap_type &storage() const {
            return *_storage;
        }

        [[nodiscard]] constexpr mmap_type &storage() {
            return *_storage;
        }

        void close() {
            if (_storage) {
                delete _storage;
                _storage = nullptr;
            }
        }

        [[nodiscard]] std::size_t size() const {
            if (_storage) {
                return _storage->get_size();
            }
            return 0;
        }

    protected:
        mmap_type *_storage{nullptr};

#if defined(MM2_WINDOWS)
        template<typename CreationTag>
        static void _make_sparse_file(CreationTag creation_tag, const std::string &filename, const std::size_t init) {
            {
                mmap_type storage{creation_tag, filename.c_str(), init};
                process_info(creation_tag, storage, filename);
            }
            make_sparse_file(filename);
        }

        static void _grow_file(const std::string &filename, const std::size_t offset, const std::size_t size) {
            if (size > offset) {
                grow_file(filename, size);
                mmap_type storage{boost::interprocess::open_only, filename.c_str()};
                process_info(boost::interprocess::open_only, storage, filename);
                storage.get_segment_manager()->grow(size - offset);
            }
        }

        static void _shrink_to_fit(const std::string &filename) {
            std::size_t new_size = 0;

            try {
                mmap_type storage{boost::interprocess::open_only, filename.c_str()};
                process_info(boost::interprocess::open_only, storage, filename);
                storage.get_segment_manager()->shrink_to_fit();
                new_size = storage.get_segment_manager()->get_size();
            } catch (...) {
                throw;
            }

            shrink_to_fit(filename, new_size);
        }
#endif

    };

    template<typename Types>
    class mmap_container;

    template<is_managed_shared_memory Types>
    class mmap_container<Types> : public mmap_base_container<Types> {

    public:
        using types = Types;
        using base_type = mmap_base_container<types>;
        using mmap_type = typename types::mmap_type;

        explicit constexpr mmap_container(std::string name)
            : base_type{}, _name{std::move(name)} {}

        constexpr mmap_container(const mmap_container &other) = delete;

        constexpr mmap_container(mmap_container &&other) noexcept = default;

        constexpr mmap_container &operator=(const mmap_container &other) = delete;

        constexpr mmap_container &operator=(mmap_container &&other) noexcept = default;

        ~mmap_container() = default;

        [[nodiscard]] constexpr const std::string &name() const {
            return _name;
        }

        void create_only(std::size_t size) {
            base_type::create_only(_name, size);
        }

        void open_or_create(std::size_t size) {
            base_type::open_or_create(_name, size);
        }

        void open_read_only() {
            base_type::open_read_only(_name);
        }

        void open() {
            base_type::open(_name);
        }

        void open_copy_on_write() {
            base_type::open_copy_on_write(_name);
        }

        void destroy_and_create(std::size_t size) {
            destroy();
            create_only(size);
        }

        void destroy() {
            this->close();
            boost::interprocess::shared_memory_object::remove(_name.c_str());
        }

        [[nodiscard]] std::size_t max_retries() const {
            const auto total_memory = get_total_memory();
            return static_cast<std::size_t>(std::log2(total_memory / this->size()));
        }

        void grow() {
            grow(2 * this->size());
        }

        void grow(const std::size_t size) {
            const std::size_t offset = this->size();
            if (size > offset) {
                this->close();
                mmap_type::grow(_name.c_str(), size - offset);
                open();
            }
        }

        void drop_grow() {
            drop_grow(2 * this->size());
        }

        void drop_grow(const std::size_t size) {
            const std::size_t offset = this->size();
            if (size > offset) {
                destroy();
                create_only(size);
            }
        }

        void shrink_to_fit(bool reopen = false) {
            this->close();
            if (not _name.empty()) {
                mmap_type::shrink_to_fit(_name.c_str());
            }
            if (reopen) {
                open();
            }
        }

    protected:
        std::string _name;

    };

    template<is_managed_mapped_file Types>
    class mmap_container<Types> : public mmap_base_container<Types> {

    public:
        using types = Types;
        using base_type = mmap_base_container<types>;
        using mmap_type = typename types::mmap_type;

        explicit mmap_container(std::filesystem::path path)
            : base_type{}, _path{std::move(path)} {}

        constexpr mmap_container(const mmap_container &other) = delete;

        constexpr mmap_container(mmap_container &&other) noexcept = default;

        constexpr mmap_container &operator=(const mmap_container &other) = delete;

        constexpr mmap_container &operator=(mmap_container &&other) noexcept = default;

        ~mmap_container() = default;

        [[nodiscard]] constexpr const std::filesystem::path &path() const {
            return _path;
        }

        void create_only(std::size_t size) {
            base_type::create_only(_path.string(), size);
        }

        void open_or_create(std::size_t size) {
            base_type::open_or_create(_path.string(), size);
        }

        void open_read_only() {
            base_type::open_read_only(_path.string());
        }

        void open() {
            base_type::open(_path.string());
        }

        void open_copy_on_write() {
            base_type::open_copy_on_write(_path.string());
        }

        void destroy_and_create(std::size_t size) {
            destroy();
            create_only(size);
        }

        void destroy() {
            this->close();
            boost::interprocess::file_mapping::remove(_path.c_str());
        }

        [[nodiscard]] std::size_t max_retries() const {
            const auto space = std::filesystem::space(_path);
            return static_cast<std::size_t>(std::log2(space.available / this->size()));
        }

        void grow() {
            grow(2 * this->size());
        }

        void grow(const std::size_t size) {
            const std::size_t offset = this->size();
            if (size > offset) {
                this->close();
                const std::string path_str = _path.string();
#if defined(MM2_WINDOWS)
                base_type::_grow_file(path_str, offset, size);
#else
                mmap_type::grow(path_str.c_str(), size - offset);
#endif
                open();
            }
        }

        void drop_grow() {
            drop_grow(2 * this->size());
        }

        void drop_grow(const std::size_t size) {
            const std::size_t offset = this->size();
            if (size > offset) {
                destroy();
                create_only(size);
            }
        }

        void flush() {
            if (this->_storage) {
                auto address = this->_storage->get_address();
                auto size = this->_storage->get_size();
#if defined(MM2_WINDOWS)
                flush_address(address, size);
                this->close();
                flush_file(_path.string());
#else
                flush_address(address, size, false);
#endif
            }
        }

        void shrink_to_fit(bool reopen = false) {
            this->close();
            if (not _path.empty() and std::filesystem::exists(_path)) {
#if defined(MM2_WINDOWS)
                base_type::_shrink_to_fit(_path.string());
#else
                mmap_type::shrink_to_fit(_path.c_str());
#endif
            }
            if (reopen) {
                open();
            }
        }

    protected:
        std::filesystem::path _path;

    };

    template<typename Types>
    class mmap_base_data;

    template<is_managed_shared_memory Types>
    class mmap_base_data<Types> {

    protected:
        using types = Types;
        using mmap_type = typename types::mmap_type;

        constexpr mmap_base_data() = default;

        constexpr mmap_base_data(const mmap_base_data &other) = default;

        constexpr mmap_base_data(mmap_base_data &&other) noexcept = default;

        constexpr mmap_base_data &operator=(const mmap_base_data &other) = default;

        constexpr mmap_base_data &operator=(mmap_base_data &&other) noexcept = default;

        constexpr ~mmap_base_data() = default;

    };

    template<is_managed_mapped_file Types>
    class mmap_base_data<Types> {

    protected:
        using types = Types;
        using mmap_type = typename types::mmap_type;

        explicit constexpr mmap_base_data(const std::string &path)
            : _path{path} {}

        constexpr mmap_base_data(const mmap_base_data &other) = default;

        constexpr mmap_base_data(mmap_base_data &&other) noexcept = default;

        constexpr mmap_base_data &operator=(const mmap_base_data &other) = default;

        constexpr mmap_base_data &operator=(mmap_base_data &&other) noexcept = default;

        constexpr ~mmap_base_data() = default;

        std::filesystem::path _path;

    };

    template<has_mmap_type Types>
    class mmap_base : public mmap_base_data<Types> {

    protected:
        using types = Types;
        using base_type = mmap_base_data<types>;
        using mmap_type = typename types::mmap_type;
        using mmap_container_type = mmap_container<types>;

        constexpr mmap_base() requires is_managed_shared_memory<types> = default;

        explicit mmap_base(const std::string &directory) requires is_managed_mapped_file<types>
            : base_type{directory} {
            std::filesystem::create_directory(this->_path);
        }

        constexpr mmap_base(const mmap_base &other) = default;

        constexpr mmap_base(mmap_base &&other) noexcept = default;

        constexpr mmap_base &operator=(const mmap_base &other) = default;

        constexpr mmap_base &operator=(mmap_base &&other) noexcept = default;

        constexpr ~mmap_base() = default;

        [[nodiscard]] mmap_container_type create_only(const std::string &name, std::size_t size) requires
            is_managed_shared_memory<types> {
            mmap_container_type container{name};
            container.create_only(size);
            return container;
        }

        [[nodiscard]] mmap_container_type create_only(const std::string &filename, std::size_t size) requires
            is_managed_mapped_file<types> {
            mmap_container_type container{this->_path / filename};
            container.create_only(size);
            return container;
        }

        [[nodiscard]] mmap_container_type open_or_create(const std::string &name, std::size_t size) requires
            is_managed_shared_memory<types> {
            mmap_container_type container{name};
            container.open_or_create(size);
            return container;
        }

        [[nodiscard]] mmap_container_type open_or_create(const std::string &filename, std::size_t size) requires
            is_managed_mapped_file<types> {
            mmap_container_type container{this->_path / filename};
            container.open_or_create(size);
            return container;
        }

        [[nodiscard]] mmap_container_type open(const std::string &name) requires
            is_managed_shared_memory<types> {
            mmap_container_type container{name};
            container.open();
            return container;
        }

        [[nodiscard]] mmap_container_type open(const std::string &filename) requires
            is_managed_mapped_file<types> {
            mmap_container_type container{this->_path / filename};
            container.open();
            return container;
        }

        [[nodiscard]] mmap_container_type open_read_only(const std::string &name) requires
            is_managed_shared_memory<types> {
            mmap_container_type container{name};
            container.open_read_only();
            return container;
        }

        [[nodiscard]] mmap_container_type open_read_only(const std::string &filename) requires
            is_managed_mapped_file<types> {
            mmap_container_type container{this->_path / filename};
            container.open_read_only();
            return container;
        }

        [[nodiscard]] mmap_container_type open_copy_on_write(const std::string &name) requires
            is_managed_shared_memory<types> {
            mmap_container_type container{name};
            container.open_copy_on_write();
            return container;
        }

        [[nodiscard]] mmap_container_type open_copy_on_write(const std::string &filename) requires
            is_managed_mapped_file<types> {
            mmap_container_type container{this->_path / filename};
            container.open_copy_on_write();
            return container;
        }

        [[nodiscard]] mmap_container_type destroy_and_create(const std::string &name, std::size_t size) requires
            is_managed_shared_memory<types> {
            mmap_container_type container{name};
            container.destroy_and_create(size);
            return container;
        }

        [[nodiscard]] mmap_container_type destroy_and_create(const std::string &filename, std::size_t size) requires
            is_managed_mapped_file<types> {
            mmap_container_type container{this->_path / filename};
            container.destroy_and_create(size);
            return container;
        }

    };

}

#endif //MAP_MATCHING_2_IO_MEMORY_MAPPED_STORAGE_BASE_STORAGE_HPP
