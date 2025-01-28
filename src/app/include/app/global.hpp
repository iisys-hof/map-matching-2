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

#ifndef MAP_MATCHING_2_APP_GLOBAL_HPP
#define MAP_MATCHING_2_APP_GLOBAL_HPP

#include <string>
#include <format>
#include <iostream>

#include "util/benchmark.hpp"

namespace map_matching_2::app {

    struct console_size {
        std::size_t cols = 80;
        std::size_t rows = 24;
    };

    struct global_storage {
        bool verbose = false;
        bool quiet = false;
        console_size console{};
    };

    extern global_storage global;

    [[nodiscard]] console_size get_console_size();

    constexpr void set_global_verbose_quiet(const bool verbose, const bool quiet) {
        global.verbose = verbose;
        global.quiet = quiet;
    }

    constexpr void set_global_console_size(const console_size &console_size) {
        global.console = console_size;
    }

    template<typename Func>
    void verbose_frame(const std::string &start_message, const Func &func,
            const std::string &end_message = "done in {:.3f} s") {
        if (global.verbose) {
            std::cout << start_message << " ... " << std::flush;
            auto duration = util::benchmark(func);
            std::cout << std::vformat(end_message, std::make_format_args(duration)) << std::endl;
        } else {
            func();
        }
    }

}

#endif //MAP_MATCHING_2_APP_GLOBAL_HPP
