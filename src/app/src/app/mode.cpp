// Copyright (C) 2024-2025 Adrian Wöltche
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

#include "app/global.hpp"

#include "app/prepare.hpp"
#include "app/match_cli.hpp"
#include "app/match_web.hpp"
#include "app/compare.hpp"

namespace map_matching_2::app {

    namespace po = boost::program_options;

    void _mode(const mode_data &data, int argc, char *argv[]) {
        if (data.global.mode == MODE::PREPARE) {
            geometry::disable_cache();
            prepare(argc, argv);
        } else if (data.global.mode == MODE::MATCH) {
            geometry::enable_cache();
            match_cli(argc, argv);
        } else if (data.global.mode == MODE::SERVER) {
            geometry::enable_cache();
            // match_web(argc, argv);
        } else if (data.global.mode == MODE::COMPARE) {
            geometry::enable_cache();
            compare(argc, argv);
        }
    }

    int mode(int argc, char *argv[]) {
        // works, but some default options are so long that they distort the whole output, default works better
        // console_size console_size = get_console_size();
        // set_global_console_size(console_size);

        mode_data data;

        auto all = options_mode(data);

        po::variables_map vm;
        if (not process(argc, argv, vm, all, MODE::NONE)) {
            return 1;
        }

        try {
            data.correct(vm);
        } catch (std::exception &e) {
            show_error(e, all);
            return 2;
        }

        set_global_verbose_quiet(data.all.verbose, data.all.quiet);

        try {
            _mode(data, argc, argv);
        } catch (std::exception &e) {
            show_error(e, all);
            return 3;
        }

        return 0;
    }

}
