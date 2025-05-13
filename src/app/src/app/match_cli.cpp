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

#include "app/match_cli.hpp"

namespace map_matching_2::app {

    namespace po = boost::program_options;

    void _match_cli(const match_data &data) {
        if (not global.quiet and not data.console.console) {
            std::cout << "Start Matching ..." << std::endl;
        }

        auto duration = util::benchmark([&]() {
            if (not data.console.console) {
                std::filesystem::create_directory(data.match_output.output);
            }

            auto network_variant = open_network(data);

            std::visit([&](auto &&network) {
                _start_matching(network, data);
            }, std::move(network_variant));
        });

        if (not global.quiet and not data.console.console) {
            std::cout << std::format("Matching finished in {:.3f} s!", duration) << std::endl;
        }
    }

    int match_cli(int argc, char *argv[]) {
        match_data data;

        auto all = options_match(data);

        po::variables_map vm;
        if (not process(argc, argv, vm, all, MODE::MATCH)) {
            return 1;
        }

        data.correct(vm);

        _match_cli(data);

        return 0;
    }

}
