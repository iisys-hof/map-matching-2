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

#include "app/prepare.hpp"

namespace map_matching_2::app {

    namespace po = boost::program_options;

    void _check_files(const prepare_data &data) {
        std::size_t errors = 0;
        for (const auto &file : data.network.files) {
            if (not std::filesystem::exists(file)) {
                errors++;
                std::cerr << std::format("Error: file '{}' does not exist!", file) << std::endl;
            }
        }

        if (errors > 0) {
            throw std::runtime_error{std::format("input file{} did not exist, please check.", errors > 1 ? "s" : "")};
        }
    }

    void _prepare(const prepare_data &data) {
        _check_files(data);

        if (not global.quiet) {
            std::cout << "Start Preparation ..." << std::endl;
        }

        auto duration = util::benchmark([&]() {
            std::filesystem::create_directory(data.network.output);

            auto network_import_variant = create_network_import(data);

            std::visit([&](auto &&network_import) {
                _import_network(network_import, data);
            }, std::move(network_import_variant));
        });

        if (not global.quiet) {
            std::cout << std::format("Preparation finished in {:.3f} s!", duration) << std::endl;
        }
    }

    int prepare(int argc, char *argv[]) {
        prepare_data data;

        auto all = options_prepare(data);

        po::variables_map vm;
        if (not process(argc, argv, vm, all, MODE::PREPARE)) {
            return 1;
        }

        data.correct(vm);

        _prepare(data);

        return 0;
    }

}
