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

#include <filesystem>

#include "app/compare.hpp"
#include "app/settings.hpp"

#include "types/io/track/csv_track_importer.hpp"
#include "types/io/track/gpx_track_importer.hpp"

#include "compare/comparator/comparator_forwarder_data.hpp"
#include "compare/comparator/comparator_forwarder_matches.hpp"
#include "compare/comparator/comparator_forwarder_compares.hpp"

#include "util/benchmark.hpp"

namespace map_matching_2::app {

    namespace po = boost::program_options;

    std::unique_ptr<compare::comparator> _start_comparator(const compare_data &data) {
        const auto output_file = std::filesystem::path{data.compare_output.output} / data.compare_output.filename;
        const auto output_filename = output_file.string();
        const compare::comparator_output_settings _output_settings{
                output_filename, data.compare_output.columns, data.console.console, global.verbose
        };
        auto comparator = std::make_unique<compare::comparator>(
                _output_settings, data.performance.threads);
        comparator->start();
        return comparator;
    }

    void _join_comparator(std::unique_ptr<compare::comparator> &comparator) {
        comparator->join();
        comparator.reset();
    }

    void _read_ground_truth(compare::comparator_forwarder_data &forwarder_data, const compare_data &data) {
        compare::comparator_forwarder_compares forwarder_compares{forwarder_data};
        const auto compares_reprojector_variant = geometry::create_point_reprojector(data.srs_compare.srs_transform);
        if (data.comparison.file_extension == FILE_TYPE_CSV) {
            io::track::csv_track_importer csv_ground_truth_importer{
                    data.comparison.files, data.csv_compare.selectors, _csv_settings(data.csv_compare),
                    forwarder_compares, data.srs_compare.srs_transform, compares_reprojector_variant
            };
            verbose_frame("Reading Ground Truth", [&]() {
                csv_ground_truth_importer.read();
            });
        } else if (data.comparison.file_extension == FILE_TYPE_GPX) {
            io::track::gpx_track_importer gpx_ground_truth_importer{
                    data.comparison.files, data.csv_compare.selectors, data.csv_compare.no_id,
                    data.csv_compare.time_format, data.csv_compare.no_parse_time,
                    forwarder_compares, data.srs_compare.srs_transform, compares_reprojector_variant
            };
            verbose_frame("Reading Ground Truth", [&]() {
                gpx_ground_truth_importer.read();
            });
        }
    }

    void _read_matches(compare::comparator_forwarder_data &forwarder_data, const compare_data &data) {
        compare::comparator_forwarder_matches forwarder_matches{forwarder_data};
        const auto matches_reprojector_variant = geometry::create_point_reprojector(data.srs_match.srs_transform);
        if (data.match.file_extension == FILE_TYPE_CSV) {
            io::track::csv_track_importer csv_matches_importer{
                    data.match.files, data.csv_match.selectors, _csv_settings(data.csv_match),
                    forwarder_matches, data.srs_match.srs_transform, matches_reprojector_variant
            };
            csv_matches_importer.read();
        } else if (data.match.file_extension == FILE_TYPE_GPX) {
            io::track::gpx_track_importer gpx_matches_importer{
                    data.match.files, data.csv_match.selectors, data.csv_match.no_id,
                    data.csv_match.time_format, data.csv_match.no_parse_time,
                    forwarder_matches, data.srs_match.srs_transform, matches_reprojector_variant
            };
            gpx_matches_importer.read();
        }
    }

    void _read_compares(std::unique_ptr<compare::comparator> &comparator, const compare_data &data) {
        if (data.match.file_extension != FILE_TYPE_CSV and data.match.file_extension != FILE_TYPE_GPX) {
            throw std::invalid_argument{"match file(s) need to be .csv or .gpx files"};
        }
        if (data.comparison.file_extension != FILE_TYPE_CSV and data.comparison.file_extension != FILE_TYPE_GPX) {
            throw std::invalid_argument{"ground truth file(s) need to be .csv or .gpx files"};
        }

        // shared data
        compare::comparator_forwarder_data forwarder_data{*comparator, _compare_settings(data)};

        // read ground truth and matches
        _read_ground_truth(forwarder_data, data);
        _read_matches(forwarder_data, data);

        // reading finished, close queue
        comparator->stop();
    }

    void _start_comparison(const compare_data &data) {
        auto comparator = _start_comparator(data);
        _read_compares(comparator, data);
        _join_comparator(comparator);
    }

    void _compare(const compare_data &data) {
        if (not global.quiet and not data.console.console) {
            std::cout << "Start Comparison ..." << std::endl;
        }

        auto duration = util::benchmark([&]() {
            if (not data.console.console) {
                std::filesystem::create_directory(data.compare_output.output);
            }

            _start_comparison(data);
        });

        if (not global.quiet and not data.console.console) {
            std::cout << std::format("Comparison finished in {:.3f} s!", duration) << std::endl;
        }
    }

    int compare(int argc, char *argv[]) {
        compare_data data;

        auto all = options_compare(data);

        po::variables_map vm;
        if (not process(argc, argv, vm, all, MODE::COMPARE)) {
            return 1;
        }

        data.correct(vm);

        _compare(data);

        return 0;
    }

}
