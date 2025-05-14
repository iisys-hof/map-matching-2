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

#include <stdexcept>

#include <boost/type_index.hpp>

#include "compare/comparator.hpp"

namespace map_matching_2::compare {

    comparator::comparator(const comparator_output_settings &output_settings, const std::uint32_t threads)
        : base_type{threads},
        _exporter{output_settings.filename, output_settings.columns},
        _output_printer{output_settings.console, output_settings.verbose} {
        _output_printer.start();
        _exporter.start();
    }

    comparator::~comparator() {
        _output_printer.stop();
        _exporter.stop();

        _output_printer.join();
        _exporter.join();
    }

    void comparator::compare(import_multi_track_variant_type match, import_multi_track_variant_type ground_truth,
            compare::settings settings) {
        this->queue(task_type{std::move(match), std::move(ground_truth), std::move(settings)});
    }

    void comparator::_compare_dispatch(task_type task) {
        std::visit([this, &task]<typename GroundTruthImportMultiTrack>(
                const GroundTruthImportMultiTrack &ground_truth) {
                    using ground_truth_import_multi_track_type = std::remove_reference_t<GroundTruthImportMultiTrack>;
                    using ground_truth_import_multi_rich_line_type =
                            typename ground_truth_import_multi_track_type::multi_rich_line_type;
                    using ground_truth_import_point_type =
                            typename ground_truth_import_multi_rich_line_type::point_type;
                    using ground_truth_multi_track_type =
                            geometry::track::multi_track_type<ground_truth_import_point_type>;

                    std::visit([this, &task, &ground_truth]<typename MatchImportMultiTrack>(
                            const MatchImportMultiTrack &match) {
                                using match_import_multi_track_type = std::remove_reference_t<MatchImportMultiTrack>;
                                using match_import_multi_rich_line_type =
                                        typename match_import_multi_track_type::multi_rich_line_type;
                                using match_import_point_type = typename match_import_multi_rich_line_type::point_type;
                                using match_multi_track_type =
                                        geometry::track::multi_track_type<match_import_point_type>;

                                if constexpr (std::same_as<
                                    typename geometry::data<ground_truth_import_point_type>::coordinate_system_type,
                                    typename geometry::data<match_import_point_type>::coordinate_system_type>) {

                                    _compare_impl(ground_truth_multi_track_type{ground_truth},
                                            match_multi_track_type{match}, task.compare_settings);
                                } else {
                                    throw std::invalid_argument{
                                            std::format(
                                                    "coordinate systems do not match: {} and {}",
                                                    boost::typeindex::type_id<typename geometry::data<
                                                        ground_truth_import_point_type>::coordinate_system_type>().
                                                    pretty_name(),
                                                    boost::typeindex::type_id<typename geometry::data<
                                                        match_import_point_type>::coordinate_system_type>().
                                                    pretty_name())
                                    };
                                }
                            }, task.match);
                }, task.ground_truth);
    }

    void comparator::execute(task_type &&task) {
        _compare_dispatch(std::move(task));
    }


}
