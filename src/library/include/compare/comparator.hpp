// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_COMPARE_COMPARATOR_HPP
#define MAP_MATCHING_2_COMPARE_COMPARATOR_HPP

#include "types/geometry/algorithm/compare.hpp"

#include "types/geometry/track/multi_track.hpp"

#include "io/results/compare/csv_compare_exporter.hpp"
#include "io/results/compare/compare_output_printer.hpp"

#include "comparator/compare_output_settings.hpp"
#include "comparator/compare_task.hpp"
#include "comparator/compare_result.hpp"

#include "util/coordinator.hpp"
#include "util/benchmark.hpp"

#include "settings.hpp"

namespace map_matching_2::compare {

    class comparator : public util::coordinator<compare_task> {

    public:
        using multi_track_variant_type = geometry::track::multi_track_variant_type;
        using task_type = compare_task;
        using base_type = util::coordinator<task_type>;

        explicit comparator(const comparator_output_settings &output_settings,
                std::uint32_t threads = boost::thread::hardware_concurrency());

        ~comparator() override;

        void compare(multi_track_variant_type match, multi_track_variant_type ground_truth,
                compare::settings settings);

    private:
        io::results::csv_compare_exporter<> _exporter;
        io::results::compare_output_printer _output_printer;

        template<typename GroundTruthMultiTrack, typename MatchMultiTrack>
        void _compare_impl(GroundTruthMultiTrack ground_truth, MatchMultiTrack match,
                const compare::settings &settings) {
            using ground_truth_multi_track_type = GroundTruthMultiTrack;
            using ground_truth_multi_rich_line_type = typename ground_truth_multi_track_type::multi_rich_line_type;
            using comparator_type = geometry::line_comparator<ground_truth_multi_rich_line_type>;
            using result_type = typename comparator_type::result_type;
            using comparator_result_type = compare_result<ground_truth_multi_track_type>;

            comparator_type comparator{
                    settings.simplifying_tolerance,
                    settings.simplifying_reverse_tolerance,
                    settings.adoption_distance_tolerance,
                    settings.split_distance_tolerance,
                    settings.split_direction_tolerance,
                    settings.debug
            };

            comparator_result_type _compare_result;

            result_type result;
            double duration = util::benchmark([&]() {
                result = comparator.compare(ground_truth.multi_rich_line, match.multi_rich_line);
            });

            _compare_result.ground_truth = std::move(ground_truth);
            _compare_result.match = std::move(match);
            _compare_result.comparison = std::move(result);
            _compare_result.duration = duration;

            _output_printer.pass(_output_printer.parse_result(_compare_result));
            if (not _output_printer.is_console()) {
                _exporter.pass(_exporter.parse_result(_compare_result));
            }

            geometry::reset_distance_cache();
            geometry::reset_azimuth_cache();
        }

        void _compare_dispatch(task_type task);

        void execute(task_type &&task) override;

    };

}

#endif //MAP_MATCHING_2_COMPARE_COMPARATOR_HPP
