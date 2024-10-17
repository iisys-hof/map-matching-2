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

#ifndef MAP_MATCHING_2_MATCHING_MATCHER_HPP
#define MAP_MATCHING_2_MATCHING_MATCHER_HPP

#include <boost/type_index.hpp>

#include "types/geometry/index/matcher.hpp"

#include "types/matching/algorithm/algorithms.hpp"

#include "types/environment/environments/hmm.hpp"
#include "types/environment/environments/single.hpp"

#include "types/learning/algorithms/policy_iteration.hpp"
#include "types/learning/algorithms/value_iteration.hpp"
#include "types/learning/algorithms/q_learning.hpp"
#include "types/learning/algorithms/viterbi.hpp"

#include "io/results/match/csv_match_exporter.hpp"
#include "io/results/match/csv_candidates_exporter.hpp"
#include "io/results/match/match_output_printer.hpp"

#include "util/coordinator.hpp"
#include "util/benchmark.hpp"
#include "util/time_helper.hpp"

#include "traits/network.hpp"

#include "matcher/matcher_output_settings.hpp"
#include "matcher/match_task.hpp"
#include "matcher/match_result.hpp"

#include "settings.hpp"

namespace map_matching_2::matching {

    template<typename Network>
    class matcher : public util::coordinator<match_task> {

    public:
        using network_type = Network;
        using multi_track_variant_type = geometry::track::multi_track_variant_type;
        using task_type = match_task;
        using base_type = util::coordinator<task_type>;

        using track_type = typename network_traits<network_type>::track_type;
        using route_type = typename network_traits<network_type>::route_type;
        using policy_type = typename network_traits<network_type>::policy_type;
        using multi_track_multi_rich_line_type = typename
        network_traits<network_type>::multi_track_multi_rich_line_type;
        using track_rich_line_type = typename network_traits<network_type>::track_rich_line_type;
        using lazy_rich_line_type = typename network_traits<network_type>::lazy_rich_line_type;

        matcher(const network_type &network, const matcher_output_settings &output_settings,
                std::uint32_t threads = boost::thread::hardware_concurrency())
            : base_type{threads}, _network{network},
            _exporter{output_settings.filename, output_settings.columns},
            _candidates_exporter{output_settings.candidates_filename, output_settings.candidates_columns},
            _output_printer{output_settings.console, output_settings.verbose} {
            _output_printer.start();
            _exporter.start();
            _candidates_exporter.start();
        }

        ~matcher() override {
            _output_printer.stop();
            _exporter.stop();
            _candidates_exporter.stop();

            _output_printer.join();
            _exporter.join();
            _candidates_exporter.join();
        }

        [[nodiscard]] constexpr const network_type &network() const {
            return _network;
        }

        void match(multi_track_variant_type multi_track, matching::settings settings) {
            this->queue(task_type{std::move(multi_track), std::move(settings)});
        }

    private:
        const network_type &_network;
        io::results::csv_match_exporter<> _exporter;
        io::results::csv_candidates_exporter<> _candidates_exporter;
        io::results::match_output_printer _output_printer;

        template<typename Learner, typename MultiTrack>
        void _match_impl(MultiTrack multi_track, const matching::settings &settings) {
            using learner_type = Learner;
            using multi_track_type = MultiTrack;
            using environment_type = typename learner_type::environment_type;
            using match_result_type = match_result<learner_type>;

            util::time_helper _time_helper{settings.max_time};
            matching::algorithms<network_type> _algorithms{_network, _time_helper};

            match_result_type _match_result;
            _match_result.environments.reserve(multi_track.size());
            _match_result.learner.reserve(multi_track.size());

            typename multi_track_multi_rich_line_type::rich_lines_container_type _prepared;
            _prepared.reserve(multi_track.size());

            std::vector<policy_type> _policies;
            _policies.reserve(multi_track.size());

            std::vector<route_type> _matches;
            _matches.reserve(multi_track.size());

            std::vector<osmium::object_id_type> _edge_ids;

            double duration{0.0};
            for (std::size_t i = 0; i < multi_track.size(); ++i) {
                track_type track = multi_track.get_track(i);

                environment_type &environment = _match_result.environments.emplace_back(
                        _network, _algorithms, settings);
                learner_type &learner = _match_result.learner.emplace_back(
                        environment, settings);

                // match
                policy_type policy;
                duration += util::benchmark([&]() {
                    environment.set(track);
                    if (not _time_helper.update_and_has_reached()) {
                        policy = learner();
                    }
                });

                _prepared.emplace_back(environment.track().rich_line);

                // result
                auto route = environment.result(policy);
                if (settings.export_edge_ids or settings.export_edges) {
                    const auto edge_list = _algorithms.router.edges_list(route);
                    if (settings.export_edges) {
                        route = _algorithms.router.export_edges_route(edge_list);
                    }
                    if (settings.export_edge_ids) {
                        auto ids = _algorithms.router.edge_ids(edge_list);
                        std::move(std::begin(ids), std::end(ids), std::back_inserter(_edge_ids));
                    }
                }
                _matches.emplace_back(route_type{route});

                _policies.emplace_back(std::move(policy));

                if (_time_helper.reached_max_duration()) {
                    _match_result.aborted = true;
                    break;
                }
            }

            _match_result.prepared = multi_track_type{
                    multi_track.id, multi_track_multi_rich_line_type{std::move(_prepared)}
            };
            _match_result.track = std::move(multi_track);
            _match_result.policies = std::move(_policies);
            _match_result.match = route_type::template merge<lazy_rich_line_type>({_matches.begin(), _matches.end()});
            _match_result.edge_ids = std::move(_edge_ids);
            _match_result.duration = duration;

            _output_printer.pass(_output_printer.parse_result(_match_result));
            if (not _output_printer.is_console()) {
                _exporter.pass(_exporter.parse_result(_match_result));
                _candidates_exporter.pass(_candidates_exporter.parse_result(_network, _match_result));
            }

            geometry::reset_distance_cache();
            geometry::reset_azimuth_cache();
        }

        template<typename Learner>
        void _match_dispatch(task_type task) {
            using network_multi_track_type = typename network_traits<network_type>::multi_track_type;
            using network_point_type = typename network_multi_track_type::point_type;

            std::visit([this, &task]<typename MultiTrack>(MultiTrack &&multi_track) {
                using multi_track_type = std::remove_reference_t<MultiTrack>;
                using point_type = typename multi_track_type::point_type;

                if constexpr (std::same_as<
                    typename geometry::data<network_point_type>::coordinate_system_type,
                    typename geometry::data<point_type>::coordinate_system_type>) {

                    _match_impl<Learner, multi_track_type>(std::forward<MultiTrack>(multi_track), task.match_settings);
                } else {
                    throw std::invalid_argument{
                            std::format(
                                    "coordinate systems do not match: {} and {}",
                                    boost::typeindex::type_id<typename geometry::data<
                                        network_point_type>::coordinate_system_type>().pretty_name(),
                                    boost::typeindex::type_id<typename geometry::data<
                                        point_type>::coordinate_system_type>().pretty_name())
                    };
                }
            }, std::move(task.multi_track));
        }

        void execute(task_type &&task) override {
            using single_type = environment::single<network_type>;
            using hmm_type = environment::hmm<network_type>;

            using policy_iteration_type = learning::policy_iteration<single_type>;
            using value_iteration_type = learning::value_iteration<single_type>;
            using q_learning_type = learning::q_learning<single_type>;
            using viterbi_type = learning::viterbi<hmm_type>;

            if (task.match_settings.model == settings::MODEL::POLICY_ITERATION) {
                _match_dispatch<policy_iteration_type>(std::move(task));
            } else if (task.match_settings.model == settings::MODEL::VALUE_ITERATION) {
                _match_dispatch<value_iteration_type>(std::move(task));
            } else if (task.match_settings.model == settings::MODEL::Q_LEARNING) {
                _match_dispatch<q_learning_type>(std::move(task));
            } else if (task.match_settings.model == settings::MODEL::VITERBI) {
                _match_dispatch<viterbi_type>(std::move(task));
            }
        }

    };

}

#endif //MAP_MATCHING_2_MATCHING_MATCHER_HPP
