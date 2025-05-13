// Copyright (C) 2021-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_ENVIRONMENT_ENVIRONMENTS_HMM_HPP
#define MAP_MATCHING_2_ENVIRONMENT_ENVIRONMENTS_HMM_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "types/matching/algorithm/algorithms.hpp"

#include "geometry/algorithm/distance.hpp"
#include "geometry/algorithm/direction.hpp"
#include "geometry/algorithm/simplify.hpp"
#include "geometry/algorithm/median_merge.hpp"

#include "matching/traits/network.hpp"
#include "matching/settings.hpp"

#include "util/time_helper.hpp"

namespace map_matching_2::environment {

    template<typename Network>
    class hmm {
    public:
        using network_type = Network;
        using algorithms_type = matching::algorithms<network_type>;
        using track_type = typename matching::network_traits<network_type>::track_type;
        using route_type = typename matching::network_traits<network_type>::route_type;
        using candidate_type = typename matching::network_traits<network_type>::candidate_type;
        using length_type = typename matching::network_traits<network_type>::length_type;
        using angle_type = typename matching::network_traits<network_type>::angle_type;

        using action_type = std::int64_t;

        const action_type no_action = -1;

        static constexpr bool performance = false;
        const std::string name = "hmm";

        constexpr hmm(const network_type &network, algorithms_type &algorithms,
                const matching::settings &settings = {})
            : _network{network}, _settings{settings}, _algorithms{algorithms} {}

        constexpr hmm(const hmm &other) = delete;

        constexpr hmm(hmm &&other) noexcept
            : _network{other._network}, _settings{other._settings}, _algorithms{other._algorithms},
            _track{std::move(other._track)}, _candidates{std::move(other._candidates)},
            _observations{std::move(other._observations)},
            _transitions{std::move(other._transitions)} {
            other._observations.clear();
            other._transitions.clear();
        }

        constexpr hmm &operator=(const hmm &other) = delete;

        constexpr hmm &operator=(hmm &&other) noexcept = delete;

        constexpr ~hmm() = default;

        void set(track_type track) {
            _track = std::move(track);

            if (_settings.filter_duplicates) {
                const auto defects = _algorithms.detector.detect(_track.rich_line, _settings.filter_duplicates);
                _algorithms.detector.remove_defects(_track.rich_line, defects);

                if (abort()) {
                    return;
                }
            }

            if (_settings.simplify_track) {
                geometry::simplify_rich_line(_track.rich_line, false, _settings.simplify_track_distance_tolerance);

                if (abort()) {
                    return;
                }
            }

            if (_settings.median_merge) {
                _track.rich_line = geometry::median_merge(_track.rich_line.line(),
                        _settings.median_merge_distance_tolerance, _settings.adaptive_median_merge, &_network);

                if (abort()) {
                    return;
                }
            }

            _candidates = _algorithms.searcher.candidate_search(_track, _settings);

            if (abort()) {
                return;
            }

            // clear buffer
            _observations.clear();
            _transitions.clear();

            // observations
            _observations.reserve(_track.rich_line.size());
            for (const auto &candidate : _candidates) {
                double max_distance = 0.0;
                for (const auto &candidate_edge : candidate.edges) {
                    if (candidate_edge.distance > max_distance) {
                        max_distance = candidate_edge.distance;
                    }
                }

                // fix for dividing through zero and for removing -inf values
                max_distance += 0.01;
                max_distance *= 1.01;

                std::vector<double> candidate_observations;
                candidate_observations.reserve(candidate.edges.size());
                for (const auto &candidate_edge : candidate.edges) {
                    // max(0.0, (100 - DISTANCE) / 100)
                    double observation_probability =
                            std::max(1e-20, (max_distance - candidate_edge.distance) / max_distance);
                    if (candidate.index == 0 or candidate.index + 1 >= _candidates.size()) {
                        // on first and last candidate, try to be closer to point
                        observation_probability = std::pow(observation_probability, 3.0);
                    }
                    observation_probability = std::log(
                            std::pow(observation_probability, _settings.hmm_distance_factor));
                    assert(observation_probability > -std::numeric_limits<double>::infinity() and
                            observation_probability <= 0.0);
                    candidate_observations.emplace_back(observation_probability);
                }
                _observations.emplace_back(std::move(candidate_observations));

                if (abort()) {
                    return;
                }
            }

            // transitions
            _transitions.reserve(_candidates.size());
            for (std::int64_t from = -1, to = 0; to < _candidates.size(); ++from, ++to) {
                if (from < 0) {
                    // initial probabilities
                    const auto initial_states = _candidates[to].edges.size();
                    std::vector<std::vector<double>> start_probabilities;
                    start_probabilities.reserve(initial_states);
                    double initial_probability = std::log(1.0 / initial_states);
                    for (std::size_t target = 0; target < initial_states; ++target) {
                        std::vector<double> transition_probabilities{initial_probability};
                        start_probabilities.emplace_back(std::move(transition_probabilities));
                    }
                    _transitions.emplace_back(std::move(start_probabilities));
                } else {
                    const auto &from_candidate = _candidates[from];
                    const auto &to_candidate = _candidates[to];

                    std::vector<std::vector<double>> candidate_probabilities;
                    candidate_probabilities.reserve(to_candidate.edges.size());
                    for (std::size_t target = 0; target < to_candidate.edges.size(); ++target) {
                        std::vector<double> transition_probabilities;
                        transition_probabilities.reserve(from_candidate.edges.size());
                        for (std::size_t source = 0; source < from_candidate.edges.size(); ++source) {
                            const auto route = _algorithms.router.candidate_route(
                                    _candidates, from, source, to, target, _settings.within_edge_turns,
                                    _settings.a_star, _settings.routing_max_distance_factor);

                            if (route.is_invalid()) {
                                // invalid route, not possible
                                transition_probabilities.emplace_back(std::log(0.0));
                            } else {
                                double transition_probability = std::log(1.0);
                                if (from_candidate.next_equal and not route.has_length()) {
                                    // no distance between candidates and empty route, we stay here
                                    transition_probability += std::log(1.0);
                                } else {
                                    // calculate transition probability
                                    length_type next_distance = geometry::default_float_type<length_type>::v1;
                                    if (to_candidate.track->rich_line.has_length(to_candidate.index - 1)) {
                                        next_distance =
                                                to_candidate.track->rich_line.length(to_candidate.index - 1);
                                    }
                                    length_type route_length = geometry::default_float_type<length_type>::v1;
                                    if (route.has_length()) {
                                        route_length = route.length();
                                    }

                                    // double length_probability = std::max(1e-20L, std::min(next_distance, route_length) /
                                    //         std::max(next_distance, route_length));

                                    const auto route_diff = std::fabs(next_distance - route_length);

                                    double length_probability =
                                            1.0 - (route_diff / std::max(next_distance, route_length));
                                    length_probability = std::max(1e-20, length_probability);

                                    if (route_length > 0.0 and next_distance > 0.0) {
                                        const auto route_factor = route_length / next_distance;
                                        // when route is longer than next distance, it might be bad,
                                        // if it is shorter, it might be good
                                        length_probability *= route_factor > 1.0 ? 1.0 / route_factor : route_factor;

                                        if (route_factor > 10.0) {
                                            // when route is more than 10 times longer than next distance, low probability
                                            length_probability *= 0.01;
                                        }
                                    }

                                    length_probability = std::log(
                                            std::pow(length_probability, _settings.hmm_length_factor));
                                    assert(length_probability > -std::numeric_limits<double>::infinity() and
                                            length_probability <= 0.0);
                                    transition_probability += length_probability;

                                    if (route.has_azimuth()) {
                                        const angle_type route_azimuth = route.azimuth();
                                        angle_type next_azimuth = route_azimuth;
                                        if (to_candidate.track->rich_line.has_azimuth(to_candidate.index - 1)) {
                                            next_azimuth =
                                                    to_candidate.track->rich_line.azimuth(to_candidate.index - 1);
                                        }

                                        const auto diff_azimuth = geometry::angle_diff(next_azimuth, route_azimuth);
                                        double azimuth_probability =
                                                std::max(1e-20, (180.0 - std::fabs(diff_azimuth)) / 180.0);

                                        azimuth_probability = std::log(
                                                std::pow(azimuth_probability, _settings.hmm_azimuth_factor));
                                        assert(azimuth_probability > -std::numeric_limits<double>::infinity() and
                                                azimuth_probability <= 0.0);
                                        transition_probability += azimuth_probability;

                                        // calculate turn
                                        if (from_candidate.index >= 1 and to_candidate.index >= 2 and
                                            from_candidate.track->rich_line.has_azimuth(from_candidate.index - 1) and
                                            to_candidate.track->rich_line.has_direction(to_candidate.index - 2)) {
                                            const angle_type current_azimuth =
                                                    from_candidate.track->rich_line.azimuth(from_candidate.index - 1);
                                            const angle_type next_turn =
                                                    to_candidate.track->rich_line.direction(to_candidate.index - 2);

                                            const auto route_turn = geometry::direction_deg(
                                                    current_azimuth, route_azimuth);
                                            const auto turn_diff = geometry::angle_diff(next_turn, route_turn);

                                            double turn_probability =
                                                    std::max(1e-20, (360.0 - std::fabs(turn_diff)) / 360.0);

                                            turn_probability = std::log(
                                                    std::pow(turn_probability, _settings.hmm_turn_factor));
                                            assert(turn_probability > -std::numeric_limits<double>::infinity() and
                                                    turn_probability <= 0.0);
                                            transition_probability += turn_probability;
                                        }
                                    }

                                    if (route.has_directions()) {
                                        double absolute_directions_probability =
                                                std::max(1e-20, (360.0 - route.absolute_directions()) / 360.0);

                                        absolute_directions_probability = std::log(
                                                std::pow(absolute_directions_probability,
                                                        _settings.hmm_direction_factor));
                                        assert(absolute_directions_probability >
                                                -std::numeric_limits<double>::infinity() and
                                                absolute_directions_probability <= 0.0);
                                        transition_probability += absolute_directions_probability;
                                    }
                                }
                                transition_probabilities.emplace_back(transition_probability);
                            }
                        }
                        candidate_probabilities.emplace_back(std::move(transition_probabilities));
                    }
                    _transitions.emplace_back(std::move(candidate_probabilities));
                }

                if (abort()) {
                    return;
                }
            }
        }

        [[nodiscard]] const auto &track() const {
            return _track;
        }

        [[nodiscard]] const auto &candidates() const {
            return _candidates;
        }

        [[nodiscard]] std::size_t states() const {
            std::size_t states = 0;
            for (std::int64_t from = -1, to = 0; to < _candidates.size(); ++from, ++to) {
                if (from < 0) {
                    if (to < _transitions.size()) {
                        states += _transitions[to].size();
                    }
                } else {
                    if (from < _transitions.size() and to < _transitions.size()) {
                        states += _transitions[to].size() * _transitions[from].size();
                    }
                }
            }
            return states;
        }

        [[nodiscard]] std::size_t edges() const {
            std::size_t edges = 0;
            for (const auto &candidate : _candidates) {
                edges += candidate.edges.size();
            }
            return edges;
        }

        const auto &observations() const {
            return _observations;
        }

        const auto &transitions() const {
            return _transitions;
        }

        [[nodiscard]] bool abort() {
            return _algorithms.time_helper.update_and_has_reached();
        }

        [[nodiscard]] route_type result(
                const std::vector<std::pair<std::size_t, std::size_t>> &policy) const {
            return _algorithms.router.candidates_route(
                    _candidates, policy, _settings.within_edge_turns, _settings.join_merges,
                    _settings.a_star, _settings.routing_max_distance_factor);
        }

    private:
        using observations_type = std::vector<std::vector<double>>;
        using transitions_type = std::vector<std::vector<std::vector<double>>>;

        const network_type &_network;
        const matching::settings &_settings;
        matching::algorithms<network_type> &_algorithms;

        track_type _track;
        std::vector<candidate_type> _candidates;

        observations_type _observations{};
        transitions_type _transitions{};

    };

}

#endif //MAP_MATCHING_2_ENVIRONMENT_ENVIRONMENTS_HMM_HPP
