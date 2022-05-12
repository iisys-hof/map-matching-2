// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_MATCHER_HPP
#define MAP_MATCHING_2_MATCHER_HPP

#ifndef BOOST_THREAD_VERSION
#define BOOST_THREAD_VERSION 5
#endif

#include <boost/thread.hpp>
#include <boost/thread/executors/basic_thread_pool.hpp>

#include <geometry/track/multi_track.hpp>
#include <geometry/util.hpp>
#include <util/benchmark.hpp>
#include <learning/settings.hpp>

#include "candidate.hpp"
#include "settings.hpp"
#include "detector.hpp"

namespace map_matching_2::matching {

    template<typename Network, typename Track>
    class matcher {

    public:
        using network_type = Network;
        using track_type = Track;
        using measurement_type = typename Track::measurement_type;
        using multi_track_type = geometry::track::multi_track<measurement_type>;
        using graph_type = typename Network::graph_type;
        using node_type = typename Network::node_type;
        using edge_type = typename Network::edge_type;
        using point_type = typename node_type::point_type;
        using segment_type = typename edge_type::segment_type;
        using line_type = typename edge_type::line_type;
        using rich_line_type = typename edge_type::rich_line_type;
        using route_type = typename Network::route_type;
        using vertex_descriptor = typename Network::vertex_descriptor;
        using edge_descriptor = typename Network::edge_descriptor;
        using coordinate_type = typename boost::geometry::coordinate_type<point_type>::type;
        using distance_type = typename boost::geometry::default_distance_result<point_type, point_type>::type;
        using length_type = typename boost::geometry::default_length_result<line_type>::type;

        using buffer_distance_strategy_type = boost::geometry::strategy::buffer::distance_symmetric<coordinate_type>;
        using buffer_side_strategy_type = boost::geometry::strategy::buffer::side_straight;
        using buffer_join_strategy_type = boost::geometry::strategy::buffer::join_round;
        using buffer_end_strategy_type = boost::geometry::strategy::buffer::end_round;

        const buffer_side_strategy_type buffer_side_strategy{};
        const buffer_join_strategy_type buffer_join_strategy{};
        const buffer_end_strategy_type buffer_end_strategy{};

        using candidate_node_type = candidate_node<Network>;
        using candidate_edge_type = candidate_edge<Network>;
        using candidate_type = candidate<Network, Track>;

        using candidate_node_distance_comparator_type = decltype(&candidate_node_type::distance_comparator);
        using candidate_edge_distance_comparator_type = decltype(&candidate_edge_type::distance_comparator);

        const Network &network;

        explicit matcher(const Network &network, bool single_threaded = false);

        explicit matcher(const Network &network, std::uint32_t threads = boost::thread::hardware_concurrency());

        [[nodiscard]] std::vector<candidate_type> candidate_search_buffer(
                const Track &track, std::size_t buffer_points,
                double buffer_radius, double buffer_upper_radius = 10000.0, double buffer_lower_radius = 200.0,
                bool adaptive_radius = true, bool candidate_adoption_siblings = false,
                bool candidate_adoption_nearby = false, bool candidate_adoption_reverse = false);

        [[nodiscard]] std::vector<candidate_type> candidate_search_nearest(
                const Track &track, std::size_t number, bool with_reverse = false, bool with_adjacent = false,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false);

        [[nodiscard]] std::vector<candidate_type> candidate_search_combined(
                const Track &track, double buffer_radius, std::size_t number, double buffer_upper_radius = 10000.0,
                double buffer_lower_radius = 200.0, bool adaptive_radius = true, bool with_reverse = false,
                bool with_adjacent = false, bool candidate_adoption_siblings = false,
                bool candidate_adoption_nearby = false, bool candidate_adoption_reverse = false);

        [[nodiscard]] std::vector<candidate_type> candidate_search(
                const Track &track, std::size_t buffer_points, double buffer_radius, std::size_t number,
                bool buffer_candidate_search = true, bool nearest_candidate_search = false,
                double buffer_upper_radius = 10000.0, double buffer_lower_radius = 200.0, bool adaptive_radius = true,
                bool with_reverse = false, bool with_adjacent = false, bool candidate_adoption_siblings = false,
                bool candidate_adoption_nearby = false, bool candidate_adoption_reverse = false);

        void resize_candidates_buffer(
                const Track &track, std::vector<candidate_type> &candidates, const std::vector<std::size_t> &positions,
                std::size_t round, std::size_t buffer_points, double buffer_upper_radius = 10000.0,
                double buffer_lower_radius = 200.0, bool adaptive_radius = true, bool adaptive_resize = true,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false);

        void resize_candidates_nearest(
                const Track &track, std::vector<candidate_type> &candidates, const std::vector<std::size_t> &positions,
                std::size_t round, bool with_reverse = false, bool with_adjacent = false,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false);

        void resize_candidates_combined(
                const Track &track, std::vector<candidate_type> &candidates, const std::vector<std::size_t> &positions,
                std::size_t round, double buffer_upper_radius = 10000.0, double buffer_lower_radius = 200.0,
                bool adaptive_radius = true, bool adaptive_resize = true, bool with_reverse = false,
                bool with_adjacent = false, bool candidate_adoption_siblings = false,
                bool candidate_adoption_nearby = false, bool candidate_adoption_reverse = false);

        [[nodiscard]] std::vector<std::list<vertex_descriptor>> shortest_paths(
                const std::vector<candidate_type> &candidates,
                std::size_t from, std::size_t source, std::size_t to, double max_distance_factor = 10.0);

        [[nodiscard]] const route_type &route(
                const std::vector<candidate_type> &candidates,
                std::size_t from, std::size_t source, std::size_t to, std::size_t target,
                double max_distance_factor = 10.0);

        void clear_route_cache();

        [[nodiscard]] route_type candidate_route(
                const std::vector<candidate_type> &candidates,
                std::size_t from, std::size_t source, std::size_t to, std::size_t target,
                bool within_edge_turns = false, double max_distance_factor = 10.0);

        [[nodiscard]] route_type candidates_route(
                const std::vector<candidate_type> &candidates,
                const std::vector<std::pair<std::size_t, std::size_t>> &policy,
                bool within_edge_turns = false, bool export_edges = false, bool join_merges = true,
                double max_distance_factor = 10.0);

        [[nodiscard]] std::vector<candidate_type> candidates_policy(
                const std::vector<candidate_type> &candidates,
                const std::vector<std::pair<std::size_t, std::size_t>> &policy);

        void save_candidates(std::string filename, const std::vector<candidate_type> &candidates) const;

        [[nodiscard]] std::vector<std::set<defect>> detect(
                const Track &track, bool detect_duplicates = true, bool detect_forward_backward = true) const;

        [[nodiscard]] std::set<defect> detect(
                const Track &track, std::size_t from, std::size_t to, bool detect_forward_backward = true) const;

        [[nodiscard]] Track remove_defects(const Track &track, const std::vector<std::set<defect>> &defects) const;

        template<typename Learner>
        void match(const multi_track_type &multi_track,
                   const learning::settings &learn_settings = learning::settings{},
                   const matching::settings &match_settings = matching::settings{},
                   const std::function<void(const std::vector<typename Learner::environment_type> &,
                                            const std::vector<Learner> &,
                                            const std::vector<std::vector<std::pair<std::size_t, std::size_t>>> &,
                                            const multi_track_type &, const multi_track_type &, const route_type &,
                                            const double)>
                   &callback = [](const auto &environments, const auto &learners, const auto &policies,
                                  const auto &track, const auto &prepared, const auto &route, const auto duration) {}) {
            using environment_type = typename Learner::environment_type;
            std::vector<environment_type> environments;
            environments.reserve(multi_track.tracks.size());
            std::vector<Learner> learners;
            learners.reserve(multi_track.tracks.size());

            std::vector<std::vector<std::pair<std::size_t, std::size_t>>> policies;
            policies.reserve(multi_track.tracks.size());

            std::vector<Track> prepared;
            prepared.reserve(multi_track.tracks.size());
            std::vector<route_type> results;
            results.reserve(multi_track.tracks.size());

            double duration = 0.0;
            for (const auto &track_part: multi_track.tracks) {
                environments.emplace_back(environment_type{*this, match_settings});
                auto &environment = environments.back();

                learners.emplace_back(Learner{environments.back(), learn_settings});
                auto &learner = learners.back();

                std::vector<std::pair<std::size_t, std::size_t>> policy;

                duration += util::benchmark([&]() {
                    environment.set(track_part);
                    policy = learner();
                });

                prepared.emplace_back(environment.track());
                results.emplace_back(environment.template result<route_type>(policy));
                policies.emplace_back(std::move(policy));
            }

            callback(environments, learners, policies, multi_track,
                     multi_track_type{multi_track.id, std::move(prepared)},
                     route_type::merge({results.begin(), results.end()}), duration);

            clear_route_cache();
            geometry::clear_distance_cache();
            geometry::clear_azimuth_cache();
        }

        template<typename Learner>
        void match(const Track &track,
                   const learning::settings &learn_settings = learning::settings{},
                   const matching::settings &match_settings = matching::settings{},
                   const std::function<void(const std::vector<typename Learner::environment_type> &,
                                            const std::vector<Learner> &,
                                            const std::vector<std::vector<std::pair<std::size_t, std::size_t>>> &,
                                            const multi_track_type &, const multi_track_type &, const route_type &,
                                            const double)>
                   &callback = [](const auto &environments, const auto &learners, const auto &policies,
                                  const auto &track, const auto &prepared, const auto &route, const auto duration) {}) {
            multi_track_type multi_track{track.id, track};
            this->template match<Learner>(multi_track, learn_settings, match_settings, callback);
        }

        template<typename Learner>
        void match_all(const std::unordered_map<std::string, multi_track_type> &tracks,
                       const learning::settings &learn_settings = learning::settings{},
                       const matching::settings &match_settings = matching::settings{},
                       const std::function<void(const std::vector<typename Learner::environment_type> &,
                                                const std::vector<Learner> &,
                                                const std::vector<std::vector<std::pair<std::size_t, std::size_t>>> &,
                                                const multi_track_type &, const multi_track_type &, const route_type &,
                                                const double)>
                       &callback = [](const auto &environments, const auto &learners, const auto &policies,
                                      const auto &track, const auto &prepared, const auto &route,
                                      const auto duration) {}) {
            if (_single_threaded) {
                for (const auto &track: tracks) {
                    this->template match<Learner>(track.second, learn_settings, match_settings, callback);
                }
            } else {
                std::vector<multi_track_type> sorted_tracks;
                sorted_tracks.reserve(tracks.size());
                for (const auto &track: tracks) {
                    sorted_tracks.emplace_back(track.second);
                }

                multi_track_type::sort_tracks(sorted_tracks, false);

                std::list<boost::future<void>> futures;
                for (const auto &track: sorted_tracks) {
                    futures.emplace_back(boost::async(_matching_thread_pool, [&]() {
                        this->template match<Learner>(track, learn_settings, match_settings, callback);
                    }));
                }

                while (not futures.empty()) {
                    futures.front().get();
                    futures.pop_front();
                }
            }
        }

    private:
        using route_cache_key_type = std::pair<vertex_descriptor, vertex_descriptor>;

        detector<Track> _detector;

        bool _single_threaded;
        boost::basic_thread_pool _matching_thread_pool;

        boost::thread_specific_ptr<std::vector<vertex_descriptor>> _vertices;
        boost::thread_specific_ptr<std::vector<vertex_descriptor>> _predecessors;
        boost::thread_specific_ptr<std::vector<length_type>> _distances;
        boost::thread_specific_ptr<std::vector<typename boost::default_color_type>> _colors;
        boost::thread_specific_ptr<std::vector<std::size_t>> _heap_index;
        boost::thread_specific_ptr<std::vector<vertex_descriptor>> _queue;

        using route_cache_type = std::unordered_map<route_cache_key_type, route_type, boost::hash<route_cache_key_type>>;
        using route_cache_iterator_type = typename route_cache_type::iterator;

        boost::thread_specific_ptr<route_cache_type> _route_cache;

        [[nodiscard]] boost::geometry::model::multi_polygon<boost::geometry::model::polygon<point_type>>
        _create_buffer(const point_type &point, double buffer_radius, std::size_t buffer_points = 0) const;

        [[nodiscard]]  double _candidate_search_buffer_measurement(
                std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type> &candidate_edge_set,
                std::size_t round, const Track &track, std::size_t index, std::size_t buffer_points,
                double buffer_radius, double buffer_upper_radius = 10000.0, double buffer_lower_radius = 200.0,
                bool adaptive_radius = true) const;

        [[nodiscard]] std::size_t _candidate_search_nearest_measurement(
                std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type> &candidate_edge_set,
                std::size_t round, const Track &track, std::size_t index, std::size_t number,
                bool with_reverse = false, bool with_adjacent = false) const;

        [[nodiscard]]  std::pair<double, std::size_t> _candidate_search_combined_measurement(
                std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type> &candidate_edge_set,
                std::size_t round, const Track &track, std::size_t index, double buffer_radius, std::size_t number,
                double buffer_upper_radius = 10000.0, double buffer_lower_radius = 200.0, bool adaptive_radius = true,
                bool with_reverse = false, bool with_adjacent = false) const;

        void _process_candidate_query(
                std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type> &candidate_edge_set,
                std::size_t round, const Track &track, std::size_t index, std::set<edge_descriptor> &edge_result) const;

        void _finalize_candidates(
                std::vector<candidate_type> &candidates, std::size_t round, const Track &track,
                std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> &candidate_edge_sets,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) const;

        void _finalize_candidates_buffer(
                std::vector<candidate_type> &candidates, std::size_t round, const Track &track,
                const std::vector<double> &radii,
                std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> &candidate_edge_sets,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) const;

        void _finalize_candidates_nearest(
                std::vector<candidate_type> &candidates, std::size_t round, const Track &track,
                const std::vector<std::size_t> &numbers,
                std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> &candidate_edge_sets,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) const;

        void _finalize_candidates_combined(
                std::vector<candidate_type> &candidates, std::size_t round, const Track &track,
                const std::vector<std::pair<double, std::size_t>> &radii_numbers,
                std::vector<std::multiset<candidate_edge_type, candidate_edge_distance_comparator_type>> &candidate_edge_sets,
                bool candidate_adoption_siblings = false, bool candidate_adoption_nearby = false,
                bool candidate_adoption_reverse = false) const;

        void _prepare_shortest_path_memory();

    };

}

#endif //MAP_MATCHING_2_MATCHER_HPP
