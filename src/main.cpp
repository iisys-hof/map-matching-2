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

#ifdef _WIN32

#include <windows.h>
#include <cstdio>

#endif

#include <filesystem>

#include <rpnew.h>

#ifndef BOOST_THREAD_VERSION
#define BOOST_THREAD_VERSION 5
#endif

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/thread/executors/basic_thread_pool.hpp>
#include <boost/program_options.hpp>

#include <io/network/arc_node_importer.hpp>
#include <io/network/network_loader.hpp>
#include <io/network/network_saver.hpp>
#include <io/network/osm_car_importer.hpp>
#include <io/network/osm_exporter.hpp>
#include <geometry/network/types.hpp>
#include <io/track/csv_track_importer.hpp>
#include <io/track/edges_list_importer.hpp>
#include <io/track/gpx_track_importer.hpp>
#include <io/track/input_importer.hpp>
#include <geometry/track/types.hpp>
#include <matching/types.hpp>
#include <geometry/compare.hpp>
#include <geometry/util.hpp>
#include <io/csv_exporter.hpp>
#include <environment/environments/hmm.hpp>
#include <environment/environments/single.hpp>
#include <learning/algorithms/viterbi.hpp>
#include <learning/algorithms/policy_iteration.hpp>
#include <learning/algorithms/value_iteration.hpp>
#include <learning/algorithms/q_learning.hpp>
#include <util/benchmark.hpp>

template<typename Network>
void build_spatial_indices(Network &network, const bool verbose = false) {
    if (verbose) {
        std::cout << "Building spatial indices ... " << std::flush;
    }
    auto duration = map_matching_2::util::benchmark([&]() { network.rebuild_spatial_indices(); });
    if (verbose) {
        std::cout << "done in " << duration << "s" << std::endl;
    }
}

template<typename MultiTrack>
std::pair<bool, const MultiTrack &>
find_ground_truth(const MultiTrack &track, const std::unordered_map<std::string, MultiTrack> &compare_tracks) {
    const auto &compare_track = compare_tracks.find(track.id);
    if (compare_track != compare_tracks.cend()) {
        return std::pair{true, std::cref(compare_track->second)};
    }

    return std::pair{false, MultiTrack{}};
}

struct compare_settings {
    double simplifying_tolerance;
    double simplifying_reverse_tolerance;
    double adoption_distance_tolerance;
    double split_distance_tolerance;
    double split_direction_tolerance;
};

template<typename MultiTrack>
std::pair<bool, map_matching_2::geometry::comparison<typename MultiTrack::rich_line_type>>
compare_track(const map_matching_2::geometry::line_comparator<typename MultiTrack::rich_line_type> &comparator,
              const std::pair<bool, const MultiTrack &> &ground_truth,
              const typename MultiTrack::multi_rich_line_type &match) {
    if (ground_truth.first) {
        return std::pair{true, comparator.compare(ground_truth.second, match)};
    } else {
        return std::pair{false, map_matching_2::geometry::comparison<typename MultiTrack::rich_line_type>{}};
    }
}

template<typename MultiTrack, typename Route, typename CSV>
void
result_processor(const MultiTrack &track, const MultiTrack &prepared, const Route &route,
                 const MultiTrack &ground_truth,
                 const std::pair<bool, map_matching_2::geometry::comparison<typename MultiTrack::rich_line_type>> &comparison,
                 const double duration, const bool console, const bool verbose,
                 CSV &output_csv, CSV &compare_output_csv) {
    if (console) {
        if (verbose) {
            std::cout << "Input: " << track.wkt() << "\n" << std::flush;
            std::cout << "Prepared: " << prepared.wkt() << "\n" << std::flush;
            std::cout << "Result: " << std::flush;
        }
        std::cout << route.wkt() << std::endl;
    }
    if (output_csv.is_writable()) {
        output_csv << std::vector{track.id, std::to_string(duration), track.wkt(), prepared.wkt(), route.wkt()};
        if (verbose) {
            output_csv.flush();
        }
    }
    if (comparison.first) {
        using line_type = typename MultiTrack::rich_line_type::line_type;
        boost::geometry::model::multi_linestring<line_type> error_adds, error_misses;
        if (verbose and console or compare_output_csv.is_writable()) {
            error_adds.reserve(comparison.second.error_adds.size());
            error_misses.reserve(comparison.second.error_adds.size());
            for (const auto &error_add: comparison.second.error_adds) {
                error_adds.emplace_back(error_add.line());
            }
            for (const auto &error_miss: comparison.second.error_misses) {
                error_misses.emplace_back(error_miss.line());
            }
        }

        if (verbose) {
            if (console) {
                std::cout << "Ground Truth: " << ground_truth.wkt() << std::endl;

                std::cout << "Comparison:\n";
                std::cout << "Error Adds:\n";
                for (const auto &error_add: error_adds) {
                    std::cout << map_matching_2::geometry::to_wkt(error_add) << "\n";
                }
                std::cout << "Error Misses:\n";
                for (const auto &error_miss: error_misses) {
                    std::cout << map_matching_2::geometry::to_wkt(error_miss) << "\n";
                }
            }
            std::cout << "Error Fraction: " << comparison.second.error_fraction << ", "
                      << "Error Added: " << comparison.second.error_added << ", "
                      << "Error Missed: " << comparison.second.error_missed << std::endl;
        }
        if (compare_output_csv.is_writable()) {
            compare_output_csv << std::vector{
                    track.id, track.wkt(), prepared.wkt(), route.wkt(), ground_truth.wkt(),
                    std::to_string(comparison.second.error_fraction), std::to_string(comparison.second.error_added),
                    std::to_string(comparison.second.error_missed), map_matching_2::geometry::to_wkt(error_adds),
                    map_matching_2::geometry::to_wkt(error_misses)};
            if (verbose) {
                compare_output_csv.flush();
            }
        }
    }
}

template<typename Environment, typename LearnFunction, typename Matcher, typename MultiTrack>
void
matching(Matcher &matcher, const std::unordered_map<std::string, MultiTrack> &tracks,
         const std::unordered_map<std::string, MultiTrack> &compare_tracks,
         const map_matching_2::geometry::line_comparator<typename MultiTrack::rich_line_type> &comparator,
         std::string output_file, std::string compare_output_file, std::string candidates_folder,
         const map_matching_2::learning::settings &learn_settings, map_matching_2::matching::settings &match_settings,
         const bool single_threading = false, const bool console = false, const bool quiet = false,
         const bool verbose = false) {
    if (not quiet) {
        std::cout << "Start Matching ..." << std::endl;
    }

    // Prepare output
    map_matching_2::io::csv_exporter<',', '"'> output_csv{std::move(output_file)};
    if (output_csv.is_writable()) {
        output_csv << std::vector{"id", "duration", "track", "prepared", "match"};
    }

    map_matching_2::io::csv_exporter<',', '"'> compare_output_csv{std::move(compare_output_file)};
    if (compare_output_csv.is_writable()) {
        compare_output_csv << std::vector{"id", "track", "prepared", "match", "ground_truth", "error_fraction",
                                          "error_added", "error_missed", "error_adds", "error_misses"};
    }

    const auto candidate_export = [&](const auto &environments, const auto &learners,
                                      const auto &policies, const auto &track) {
        if (not candidates_folder.empty() and environments.size() > 0) {
            std::vector<typename Matcher::candidate_type> candidates;
            std::vector<typename Matcher::candidate_type> policy;
            for (std::size_t i = 0; i < environments.size(); ++i) {
                const auto &environment_candidates = environments[i].candidates();
                candidates.insert(candidates.end(), environment_candidates.cbegin(), environment_candidates.cend());
                auto candidate_policy = matcher.candidates_policy(environment_candidates, policies[i]);
                policy.insert(policy.end(), candidate_policy.cbegin(), candidate_policy.cend());
            }

            std::string id = track.id;
            std::regex reg{"[^_[:alnum:]]"};
            std::regex_replace(id, reg, "");

            std::filesystem::path prefix_path{candidates_folder};
            prefix_path.append(id + "_" + environments[0].name + "_" + learners[0].name);

            std::filesystem::path candidate_path = prefix_path;
            std::filesystem::path policy_path = prefix_path;

            matcher.save_candidates(candidate_path.concat("_candidates.csv").string(), candidates);
            matcher.save_candidates(policy_path.concat("_policy.csv").string(), policy);
        }
    };

    std::size_t original_points = 0;
    std::size_t filtered_points = 0;
    double duration;
    if (single_threading) {
        duration = map_matching_2::util::benchmark([&]() {
            std::size_t i = 0;
            for (const auto &track: tracks) {
                if (verbose) {
                    std::cout << "Matching " << (i + 1) << "/" << tracks.size() << " " << track.first << " "
                              << std::flush;
                }

                matcher.template match<LearnFunction>(
                        track.second, learn_settings, match_settings,
                        [&](const auto &environments, const auto &learners, const auto &policies,
                            const auto &track, const auto &prepared, const auto &route, const auto duration) {
                            if (verbose) {
                                std::size_t edges = 0, combinations = 0;
                                for (std::size_t i = 0; i < environments.size(); ++i) {
                                    edges += environments[i].edges();
                                    combinations += environments[i].states();
                                }
                                std::cout << "done in " << duration << "s, track points: " << track.sizes()
                                          << ", prepared points: " << prepared.sizes() << ", candidates: " << edges
                                          << ", combinations: " << combinations << std::endl;
                                original_points += track.sizes();
                                filtered_points += prepared.sizes();
                            }
                            candidate_export(environments, learners, policies, track);
                            const auto ground_truth = find_ground_truth(track, compare_tracks);
                            const auto comparison = compare_track(comparator, ground_truth,
                                                                  route.get_multi_rich_line());
                            result_processor(track, prepared, route, ground_truth.second, comparison, duration,
                                             console, verbose, output_csv, compare_output_csv);
                            i++;
                        });
            }
        });
    } else {
        // all tracks
        duration = map_matching_2::util::benchmark([&]() {
            std::size_t i = 0;
            boost::mutex mtx;
            matcher.template match_all<LearnFunction>(
                    tracks, learn_settings, match_settings,
                    [&](const auto &environments, const auto &learners, const auto &policies,
                        const auto &track, const auto &prepared, const auto &route, const auto duration) {
                        candidate_export(environments, learners, policies, track);
                        const auto ground_truth = find_ground_truth(track, compare_tracks);
                        const auto comparison = compare_track(comparator, ground_truth, route.get_multi_rich_line());
                        boost::lock_guard lock{mtx};
                        if (verbose) {
                            std::size_t edges = 0, combinations = 0;
                            for (std::size_t i = 0; i < environments.size(); ++i) {
                                edges += environments[i].edges();
                                combinations += environments[i].states();
                            }
                            std::cout << "Matching " << (i + 1) << "/" << tracks.size() << " " << track.id
                                      << " done in " << duration << "s, track points: " << track.sizes()
                                      << ", prepared points: " << prepared.sizes() << ", candidates: " << edges
                                      << ", combinations: " << combinations << std::endl;
                            original_points += track.sizes();
                            filtered_points += prepared.sizes();
                        }
                        result_processor(track, prepared, route, ground_truth.second, comparison, duration,
                                         console, verbose, output_csv, compare_output_csv);
                        i++;
                    });
        });
    }

    if (verbose) {
        std::cout << "Points: Original: " << original_points << ", Filtered: " << filtered_points << std::endl;
    }

    if (not quiet) {
        std::cout << "Finished Matching after " << duration << " seconds." << std::endl;
    }
}

template<typename MultiTrack>
void filter_tracks(std::unordered_map<std::string, MultiTrack> &tracks,
                   const std::vector<std::string> &selectors) {
    std::unordered_map<std::string, MultiTrack> selected_tracks;
    if (not selectors.empty()) {
        std::unordered_set<std::string> selectors_set{selectors.cbegin(), selectors.cend()};
        for (const auto &track: tracks) {
            if (selectors_set.contains(track.first)) {
                selected_tracks.emplace(track);

                selectors_set.erase(track.first);
                if (selectors_set.empty()) {
                    break;
                }
            }
        }

        tracks = selected_tracks;
    }
}

template<typename Matcher, typename MultiTrack>
void match(Matcher &matcher, std::unordered_map<std::string, MultiTrack> tracks,
           const std::unordered_map<std::string, MultiTrack> &compare_tracks, std::string output_file,
           std::string compare_output_file, const compare_settings &compare_settings,
           std::string candidates_folder, const map_matching_2::learning::settings &learn_settings,
           map_matching_2::matching::settings &match_settings, const std::string &model,
           const std::vector<std::string> &selectors, const bool single_threading = false, const bool console = false,
           const bool quiet = false, const bool verbose = false) {
    using mdp = map_matching_2::environment::single<Matcher>;
    using policy_iteration = map_matching_2::learning::policy_iteration<mdp>;
    using value_iteration = map_matching_2::learning::value_iteration<mdp>;
    using q_learning = map_matching_2::learning::q_learning<mdp>;

    using hmm = map_matching_2::environment::hmm<Matcher>;
    using viterbi = map_matching_2::learning::viterbi<hmm>;

    // only filters if selectors is not empty
    filter_tracks(tracks, selectors);

    using rich_line_type = typename MultiTrack::rich_line_type;
    map_matching_2::geometry::line_comparator<rich_line_type> comparator;
    comparator.simplifying_tolerance = compare_settings.simplifying_tolerance;
    comparator.simplifying_reverse_tolerance = compare_settings.simplifying_reverse_tolerance;
    comparator.adoption_distance_tolerance = compare_settings.adoption_distance_tolerance;
    comparator.split_distance_tolerance = compare_settings.split_distance_tolerance;
    comparator.split_direction_tolerance = compare_settings.split_direction_tolerance;

    if (model == "policy-iteration") {
        matching<mdp, policy_iteration>(matcher, tracks, compare_tracks, comparator,
                                        std::move(output_file), std::move(compare_output_file),
                                        std::move(candidates_folder), learn_settings, match_settings, single_threading,
                                        console, quiet, verbose);
    } else if (model == "value-iteration") {
        matching<mdp, value_iteration>(matcher, tracks, compare_tracks, comparator,
                                       std::move(output_file), std::move(compare_output_file),
                                       std::move(candidates_folder), learn_settings, match_settings, single_threading,
                                       console, quiet, verbose);
    } else if (model == "q-learning") {
        matching<mdp, q_learning>(matcher, tracks, compare_tracks, comparator,
                                  std::move(output_file), std::move(compare_output_file),
                                  std::move(candidates_folder), learn_settings, match_settings, single_threading,
                                  console, quiet, verbose);
    } else if (model == "viterbi") {
        matching<hmm, viterbi>(matcher, tracks, compare_tracks, comparator,
                               std::move(output_file), std::move(compare_output_file),
                               std::move(candidates_folder), learn_settings, match_settings, single_threading,
                               console, quiet, verbose);
    } else {
        throw std::invalid_argument{"model unknown, please review help"};
    }
}

template<typename MultiTrack>
void
compare(std::unordered_map<std::string, MultiTrack> tracks,
        const std::unordered_map<std::string, MultiTrack> &compare_tracks, std::string compare_output_file,
        const compare_settings &compare_settings,
        const std::vector<std::string> &selectors, const bool single_threading = false,
        const bool console = false, const bool quiet = false, const bool verbose = false) {
    if (not quiet) {
        std::cout << "Start Comparing ..." << std::endl;
    }

    // only filters if selectors is not empty
    filter_tracks(tracks, selectors);

    // Prepare output
    map_matching_2::io::csv_exporter<',', '"'> output_csv{""};

    map_matching_2::io::csv_exporter<',', '"'> compare_output_csv{std::move(compare_output_file)};
    if (compare_output_csv.is_writable()) {
        compare_output_csv << std::vector{"id", "track", "prepared", "match", "ground_truth", "error_fraction",
                                          "error_added", "error_missed", "error_adds", "error_misses"};
    }

    using route_type = map_matching_2::geometry::network::route<typename MultiTrack::line_type>;

    using rich_line_type = typename MultiTrack::rich_line_type;
    map_matching_2::geometry::line_comparator<rich_line_type> comparator;
    comparator.simplifying_tolerance = compare_settings.simplifying_tolerance;
    comparator.simplifying_reverse_tolerance = compare_settings.simplifying_reverse_tolerance;
    comparator.adoption_distance_tolerance = compare_settings.adoption_distance_tolerance;
    comparator.split_distance_tolerance = compare_settings.split_distance_tolerance;
    comparator.split_direction_tolerance = compare_settings.split_direction_tolerance;

    double duration;
    if (single_threading) {
        duration = map_matching_2::util::benchmark([&]() {
            std::size_t i = 0;
            for (const auto &track: tracks) {
                if (verbose) {
                    std::cout << "Comparing " << (i + 1) << "/" << tracks.size() << " " << track.first << " "
                              << std::flush;
                }

                const auto ground_truth = find_ground_truth(track.second, compare_tracks);

                std::pair<bool, map_matching_2::geometry::comparison<typename MultiTrack::rich_line_type>> comparison;
                double inner_duration = map_matching_2::util::benchmark([&]() {
                    comparison = compare_track(comparator, ground_truth, track.second);
                });

                if (verbose) {
                    std::cout << "done in " << inner_duration << "s" << std::endl;
                }

                result_processor(track.second, track.second, route_type{}, ground_truth.second, comparison,
                                 inner_duration, console, verbose, output_csv, compare_output_csv);
                i++;
            }
        });
    } else {
        // all tracks
        duration = map_matching_2::util::benchmark([&]() {
            std::size_t i = 0;
            boost::mutex mtx;

            boost::basic_thread_pool _compare_thread_pool{boost::thread::hardware_concurrency()};

            std::vector<MultiTrack> sorted_tracks;
            sorted_tracks.reserve(tracks.size());
            for (const auto &track: tracks) {
                sorted_tracks.emplace_back(track.second);
            }

            MultiTrack::sort_tracks(sorted_tracks, false);

            std::list<boost::future<void>>
                    futures;
            for (const auto &track: sorted_tracks) {
                futures.emplace_back(boost::async(_compare_thread_pool, [&]() {
                    const auto ground_truth = find_ground_truth(track, compare_tracks);

                    std::pair<bool, map_matching_2::geometry::comparison<typename MultiTrack::rich_line_type>> comparison;
                    double inner_duration = map_matching_2::util::benchmark([&]() {
                        comparison = compare_track(comparator, ground_truth, track);
                    });

                    boost::lock_guard lock{mtx};
                    if (verbose) {
                        std::cout << "Comparison " << (i + 1) << "/" << tracks.size() << " " << track.id
                                  << " done in " << inner_duration << "s" << std::endl;
                    }

                    result_processor(track, track, route_type{}, ground_truth.second, comparison, inner_duration,
                                     console, verbose, output_csv, compare_output_csv);
                    i++;
                }));
            }

            while (not futures.empty()) {
                futures.front().get();
                futures.pop_front();
            }
        });
    }

    if (not quiet) {
        std::cout << "Finished Comparison after " << duration << " seconds." << std::endl;
    }
}

void detect_extension(const std::string &file, std::string &file_type) {
    if (not file.empty()) {
        std::filesystem::path path{file};
        file_type = path.extension();
    }

    boost::algorithm::to_lower(file_type);

    if (not(file_type == ".csv" or file_type == ".gpx")) {
        file_type = ".csv";
    }
}

int main(int argc, char *argv[]) {
#ifdef _WIN32
    // Set console code page to UTF-8 so console known how to interpret string data
    SetConsoleOutputCP(CP_UTF8);

    // Enable buffering to prevent VS from chopping up UTF-8 byte sequences
    setvbuf(stdout, nullptr, _IOFBF, 1000);
#endif

    namespace po = boost::program_options;

    std::string network_file, arcs_file, nodes_file, network_srs, network_transform_srs, network_srs_type,
            network_transform_srs_type, tracks_file_type, tracks_srs, tracks_transform_srs, tracks_srs_type,
            tracks_transform_srs_type, output_file, candidates_folder, compare_file_type, compare_srs, compare_srs_type,
            compare_transform_srs, compare_transform_srs_type, compare_output_file, model, candidate_search;
    std::vector<std::string> tracks_file, compare_file, id, compare_id, selectors, export_network,
            export_simplified_network, export_retained_network, export_transformed_network;
    std::size_t state_size, k_nearest, episodes;
    char delimiter, compare_delimiter;
    double routing_max_distance_factor, radius, radius_upper_limit, radius_lower_limit, learning_rate, epsilon,
            discount, threshold, simplify_track_distance_tolerance, median_merge_distance_tolerance,
            mdp_distance_factor, mdp_length_factor, mdp_azimuth_factor, mdp_direction_factor, hmm_distance_factor,
            hmm_length_factor, hmm_azimuth_factor, hmm_direction_factor, compare_simplifying_tolerance,
            compare_simplifying_reverse_tolerance, compare_adoption_distance_tolerance,
            compare_split_distance_tolerance, compare_split_direction_tolerance;
    bool read_line, console, wkt, no_header, no_id, no_parse_time, export_edges, join_merges, compare_only, no_compare,
            compare_edges_list_mode, compare_wkt, compare_no_header, compare_no_id, filter_duplicates,
            filter_defects, simplify_track, median_merge, adaptive_median_merge, adaptive_radius, skip_errors,
            candidate_adoption_siblings, candidate_adoption_nearby, candidate_adoption_reverse,
            simplify_network, simplify_network_complete, remove_unconnected, single_threading, quiet, verbose;
    std::string id_aggregator, compare_id_aggregator, x, y, compare_x, compare_y,
            geometry, compare_geometry, time, time_format, network_output, network_save, network_load,
            export_network_nodes, export_network_edges,
            export_simplified_network_nodes, export_simplified_network_edges,
            export_retained_network_nodes, export_retained_network_edges,
            export_transformed_network_nodes, export_transformed_network_edges;

    po::variables_map vm;
    po::options_description all("Available Options"),
            options_network("Network Options"),
            options_tracks("Track Options"),
            options_output("Output Options"),
            options_compare("Compare Options"),
            options_export("Network Export Options"),
            options_console("Console Options"),
            options_performance("Performance Options"),
            options_preparation("Track Preparation Options"),
            options_model("Model Options"),
            options_mdp("Markov-Decision-Process Options"),
            options_hmm("Hidden-Markov-Model Options"),
            options_dp_iteration("Dynamic Programming Options"),
            options_q_learning("Q-Learning Options");

    try {
        options_network.add_options()
                ("network", po::value<std::string>(&network_file),
                 "path to .osm.pbf network file")
                ("arcs", po::value<std::string>(&arcs_file),
                 "path to arcs network file, requires nodes file")
                ("nodes", po::value<std::string>(&nodes_file),
                 "path to nodes network file, requires arcs file")
                ("network-load", po::value<std::string>(&network_load),
                 "loads network from binary file in deserialized form, works together with network-save;"
                 "is much faster than initial import from original network source files, should be preferred; "
                 "network is seen as ready and is not processed, transformed or changed in any way any further; "
                 "with compare-edge-lists-mode an input network is still needed but only used for import of compare tracks")
                ("network-srs",
                 po::value<std::string>(&network_srs)->default_value("+proj=longlat +datum=WGS84 +no_defs"),
                 "network spatial reference system (srs) as PROJ string, default is WGS84, "
                 "tracks and compare tracks if given need to be in the same reference system as the network")
                ("network-srs-type", po::value<std::string>(&network_srs_type),
                 "network srs type, can be set to 'geographic' or 'cartesian'; "
                 "by default automatically detects type depending on set srs")
                ("network-transform-srs", po::value<std::string>(&network_transform_srs),
                 "network transform spatial reference system (srs) as PROJ string; "
                 "when defined, network is transformed to given srs, "
                 "tracks and compare tracks if given need to be in the same reference system as the network")
                ("network-transform-srs-type", po::value<std::string>(&network_transform_srs_type),
                 "network transform srs type, can be set to 'geographic' or 'cartesian'; "
                 "by default automatically detects type depending on set transform srs")
                ("simplify-network", po::value<bool>(&simplify_network)->default_value(true, "on"),
                 "simplify network graph")
                ("simplify-network-complete", po::value<bool>(&simplify_network_complete)->default_value(true, "on"),
                 "simplify as much as possible in network graph, osm ids and tags are not seperated, "
                 "if only edges with same osm id and tags should be simplified, disable this option")
                ("remove-unconnected", po::value<bool>(&remove_unconnected)->default_value(false, "off"),
                 "remove weakly unconnected components so that only the largest subgraph remains")
                ("routing-max-distance-factor",
                 po::value<double>(&routing_max_distance_factor)->default_value(10.0, "10.0"),
                 "max distance factor for upper bound for routing algorithm, "
                 "removes all nodes from routing that are too far away from the search area between a start and end node, "
                 "dramatically reduces routing duration in networks significantly larger than the given track");

        options_tracks.add_options()
                ("tracks", po::value<std::vector<std::string>>(&tracks_file),
                 "path to tracks file; "
                 "can be specified multiple times for multiple files but all files need the same format per file type")
                ("tracks-file-type", po::value<std::string>(&tracks_file_type),
                 "tracks file type; "
                 "by default is automatically detected from filename extension"
                 "falls back to .csv automatically if no other supported extension is detected; "
                 "see remaining options for processing possibilities; "
                 "can be manually overridden to supported file types:\n"
                 ".csv\n"
                 ".gpx")
                ("tracks-srs",
                 po::value<std::string>(&tracks_srs)->default_value("+proj=longlat +datum=WGS84 +no_defs"),
                 "tracks spatial reference system (srs) as PROJ string, default is WGS84")
                ("tracks-srs-type", po::value<std::string>(&tracks_srs_type),
                 "tracks srs type, can be set to 'geographic' or 'cartesian'; "
                 "by default automatically detects type depending on set srs")
                ("tracks-transform-srs", po::value<std::string>(&tracks_transform_srs),
                 "tracks transform spatial reference system (srs) as PROJ string; "
                 "when defined, tracks are transformed to given srs")
                ("tracks-transform-srs-type", po::value<std::string>(&tracks_transform_srs_type),
                 "tracks transform srs type, can be set to 'geographic' or 'cartesian'; "
                 "by default automatically detects type depending on set transform srs")
                ("delimiter", po::value<char>(&delimiter)->default_value(','),
                 "delimiter for .csv tracks file, "
                 "only for import, export always uses ',' (because csv is 'comma' separated)")
                ("no-header", po::bool_switch(&no_header),
                 "csv has no header, column specifications are interpreted as indices of columns")
                ("no-id", po::bool_switch(&no_id),
                 "csv has no id column, in this case, an id of 0 is used and all points are asumed to be one track, "
                 "except when WKT read mode and linestrings are given, then the row index is used as id")
                ("id", po::value<std::vector<std::string>>(&id)->default_value({"id"}, "id"),
                 "id column of tracks .csv file, "
                 "can be specified multiple times for composite primary keys over multiple columns, "
                 "internally, the id-aggregator field is then used for concatenation of the values")
                ("id-aggregator", po::value<std::string>(&id_aggregator)->default_value("_"),
                 "used for aggregating the values of composite primary keys, see id")
                ("wkt", po::bool_switch(&wkt),
                 "use WKT read mode for .csv tracks file instead of default x,y coordinates read mode")
                ("x", po::value<std::string>(&x)->default_value("x"),
                 "x column (or longitude) of tracks .csv file in coordinates read mode")
                ("y", po::value<std::string>(&y)->default_value("y"),
                 "y column (or latitude) of tracks .csv file in coordinates read mode")
                ("geometry", po::value<std::string>(&geometry)->default_value("geometry"),
                 "geometry column of tracks .csv file in WKT read mode "
                 "can either be of type POINT or LINESTRING, "
                 "in POINT mode, points are sorted ascending by time column, if available, else by appearance, "
                 "in LINESTRING mode, timestamp is just an incremented value for each point")
                ("time", po::value<std::string>(&time)->default_value("time"),
                 "time column of tracks .csv file, is optional if no such column exists, for example in WKT LINESTRING read mode, "
                 "currently, the time column is only used for sorting points while reading tracks .csv file")
                ("time-format", po::value<std::string>(&time_format)->default_value("%FT%T%Oz"),
                 "time format for parsing the time value in the time column into unix timestamp format, "
                 "please review std::chrono::parse documentation for syntax, "
                 "default reads ISO 8601 date and time with time zone")
                ("no-parse-time", po::bool_switch(&no_parse_time),
                 "do not parse time, is needed when the time is given in seconds only")
                ("selector", po::value<std::vector<std::string >>(&selectors),
                 "track selector for matching a specific track from the tracks file, can be specified multiple times");

        options_output.add_options()
                ("output", po::value<std::string>(&output_file),
                 "path to .csv output file for matched results")
                ("export-edges", po::value<bool>(&export_edges)->default_value(false, "off"),
                 "export matched routes based on original edges from the network graph, "
                 "this gives route segments that are exactly comparable to the underlying road network")
                ("join-merges", po::value<bool>(&join_merges)->default_value(true, "on"),
                 "by default or when export-edges is off, the routes between candidates are concatenated, "
                 "this parameter removes additional joint points when possible, "
                 "so all joint points are removed except when reverse movements within an edge happen")
                ("console", po::bool_switch(&console),
                 "output WKT to console");

        options_compare.add_options()
                ("compare-only", po::bool_switch(&compare_only),
                 "disables map-matching, uses tracks only for comparing with given ground truth matches, "
                 "you can for example input pre-matched tracks for comparing them to given ground truth matches")
                ("no-compare", po::bool_switch(&no_compare),
                 "disables comparison completely")
                ("compare-edges-list-mode", po::bool_switch(&compare_edges_list_mode),
                 "in this mode, compare is no .csv file but a file that contains a list of edges that form the ground truth route, "
                 "in this case, the number of the edge is the id of the edge in the network graph, "
                 "also the network-srs and network-transform-srs as well as the corresponding types are used")
                ("compare", po::value<std::vector<std::string>>(&compare_file),
                 "path to compare file for ground truth matches for statistics on results; "
                 "can be specified multiple times for multiple files but all files need the same format per file type")
                ("compare-file-type", po::value<std::string>(&compare_file_type),
                 "compare file type; "
                 "by default is automatically detected from filename extension; "
                 "falls back to .csv automatically if no other supported extension is detected; "
                 "see remaining options for processing possibilities; "
                 "can be manually overridden to supported file types:\n"
                 ".csv\n"
                 ".gpx")
                ("compare-srs",
                 po::value<std::string>(&compare_srs)->default_value("+proj=longlat +datum=WGS84 +no_defs"),
                 "compare spatial reference system (srs) as PROJ string, default is WGS84")
                ("compare-srs-type", po::value<std::string>(&compare_srs_type),
                 "compare srs type, can be set to 'geographic' or 'cartesian'; "
                 "by default automatically detects type depending on set srs")
                ("compare-transform-srs", po::value<std::string>(&compare_transform_srs),
                 "compare transform spatial reference system (srs) as PROJ string; "
                 "when defined, compare tracks are transformed to given srs")
                ("compare-transform-srs-type", po::value<std::string>(&compare_transform_srs_type),
                 "compare transform srs type, can be set to 'geographic' or 'cartesian'; "
                 "by default automatically detects type depending on set transform srs")
                ("compare-output", po::value<std::string>(&compare_output_file),
                 "path to .csv compare output file for results of comparison, "
                 "in compare-only mode, this is filled from output parameter if not specified")
                ("compare-delimiter", po::value<char>(&compare_delimiter)->default_value(','),
                 "see delimiter for tracks .csv")
                ("compare-no-header", po::bool_switch(&compare_no_header),
                 "see no-header for tracks .csv")
                ("compare-no-id", po::bool_switch(&compare_no_id),
                 "see no-id for tracks .csv")
                ("compare-id", po::value<std::vector<std::string >>(&compare_id)->default_value({"id"}, "id"),
                 "see id for tracks .csv")
                ("compare-id-aggregator", po::value<std::string>(&compare_id_aggregator)->default_value("_"),
                 "see id-aggregator for tracks .csv")
                ("compare-wkt", po::bool_switch(&compare_wkt),
                 "see wkt for tracks .csv")
                ("compare-x", po::value<std::string>(&compare_x)->default_value("x"),
                 "see x for tracks .csv")
                ("compare-y", po::value<std::string>(&compare_y)->default_value("y"),
                 "see y for tracks .csv")
                ("compare-geometry", po::value<std::string>(&compare_geometry)->default_value("geometry"),
                 "see geometry for tracks .csv")
                ("compare-simplify-distance-tolerance",
                 po::value<double>(&compare_simplifying_tolerance)->default_value(0.1, "0.1"),
                 "compare simplify distance tolerance in meters around comparison lines for Douglas-Peucker algorithm")
                ("compare-simplify-reverse-tolerance",
                 po::value<double>(&compare_simplifying_reverse_tolerance)->default_value(0.1, "0.1"),
                 "compare reverse detection azimuth tolerance in degrees for splitting simplifications")
                ("compare-adoption-distance-tolerance",
                 po::value<double>(&compare_adoption_distance_tolerance)->default_value(0.1, "0.1"),
                 "compare adoption distance tolerance in meters around a point for merging close points to remove tiny differences")
                ("compare-split-distance-tolerance",
                 po::value<double>(&compare_split_distance_tolerance)->default_value(1.0, "1.0"),
                 "compare split distance tolerance in meters for splitting lines at closest points between lines")
                ("compare-split-direction-tolerance",
                 po::value<double>(&compare_split_direction_tolerance)->default_value(90.0, "90.0"),
                 "compare split direction tolerance in degrees for allowing only splits at points "
                 "that have no more direction change degrees than specified");

        options_export.add_options()
                ("network-save", po::value<std::string>(&network_save),
                 "saves network in serialized form in binary file, works together with network-load")
                ("export-candidates", po::value<std::string>(&candidates_folder),
                 "export candidates to the folder given, if no folder is given, no candidates are exported")
                ("network-output", po::value<std::string>(&network_output),
                 "output final network to .osm.pbf file for faster reimport on next run")
                ("export-network", po::value<std::vector<std::string >>(&export_network),
                 "apply two times for exporting nodes.csv and edges.csv of network graph")
                ("export-simplified-network", po::value<std::vector<std::string >>(&export_simplified_network),
                 "apply two times for exporting nodes.csv and edges.csv of simplified network graph")
                ("export-retained-network", po::value<std::vector<std::string >>(&export_retained_network),
                 "apply two times for exporting nodes.csv and edges.csv of retained network graph "
                 "after removal of weakly unconnected components")
                ("export-transformed-network", po::value<std::vector<std::string >>(&export_transformed_network),
                 "apply two times for exporting nodes.csv and edges.csv of transformed network graph");

        options_console.add_options()
                ("readline", po::bool_switch(&read_line),
                 "read WKT track from console input")
                ("quiet", po::bool_switch(&quiet),
                 "do not output anything except if console output is activated, "
                 "deactivates verbose")
                ("verbose", po::bool_switch(&verbose),
                 "show more output during matching");

        options_performance.add_options()
                ("single-threading", po::bool_switch(&single_threading),
                 "disable multi-threading and use a single thread for all map matching algorithms, "
                 "multi-threading is enabled by default, which allows to map match multiple tracks simultaneously");

        options_preparation.add_options()
                ("filter-duplicates", po::value<bool>(&filter_duplicates)->default_value(true, "on"),
                 "filter duplicate (equal) points in tracks before matching")
                ("filter-defects", po::value<bool>(&filter_defects)->default_value(false, "off"),
                 "filter defects in tracks before matching, currently removes zig-zag curves manually, "
                 "not recommended, better use simplify-track which is on by default")
                ("simplify-track", po::value<bool>(&simplify_track)->default_value(true, "on"),
                 "simplify track with Douglas-Peucker algorithm")
                ("simplify-track-distance-tolerance",
                 po::value<double>(&simplify_track_distance_tolerance)->default_value(5.0, "5.0"),
                 "distance tolerance in meters around line for Douglas-Peucker algorithm")
                ("median-merge", po::value<bool>(&median_merge)->default_value(true, "on"),
                 "simplify track by merging point clouds to their median point coordinates, "
                 "this reduces the amount of points when vehicles emit positions from static positions")
                ("median-merge-distance-tolerance",
                 po::value<double>(&median_merge_distance_tolerance)->default_value(10.0, "10.0"),
                 "distance tolerance in meters around a point to merge all points within the given radius")
                ("adaptive-median-merge", po::value<bool>(&adaptive_median_merge)->default_value(true, "on"),
                 "adapt median-merge-distance-tolerance automatically for point clouds that are far from streets, "
                 "the distance can be raised up to half the distance of the nearest road edge for merging")
                ("candidate-adoption-siblings",
                 po::value<bool>(&candidate_adoption_siblings)->default_value(true, "on"),
                 "for each measurement, adopt candidates from preceding and succeeding candidate, "
                 "in other words, each measurement can choose from at least three road positions "
                 "and each road position is referenced by three candidates, "
                 "gives much better results but has a huge calculation speed impact")
                ("candidate-adoption-nearby", po::value<bool>(&candidate_adoption_nearby)->default_value(true, "on"),
                 "for each measurement, adopt candidates from searching nearby candidates "
                 "intersecting a search circle with distance of the farthest current candidate, "
                 "in dense areas with very near records this exchanges all candidates between all records, "
                 "gives much better results in dense record areas but has a very huge calculation speed impact")
                ("candidate-adoption-reverse",
                 po::value<bool>(&candidate_adoption_reverse)->default_value(false, "off"),
                 "for each nearby adoption, inject current candidate into candidate sets of adopting candidates "
                 "this is especially useful when adaptive optimization rounds are enabled, "
                 "new found candidates are then deployed to preceding and following locations around the high error location, "
                 "has the highest calculation speed impact")
                ("candidate-search", po::value<std::string>(&candidate_search)->default_value("circle"),
                 "candidate search algorithm to use, possible options:\n"
                 "circle\n"
                 "searches for all edges around a point intersecting a circle with at least given radius\n"
                 "nearest\n"
                 "searches the k-nearest edges around a point\n"
                 "next\n"
                 "searches only the next nearest edge (within the nearest and adjacent as well as reversed edges)")
                ("radius", po::value<double>(&radius)->default_value(200.0, "200.0"),
                 "radius in meters for candidate search when circle algorithm is selected, "
                 "is automatically doubled each time when no candidates within given radius are found, "
                 "radius is starting radius if adaptive-radius is enabled (which is by default)")
                ("radius-upper-limit", po::value<double>(&radius_upper_limit)->default_value(10000.0, "10000.0"),
                 "radius upper limit in meters for candidate search when circle algorithm is selected, "
                 "eventually stops automatically doubling of radius when no candidates are found")
                ("radius-lower-limit", po::value<double>(&radius_lower_limit)->default_value(200.0, "200.0"),
                 "radius lower limit in meters for candidate search when adaptive-radius is used, "
                 "prevents too small search radii in areas with dense track measurements")
                ("adaptive-radius", po::value<bool>(&adaptive_radius)->default_value(true, "on"),
                 "adaptive-radius is only used when circle algorithm is selected, "
                 "the radius is then reduced to the minimum distance to the next and previous candidate, "
                 "the radius-lower-limit is never undershot, when no candidate is found, the radius is doubled until radius-upper-limit, "
                 "this reduces the amount of candidates in dense road network areas, it works best with candidate-adoption enabled")
                ("k-nearest", po::value<std::size_t>(&k_nearest)->default_value(16),
                 "k-nearest edges to search for in candidate search when nearest algorithm is selected");

        options_model.add_options()
                ("model", po::value<std::string>(&model)->default_value("value-iteration"),
                 "model to use, possible options:\n"
                 "policy-iteration\n"
                 "uses policy iteration algorithm with markov decision process\n"
                 "value-iteration\n"
                 "uses value iteration algorithm with markov decision process\n"
                 "q-learning\n"
                 "uses Q-learning algorithm with markov decision process\n"
                 "viterbi\n"
                 "uses Viterbi algorithm with hidden markov model")
                ("skip-errors", po::value<bool>(&skip_errors)->default_value(true, "on"),
                 "enable skipping high error situations in markov decision processes, "
                 "leads to gaps in the exported track, but this is much better than high errors")
                ("state-size", po::value<std::size_t>(&state_size)->default_value(2),
                 "amount of adjacent track points to consider for a state in value iteration and q-learning, "
                 "default value of 2 means two directly succeeding points, value of 3 means three directly succeeding points, "
                 "with 3 succeeding points, additional metrics for direction changes and zig-zag movements are processed, "
                 "3 has a huge impact on calculation speed because the state space becomes very large, "
                 "other values are not permitted, also does not work with viterbi algorithm due to design principles")
                ("discount", po::value<double>(&discount)->default_value(0.99, "0.99"),
                 "discount factor for value-iteration and q-learning, "
                 "discount of 0.0 means only the current state is of value, "
                 "discount of 1.0 means all state sequences up to the goal are of value fur the current state, "
                 "defaults to 0.99 for making sure that loops eventually unroll");

        options_mdp.add_options()
                ("mdp-distance-factor", po::value<double>(&mdp_distance_factor)->default_value(0.9, "0.9"),
                 "distance factor for rewarding distances between measurement point and candidate position")
                ("mdp-length-factor", po::value<double>(&mdp_length_factor)->default_value(0.15, "0.15"),
                 "length factor for rewarding length differences between track segments and routes in network")
                ("mdp-azimuth-factor", po::value<double>(&mdp_azimuth_factor)->default_value(0.3, "0.3"),
                 "azimuth factor for rewarding azimuth differences between track segments and routes in network")
                ("mdp-direction-factor", po::value<double>(&mdp_direction_factor)->default_value(0.8, "0.8"),
                 "direction factor for rewarding direction changes of routes in network");

        options_hmm.add_options()
                ("hmm-distance-factor", po::value<double>(&hmm_distance_factor)->default_value(0.7, "0.7"),
                 "distance factor for probability of distances between measurement point and candidate position")
                ("hmm-length-factor", po::value<double>(&hmm_length_factor)->default_value(0.05, "0.05"),
                 "length factor for probability of length differences between track segments and routes in network")
                ("hmm-azimuth-factor", po::value<double>(&hmm_azimuth_factor)->default_value(0.2, "0.2"),
                 "azimuth factor for probability of azimuth differences between track segments and routes in network")
                ("hmm-direction-factor", po::value<double>(&hmm_direction_factor)->default_value(0.5, "0.5"),
                 "direction factor for probability of direction changes of routes in network");

        options_dp_iteration.add_options()
                ("threshold", po::value<double>(&threshold)->default_value(1e-3, "1e-3"),
                 "threshold for stopping iterations in policy- and value-iteration when the previous round improvement was below the given threshold");

        options_q_learning.add_options()
                ("learning-rate", po::value<double>(&learning_rate)->default_value(0.9, "0.9"),
                 "learning rate for q-learning, only applies if q-learning model is selected, "
                 "higher learning-rate means faster learning, "
                 "lower learning-rate means slower learning")
                ("epsilon", po::value<double>(&epsilon)->default_value(0.5, "0.5"),
                 "epsilon random action selection value for q-learning, only applies if q-learning model is selected, "
                 "epsilon of 0.0 means no random action selection, always choose next best, "
                 "epsilon of 1.0 means always random selection, never select next best")
                ("max-episodes", po::value<std::size_t>(&episodes)->default_value(1000000000),
                 "maximum episodes for definitive stop in q-learning, do not change except you know what you are doing, "
                 "training is generally stopped early before max-episodes is ever reached, "
                 "this is calculated from the found candidates, the more possible candidates, the longer the training");

        all.add_options()
                ("help",
                 "show this help");
        all.add(options_network)
                .add(options_tracks)
                .add(options_output)
                .add(options_compare)
                .add(options_export)
                .add(options_console)
                .add(options_performance)
                .add(options_preparation)
                .add(options_model)
                .add(options_mdp)
                .add(options_hmm)
                .add(options_dp_iteration)
                .add(options_q_learning);

        po::store(po::parse_command_line(argc, argv, all), vm);
        po::notify(vm);

        if (quiet) {
            verbose = false;
        }

        if (vm.count("help")) {
            if (not quiet) {
                std::cout << all << std::endl;
            }
            return 1;
        }

        if (network_file.empty() and nodes_file.empty() and arcs_file.empty() and network_load.empty()) {
            throw std::invalid_argument{"Please specify a network."};
        }

        if (not nodes_file.empty() and arcs_file.empty() or nodes_file.empty() and not arcs_file.empty()) {
            throw std::invalid_argument{"When specifying nodes and arcs, both files are needed."};
        }

        if (network_output.empty() and network_save.empty() and tracks_file.empty() and not read_line) {
            throw std::invalid_argument{"Please specify a tracks file or enable readline mode."};
        }

        if (not(candidate_search == "circle" or candidate_search == "nearest" or candidate_search == "next")) {
            throw std::invalid_argument{"Candidate search algorithm unknown."};
        }

        if (not quiet) {
            if (compare_only) {
                if (not no_compare) {
                    std::cout << "Compare-only mode, disabled map-matching." << std::endl;
                }
            } else {
                if (not network_output.empty()) {
                    std::cout << "Enabled network output." << std::endl;
                }

                if (not filter_duplicates) {
                    std::cout << "Disabled duplicates filtering." << std::endl;
                }

                if (filter_defects) {
                    std::cout << "Enabled manual defects filtering." << std::endl;
                }

                if (not candidate_adoption_siblings) {
                    std::cout << "Disabled candidate adoption of siblings." << std::endl;
                }

                if (not candidate_adoption_nearby) {
                    std::cout << "Disabled candidate adoption of nearby candidates." << std::endl;
                }

                if (not adaptive_radius) {
                    std::cout << "Disabled adaptive radius." << std::endl;
                }

                if (export_edges) {
                    std::cout << "Enabled export edges mode." << std::endl;
                }

                if (not join_merges) {
                    std::cout << "Disabled joining merged routes." << std::endl;
                }
            }

            if (single_threading) {
                std::cout << "Disabled multi-threading." << std::endl;
            }

            if (not candidates_folder.empty()) {
                std::filesystem::path candidates_path{candidates_folder};
                if (not std::filesystem::exists(candidates_path)) {
                    std::filesystem::create_directory(candidates_path);
                }

                std::cout << "Using candidates folder: " << candidates_folder << std::endl;
            }

            if (not selectors.empty()) {
                std::cout << "Selections:";
                for (const auto &selector: selectors) {
                    std::cout << "\n" << selector;
                }
                std::cout << std::endl;
            }
        }

        if (not compare_only) {
            if (radius < 2.0) {
                throw std::invalid_argument{
                        "radius must at least be 2.0 because automatic doubling does not work else."};
            }

            if (radius > radius_upper_limit) {
                throw std::invalid_argument{"radius cannot be larger than radius-limit."};
            }

            if (state_size < 2 or state_size > 3) {
                throw std::invalid_argument{"state_size must be 2 or 3."};
            }
        }

        if (compare_only and compare_output_file.empty()) {
            compare_output_file = output_file;
        }

        if (export_network.size() >= 2) {
            export_network_nodes = export_network.at(0);
            export_network_edges = export_network.at(1);
        }

        if (export_simplified_network.size() >= 2) {
            export_simplified_network_nodes = export_simplified_network.at(0);
            export_simplified_network_edges = export_simplified_network.at(1);
        }

        if (export_retained_network.size() >= 2) {
            export_retained_network_nodes = export_retained_network.at(0);
            export_retained_network_edges = export_retained_network.at(1);
        }

        if (export_transformed_network.size() >= 2) {
            export_transformed_network_nodes = export_transformed_network.at(0);
            export_transformed_network_edges = export_transformed_network.at(1);
        }
    } catch (std::exception &e) {
        std::clog << "Error:\n" << e.what() << std::endl;
        std::cout << "\n" << all << std::endl;
        return 1;
    }

    const std::string geographic{"geographic"};
    const std::string cartesian{"cartesian"};

    const auto set_srs_type = [&geographic, &cartesian](std::string &srs_type, boost::geometry::srs::proj4 &proj) {
        if (srs_type != geographic and srs_type != cartesian) {
            boost::geometry::projections::proj_wrapper<boost::geometry::srs::dynamic, double> proj_wrapper{proj};
            const auto &params = proj_wrapper.proj().params();
            bool is_geographic = not params.is_geocent and params.is_latlong;
            if (is_geographic) {
                srs_type = geographic;
            } else {
                srs_type = cartesian;
            }
        }
    };

    auto network_final_srs = network_srs;
    boost::geometry::srs::proj4 network_proj{network_srs};
    boost::geometry::srs::proj4 *network_transform_proj;
    set_srs_type(network_srs_type, network_proj);
    bool network_transform = false;
    if (not network_transform_srs.empty()) {
        network_final_srs = network_transform_srs;
        network_transform_proj = new boost::geometry::srs::proj4{network_transform_srs};
        set_srs_type(network_transform_srs_type, *network_transform_proj);
        network_transform = true;
    }

    auto tracks_final_srs = tracks_srs;
    boost::geometry::srs::proj4 tracks_proj{tracks_srs};
    boost::geometry::srs::proj4 *tracks_transform_proj;
    set_srs_type(tracks_srs_type, tracks_proj);
    bool tracks_transform = false;
    if (not tracks_transform_srs.empty()) {
        tracks_final_srs = tracks_transform_srs;
        tracks_transform_proj = new boost::geometry::srs::proj4{tracks_transform_srs};
        set_srs_type(tracks_transform_srs_type, *tracks_transform_proj);
        tracks_transform = true;
    }

    auto compare_final_srs = tracks_final_srs;
    boost::geometry::srs::proj4 *compare_proj;
    boost::geometry::srs::proj4 *compare_transform_proj;
    bool compare_transform = false;
    if (not compare_file.empty()) {
        if (compare_edges_list_mode) {
            compare_srs = network_srs;
            compare_transform_srs = network_transform_srs;
        }
        compare_final_srs = compare_srs;
        compare_proj = new boost::geometry::srs::proj4{compare_srs};
        set_srs_type(compare_srs_type, *compare_proj);
        if (not compare_transform_srs.empty()) {
            compare_final_srs = compare_transform_srs;
            compare_transform_proj = new boost::geometry::srs::proj4{compare_transform_srs};
            set_srs_type(compare_transform_srs_type, *compare_transform_proj);
            compare_transform = true;
        }
    }

    // check srs equality
    if (not(network_final_srs == tracks_final_srs and network_final_srs == compare_final_srs)) {
        std::clog << "Error:\n"
                  << "Final spatial reference systems of network, tracks and compare tracks (if defined) must be equal.\n"
                  << "Final network srs: " << network_final_srs << "\n"
                  << "Final tracks srs : " << tracks_final_srs << "\n"
                  << "Final compare srs: " << compare_final_srs << "\n"
                  << std::endl;
        std::cout << "\n" << all << std::endl;
        return 1;
    }

    // prepared checks
    bool import_compare_edge_lists = not no_compare and not compare_file.empty() and compare_edges_list_mode;

    if (not quiet) {
        std::cout << "\nStart Preparation ..." << std::endl;
    }

    map_matching_2::geometry::network::types_geographic::network_static *network_geographic;
    map_matching_2::geometry::network::types_geographic::network_modifiable *network_geographic_import;
    map_matching_2::geometry::network::types_cartesian::network_static *network_cartesian;
    map_matching_2::geometry::network::types_cartesian::network_modifiable *network_cartesian_import;

    std::unordered_map<std::string, map_matching_2::geometry::track::types_geographic::multi_track_type> tracks_geographic;
    std::unordered_map<std::string, map_matching_2::geometry::track::types_cartesian::multi_track_type> tracks_cartesian;
    std::unordered_map<std::string, map_matching_2::geometry::track::types_geographic::multi_track_type> compare_tracks_geographic;
    std::unordered_map<std::string, map_matching_2::geometry::track::types_cartesian::multi_track_type> compare_tracks_cartesian;

    auto duration = map_matching_2::util::benchmark([&]() {
        // Network
        if (network_load.empty() or import_compare_edge_lists) {
            // Regular Network Import
            if (not network_file.empty()) {
                if (verbose) {
                    std::cout << "Import network ... " << std::flush;
                }

                double duration;
                if (network_srs_type == geographic) {
                    network_geographic_import = new map_matching_2::geometry::network::types_geographic::network_modifiable;
                    map_matching_2::io::network::osm_car_importer osm_car_importer{network_file,
                                                                                   *network_geographic_import};
                    duration = map_matching_2::util::benchmark([&]() { osm_car_importer.read(); });
                } else {
                    network_cartesian_import = new map_matching_2::geometry::network::types_cartesian::network_modifiable;
                    map_matching_2::io::network::osm_car_importer osm_car_importer{network_file,
                                                                                   *network_cartesian_import};
                    duration = map_matching_2::util::benchmark([&]() { osm_car_importer.read(); });
                }

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;
                }
            } else if (not arcs_file.empty() and not nodes_file.empty()) {
                if (verbose) {
                    std::cout << "Import arcs and nodes ... " << std::flush;
                }

                double duration;
                if (network_srs_type == geographic) {
                    network_geographic_import = new map_matching_2::geometry::network::types_geographic::network_modifiable;
                    map_matching_2::io::network::arc_node_importer arc_node_importer{
                            arcs_file, nodes_file, *network_geographic_import};
                    duration = map_matching_2::util::benchmark([&]() { arc_node_importer.read(); });
                } else {
                    network_cartesian_import = new map_matching_2::geometry::network::types_cartesian::network_modifiable;
                    map_matching_2::io::network::arc_node_importer arc_node_importer{
                            arcs_file, nodes_file, *network_cartesian_import};
                    duration = map_matching_2::util::benchmark([&]() { arc_node_importer.read(); });
                }

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;
                }
            }

            if (verbose) {
                if (network_srs_type == geographic) {
                    std::cout << "Graph has " << boost::num_vertices(network_geographic_import->graph)
                              << " vertices and " << boost::num_edges(network_geographic_import->graph)
                              << " edges" << std::endl;
                } else {
                    std::cout << "Graph has " << boost::num_vertices(network_cartesian_import->graph)
                              << " vertices and " << boost::num_edges(network_cartesian_import->graph)
                              << " edges" << std::endl;
                }
            }

            // Compare Edge List Tracks
            if (import_compare_edge_lists) {
                if (verbose) {
                    std::cout << "Import edge list ground truth tracks for comparison ... " << std::flush;
                }

                double duration = 0.0;
                for (const auto &file: compare_file) {
                    if (network_srs_type == geographic) {
                        map_matching_2::io::track::edges_list_importer<
                                map_matching_2::geometry::network::types_geographic::network_modifiable,
                                map_matching_2::geometry::track::types_geographic::multi_track_type>
                                edges_list_importer{file, *network_geographic_import,
                                                    compare_tracks_geographic};
                        duration += map_matching_2::util::benchmark([&]() { edges_list_importer.read(); });
                    } else {
                        map_matching_2::io::track::edges_list_importer<
                                map_matching_2::geometry::network::types_cartesian::network_modifiable,
                                map_matching_2::geometry::track::types_cartesian::multi_track_type>
                                edges_list_importer{file, *network_cartesian_import, compare_tracks_cartesian};
                        duration += map_matching_2::util::benchmark([&]() { edges_list_importer.read(); });
                    }
                }

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;

                    if (network_srs_type == geographic) {
                        std::cout << "Number of ground truth tracks for comparison: "
                                  << compare_tracks_geographic.size() << std::endl;
                    } else {
                        std::cout << "Number of ground truth tracks for comparison: "
                                  << compare_tracks_cartesian.size() << std::endl;
                    }
                }
            }
        }

        if (network_load.empty()) {
            // Save imported network
            if (not export_network_nodes.empty() and not export_network_edges.empty()) {
                if (verbose) {
                    std::cout << "Saving graph ... " << std::flush;
                }

                double duration;
                if (network_srs_type == geographic) {
                    duration = map_matching_2::util::benchmark(
                            [&]() { network_geographic_import->save(export_network_nodes, export_network_edges); });
                } else {
                    duration = map_matching_2::util::benchmark(
                            [&]() { network_cartesian_import->save(export_network_nodes, export_network_edges); });
                }

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;
                }
            }

            // Simplify network
            if (simplify_network) {
                if (verbose) {
                    std::cout << "Simplifying ... " << std::flush;
                }

                double duration;
                if (network_srs_type == geographic) {
                    duration = map_matching_2::util::benchmark([&]() {
                        network_geographic_import->simplify(simplify_network_complete);
                    });
                } else {
                    duration = map_matching_2::util::benchmark([&]() {
                        network_cartesian_import->simplify(simplify_network_complete);
                    });
                }

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;

                    if (network_srs_type == geographic) {
                        std::cout << "Simplified graph has " << boost::num_vertices(network_geographic_import->graph)
                                  << " vertices and " << boost::num_edges(network_geographic_import->graph)
                                  << " edges" << std::endl;
                    } else {
                        std::cout << "Simplified graph has " << boost::num_vertices(network_cartesian_import->graph)
                                  << " vertices and " << boost::num_edges(network_cartesian_import->graph)
                                  << " edges" << std::endl;
                    }
                }

                // Save simplified network
                if (not export_simplified_network_nodes.empty() and not export_simplified_network_edges.empty()) {
                    if (verbose) {
                        std::cout << "Saving simplified graph ... " << std::flush;
                    }

                    if (network_srs_type == geographic) {
                        duration = map_matching_2::util::benchmark([&]() {
                            network_geographic_import->save(
                                    export_simplified_network_nodes, export_simplified_network_edges);
                        });
                    } else {
                        duration = map_matching_2::util::benchmark([&]() {
                            network_cartesian_import->save(
                                    export_simplified_network_nodes, export_simplified_network_edges);
                        });
                    }

                    if (verbose) {
                        std::cout << "done in " << duration << "s" << std::endl;
                    }
                }
            }

            // Retain network
            if (remove_unconnected) {
                if (verbose) {
                    std::cout << "Remove weakly unconnected components ... " << std::flush;
                }

                double duration;
                if (network_srs_type == geographic) {
                    duration = map_matching_2::util::benchmark([&]() {
                        network_geographic_import->remove_unconnected_components();
                    });
                } else {
                    duration = map_matching_2::util::benchmark([&]() {
                        network_cartesian_import->remove_unconnected_components();
                    });
                }

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;

                    if (network_srs_type == geographic) {
                        std::cout << "Retained graph has " << boost::num_vertices(network_geographic_import->graph)
                                  << " vertices and " << boost::num_edges(network_geographic_import->graph)
                                  << " edges" << std::endl;
                    } else {
                        std::cout << "Retained graph has " << boost::num_vertices(network_cartesian_import->graph)
                                  << " vertices and " << boost::num_edges(network_cartesian_import->graph)
                                  << " edges" << std::endl;
                    }
                }

                // Save simplified network
                if (not export_retained_network_nodes.empty() and not export_retained_network_edges.empty()) {
                    if (verbose) {
                        std::cout << "Saving retained graph ... " << std::flush;
                    }

                    if (network_srs_type == geographic) {
                        duration = map_matching_2::util::benchmark([&]() {
                            network_geographic_import->save(export_retained_network_nodes,
                                                            export_retained_network_edges);
                        });
                    } else {
                        duration = map_matching_2::util::benchmark([&]() {
                            network_cartesian_import->save(export_retained_network_nodes,
                                                           export_retained_network_edges);
                        });
                    }

                    if (verbose) {
                        std::cout << "done in " << duration << "s" << std::endl;
                    }
                }
            }

            // Transform network
            if (network_transform) {
                if (verbose) {
                    std::cout << "Transform graph ... " << std::flush;
                }

                double duration;
                if (network_srs_type == geographic and network_transform_srs_type == cartesian) {
                    network_cartesian_import = new map_matching_2::geometry::network::types_cartesian::network_modifiable;
                    duration = map_matching_2::util::benchmark([&]() {
                        network_geographic_import->reproject(*network_cartesian_import,
                                                             network_proj, *network_transform_proj);
                    });
                    delete network_geographic_import;
                    network_srs_type = cartesian;
                } else if (network_srs_type == cartesian and network_transform_srs_type == geographic) {
                    network_geographic_import = new map_matching_2::geometry::network::types_geographic::network_modifiable;
                    duration = map_matching_2::util::benchmark([&]() {
                        network_cartesian_import->reproject(*network_geographic_import,
                                                            network_proj, *network_transform_proj);
                    });
                    delete network_cartesian_import;
                    network_srs_type = geographic;
                }
                delete network_transform_proj;

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;

                    if (network_srs_type == geographic) {
                        std::cout << "Transformed graph has " << boost::num_vertices(network_geographic_import->graph)
                                  << " vertices and " << boost::num_edges(network_geographic_import->graph)
                                  << " edges" << std::endl;
                    } else {
                        std::cout << "Transformed graph has " << boost::num_vertices(network_cartesian_import->graph)
                                  << " vertices and " << boost::num_edges(network_cartesian_import->graph)
                                  << " edges" << std::endl;
                    }
                }

                // Save transformed network
                if (not export_transformed_network_nodes.empty() and not export_transformed_network_edges.empty()) {
                    if (verbose) {
                        std::cout << "Saving transformed graph ... " << std::flush;
                    }

                    if (network_srs_type == geographic) {
                        duration = map_matching_2::util::benchmark([&]() {
                            network_geographic_import->save(
                                    export_transformed_network_nodes, export_transformed_network_edges);
                        });
                    } else {
                        duration = map_matching_2::util::benchmark([&]() {
                            network_cartesian_import->save(
                                    export_transformed_network_nodes, export_transformed_network_edges);
                        });
                    }

                    if (verbose) {
                        std::cout << "done in " << duration << "s" << std::endl;
                    }
                }
            }

            // Baking network
            if (verbose) {
                std::cout << "Baking graph ... " << std::flush;
            }

            double duration;
            if (network_srs_type == geographic) {
                network_geographic = new map_matching_2::geometry::network::types_geographic::network_static;
                duration = map_matching_2::util::benchmark([&]() {
                    network_geographic_import->convert(*network_geographic);
                });
                delete network_geographic_import;
            } else {
                network_cartesian = new map_matching_2::geometry::network::types_cartesian::network_static;
                duration = map_matching_2::util::benchmark([&]() {
                    network_cartesian_import->convert(*network_cartesian);
                });
                delete network_cartesian_import;
            }

            if (verbose) {
                std::cout << "done in " << duration << "s" << std::endl;

                if (network_srs_type == geographic) {
                    std::cout << "Baked graph has " << boost::num_vertices(network_geographic->graph)
                              << " vertices and " << boost::num_edges(network_geographic->graph)
                              << " edges" << std::endl;
                } else {
                    std::cout << "Baked graph has " << boost::num_vertices(network_cartesian->graph)
                              << " vertices and " << boost::num_edges(network_cartesian->graph)
                              << " edges " << std::endl;
                }
            }

            // Export imported network
            if (not network_output.empty()) {
                if (verbose) {
                    std::cout << "Saving osm file ... " << std::flush;
                }

                if (network_srs_type == geographic) {
                    map_matching_2::io::network::osm_exporter osm_exporter{network_output, *network_geographic};
                    duration = map_matching_2::util::benchmark([&]() { osm_exporter.write(); });
                } else {
                    // cartesian export is unsupported by osm file format as it needs WGS84 coordinates
                    std::cout << "skipping because cartesian is unsupported in osm format ... " << std::flush;
//                map_matching_2::io::network::osm_exporter osm_exporter{network_output, *network_cartesian};
//                duration = map_matching_2::util::benchmark([&]() { osm_exporter.write(); });
                }

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;
                }
            }

            // Building spatial indices
            if (network_srs_type == geographic) {
                build_spatial_indices(*network_geographic, verbose);
            } else {
                build_spatial_indices(*network_cartesian, verbose);
            }

            // Network Save
            if (not network_save.empty()) {
                if (verbose) {
                    std::cout << "Saving network graph ... " << std::flush;
                }

                if (network_srs_type == geographic) {
                    map_matching_2::io::network::network_saver network_saver{network_save, *network_geographic};
                    duration = map_matching_2::util::benchmark([&]() { network_saver.write(); });
                } else {
                    map_matching_2::io::network::network_saver network_saver{network_save, *network_cartesian};
                    duration = map_matching_2::util::benchmark([&]() { network_saver.write(); });
                }

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;
                }
            }
        } else {
            if (import_compare_edge_lists) {
                // Imported Network for Compare Edge Lists Mode, remove as it is no longer used
                if (network_srs_type == geographic) {
                    delete network_geographic_import;
                } else {
                    delete network_cartesian_import;
                }
            }

            // Network Load
            if (verbose) {
                std::cout << "Loading network graph ... " << std::flush;
            }

            double duration;
            if (network_srs_type == geographic) {
                network_geographic = new map_matching_2::geometry::network::types_geographic::network_static;
                map_matching_2::io::network::network_loader network_loader{network_load, *network_geographic};
                duration = map_matching_2::util::benchmark([&]() { network_loader.read(); });
            } else {
                network_cartesian = new map_matching_2::geometry::network::types_cartesian::network_static;
                map_matching_2::io::network::network_loader network_loader{network_load, *network_cartesian};
                duration = map_matching_2::util::benchmark([&]() { network_loader.read(); });
            }

            if (verbose) {
                std::cout << "done in " << duration << "s" << std::endl;

                if (network_srs_type == geographic) {
                    std::cout << "Loaded graph has " << boost::num_vertices(network_geographic->graph)
                              << " vertices and " << boost::num_edges(network_geographic->graph)
                              << " edges" << std::endl;
                } else {
                    std::cout << "Loaded graph has " << boost::num_vertices(network_cartesian->graph)
                              << " vertices and " << boost::num_edges(network_cartesian->graph)
                              << " edges" << std::endl;
                }
            }
        }

        // Tracks
        if (read_line) {
            if (not quiet) {
                std::cout << "Please input tracks one per line, "
                             "either as LINESTRING or comma separated sequence of POINT "
                             "(enter empty line for stopping):" << std::endl;
            }

            if (tracks_srs_type == geographic) {
                map_matching_2::io::track::input_importer<map_matching_2::geometry::track::types_geographic::multi_track_type>
                        input_importer{tracks_geographic};
                input_importer.read();
            } else {
                map_matching_2::io::track::input_importer<map_matching_2::geometry::track::types_cartesian::multi_track_type>
                        input_importer{tracks_cartesian};
                input_importer.read();
            }

            if (verbose) {
                if (tracks_srs_type == geographic) {
                    std::cout << "Number of tracks: " << tracks_geographic.size() << std::endl;
                } else {
                    std::cout << "Number of tracks: " << tracks_cartesian.size() << std::endl;
                }
            }
        }

        double duration;
        if (not tracks_file.empty()) {
            if (verbose) {
                std::cout << "Import tracks ... " << std::flush;
            }

            map_matching_2::io::track::csv_track_importer_settings csv_track_importer_settings;
            csv_track_importer_settings.delimiter = delimiter;
            csv_track_importer_settings.no_header = no_header;
            csv_track_importer_settings.no_id = no_id;
            csv_track_importer_settings.wkt = wkt;
            csv_track_importer_settings.field_id = id;
            csv_track_importer_settings.id_aggregator = id_aggregator;
            csv_track_importer_settings.field_x = x;
            csv_track_importer_settings.field_y = y;
            csv_track_importer_settings.field_time = time;
            csv_track_importer_settings.time_format = time_format;
            csv_track_importer_settings.no_parse_time = no_parse_time;
            csv_track_importer_settings.field_geometry = geometry;

            duration = 0.0;
            for (const auto &file: tracks_file) {
                detect_extension(file, tracks_file_type);

                if (tracks_file_type == ".csv") {
                    if (tracks_srs_type == geographic) {
                        map_matching_2::io::track::csv_track_importer<map_matching_2::geometry::track::types_geographic::multi_track_type>
                                csv_track_importer{file, tracks_geographic};
                        csv_track_importer.settings = csv_track_importer_settings;
                        duration += map_matching_2::util::benchmark([&]() { csv_track_importer.read(); });
                    } else {
                        map_matching_2::io::track::csv_track_importer<map_matching_2::geometry::track::types_cartesian::multi_track_type>
                                csv_track_importer{file, tracks_cartesian};
                        csv_track_importer.settings = csv_track_importer_settings;
                        duration += map_matching_2::util::benchmark([&]() { csv_track_importer.read(); });
                    }
                } else if (tracks_file_type == ".gpx") {
                    if (tracks_srs_type == geographic) {
                        map_matching_2::io::track::gpx_track_importer<map_matching_2::geometry::track::types_geographic::multi_track_type>
                                gpx_track_importer{file, tracks_geographic};
                        duration += map_matching_2::util::benchmark([&]() { gpx_track_importer.read(); });
                    } else {
                        map_matching_2::io::track::gpx_track_importer<map_matching_2::geometry::track::types_cartesian::multi_track_type>
                                gpx_track_importer{file, tracks_cartesian};
                        duration += map_matching_2::util::benchmark([&]() { gpx_track_importer.read(); });
                    }
                }
            }

            if (verbose) {
                std::cout << "done in " << duration << "s" << std::endl;

                if (tracks_srs_type == geographic) {
                    std::cout << "Number of tracks: " << tracks_geographic.size() << std::endl;
                } else {
                    std::cout << "Number of tracks: " << tracks_cartesian.size() << std::endl;
                }
            }
        }

        // Transform tracks
        if (tracks_transform) {
            if (verbose) {
                std::cout << "Transform tracks ... " << std::flush;
            }

            if (tracks_srs_type == geographic) {
                using track_cartesian = map_matching_2::geometry::track::types_cartesian::multi_track_type;
                duration = map_matching_2::util::benchmark([&]() {
                    for (const auto &track: tracks_geographic) {
                        tracks_cartesian.emplace(track.first, track.second.template reproject<track_cartesian>(
                                tracks_proj, *tracks_transform_proj));
                    }
                });
                tracks_geographic.clear();
                tracks_srs_type = cartesian;
            } else {
                using track_geographic = map_matching_2::geometry::track::types_geographic::multi_track_type;
                duration = map_matching_2::util::benchmark([&]() {
                    for (const auto &track: tracks_cartesian) {
                        tracks_geographic.emplace(track.first, track.second.template reproject<track_geographic>(
                                tracks_proj, *tracks_transform_proj));
                    }
                });
                tracks_cartesian.clear();
                tracks_srs_type = geographic;
            }

            delete tracks_transform_proj;

            if (verbose) {
                std::cout << "done in " << duration << "s" << std::endl;

                if (tracks_srs_type == geographic) {
                    std::cout << "Number of transformed tracks: " << tracks_geographic.size() << std::endl;
                } else {
                    std::cout << "Number of transformed tracks: " << tracks_cartesian.size() << std::endl;
                }
            }
        }

        // Compare Tracks
        if (not no_compare) {
            if (read_line) {
                if (not quiet) {
                    std::cout << "Please input ground truth tracks for comparison one per line, "
                                 "either as LINESTRING or comma separated sequence of POINT "
                                 "(enter empty line for stopping):" << std::endl;
                }

                if (compare_srs_type == geographic) {
                    map_matching_2::io::track::input_importer<map_matching_2::geometry::track::types_geographic::multi_track_type>
                            input_importer{compare_tracks_geographic};
                    input_importer.read();
                } else {
                    map_matching_2::io::track::input_importer<map_matching_2::geometry::track::types_cartesian::multi_track_type>
                            input_importer{compare_tracks_cartesian};
                    input_importer.read();
                }

                if (verbose) {
                    if (compare_srs_type == geographic) {
                        std::cout << "Number of ground truth tracks for comparison: "
                                  << compare_tracks_geographic.size() << std::endl;
                    } else {
                        std::cout << "Number of ground truth tracks for comparison: "
                                  << compare_tracks_cartesian.size() << std::endl;
                    }
                }
            }

            if (not compare_file.empty() and not compare_edges_list_mode) {
                if (verbose) {
                    std::cout << "Import ground truth tracks for comparison ... " << std::flush;
                }

                map_matching_2::io::track::csv_track_importer_settings csv_track_importer_settings;
                csv_track_importer_settings.delimiter = compare_delimiter;
                csv_track_importer_settings.no_header = compare_no_header;
                csv_track_importer_settings.no_id = compare_no_id;
                csv_track_importer_settings.wkt = compare_wkt;
                csv_track_importer_settings.field_id = compare_id;
                csv_track_importer_settings.id_aggregator = compare_id_aggregator;
                csv_track_importer_settings.field_x = compare_x;
                csv_track_importer_settings.field_y = compare_y;
                csv_track_importer_settings.field_geometry = compare_geometry;

                duration = 0.0;
                for (const auto &file: compare_file) {
                    detect_extension(file, compare_file_type);

                    if (compare_file_type == ".csv") {
                        if (compare_srs_type == geographic) {
                            map_matching_2::io::track::csv_track_importer<map_matching_2::geometry::track::types_geographic::multi_track_type>
                                    csv_track_importer{file, compare_tracks_geographic};
                            csv_track_importer.settings = csv_track_importer_settings;
                            duration += map_matching_2::util::benchmark([&]() { csv_track_importer.read(); });
                        } else {
                            map_matching_2::io::track::csv_track_importer<map_matching_2::geometry::track::types_cartesian::multi_track_type>
                                    csv_track_importer{file, compare_tracks_cartesian};
                            csv_track_importer.settings = csv_track_importer_settings;
                            duration += map_matching_2::util::benchmark([&]() { csv_track_importer.read(); });
                        }
                    } else if (compare_file_type == ".gpx") {
                        if (compare_srs_type == geographic) {
                            map_matching_2::io::track::gpx_track_importer<map_matching_2::geometry::track::types_geographic::multi_track_type>
                                    gpx_track_importer{file, compare_tracks_geographic};
                            duration += map_matching_2::util::benchmark([&]() { gpx_track_importer.read(); });
                        } else {
                            map_matching_2::io::track::gpx_track_importer<map_matching_2::geometry::track::types_cartesian::multi_track_type>
                                    gpx_track_importer{file, compare_tracks_cartesian};
                            duration += map_matching_2::util::benchmark([&]() { gpx_track_importer.read(); });
                        }
                    }
                }

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;

                    if (compare_srs_type == geographic) {
                        std::cout << "Number of ground truth tracks for comparison: "
                                  << compare_tracks_geographic.size() << std::endl;
                    } else {
                        std::cout << "Number of ground truth tracks for comparison: "
                                  << compare_tracks_cartesian.size() << std::endl;
                    }
                }
            }

            // Transform compare tracks
            if (compare_transform) {
                if (verbose) {
                    std::cout << "Transform ground truth tracks for comparison ... " << std::flush;
                }

                if (compare_srs_type == geographic) {
                    using track_cartesian = map_matching_2::geometry::track::types_cartesian::multi_track_type;
                    duration = map_matching_2::util::benchmark([&]() {
                        for (const auto &track: compare_tracks_geographic) {
                            compare_tracks_cartesian.emplace(
                                    track.first, track.second.template reproject<track_cartesian>(
                                            *compare_proj, *compare_transform_proj));
                        }
                    });
                    compare_tracks_geographic.clear();
                    compare_srs_type = cartesian;
                } else {
                    using track_geographic = map_matching_2::geometry::track::types_geographic::multi_track_type;
                    duration = map_matching_2::util::benchmark([&]() {
                        for (const auto &track: compare_tracks_cartesian) {
                            compare_tracks_geographic.emplace(
                                    track.first, track.second.template reproject<track_geographic>(
                                            *compare_proj, *compare_transform_proj));
                        }
                    });
                    compare_tracks_cartesian.clear();
                    compare_srs_type = geographic;
                }

                delete compare_proj;
                delete compare_transform_proj;

                if (verbose) {
                    std::cout << "done in " << duration << "s" << std::endl;

                    if (compare_srs_type == geographic) {
                        std::cout << "Number of transformed ground truth tracks for comparison: "
                                  << compare_tracks_geographic.size() << std::endl;
                    } else {
                        std::cout << "Number of transformed ground truth tracks for comparison: "
                                  << compare_tracks_cartesian.size() << std::endl;
                    }
                }
            }
        }

        // Clear preparation caches
        map_matching_2::geometry::clear_distance_cache();
        map_matching_2::geometry::clear_azimuth_cache();
    });

    if (not quiet) {
        std::cout << "Finished Preparation after " << duration << " seconds.\n" << std::endl;
    }

    compare_settings compare_settings{};
    compare_settings.simplifying_tolerance = compare_simplifying_tolerance;
    compare_settings.simplifying_reverse_tolerance = compare_simplifying_reverse_tolerance;
    compare_settings.adoption_distance_tolerance = compare_adoption_distance_tolerance;
    compare_settings.split_distance_tolerance = compare_split_distance_tolerance;
    compare_settings.split_direction_tolerance = compare_split_direction_tolerance;

    if (not compare_only) {
        // Settings
        map_matching_2::learning::settings learn_settings;
        learn_settings.threshold = threshold;
        learn_settings.discount = discount;
        learn_settings.episodes = episodes;
        learn_settings.learning_rate = learning_rate;
        learn_settings.epsilon = epsilon;

        map_matching_2::matching::settings match_settings;
        match_settings.state_size = state_size;
        match_settings.skip_errors = skip_errors;
        match_settings.filter_duplicates = filter_duplicates;
        match_settings.filter_defects = filter_defects;
        match_settings.simplify = simplify_track;
        match_settings.simplify_distance_tolerance = simplify_track_distance_tolerance;
        match_settings.median_merge = median_merge;
        match_settings.median_merge_distance_tolerance = median_merge_distance_tolerance;
        match_settings.adaptive_median_merge = adaptive_median_merge;
        match_settings.candidate_adoption_siblings = candidate_adoption_siblings;
        match_settings.candidate_adoption_nearby = candidate_adoption_nearby;
        match_settings.candidate_adoption_reverse = candidate_adoption_reverse;
        match_settings.routing_max_distance_factor = routing_max_distance_factor;
        match_settings.export_edges = export_edges;
        match_settings.join_merges = join_merges;

        match_settings.mdp_distance_factor = mdp_distance_factor;
        match_settings.mdp_length_factor = mdp_length_factor;
        match_settings.mdp_azimuth_factor = mdp_azimuth_factor;
        match_settings.mdp_direction_factor = mdp_direction_factor;
        match_settings.hmm_distance_factor = hmm_distance_factor;
        match_settings.hmm_length_factor = hmm_length_factor;
        match_settings.hmm_azimuth_factor = hmm_azimuth_factor;
        match_settings.hmm_direction_factor = hmm_direction_factor;

        if (candidate_search == "circle") {
            match_settings.k_nearest_candidate_search = false;
            match_settings.adaptive_radius = adaptive_radius;
            match_settings.buffer_radius = radius;
            match_settings.buffer_upper_radius = radius_upper_limit;
            match_settings.buffer_lower_radius = radius_lower_limit;
        } else if (candidate_search == "nearest") {
            match_settings.k_nearest_candidate_search = true;
            match_settings.k_nearest = k_nearest;
        } else if (candidate_search == "next") {
            match_settings.k_nearest_candidate_search = true;
            match_settings.k_nearest = 1;
            match_settings.k_nearest_adjacent = true;
            match_settings.k_nearest_reverse = true;
        }

        // Match
        if (network_srs_type == geographic and tracks_srs_type == geographic and not tracks_geographic.empty() and
            (compare_tracks_geographic.empty() or compare_srs_type == geographic)) {
            map_matching_2::matching::types_geographic::matcher_static matcher{*network_geographic, single_threading};
            match(matcher, std::move(tracks_geographic), compare_tracks_geographic,
                  std::move(output_file), std::move(compare_output_file), compare_settings,
                  std::move(candidates_folder), learn_settings, match_settings, model, selectors,
                  single_threading, console, quiet, verbose);
        } else if (network_srs_type == cartesian and tracks_srs_type == cartesian and not tracks_cartesian.empty() and
                   (compare_tracks_cartesian.empty() or compare_srs_type == cartesian)) {
            map_matching_2::matching::types_cartesian::matcher_static matcher{*network_cartesian, single_threading};
            match(matcher, std::move(tracks_cartesian), compare_tracks_cartesian,
                  std::move(output_file), std::move(compare_output_file), compare_settings,
                  std::move(candidates_folder), learn_settings, match_settings, model, selectors,
                  single_threading, console, quiet, verbose);
        }
    } else {
        // Compare
        if (tracks_srs_type == geographic and
            (compare_tracks_geographic.empty() or compare_srs_type == geographic)) {
            compare(std::move(tracks_geographic), compare_tracks_geographic, std::move(compare_output_file),
                    compare_settings, selectors, single_threading, console, quiet, verbose);
        } else if (tracks_srs_type == cartesian and
                   (compare_tracks_cartesian.empty() or compare_srs_type == cartesian)) {
            compare(std::move(tracks_cartesian), compare_tracks_cartesian, std::move(compare_output_file),
                    compare_settings, selectors, single_threading, console, quiet, verbose);
        }
    }

    if (network_srs_type == geographic) {
        delete network_geographic;
    } else {
        delete network_cartesian;
    }

    return 0;
}
