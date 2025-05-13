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

#ifndef MAP_MATCHING_2_APP_OPTIONS_HPP
#define MAP_MATCHING_2_APP_OPTIONS_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include "geometry/srs.hpp"

namespace map_matching_2::app {

    namespace po = boost::program_options;

    enum MODE {
        NONE, PREPARE, MATCH, SERVER, COMPARE
    };

    const std::string FORMAT_OSM{"osm"};
    const std::string FORMAT_ARC_NODE{"arc-node"};
    const std::string FORMAT_SEATTLE{"seattle"};
    const std::string FORMAT_MELBOURNE{"melbourne"};
    const std::string FORMAT_GISCUP{"giscup"};

    const std::string OSM_TAGS_PROFILE_CAR{"car"};
    const std::string OSM_TAGS_PROFILE_MANUAL{"manual"};

    const std::string SIMPLIFY_ALL{"all"};
    const std::string SIMPLIFY_NONE{"none"};
    const std::string SIMPLIFY_OSM_AWARE{"osm-aware"};

    const std::string FILE_TYPE_CSV{"csv"};
    const std::string FILE_TYPE_GPX{"gpx"};
    const std::string FILE_TYPE_LIST{"list"};
    const std::string FILE_TYPE_SEATTLE{"seattle"};

    const std::string FILTER_METHOD_WITHIN{"within"};
    const std::string FILTER_METHOD_INTERSECTS{"intersects"};

    const std::string CANDIDATE_SEARCH_CIRCLE{"circle"};
    const std::string CANDIDATE_SEARCH_NEAREST{"nearest"};
    const std::string CANDIDATE_SEARCH_COMBINED{"combined"};
    const std::string CANDIDATE_SEARCH_NEXT{"next"};

    const std::string MODEL_POLICY_ITERATION{"policy-iteration"};
    const std::string MODEL_VALUE_ITERATION{"value-iteration"};
    const std::string MODEL_Q_LEARNING{"q-learning"};
    const std::string MODEL_VITERBI{"viterbi"};

    struct base_data {
        virtual void correct(const po::variables_map &vm);

        virtual ~base_data() = default;
    };

    struct all_data : base_data {
        bool verbose, quiet;
        std::string config;

        void correct(const po::variables_map &vm) override;
    };

    struct global_data : base_data {
        bool prepare, match, server, compare;
        MODE mode;

        void correct(const po::variables_map &vm) override;
    };

    struct input_data : base_data {
        std::string input;

        void correct(const po::variables_map &vm) override;
    };

    struct output_data : base_data {
        std::string output, filename;
    };

    struct match_output_data : output_data {
        bool export_timestamps, export_edges, export_edge_ids, join_merges;
        std::string columns, export_time_zone;

        void correct(const po::variables_map &vm) override;
    };

    struct compare_output_data : output_data {
        std::string columns;
    };

    struct console_data : base_data {
        bool read_line, console;
    };

    struct prepare_network_data : base_data {
        std::string format, output;
        std::vector<std::string> files;

        void correct(const po::variables_map &vm) override;
    };

    struct prepare_osm_import_data : base_data {
        std::string profile, query, filter, oneway, oneway_reverse;

        void correct(const po::variables_map &vm) override;
    };

    struct srs_data : base_data {
        std::int32_t _srs;
        std::string _srs_type;
        geometry::srs srs;

        void correct(const po::variables_map &vm) override;
    };

    struct srs_transform_data : srs_data {
        std::int32_t _transform_srs = -1;
        std::string _transform_srs_type;
        geometry::srs_transform srs_transform;

        void correct(const po::variables_map &vm) override;
    };

    struct prepare_srs_data : srs_transform_data {};

    struct files_data : base_data {
        std::string graph_file, tags_file, index_file;

        void correct(const po::variables_map &vm) override;
    };

    struct prepare_files_data : files_data {
        std::size_t initial_size;
        std::string osm_vertices_file, osm_tags_file, graph_import_file, tags_import_file, index_build_file;
    };

    struct export_data : base_data {
        std::string candidates, candidates_columns;
    };

    struct prepare_export_data : base_data {
        std::string export_nodes_csv_file, export_edges_csv_file, export_network_osm_file;
    };

    struct prepare_process_data : base_data {
        bool remove_unconnected, remove_duplicate_nodes, remove_duplicate_edges;
        std::string simplify;

        void correct(const po::variables_map &vm) override;
    };

    struct memory_data : base_data {
        bool memory_mapping, mmap_graph, mmap_indices, mmap_tags;

        void correct(const po::variables_map &vm) override;
    };

    struct prepare_memory_data : memory_data {
        bool mmap_preparation, mmap_osm_vertices, mmap_osm_tags, mmap_graph_import, mmap_tags_import, mmap_index_build;

        void correct(const po::variables_map &vm) override;
    };

    struct performance_data : base_data {
        std::uint32_t threads;
    };

    struct tracks_data : base_data {
        std::string file_extension;
        std::vector<std::string> files;

        void correct(const po::variables_map &vm) override;
    };

    struct csv_data : base_data {
        bool grouped, no_header, no_id, no_id_whole_file, wkt;
        std::size_t skip_lines;
        std::string delimiter, id_aggregator, x, y, geometry;
        std::vector<std::string> ids;
        std::vector<std::string> selectors;
    };

    struct csv_tracks_data : csv_data {
        bool no_parse_time;
        std::string time_format, time_aggregator;
        std::vector<std::string> time;
    };

    struct matching_data : base_data {
        bool filter_duplicates, simplify_track, median_merge, adaptive_median_merge, within_edge_turns, a_star,
                candidate_adoption_siblings, candidate_adoption_nearby, candidate_adoption_reverse,
                adaptive_radius;
        double max_time, split_time, routing_max_distance_factor, simplify_track_distance_tolerance,
                median_merge_distance_tolerance, radius, radius_upper_limit, radius_lower_limit;
        std::size_t k_nearest;
        std::string filter_polygon, filter_method, candidate_search;

        void correct(const po::variables_map &vm) override;
    };

    struct model_data : base_data {
        bool skip_errors;
        std::size_t state_size;
        double discount;
        std::string model;

        void correct(const po::variables_map &vm) override;
    };

    struct weight_data : base_data {
        double distance_factor, length_factor, azimuth_factor, direction_factor, turn_factor;
    };

    struct weight_mdp_data : weight_data {};

    struct weight_hmm_data : weight_data {};

    struct dynamic_programming_data : base_data {
        double threshold;
    };

    struct learning_data : base_data {
        double learning_rate, epsilon, early_stop_factor;
        std::size_t episodes;
    };

    struct comparison_data : tracks_data {
        bool ignore_non_existent;
        double simplifying_tolerance, simplifying_reverse_tolerance,
                adoption_distance_tolerance,
                split_distance_tolerance, split_direction_tolerance;
    };

    struct mode_data : base_data {
        all_data all;
        global_data global;

        void correct(const po::variables_map &vm) override;
    };

    struct prepare_data : mode_data {
        prepare_network_data network;
        prepare_osm_import_data osm_import;
        prepare_srs_data srs;
        prepare_files_data files;
        prepare_export_data expo;
        prepare_process_data process;
        prepare_memory_data memory;

        void correct(const po::variables_map &vm) override;
    };

    struct work_data : mode_data {
        console_data console;
        performance_data performance;

        void correct(const po::variables_map &vm) override;
    };

    struct match_data : work_data {
        input_data input;
        srs_data srs;
        files_data files;
        memory_data memory;
        match_output_data match_output;
        tracks_data tracks;
        srs_transform_data srs_tracks;
        csv_tracks_data csv;
        export_data expo;
        matching_data matching;
        model_data model;
        weight_mdp_data weights_mdp;
        weight_hmm_data weights_hmm;
        dynamic_programming_data dynamic_programming;
        learning_data learning;

        void correct(const po::variables_map &vm) override;
    };

    struct compare_data : work_data {
        compare_output_data compare_output;
        tracks_data match;
        srs_transform_data srs_match;
        csv_tracks_data csv_match;
        comparison_data comparison;
        srs_transform_data srs_compare;
        csv_tracks_data csv_compare;

        void correct(const po::variables_map &vm) override;
    };

    po::options_description options_all(all_data &data);

    po::options_description options_global(global_data &data, MODE mode = MODE::NONE);

    po::options_description options_input(input_data &data);

    po::options_description options_output(output_data &data, const std::string &default_filename);

    po::options_description options_match_output(match_output_data &data);

    po::options_description options_compare_output(compare_output_data &data);

    po::options_description options_console(console_data &data);

    po::options_description options_prepare_network(prepare_network_data &data);

    po::options_description options_prepare_osm_import(prepare_osm_import_data &data);

    po::options_description options_srs(srs_data &data, const std::string &prefix);

    po::options_description options_srs_transform(srs_transform_data &data, const std::string &prefix);

    po::options_description options_prepare_srs(prepare_srs_data &data);

    po::options_description options_files(files_data &data);

    po::options_description options_prepare_files(prepare_files_data &data);

    po::options_description options_export(export_data &data);

    po::options_description options_prepare_export(prepare_export_data &data);

    po::options_description options_prepare_process(prepare_process_data &data);

    po::options_description options_memory(memory_data &data);

    po::options_description options_prepare_memory(prepare_memory_data &data);

    po::options_description options_performance(performance_data &data);

    po::options_description options_tracks(tracks_data &data, const std::string &prefix = "");

    po::options_description options_csv(csv_data &data, const std::string &prefix = "");

    po::options_description options_csv_tracks(csv_tracks_data &data, const std::string &prefix = "");

    po::options_description options_matching(matching_data &data);

    po::options_description options_model(model_data &data);

    po::options_description options_weight_mdp(weight_mdp_data &data);

    po::options_description options_weight_hmm(weight_hmm_data &data);

    po::options_description options_dynamic_programming(dynamic_programming_data &data);

    po::options_description options_learning(learning_data &data);

    po::options_description options_comparison(comparison_data &data);

    po::options_description options_mode(mode_data &data, MODE mode = MODE::NONE);

    po::options_description options_prepare(prepare_data &data);

    po::options_description options_work(work_data &data, MODE mode);

    po::options_description options_match(match_data &data);

    po::options_description options_compare(compare_data &data);

    po::variables_map parse(int argc, char *argv[], po::options_description &all);

    bool show_help(const po::options_description &all, const po::variables_map &vm, MODE mode = MODE::NONE);

    void show_error(const std::exception &e, const po::options_description &all);

    bool process(int argc, char *argv[], po::variables_map &vm, po::options_description &all, MODE mode = MODE::NONE);

}

#endif //MAP_MATCHING_2_APP_OPTIONS_HPP
