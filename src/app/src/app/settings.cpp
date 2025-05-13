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

#include "app/global.hpp"

#include "app/settings.hpp"

namespace map_matching_2::app {

    matching::settings _match_settings(const match_data &data) {
        matching::settings settings{};

        settings.verbose = global.verbose;
        settings.quiet = global.quiet;

        settings.tracks_srs_transform = data.srs_tracks.srs_transform;

        settings.export_timestamps = data.match_output.export_timestamps;
        settings.export_edges = data.match_output.export_edges;
        settings.export_edge_ids = data.match_output.export_edge_ids;
        settings.join_merges = data.match_output.join_merges;
        settings.output = data.match_output.output;
        settings.filename = data.match_output.filename;
        settings.export_time_zone = data.match_output.export_time_zone;
        settings.export_candidates = data.expo.candidates;
        settings.console = data.console.console;

        settings.max_time = data.matching.max_time;
        settings.filter_duplicates = data.matching.filter_duplicates;
        settings.simplify_track = data.matching.simplify_track;
        settings.median_merge = data.matching.median_merge;
        settings.adaptive_median_merge = data.matching.adaptive_median_merge;
        settings.within_edge_turns = data.matching.within_edge_turns;
        settings.a_star = data.matching.a_star;
        settings.candidate_adoption_siblings = data.matching.candidate_adoption_siblings;
        settings.candidate_adoption_nearby = data.matching.candidate_adoption_nearby;
        settings.candidate_adoption_reverse = data.matching.candidate_adoption_reverse;
        settings.adaptive_radius = data.matching.adaptive_radius;
        settings.routing_max_distance_factor = data.matching.routing_max_distance_factor;
        settings.simplify_track_distance_tolerance = data.matching.simplify_track_distance_tolerance;
        settings.median_merge_distance_tolerance = data.matching.median_merge_distance_tolerance;
        settings.radius = data.matching.radius;
        settings.radius_upper_limit = data.matching.radius_upper_limit;
        settings.radius_lower_limit = data.matching.radius_lower_limit;
        settings.k_nearest = data.matching.k_nearest;
        settings.skip_errors = data.model.skip_errors;
        settings.state_size = data.model.state_size;
        settings.discount = data.model.discount;

        settings.mdp_distance_factor = data.weights_mdp.distance_factor;
        settings.mdp_length_factor = data.weights_mdp.length_factor;
        settings.mdp_azimuth_factor = data.weights_mdp.azimuth_factor;
        settings.mdp_direction_factor = data.weights_mdp.direction_factor;
        settings.mdp_turn_factor = data.weights_mdp.turn_factor;

        settings.hmm_distance_factor = data.weights_hmm.distance_factor;
        settings.hmm_length_factor = data.weights_hmm.length_factor;
        settings.hmm_azimuth_factor = data.weights_hmm.azimuth_factor;
        settings.hmm_direction_factor = data.weights_hmm.direction_factor;
        settings.hmm_turn_factor = data.weights_hmm.turn_factor;

        settings.threshold = data.dynamic_programming.threshold;

        settings.learning_rate = data.learning.learning_rate;
        settings.epsilon = data.learning.epsilon;
        settings.early_stop_factor = data.learning.early_stop_factor;
        settings.episodes = data.learning.episodes;

        if (data.matching.candidate_search == CANDIDATE_SEARCH_CIRCLE) {
            settings.candidate_search = matching::settings::CANDIDATE_SEARCH::CIRCLE;
        } else if (data.matching.candidate_search == CANDIDATE_SEARCH_NEAREST) {
            settings.candidate_search = matching::settings::CANDIDATE_SEARCH::NEAREST;
        } else if (data.matching.candidate_search == CANDIDATE_SEARCH_COMBINED) {
            settings.candidate_search = matching::settings::CANDIDATE_SEARCH::COMBINED;
        } else if (data.matching.candidate_search == CANDIDATE_SEARCH_NEXT) {
            settings.candidate_search = matching::settings::CANDIDATE_SEARCH::NEXT;
        }

        if (data.model.model == MODEL_POLICY_ITERATION) {
            settings.model = matching::settings::MODEL::POLICY_ITERATION;
        } else if (data.model.model == MODEL_VALUE_ITERATION) {
            settings.model = matching::settings::MODEL::VALUE_ITERATION;
        } else if (data.model.model == MODEL_Q_LEARNING) {
            settings.model = matching::settings::MODEL::Q_LEARNING;
        } else if (data.model.model == MODEL_VITERBI) {
            settings.model = matching::settings::MODEL::VITERBI;
        }

        return settings;
    }

    compare::settings _compare_settings(const compare_data &data) {
        compare::settings settings{};

        settings.verbose = global.verbose;
        settings.quiet = global.quiet;

        settings.matches_srs_transform = data.srs_match.srs_transform;
        settings.ground_truth_srs_transform = data.srs_compare.srs_transform;

        settings.output = data.compare_output.output;
        settings.filename = data.compare_output.filename;

        settings.console = data.console.console;

        settings.ignore_non_existent = data.comparison.ignore_non_existent;
        settings.simplifying_tolerance = data.comparison.simplifying_tolerance;
        settings.simplifying_reverse_tolerance = data.comparison.simplifying_reverse_tolerance;
        settings.adoption_distance_tolerance = data.comparison.adoption_distance_tolerance;
        settings.split_distance_tolerance = data.comparison.split_distance_tolerance;
        settings.split_direction_tolerance = data.comparison.split_direction_tolerance;

        return settings;
    }

    io::track::importer_settings _importer_settings(const match_data &data) {
        io::track::importer_settings settings{};

        if (data.tracks.file_extension == FILE_TYPE_CSV) {
            settings.file_extension = io::track::importer_settings::FILE_TYPE::CSV;
        } else if (data.tracks.file_extension == FILE_TYPE_GPX) {
            settings.file_extension = io::track::importer_settings::FILE_TYPE::GPX;
        }

        settings.files = data.tracks.files;

        settings.tracks_srs_transform = data.srs_tracks.srs_transform;

        settings.csv_settings = _csv_tracks_settings(data.csv);

        return settings;
    }

    io::track::importer_settings _matches_importer_settings(const compare_data &data) {
        io::track::importer_settings settings{};

        if (data.match.file_extension == FILE_TYPE_CSV) {
            settings.file_extension = io::track::importer_settings::FILE_TYPE::CSV;
        } else if (data.match.file_extension == FILE_TYPE_GPX) {
            settings.file_extension = io::track::importer_settings::FILE_TYPE::GPX;
        }

        settings.files = data.match.files;

        settings.tracks_srs_transform = data.srs_match.srs_transform;

        settings.csv_settings = _csv_tracks_settings(data.csv_match);

        return settings;
    }

    io::track::importer_settings _compares_importer_settings(const compare_data &data) {
        io::track::importer_settings settings{};

        if (data.comparison.file_extension == FILE_TYPE_CSV) {
            settings.file_extension = io::track::importer_settings::FILE_TYPE::CSV;
        } else if (data.comparison.file_extension == FILE_TYPE_GPX) {
            settings.file_extension = io::track::importer_settings::FILE_TYPE::GPX;
        }

        settings.files = data.comparison.files;

        settings.tracks_srs_transform = data.srs_compare.srs_transform;

        settings.csv_settings = _csv_tracks_settings(data.csv_compare);

        return settings;
    }

    io::csv_settings _csv_settings(const csv_data &csv) {
        io::csv_settings csv_settings{};

        csv_settings.delimiter = csv.delimiter;
        csv_settings.grouped = csv.grouped;
        csv_settings.no_header = csv.no_header;
        csv_settings.no_id = csv.no_id;
        csv_settings.no_id_whole_file = csv.no_id_whole_file;
        csv_settings.wkt = csv.wkt;
        csv_settings.skip_lines = csv.skip_lines;
        csv_settings.field_id = csv.ids;
        csv_settings.field_x = csv.x;
        csv_settings.field_y = csv.y;
        csv_settings.field_geometry = csv.geometry;
        csv_settings.id_aggregator = csv.id_aggregator;
        csv_settings.selectors = csv.selectors;

        csv_settings.no_parse_time = true;

        return csv_settings;
    }

    io::csv_settings _csv_tracks_settings(const csv_tracks_data &csv) {
        io::csv_settings csv_settings = _csv_settings(csv);

        csv_settings.no_parse_time = csv.no_parse_time;
        csv_settings.field_time = csv.time;
        csv_settings.time_aggregator = csv.time_aggregator;
        csv_settings.time_format = csv.time_format;

        return csv_settings;
    }

}
