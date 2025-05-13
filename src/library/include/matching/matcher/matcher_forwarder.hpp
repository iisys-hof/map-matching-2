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

#ifndef MAP_MATCHING_2_MATCHING_MATCHER_MATCHER_FORWARDER_HPP
#define MAP_MATCHING_2_MATCHING_MATCHER_MATCHER_FORWARDER_HPP

#include "../settings.hpp"

namespace map_matching_2::matching {

    template<typename Matcher>
    class matcher_forwarder {

    public:
        using matcher_type = Matcher;
        using multi_track_variant_type = typename matcher_type::multi_track_variant_type;

        matcher_forwarder(matcher_type &matcher, matching::settings settings)
            : _matcher{matcher}, _settings{std::move(settings)} {}

        void pass(multi_track_variant_type multi_track) {
            _matcher.match(std::move(multi_track), _settings);
        }

    private:
        matcher_type &_matcher;
        const matching::settings _settings;

    };

}

#endif //MAP_MATCHING_2_MATCHING_MATCHER_MATCHER_FORWARDER_HPP
