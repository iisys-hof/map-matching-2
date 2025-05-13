// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_OSM_RELATIONS_MANAGER_HPP
#define MAP_MATCHING_2_OSM_RELATIONS_MANAGER_HPP

#include <osmium/relations/relations_manager.hpp>

namespace map_matching_2::io::network {

    template<typename Network>
    class osm_relations_manager
            : public osmium::relations::RelationsManager<osm_relations_manager<Network>, true, true, false> {

    public:
        explicit osm_relations_manager(Network &network)
            : _network{network} {}

        bool new_relation(const osmium::Relation &relation) noexcept {
            return relation.tags().has_tag("type", "restriction") and
                    relation.tags().has_key("restriction");
        }

        bool new_member(const osmium::Relation &relation,
                const osmium::RelationMember &member, std::size_t n) noexcept {
            return true;
        }

        void complete_relation(const osmium::Relation &relation) {
            const auto &restriction = relation.tags().get_value_by_key("restriction");

            // std::cout << "Relation: " << relation.id() << ": ";
            // for (const auto &tag : relation.tags()) {
            //     std::cout << tag << ", ";
            // }
            // std::cout << std::endl;

            for (const auto &member : relation.members()) {
                if (member.ref() != 0) {
                    const osmium::OSMObject *obj = this->get_member_object(member);

                    if (obj->type() == osmium::item_type::node) {
                        const osmium::Node *node = this->get_member_node(member.ref());
                    } else if (obj->type() == osmium::item_type::way) {
                        const osmium::Way *way = this->get_member_way(member.ref());
                    }

                    // if (obj->type() == osmium::item_type::way and std::strcmp(member.role(), "via") == 0) {
                    //     std::cout << "Relation: " << relation.id() << ": ";
                    //     for (const auto &tag : relation.tags()) {
                    //         std::cout << tag << ", ";
                    //     }
                    //     std::cout << std::endl;
                    //
                    //     std::cout << member.role() << " " << obj->type() << " " << obj->id() << ": ";
                    //     for (const auto &tag : obj->tags()) {
                    //         std::cout << tag << ", ";
                    //     }
                    //     std::cout << std::endl;
                    // }

                    // std::cout << member.role() << " " << obj->type() << " " << obj->id() << ": ";
                    // for (const auto &tag : obj->tags()) {
                    //     std::cout << tag << ", ";
                    // }
                    // std::cout << std::endl;

                    // If you know which type you have you can also use any of these:
                    // const osmium::Node *node = this->get_member_node(member.ref());
                    // const osmium::Way *way = this->get_member_way(member.ref());
                    // const osmium::Relation *relation = this->get_member_relation(member.ref());
                }
            }
            // std::cout << std::endl;
        }

    private:
        Network &_network;

    };

}

#endif //MAP_MATCHING_2_OSM_RELATIONS_MANAGER_HPP
