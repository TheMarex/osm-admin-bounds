/*

Copyright (c) 2014, Patrick Niklaus
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <unordered_map>
#include <fstream>
#include <iomanip>

#include <osmium/io/pbf_input.hpp>
#include <osmium/tags/filter.hpp>
#include <osmium/index/multimap/vector.hpp>

#include <proj_api.h>

struct Coordinate
{
    double lat;
    double lon;
};

struct Boundary
{
    std::vector<long> node_ids;
};

class BoundaryParser
{
public:
    BoundaryParser()
    {
        boundary_filter.add(true, "boundary", "administrative");
        level_filter.add(true, "admin_level", "6");
        level_filter.add(true, "admin_level", "7");
        level_filter.add(true, "admin_level", "8");
    }

    void operator()(const osmium::memory::Buffer& buffer)
    {
        for (const auto& item : buffer)
        {
            if (item.type() == osmium::item_type::relation)
            {
                parseBoundaryRelation(static_cast<const osmium::Relation&>(item));
            }
        }
    }

    void finish(std::vector<long>& out_boundary_ways)
    {
        boundary_ways.swap(out_boundary_ways);
    }

private:
    void parseBoundaryRelation(const osmium::Relation& relation)
    {
        const auto& tags = relation.tags();
        if (std::find_if(tags.begin(), tags.end(), boundary_filter) == tags.end())
        {
            return;
        }
        if (std::find_if(tags.begin(), tags.end(), level_filter) == tags.end())
        {
            return;
        }

        for (const auto& member : relation.members())
        {
            if (member.type() == osmium::item_type::way)
            {
                boundary_ways.push_back(member.ref());
            }
        }
    }

    std::vector<long> boundary_ways;
    osmium::tags::KeyValueFilter boundary_filter;
    osmium::tags::KeyValueFilter level_filter;
};

class WayParser
{
public:
    WayParser(std::vector<long>&& in_way_id_list)
        : way_id_list(in_way_id_list)
    {
        std::sort(way_id_list.begin(), way_id_list.end());
    }

    void operator()(const osmium::memory::Buffer& buffer)
    {
        for (const auto& item : buffer)
        {
            if (item.type() == osmium::item_type::way)
            {
                parseWay(static_cast<const osmium::Way&>(item));
            }
        }
    }

    void finish(std::vector<Boundary>& out_boundaries, std::unordered_map<long, unsigned>& out_node_map)
    {
        out_boundaries.swap(boundaries);
        out_node_map.swap(node_map);
    }

private:
    void parseWay(const osmium::Way& way)
    {
        if(!std::binary_search(way_id_list.begin(), way_id_list.end(), way.id()))
        {
            return;
        }


        std::vector<long> node_ids;

        for (const auto n : way.nodes())
        {
            node_ids.push_back(n.ref());
            node_map[n.ref()] = static_cast<unsigned>(-1);
        }
        boundaries.push_back(Boundary {std::move(node_ids)});
    }

    std::vector<Boundary> boundaries;
    std::unordered_map<long, unsigned> node_map;

    std::vector<long> way_id_list;
};

class NodeParser
{
public:
    NodeParser(std::unordered_map<long, unsigned>& in_node_map)
        : node_map(in_node_map)
    {
        coordinates.reserve(node_map.size());
    }

    void operator()(const osmium::memory::Buffer& buffer)
    {
        for (const auto& item : buffer)
        {
            if (item.type() == osmium::item_type::node)
            {
                parseNode(static_cast<const osmium::Node&>(item));
            }
        }
    }

    void finish(std::vector<Coordinate>& out_coordinates)
    {
        out_coordinates.swap(coordinates);
    }

private:

    void parseNode(const osmium::Node& node)
    {
        auto iter = node_map.find(node.id());
        if (iter == node_map.end())
        {
            return;
        }
        node_map[iter->first] = coordinates.size();
        coordinates.push_back({node.location().lat(), node.location().lon()});
    }

    std::vector<Coordinate> coordinates;
    std::unordered_map<long, unsigned>& node_map;
};

void parseInput(const char* path, std::vector<Boundary>& out_boundaries, std::unordered_map<long, unsigned>& out_node_map, std::vector<Coordinate>& out_coodinates)
{
    osmium::io::File input(path);

    std::vector<long> boundary_ways;
    std::cout << "Parsing relations... " << std::flush;
    {
        osmium::io::Reader relation_reader(input, osmium::osm_entity_bits::relation);
        BoundaryParser relation_parser;
        while (osmium::memory::Buffer buffer = relation_reader.read()) {
            relation_parser(buffer);
        }
        relation_parser.finish(boundary_ways);
    }
    std::cout << " ok." << std::endl;

    std::vector<Boundary> boundaries;
    std::unordered_map<long, unsigned> node_map;
    std::cout << "Parsing ways... " << std::flush;
    {
        osmium::io::Reader way_reader(input, osmium::osm_entity_bits::way);
        WayParser way_parser(std::move(boundary_ways));
        while (osmium::memory::Buffer buffer = way_reader.read()) {
            way_parser(buffer);
        }
        way_parser.finish(boundaries, node_map);
    }
    std::cout << " ok." << std::endl;

    std::vector<Coordinate> coordinates;
    std::cout << "Parsing nodes... " << std::flush;
    {
        osmium::io::Reader node_reader(input, osmium::osm_entity_bits::node);
        NodeParser node_parser(node_map);
        while (osmium::memory::Buffer buffer = node_reader.read()) {
            node_parser(buffer);
        }
        node_parser.finish(coordinates);
    }
    std::cout << " ok." << std::endl;

    out_boundaries.swap(boundaries);
    out_node_map.swap(node_map);
    out_coodinates.swap(coordinates);
}


void writeOutput(const char* path, const std::vector<Boundary>& boundaries, const std::unordered_map<long, unsigned>& node_map, const std::vector<Coordinate>& coordinates)
{
    std::ofstream out(path);

    projPJ epsg54004 = pj_init_plus("+proj=merc +lat_ts=0 +lon_0=0 +k=1.000000 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m");
    projPJ wgs84 = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs");

    auto id = 0u;
    for (const auto& b : boundaries)
    {
        out << id++ << ":";
        out << "<gml:LineString srsName=\"EPSG:54004\" xmlns:gml=\"http://www.opengis.net/gml\"><gml:coordinates decimal=\".\" cs=\",\" ts=\" \">";
        for (auto id : b.node_ids)
        {
            auto iter = node_map.find(id);
            BOOST_ASSERT(iter != node_map.end() && iter->second < coordinates.size());

            auto x = coordinates[iter->second].lon * DEG_TO_RAD;
            auto y = coordinates[iter->second].lat * DEG_TO_RAD;
            pj_transform(wgs84, epsg54004, 1, 1, &x, &y, NULL);
            out << std::setprecision(6) << std::fixed << x << "," << y  << " ";
        }
        out << "</gml:coordinates></gml:LineString>" << std::endl;
    }
}

int main (int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Error: Not enough arguments.\nUsage: ./osm-admin-bounds INPUT.pbf" << std::endl;
        return 1;
    }

    char* input_file_path = argv[1];
    boost::filesystem::path input_path(input_file_path);
    if (!boost::filesystem::exists(input_path))
    {
        std::cout << "Error: Input file " << input_file_path << " does not exists." << std::endl;
    }

    boost::filesystem::path output_path = input_path.filename().replace_extension(".txt");

    std::vector<Boundary> boundaries;
    std::vector<Coordinate> coordinates;
    std::unordered_map<long, unsigned> node_map;

    parseInput(input_file_path, boundaries, node_map, coordinates);
    std::cout << "Writing to " << output_path.c_str() << " ..." << std::flush;
    writeOutput(output_path.c_str(), boundaries, node_map, coordinates);
    std::cout << " ok." << std::endl;

    return 0;
}
