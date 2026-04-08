import osmium
import os

import folium


class OsmHandler(osmium.SimpleHandler):
    def __init__(self, callback):
        super().__init__()

        self.categories = [
            'fast_food', 
            'restaurant', 
            'department_store', 
            'marketplace',
            'post_office',
            'variety_store',
            'ice_cream',
            'pharmacy',
            'bakery',
            'store',
            'clothes'
        ]

        self.keys = ['amenity', 'shop']
        self.tags_key = ['amenity', 'shop', 'comment', 'name', 'height']

        self.callback = callback


    def __process_tags__(self, tags):
        tags_dict = {}

        for t in tags:
            if t.k in self.tags_key:
                tags_dict[t.k] = t.v
        
        return tags_dict


    def __process_node__(self, node, hasChild):
        for tag in node.tags:
            tag_key = tag.k.lower()
            tag_value = tag.v.lower()

            if not tag_key in self.keys or not tag_value in self.categories:
                continue

            node_coords = node.nodes[0] if hasChild else node
            
            tags = self.__process_tags__(node.tags)
            coords = (node_coords.lat, node_coords.lon)

            self.callback(coords, tags)


    def way(self, n):
        self.__process_node__(n, True)

    def node(self, n):
        self.__process_node__(n, False)


class TrajectoryPlanner:
    def __init__(self, osm_path: str):
        self.osm_path = osm_path
        self.osm_handler = OsmHandler(self.callback)

        self.map = folium.Map(location=[-4.9688697, -39.017628], zoom_start=10)


    def __check_file_osm_exists__(self):
        if not os.path.isfile(self.osm_path):
            raise Exception('OSM file not exists')

    
    def callback(self, position, tags):
        name = tags['name'] if 'name' in tags.keys() else tags['comment']
        
        folium.Marker(
            location=position,
            popup=name
        ).add_to(self.map)

        self.map.save('trajectory.html')

    def generate(self):
        self.__check_file_osm_exists__()

        self.osm_handler.apply_file(self.osm_path, locations=True)
