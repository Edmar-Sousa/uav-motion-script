import osmium
import os


class OsmHandler(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()

        self.food_tags = ['fast_food', 'restaurant', 'department_store']
        self.keys = ['amenity', 'shop']

    def way(self, n):
        for tag in n.tags:
            # filtrando por tipo restaurante ou fast food
            if tag.k in self.keys and tag.v in self.food_tags:
                print()
                
                for t in n.tags:
                    print(t)
                
                for node in n.nodes: 
                    print(node.lat, node.lon)

    def node(self, n):
        # print(n)
        pass


class TrajectoryPlanner:
    def __init__(self, osm_path: str):
        self.osm_path = osm_path
        self.osm_handler = OsmHandler()


    def __check_file_osm_exists__(self):
        if not os.path.isfile(self.osm_path):
            raise Exception('OSM file not exists')


    def generate(self):
        self.__check_file_osm_exists__()

        self.osm_handler.apply_file(self.osm_path, locations=True)
