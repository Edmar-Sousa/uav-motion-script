import osmium
import os


class OsmHandler(osmium.SimpleHandler):
    def node(self, n):
        print(n.location.lat, n.location.lon)


class TrajectoryPlanner:
    def __init__(self, osm_path: str):
        self.osm_path = osm_path
        self.osm_handler = OsmHandler()


    def __check_file_osm_exists__(self):
        if not os.path.isfile(self.osm_path):
            raise Exception('OSM file not exists')


    def generate(self):
        self.__check_file_osm_exists__()

        self.handler_osm.apply_file(self.osm_path)
