import bpy
import numpy as np

import constants as const
import utils

class Ultrasonic_avoidance:

    def __init__(self, collection_name):
        # Get all obstacles objects 
        collection = bpy.data.collections[collection_name]
        self.obstacles = collection.all_objects
        self.right_sensor = bpy.data.objects["sensor_r"]


    def get_distance(self, front_axe, sides_axe, sensor_pos, obs_pos):
        front_dist = np.abs(sensor_pos[front_axe] - obs_pos[front_axe])
        sides_dist = np.abs(sensor_pos[sides_axe] - obs_pos[sides_axe])

        return front_dist, sides_dist


    def avoid_obstacle(self, front_axe, move_dist):
        sensor_pos = utils.get_child_obj_location(self.right_sensor)
        sensor_pos[front_axe] += move_dist
        sides_axe = utils.toggle_axe(front_axe)

        for obs in self.obstacles:
            front_dist, sides_dist = self.get_distance(front_axe, sides_axe, sensor_pos, obs.location)

            if self.check_obs_direction(self.right_sensor, obs, front_axe) and sides_dist <= const.SENSOR_SIDES_RANGE:
                if front_dist <= const.SENSOR_CLOSE_RANGE:
                    return const.BACKWARDS_FLAG
                elif front_dist <= const.SENSOR_FAR_RANGE:
                    return const.TURN_FLAG

            return const.OTHER_FLAG