# import bpy
import numpy as np

import bpy

from sys import path
path.append(r'C:\projets\McQueenMobile\Code')

# import ultrasonic_avoidance as ua

import constants as const
import utils

import math


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

            if utils.check_obs_direction(self.right_sensor, obs, front_axe) and sides_dist <= const.SENSOR_SIDES_RANGE:
                
#                if math.isclose(front_dist, const.SENSOR_CLOSE_RANGE, 5):
                if front_dist <= const.SENSOR_CLOSE_RANGE:
                    return const.BACKWARDS_FLAG
#                elif math.isclose(front_dist, const.SENSOR_FAR_RANGE, 5):
                elif front_dist <= const.SENSOR_FAR_RANGE:
                    return const.TURN_FLAG

        return const.OTHER_FLAG


class Car:

    def __init__(self):
        print("init")


    def init_simulation(self, car_name, obs_name, front_axe, direction):
        self.obj = bpy.data.objects[car_name]
        self.rotation_axe = const.Z_AXE
        self.front_axe = front_axe
        self.direction = direction
        self.ultrasonic_sensor = Ultrasonic_avoidance(obs_name)


    def move(self, keyframe, move_dist, backwards=False):
        new_pos = self.obj.location 
        move_dist = move_dist if not backwards else -move_dist
        new_pos[self.front_axe] += move_dist*self.direction
        self.obj.location = new_pos[:]
        self.obj.keyframe_insert(data_path="location", frame=keyframe)


    def rotate(self, angle, keyframe):
        self.obj.rotation_euler[self.rotation_axe] += angle
        self.obj.keyframe_insert(data_path="rotation_euler", frame=keyframe)
        


    def simulate(self, move_dist, frame_rate):
        curr_frame = 0
        count = 0


        while True:
            bpy.context.scene.frame_set(curr_frame)
            self.rotate(0, curr_frame)
            avoid_flag = self.ultrasonic_sensor.avoid_obstacle(self.front_axe, move_dist)

            if avoid_flag == const.BACKWARDS_FLAG:
                print("BACKWARDS")
                # Obstacle detected too close, move backwards
                self.move(curr_frame, move_dist, True)
            elif avoid_flag == const.TURN_FLAG:
                print("TURN")  
                # Obstacle detected, turn right         
                self.rotate(-np.pi/2, curr_frame)
                self.front_axe = utils.toggle_axe(self.front_axe)  
            else:
                self.move(curr_frame, move_dist)

            curr_frame += frame_rate
            count += 1

            if count > 120:
                # Trajectory end (TODO add line follower check for end)
                break
            
            
#     def test(self):
#         sensor_right = bpy.data.objects["sensor_r"]
        
#         move_dist = 2
#         front_axe = 1
#         i = 1
#         frame_rate = 2
#         curr_frame = 0
        
#         while True:
# #            print(f"sensor right {sensor_right.location}")
#             print(f"sensor matrix {sensor_right.matrix_world.to_translation()}")
#             bpy.context.scene.frame_set(curr_frame)
#             self.move(curr_frame, move_dist)
#             curr_frame += frame_rate 
#             i += 1
            
#             if i > 120:
#                 break


def main():
    move_dist = 2
    frame_rate = 2
    front_axe = const.Y_AXE
    direction = 1

    c = Car()
    c.init_simulation("car", "obstacles", front_axe, direction)
    c.simulate(move_dist, frame_rate)
#    c.test()

    

if __name__ == '__main__':
    main()


