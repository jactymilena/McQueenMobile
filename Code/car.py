# import bpy
import numpy as np
import ultrasonic_avoidance as ua

import constants as const
import utils

class Car:

    def __init__(self):
        pass

    def init_simulation(self, car_name, obs_name, front_axe, direction):
        import bpy
        self.obj = bpy.data.objects[car_name]
        self.rotation_axe = const.Z_AXE
        self.front_axe = front_axe
        self.direction = direction
        self.ultrasonic_sensor = ua.Ultrasonic_avoidance(obs_name)


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
            avoid_flag = self.ultrasonic_sensor.avoid_obstacle(self.front_axe)


            if avoid_flag == const.BACKWARDS_FLAG:
                # Obstacle detected too close, move backwards
                self.move(curr_frame, move_dist, True)
            elif avoid_flag == const.TURN_FLAG:  
                # Obstacle detected, turn right         
                self.rotate(-np.pi/2, curr_frame)
                front_axe = utils.toggle_axe(front_axe)  
            else:
                self.move(curr_frame, move_dist)

            curr_frame += frame_rate
            count += 1

            if count > 120:
                # Trajectory end (TODO add line follower check for end)
                break


