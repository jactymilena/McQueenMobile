import numpy as np
import copy
import time
import bpy

from sys import path
path.append(r'C:\projets\McQueenMobile\Code\simulation')


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


    def detect_obstacle(self, front_axe, move_dist, direction):
        sensor_pos = utils.get_child_obj_location(self.right_sensor)
        sides_axe = utils.toggle_axe(front_axe)

        for obs in self.obstacles:
            front_dist, sides_dist = self.get_distance(front_axe, sides_axe, sensor_pos, obs.location)
            if utils.check_obs_direction(self.right_sensor, obs, front_axe, direction) and sides_dist <= const.SENSOR_SIDES_RANGE:
                if front_dist <= const.SENSOR_CLOSE_RANGE:
                    return True

        return False


class Car:

    def __init__(self, car_name, obs_name, front_axe, direction, max_speed):
        self.obj = bpy.data.objects[car_name]
        self.rotation_axe = const.Z_AXE
        self.front_axe = front_axe
        self.direction = direction
        self.curr_frame = 0
        self.max_speed = max_speed
        self.ua = Ultrasonic_avoidance(obs_name)


    def move_by_dist(self, dist, frame_rate, move_dist, backwards=False):
        for i in range(int(dist/move_dist)):
            self.move(move_dist, backwards)
            self.curr_frame += frame_rate


    def move(self, move_dist, backwards=False):
        new_pos = self.obj.location 
        move_dist = move_dist if not backwards else -move_dist
        new_pos[self.front_axe] += move_dist*self.direction
        self.obj.location = new_pos[:]
        self.obj.keyframe_insert(data_path="location", frame=self.curr_frame)


    def rotate(self, angle):
        self.obj.rotation_euler[self.rotation_axe] += angle
        self.obj.keyframe_insert(data_path="rotation_euler", frame=self.curr_frame)


    def movement_points(self, angle_delta, rayon, frame_total, turn_direction):
        axe2 = utils.toggle_axe(self.front_axe)
        old_pos = self.obj.location
        new_pos = copy.deepcopy(old_pos)
        position_array = []

        for i in range(int(frame_total)):
            x = np.cos((i+1)*angle_delta) * rayon
            y = np.sin((i+1)*angle_delta) * rayon
            
            new_pos[self.front_axe] = old_pos[self.front_axe] + y*self.direction
            
            if ((self.front_axe == const.Y_AXE) and (turn_direction == const.LEFT)) or ((self.front_axe == const.X_AXE) and (turn_direction == const.RIGHT)):
                new_pos[axe2] = old_pos[axe2] - (rayon-x)*self.direction
            else:
                new_pos[axe2] = old_pos[axe2] + (rayon-x)*self.direction

            position_array.append(copy.deepcopy(new_pos))
        
        return position_array


    def rotation_points(self, angle_delta, frame_total, turn_direction):
        old_rot = self.obj.rotation_euler
        rotation_array = []

        for i in range(frame_total):
            new_rot = copy.deepcopy(old_rot)
            
            if turn_direction == const.LEFT:
                new_rot[const.Z_AXE] += (i+1)*angle_delta
            else:
                new_rot[const.Z_AXE] -= (i+1)*angle_delta
            
            rotation_array.append(new_rot)
        
        return rotation_array
    

    def apply_turn(self, positions, rotations, frame_rate):
        for x in range(len(positions)):
            self.obj.location = positions[x]
            self.obj.rotation_euler = rotations[x]
            self.obj.keyframe_insert(data_path="location", frame=self.curr_frame)
            self.obj.keyframe_insert(data_path="rotation_euler", frame=self.curr_frame)
            self.curr_frame += frame_rate


    def turn_right(self, frame_rate):
        self.turn(-np.pi/2, frame_rate)


    def turn_left(self, frame_rate): 
        self.turn(np.pi/2, frame_rate)


    def turn(self, angle, frame_rate, rayon=12):
        turn_direction = const.RIGHT if angle < 0 else const.LEFT

        angle = np.abs(angle)
        dist = angle * rayon
        frame_total = round(dist/self.max_speed)
        angle_delta = angle/frame_total

        # rotation
        self.obj.keyframe_insert(data_path="rotation_euler", frame=self.curr_frame-frame_rate)
        rotations = self.rotation_points(angle_delta, frame_total, turn_direction)
        # movement
        positions = self.movement_points(angle_delta, rayon, frame_total, turn_direction)

        self.apply_turn(positions, rotations, frame_rate)

        
        if (self.front_axe == const.Y_AXE and turn_direction == const.LEFT) or (self.front_axe == const.X_AXE and turn_direction != const.LEFT):
            self.direction = utils.toggle_direction(self.direction)

        self.front_axe = utils.toggle_axe(self.front_axe) 



    def accelerate(curr_speed, goal_speed):
        factor = 1/5
        acc = (goal_speed - curr_speed) * factor
        return acc


    def speed(frames, move_dist):
        seconds = utils.frames_to_seconds(frames)
        current_speed = move_dist / seconds
        return current_speed


    def stop(self, stop_time):
        time.sleep(stop_time)

    
    def obstacle_avoidance(self, frame_rate, move_dist, obs_size):
        self.turn_right(frame_rate) 
        self.move_by_dist(obs_size[0], frame_rate, move_dist)
        self.turn_left(frame_rate) 
        self.move_by_dist(obs_size[1] + const.ORIGIN_DISTANCE, frame_rate, move_dist)
        self.turn_left(frame_rate) 
        self.move_by_dist(obs_size[0], frame_rate, move_dist)
        self.turn_right(frame_rate)  
        # TODO : ajouter une condition au virage pour qu'il soit effectue seulement si la ligne est detectee (dans run pas ici)


    def run(self, move_dist, frame_rate):
        count = 0

        while True:
            bpy.context.scene.frame_set(self.curr_frame)
            self.rotate(0)

            if self.ua.detect_obstacle(self.front_axe, move_dist, self.direction):
                print(f"OBSTACLE DETECTED {self.curr_frame}")
                self.move_by_dist(20, frame_rate, move_dist, backwards=True)
                self.stop(2)
                self.obstacle_avoidance(frame_rate, move_dist, [10, 10])
            else:
                self.move(move_dist) # TODO add line follower move conditions

            self.curr_frame += frame_rate
            count += 1

            if count > 120:
                # Trajectory end (TODO add line follower check for end)
                break


def main():
    move_dist = 2
    frame_rate = 2
    front_axe = const.Y_AXE
    direction = 1
    max_speed = 0.297 * 100 / 24 

    c = Car("car", "obstacles", front_axe, direction, max_speed)
    c.run(move_dist, frame_rate)
    

if __name__ == '__main__':
    main()


