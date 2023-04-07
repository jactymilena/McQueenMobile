import numpy as np
import copy
import time
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

import sys, os

dir = os.path.dirname(bpy.data.filepath)
if not dir in sys.path:
    sys.path.append(dir)

import constants as const
import utils

class Line_following:

    def __init__(self, trajectories_name):
        self.sensor1_obj = bpy.data.objects["sensor1"]
        self.sensor2_obj = bpy.data.objects["sensor2"]
        self.sensor3_obj = bpy.data.objects["sensor3"]
        self.sensor4_obj = bpy.data.objects["sensor4"]
        self.sensor5_obj = bpy.data.objects["sensor5"]

        self.trajectoire = bpy.data.objects[trajectories_name]

    def detect_line(sensor, trajectoire):
        
        copy_sensor = sensor.matrix_world.copy()
        sensor_verts = [copy_sensor @ vertex.co for vertex in sensor.data.vertices]
        sensor_polys = [polygon.vertices for polygon in sensor.data.polygons]    

        copy_traj = trajectoire.matrix_world.copy()
        traj_verts = [copy_traj @ vertex.co for vertex in trajectoire.data.vertices] 
        traj_polys = [polygon.vertices for polygon in trajectoire.data.polygons]

        sensor_bvh_tree = BVHTree.FromPolygons(sensor_verts, sensor_polys)
        traj_bvh_tree = BVHTree.FromPolygons(traj_verts, traj_polys)
        intersections = sensor_bvh_tree.overlap(traj_bvh_tree)
        return intersections
    

    def line_status(self, sensor1, sensor2, sensor3, sensor4, sensor5, trajectoire):
        status = [0, 0, 0, 0, 0]
        if self.detect_line(sensor1, trajectoire):
            status[0] = 1
        if self.detect_line(sensor2, trajectoire):
            status[1] = 1
        if self.detect_line(sensor3, trajectoire):
            status[2] = 1
        if self.detect_line(sensor4, trajectoire):
            status[3] = 1
        if self.detect_line(sensor5, trajectoire):
            status[4] = 1
        return status
    
    def line_follow_angle(lt_status_now):
        step = 0
        turning_angle = 0
        a_step = np.pi/60 # 3
        b_step = np.pi/18  # 10
        c_step = np.pi/6 # 30  degré
        d_step = np.pi/4 #  45 degré

        # Angle calculate
        if	lt_status_now == [0,0,1,0,0]:
            step = 0	
        elif lt_status_now == [0,1,1,0,0] or lt_status_now == [0,0,1,1,0]:
            step = a_step
        elif lt_status_now == [0,1,0,0,0] or lt_status_now == [0,0,0,1,0]:
            step = b_step
        elif lt_status_now == [1,1,0,0,0] or lt_status_now == [0,0,0,1,1]:
            step = c_step
        elif lt_status_now == [1,0,0,0,0] or lt_status_now == [0,0,0,0,1]:
            step = d_step  
    
    # turn right
        if lt_status_now in ([0,1,1,0,0],[0,1,0,0,0],[1,1,0,0,0],[1,0,0,0,0]):
            turning_angle = -step/4
    # turn left
        elif lt_status_now in ([0,0,1,1,0],[0,0,0,1,0],[0,0,0,1,1],[0,0,0,0,1]):
            turning_angle = step/4
        elif lt_status_now == [0,0,0,0,0]:
            turning_angle = 0
        else :turning_angle = 0
        
        return turning_angle

class Ultrasonic_avoidance:

    def __init__(self, collection_name):
        # Get all obstacles objects 
        collection = bpy.data.collections[collection_name]
        self.obstacles = collection.all_objects
        self.right_sensor = bpy.data.objects["sensor_r"]


    def get_distance(self, front_axe, sides_axe, sensor_pos, obs_obj, direction):
        obstacle_distance = obs_obj.dimensions[front_axe]/2.0;
        front_dist = np.abs((sensor_pos[front_axe] + (const.FRONT_DISTANCE * direction) + (obstacle_distance * direction))  - obs_obj.location[front_axe])
        sides_dist = np.abs(sensor_pos[sides_axe] - obs_obj.location[sides_axe])
        return front_dist, sides_dist


    def detect_obstacle(self, front_axe, move_dist, direction):
        sensor_pos = utils.get_child_obj_location(self.right_sensor)
        sides_axe = utils.toggle_axe(front_axe)

        for obs in self.obstacles:
            front_dist, sides_dist = self.get_distance(front_axe, sides_axe, sensor_pos, obs, direction)
            if utils.check_obs_direction(self.right_sensor, obs, front_axe, direction) and sides_dist <= const.SENSOR_SIDES_RANGE:
                if front_dist <= const.SENSOR_CLOSE_RANGE:
                    return True

        return False


class Car:

    def __init__(self, car_name, obs_name, traject_name, front_axe, direction, max_speed, total_turning):
        self.obj = bpy.data.objects[car_name]
        self.rotation_axe = const.Z_AXE
        self.front_axe = front_axe
        self.direction = direction
        self.curr_frame = 0
        self.max_speed = max_speed
        self.angle_car = total_turning
        self.ua = Ultrasonic_avoidance(obs_name)
        self.lf = Line_following(traject_name)


    def move_by_dist(self, dist, frame_rate, move_dist, backwards=False):
        for i in range(int(dist/move_dist)):
            self.move(move_dist, backwards)
            self.curr_frame += frame_rate


    def move(self, move_dist, angle, backwards=False):
        new_pos = self.obj.location 
        move_dist = move_dist if not backwards else -move_dist
        new_pos[self.front_axe] += move_dist*self.direction
        new_pos[const.X_AXE] += move_dist*np.cos(angle)
        new_pos[const.Y_AXE] += move_dist*np.sin(angle)
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
    
    def change_direction(self):
        if ((self.angle_car < -np.pi) and (self.angle_car > -np.pi*3/2)) or ((self.angle_car > np.pi) and (self.angle_car < np.pi*3/2)):
            self.front_axe = const.X_AXE
            self.direction = -1
        elif ((self.angle_car < -np.pi/2) and (self.angle_car > -np.pi)) or ((self.angle_car > np.pi*3/2) and (self.angle_car < np.pi*2)):
            self.front_axe = const.Y_AXE
            self.direction = -1
        elif ((self.angle_car > np.pi/2) and (self.angle_car  > np.pi)) or ((self.angle_car < -np.pi*3/2) and (self.angle_car > -np.pi*2)):
            self.front_axe = const.Y_AXE
            self.direction = 1
        else: 
            self.front_axe = const.X_AXE
            self.direction = 1
        if self.angle_car > 2*np.pi or self.angle_car  < -2*np.pi:
            self.angle_car = self.angle_car - (2*np.pi)
        elif self.angle_car < -2*np.pi:
            self.angle_car = self.angle_car + (2*np.pi)


    def apply_turn(self, positions, rotations, frame_rate):
        for x in range(len(positions)):
            self.obj.location = positions[x]
            self.obj.rotation_euler = rotations[x]
            self.obj.keyframe_insert(data_path="location", frame=self.curr_frame)
            self.obj.keyframe_insert(data_path="rotation_euler", frame=self.curr_frame)
            self.curr_frame += frame_rate

    def apply_turn_with_line(self,positions, rotations, angle, frame_rate):
        tmp_angle = angle
        for x in range(len(positions)):
            self.rotate(0)
            bpy.context.scene.frame_set(self.curr_frame)
            lt_status_now = self.lf.line_status(self.lf.sensor1_obj, self.lf.sensor2_obj, self.lf.sensor3_obj, self.lf.sensor4_obj, self.lf.sensor5_obj, self.lf.trajectoire )
            turning_angle = self.lf.line_follow_angle(lt_status_now)
            print(lt_status_now, "status ", turning_angle, "en rad")
            if tmp_angle != turning_angle:
                print("doit break", tmp_angle, "! = ", turning_angle)
                break
            self.obj.location = positions[x]
            self.obj.rotation_euler = rotations[x]
            self.obj.keyframe_insert(data_path="location", frame=self.curr_frame)
            self.obj.keyframe_insert(data_path="rotation_euler", frame=self.curr_frame)
           
            self.curr_frame += frame_rate


    def turn_right(self, frame_rate):
        self.turn(-np.pi/2, frame_rate,0)


    def turn_left(self, frame_rate): 
        self.turn(np.pi/2, frame_rate, 0)


    def turn(self, angle, frame_rate, isline, rayon=12):
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

        if isline:
            self.apply_turn_with_line(positions, rotations, angle,frame_rate)
        else:
            self.apply_turn(positions, rotations, frame_rate)
        
       # if (self.front_axe == const.Y_AXE and turn_direction == const.LEFT) or (self.front_axe == const.X_AXE and turn_direction != const.LEFT):
        #    self.direction = utils.toggle_direction(self.direction)

        #self.front_axe = utils.toggle_axe(self.front_axe) 



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
        turning_angle = 0
        while True:
            bpy.context.scene.frame_set(self.curr_frame)
            self.rotate(0)

            if self.ua.detect_obstacle(self.front_axe, move_dist, self.direction):
                print(f"OBSTACLE DETECTED {self.curr_frame}")
                self.move_by_dist(20, frame_rate, move_dist, backwards=True)
                self.stop(2)
                self.obstacle_avoidance(frame_rate, move_dist, [10, 10])
            else:
                #follow_line
                lt_status_now = self.lf.line_status(self.lf.sensor1_obj, self.lf.sensor2_obj, self.lf.sensor3_obj, self.lf.sensor4_obj, self.lf.sensor5_obj, self.lf.trajectoire)
                if lt_status_now == [1, 1, 1, 1, 1]:
                    break
                turning_angle = self.lf.line_follow_angle(lt_status_now)
                self.turn(self, turning_angle, 1, frame_rate)
                self.change_direction()
                #self.move(move_dist) # TODO add line follower move conditions

            self.curr_frame += frame_rate
            count += 1

            if count > 120:
                # Trajectory end (TODO add line follower check for end)
                break


def main():
    move_dist = 2
    frame_rate = 2
    front_axe = const.X_AXE
    direction = 1
    max_speed = 0.297 * 100 / 24 
    total_turning = 0

    c = Car("car", "obstacles","road1", front_axe, direction, max_speed, total_turning)
    c.run(move_dist, frame_rate)
    

if __name__ == '__main__':
    main()


