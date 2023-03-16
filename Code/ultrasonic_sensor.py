import bpy
import math 
import numpy as np


X_AXE = 0
Y_AXE = 1
Z_AXE = 2
SENSOR_FAR_RANGE = 25
SENSOR_CLOSE_RANGE = 15
TURN_FLAG = 1
BACKWARDS_FLAG = 2
OTHER_FLAG = 0


def frames_to_seconds(frames):
    # 24 frames per second
    return frames/24


def toggle_direction(curr_axe):
    return Y_AXE if curr_axe == X_AXE else X_AXE


def rotate(obj, axe, angle, keyframe):
    obj.rotation_euler[axe] += angle
    obj.keyframe_insert(data_path="rotation_euler", frame=keyframe)
    

def move(obj, dist, axe, keyframe):
    new_pos = obj.location 
    new_pos[axe] += dist
    obj.location = new_pos[:]
    obj.keyframe_insert(data_path="location", frame=keyframe)
    

def get_child_obj_location(obj):
    return obj.matrix_world.to_translation()


def check_direction(obj, obs, front_axe):
    print(f"obs {obs.location[front_axe]} obj {obj.matrix_world.to_translation()[front_axe]} front_axe {front_axe}")
    return obs.location[front_axe] > get_child_obj_location(obj)[front_axe] 
    

def avoid_obstacle(obstacle, sensor, move_dist, front_axe) -> bool:
    sensor_pos = get_child_obj_location(sensor)
    sensor_pos[front_axe] += move_dist
    dist_from_obs = (sensor_pos - obstacle.location).length
#    front_dist = 
#    sides_dist
    
    if check_direction(sensor, obstacle, front_axe):
        print(f"distance {dist_from_obs}")
        if dist_from_obs <= SENSOR_CLOSE_RANGE:
            return BACKWARDS_FLAG
        elif dist_from_obs <= SENSOR_FAR_RANGE:
            return TURN_FLAG
        else:
            return OTHER_FLAG
 

if __name__ == '__main__':
    car_obj = bpy.data.objects["car"]
    obs_obj = bpy.data.objects["obstacle"]
    sensor_right = bpy.data.objects["sensor_r"]
    sensor_left = bpy.data.objects["sensor_l"]
    
    move_dist = 2
    count = 0
    front_axe = Y_AXE
    rotation_axe = 2
    frame_rate = 2
    curr_frame = 0
    end_avoidance = -1
    
    move(car_obj, 0, front_axe, curr_frame)
    
    while True:
        rotate(car_obj, rotation_axe, 0, curr_frame)
        bpy.context.scene.frame_set(curr_frame)

        print(curr_frame)
        
        avoid_flag = avoid_obstacle(obs_obj, sensor_right, move_dist, front_axe)
        
        if avoid_flag == BACKWARDS_FLAG:
            print("obstacle detected BACKWARDS_FLAG" + str(curr_frame))
            move(car_obj, -move_dist, front_axe, curr_frame)
            
        elif avoid_flag == TURN_FLAG:            
            print("obstacle detected TURN_FLAG" + str(curr_frame))
            rotate(car_obj, rotation_axe, -np.pi/2, curr_frame)
            front_axe = toggle_direction(front_axe)
            
            end_avoidance = (5 * move_dist) + count
            
        elif end_avoidance == count:
            rotate(car_obj, rotation_axe, np.pi/2, curr_frame)
            front_axe = toggle_direction(front_axe)
        else:
            move(car_obj, move_dist, front_axe, curr_frame)

        curr_frame += frame_rate 

        count += 1
        
        if count > 120:
            break

# TODO: verifier si l'obstacle est en x et en y
# TODO: code pour plusieurs obstacles
# TODO: Mettre ca en classes et tester plusieurs fichiers sur blender
