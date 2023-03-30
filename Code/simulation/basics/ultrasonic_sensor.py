import bpy
import numpy as np


X_AXE = 0
Y_AXE = 1
Z_AXE = 2
SENSOR_FAR_RANGE = 30
SENSOR_CLOSE_RANGE = 20
SENSOR_SIDES_RANGE = 10
TURN_FLAG = 1
BACKWARDS_FLAG = 2
OTHER_FLAG = 0
RIGHT = 0
LEFT = 1


def frames_to_seconds(frames):
    # 24 frames per second
    return frames/24


def toggle_axe(curr_axe):
    return Y_AXE if curr_axe == X_AXE else X_AXE


def toggle_direction(curr_dir):
    return -1 if curr_dir == 1 else 1


def rotate(obj, axe, angle, keyframe):
    obj.rotation_euler[axe] += angle
    obj.keyframe_insert(data_path="rotation_euler", frame=keyframe)

    
def move(obj, dist, axe, direction, keyframe):
    new_pos = obj.location 
    new_pos[axe] += dist*direction
    obj.location = new_pos[:]
    obj.keyframe_insert(data_path="location", frame=keyframe)
    

def get_child_obj_location(obj):
    return obj.matrix_world.to_translation()


def check_direction(obj, obs, front_axe):
    print(f"obs {obs.location[front_axe]} obj {obj.matrix_world.to_translation()[front_axe]} front_axe {front_axe}")
    return obs.location[front_axe] > get_child_obj_location(obj)[front_axe] 
    

def avoid_obstacle(obstacle, sensor, move_dist, front_axe) -> bool:
    sensor_pos = get_child_obj_location(sensor)
    print(f"sensor_pos {sensor_pos}")
    sensor_pos[front_axe] += move_dist
    obs_pos = obstacle.location
    
    sides_axe = toggle_axe(front_axe)
    front_dist = np.abs(sensor_pos[front_axe] - obs_pos[front_axe])
    sides_dist = np.abs(sensor_pos[sides_axe] - obs_pos[sides_axe])

    if check_direction(sensor, obstacle, front_axe) and sides_dist <= SENSOR_SIDES_RANGE:
        
        if front_dist <= SENSOR_CLOSE_RANGE:
            return BACKWARDS_FLAG
        elif front_dist <= SENSOR_FAR_RANGE:
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
    first_avoidance_turn = -1
    second_avoidance_turn = -1
    last_avoidance_turn = -1
    direction = 1
    
    while True:
        rotate(car_obj, rotation_axe, 0, curr_frame)
        bpy.context.scene.frame_set(curr_frame)

        print(curr_frame)
        
        avoid_flag = avoid_obstacle(obs_obj, sensor_right, move_dist, front_axe)
        
        if avoid_flag == BACKWARDS_FLAG:
            # Obstacle detected too close, move backwards
            move(car_obj, -move_dist, front_axe, direction, curr_frame)
            
        elif avoid_flag == TURN_FLAG:  
            # Obstacle detected, turn right         
            rotate(car_obj, rotation_axe, -np.pi/2, curr_frame)
            front_axe = toggle_axe(front_axe)
            
            first_avoidance_turn = (5 * move_dist) + count
            second_avoidance_turn = (20 * move_dist) + first_avoidance_turn
            last_avoidance_turn = (5 * move_dist) + second_avoidance_turn
            
        elif first_avoidance_turn == count:
            # Obstacle avoidance, turn left
            rotate(car_obj, rotation_axe, np.pi/2, curr_frame)
            front_axe = toggle_axe(front_axe)
        elif second_avoidance_turn == count:
            # Obstacle avoidance, turn left
            rotate(car_obj, rotation_axe, np.pi/2, curr_frame)
            front_axe = toggle_axe(front_axe)
            direction = toggle_direction(direction)
        elif last_avoidance_turn == count:
            # End of obstacle avoidance, turn right
            rotate(car_obj, rotation_axe, -np.pi/2, curr_frame)
            front_axe = toggle_axe(front_axe)
            direction = toggle_direction(direction)
            
        else:
            # Move forward
            move(car_obj, move_dist, front_axe, direction, curr_frame)

        curr_frame += frame_rate 

        count += 1
        
        if count > 120:
            # Trajectory end
            break
