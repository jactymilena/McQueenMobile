import bpy
import math
  

def accelerate(curr_speed, goal_speed):
    factor = 1/5
    acc = (goal_speed - curr_speed) * factor
    return acc

def speed(frames, move_dist):
    seconds = frames_to_seconds(frames)
    current_speed = move_dist / seconds
    return current_speed

def frames_to_seconds(frames):
    # 24 frames per second
    return frames/24

def move(obj, dist, axe, direction, keyframe):
    new_pos = obj.location 
    new_pos[axe] += dist*direction
    obj.location = new_pos[:]
    obj.keyframe_insert(data_path="location", frame=keyframe)

if __name__ == '__main__':
    
    car_obj = bpy.data.objects["car"]
    
    move_dist = 2
    front_axe = 1
    i = 1
    frame_rate = 2
    curr_frame = 0
    goal_speed = 0
    curr_speed = 0
    is_acc = False
    acc_rate = 0

    while True:
        bpy.context.scene.frame_set(curr_frame)
        move(car_obj, move_dist, front_axe, 1, curr_frame)
        curr_frame += frame_rate 
        curr_speed = speed(frame_rate, move_dist)
        i += 1
        
        if i == 20:
            goal_speed  = curr_speed *4
            acceleration = accelerate(curr_speed, goal_speed)
            is_acc = True
            acc_rate  = acceleration * frames_to_seconds(frame_rate)
        if is_acc:
            if not math.isclose(curr_speed, goal_speed):
                move_dist += acc_rate
        else:
            is_acc = False    
        
        if i > 120:
            break