import bpy
import math 
import numpy as np

def frames_to_seconds(frames):
    # 24 frames per second
    return frames/24


def move_forward(object, positions, start_frame = 0, frame_rate = 24):
    
    curr_frame = start_frame
    for position in positions:
        bpy.context.scene.frame_set(curr_frame)
        object.location = position
        object.keyframe_insert(data_path="location", index=-1)
        curr_frame += frame_rate 
        

def rotate(object, rotations, start_frame = 0, frame_rate = 24):
    curr_frame = start_frame
    for rotation in rotations:
        bpy.context.scene.frame_set(curr_frame)
        object.rotation_euler = rotation
        object.keyframe_insert(data_path="rotation_euler", index=-1)
        frame_num += frame_rate 

#def ultrasonic_sensor_distace(sensor_right, sensor_letf, 

#def ultrasonic_sensor(car, sensors):
    

#def move(object, final_pos):
#    if final_pos > object.location:
#        print("avance")
#    print("avance pas")


#def check_ultrasonic_sensor(obstacle, sensor)
 


if __name__ == '__main__':

#    start_pos = (0,0,1.5)
#    positions = start_pos,(0,4,1.5),(0,6,1.5),(0,10,1.5),(0,5,1.5),(0,2,1.5)
#    frame_num = 0
#    move_forward(ob, frame_num, positions) 
#    
#    rotations = (0,0,0),(0,0,3),
#    rotate(ob,120, rotations)
#    po=(0,0,1.5),(0,-2,1.5)
#    move_forward(ob, 140,po ) 
    
    car_obj = bpy.data.objects["car"]
    obs_obj = bpy.data.objects["obstacle"]
    sensor_right = bpy.data.objects["sensor_r"]
    sensor_left = bpy.data.objects["sensor_l"]
    
    m = 1
    i = 1
    sensor_range = 100
    backwards_range = 50
    positions = []
    curr_pos = car_obj.location
    positions.append(curr_pos[:])
    
    print("before while")
    
    while True:
        curr_pos[1] += m
#        print(curr_pos - obs_obj.location)
        
        distance = (curr_pos - obs_obj.location).length
        print(distance)
        if distance <= sensor_range:
            print("obstacle detected")
        
        positions.append(curr_pos[:])
        
        
        i += 1
        
        if i > 100:
            break
    move_forward(car_obj, positions, 0, 2) 
#    print(positions)
        
        