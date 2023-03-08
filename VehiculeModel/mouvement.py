import bpy
import math 
import numpy as np

X_AXE = 0
Y_AXE = 1
Z_AXE = 2


def frames_to_seconds(frames):
    # 24 frames per second
    return frames/24


#def move_forward(object, positions, start_frame = 0, frame_rate = 24):  
#    curr_frame = start_frame
#    for position in positions:
#        bpy.context.scene.frame_set(curr_frame)
#        object.location = position
#        object.keyframe_insert(data_path="location", index=-1)
#        curr_frame += frame_rate 


#def rotate(object, rotations, start_frame = 0, frame_rate = 24):
#    curr_frame = start_frame
#    for rotation in rotations:
#        bpy.context.scene.frame_set(curr_frame)
#        object.rotation_euler = rotation
#        object.keyframe_insert(data_path="rotation_euler", index=-1)
#        curr_frame += frame_rate 

def rotate(obj, axe, angle, keyframe):
    rotation = obj.rotation_euler
    rotation[axe] += angle
    obj.rotation_euler = rotation
    obj.keyframe_insert(data_path="rotation_euler", frame=keyframe, index=-1)
    

def move(obj, new_pos, keyframe):
    obj.location = new_pos[:]
    obj.keyframe_insert(data_path="location", frame=keyframe, index=-1)


def check_direction(obj, obs, front_axe):
    return obs.location[front_axe] > obj.location[front_axe] 
    

#def ultrasonic_sensor_distace(sensor_right, sensor_letf, 

#def ultrasonic_sensor(car, sensors):
    

#def move(object, final_pos):
#    if final_pos > object.location:
#        print("avance")
#    print("avance pas")


#def check_ultrasonic_sensor(obstacle, sensor)
 


if __name__ == '__main__':
    
    car_obj = bpy.data.objects["car"]
    obs_obj = bpy.data.objects["obstacle"]
    sensor_right = bpy.data.objects["sensor_r"]
    sensor_left = bpy.data.objects["sensor_l"]
    
    move_dist = 2
    i = 1
    sensor_range = 25
    backwards_range = 10
    front_axe = Y_AXE
    rotation_axe = 2
    frame_rate = 2
    curr_frame = 0
    
    while True:
        rotate(car_obj, rotation_axe, 0, curr_frame)
        bpy.context.scene.frame_set(curr_frame)
        curr_pos = car_obj.location 
        curr_pos[front_axe] += move_dist

        distance = (curr_pos - obs_obj.location).length
        print(curr_frame)

        if distance <= sensor_range and check_direction(car_obj, obs_obj, front_axe):
            print("obstacle detected " + str(curr_frame))
            rotate(car_obj, rotation_axe, -np.pi/2, curr_frame)
            front_axe = X_AXE
        else:
            move(car_obj, curr_pos, curr_frame)
            
        curr_frame += frame_rate 
        
#        else:
#        positions.append(curr_pos[:])
        i += 1
        
        if i > 120:
            print("fin")
#            rotate(car_obj, rotation_axe, 3*np.pi/2, curr_frame)
            break
#    move_forward(car_obj, positions, 0, frame_rate) 
        
        
#import bpy
#import math 


#def move_forward(object, frame_num, positions):
#    for position in positions:
#        bpy.context.scene.frame_set(frame_num)
#        object.location = position
#        object.keyframe_insert(data_path="location", index=-1)
#        frame_num += 20 


#def rotate(object, frame_num, rotations):
#    for rotation in rotations:
#        bpy.context.scene.frame_set(frame_num)
#        object.rotation_euler = rotation
#        object.keyframe_insert(data_path="rotation_euler", index=-1)
#        frame_num += 20 


#if __name__ == '__main__':
#    ob = bpy.data.objects["car"]
#    start_pos = (0,0,1.5)
#    positions = start_pos,(0,4,1.5),(0,6,1.5),(0,10,1.5),(0,5,1.5),(0,2,1.5)
#    frame_num = 0
#    move_forward(ob, frame_num, positions) 
#    
#    rotations = (0,0,0),(0,0,3),
#    rotate(ob,120, rotations)
#    po=(0,0,1.5),(0,-2,1.5)
#    move_forward(ob, 140,po ) 