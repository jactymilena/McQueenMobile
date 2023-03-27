import copy
import bpy
import math 
import numpy as np

X_AXE = 0
Y_AXE = 1
Z_AXE = 2


def toggle_axe(curr_axe):
    return Y_AXE if curr_axe == X_AXE else X_AXE

def toggle_direction(curr_dir):
    return -1 if curr_dir == 1 else 1
    
    
def virage_gauche(obj, axe1, angle, direction, keyframe, frame_rate, vit, rayon=12):
    angle = np.abs(angle)
    dist = angle * rayon
    frame_total = round(dist/vit)
    angle_delta = angle/frame_total

    # rotation gauche
    obj.keyframe_insert(data_path="rotation_euler", frame=keyframe-frame_rate)
    old_rot = obj.rotation_euler
    rotation_array = []
    
    for i in range(frame_total):
        new_rot = copy.deepcopy(old_rot)
        new_rot[Z_AXE] += (i+1)*angle_delta
        rotation_array.append(new_rot)
    
    # déplacement gauche
    axe2 = toggle_axe(axe1)
    old_pos = obj.location
    new_pos = copy.deepcopy(old_pos)
    position_array = []

    for i in range(int(frame_total)):
        x = np.cos((i+1)*angle_delta) * rayon
        y = np.sin((i+1)*angle_delta) * rayon
        
        if axe1 == 1:
            new_pos[axe1] = old_pos[axe1] + y*direction
            new_pos[axe2] = old_pos[axe2] - (rayon-x)*direction
        else:
            new_pos[axe1] = old_pos[axe1] + y*direction
            new_pos[axe2] = old_pos[axe2] + (rayon-x)*direction

        position_array.append(copy.deepcopy(new_pos))
            
    return position_array, rotation_array


def virage_droite(obj, axe1, angle, direction, keyframe, frame_rate, vit, rayon=12):
    angle = np.abs(angle)
    dist = angle * rayon
    frame_total = round(dist/vit)
    angle_delta = angle/frame_total

    # rotation droite
    obj.keyframe_insert(data_path="rotation_euler", frame=keyframe-frame_rate)
    old_rot = obj.rotation_euler
    rotation_array = []
    
    for i in range(frame_total):
        new_rot = copy.deepcopy(old_rot)
        new_rot[Z_AXE] -= (i+1)*angle_delta
        rotation_array.append(new_rot)      
    
    # déplacement droite
    axe2 = toggle_axe(axe1)
    old_pos = obj.location        
    new_pos = copy.deepcopy(old_pos)
    position_array = []

    for i in range(int(frame_total)):
        x = np.cos((i+1)*angle_delta) * rayon
        y = np.sin((i+1)*angle_delta) * rayon
        
        if axe1 == 1:
            new_pos[axe1] = old_pos[axe1] + y*direction
            new_pos[axe2] = old_pos[axe2] + (rayon-x)*direction
        else:
            new_pos[axe1] = old_pos[axe1] + y*direction
            new_pos[axe2] = old_pos[axe2] - (rayon-x)*direction        
        
        position_array.append(copy.deepcopy(new_pos))

    return position_array, rotation_array


def move(obj, dist, axe, direction, keyframe):
    new_pos = obj.location 
    new_pos[axe] += dist*direction
    obj.location = new_pos[:]
    obj.keyframe_insert(data_path="location", frame=keyframe)


if __name__ == '__main__':
    car_obj = bpy.data.objects["car"]
    
    vit_max = 0.297                     # m/s
    vit_max = vit_max * 100 / 24        # cm/frame
    i = 0
    frame_rate = 1
    curr_frame = 0
    front_axe = Y_AXE
    direction = -1

    while True:        
        if i==10:
            print('\nVirage')
            virage, rotation = virage_gauche(car_obj, front_axe, np.pi/2, direction, curr_frame, frame_rate, vit_max)
            
            for x in range(len(virage)):
                car_obj.location = virage[x]
                car_obj.rotation_euler = rotation[x]
                car_obj.keyframe_insert(data_path="location", frame=curr_frame)
                car_obj.keyframe_insert(data_path="rotation_euler", frame=curr_frame)
                curr_frame += frame_rate
            
            front_axe = toggle_axe(front_axe)
            direction = toggle_direction(direction)

        move(car_obj, vit_max, front_axe, direction, curr_frame)
        curr_frame += frame_rate 

        i += 1        
        if i > 40:
            break