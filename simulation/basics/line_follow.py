import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree
import numpy as np
import copy


X_AXE = 0
Y_AXE = 1
Z_AXE = 2
TURN_FLAG = 1
BACKWARDS_FLAG = 2
OTHER_FLAG = 0
RIGHT = 0
LEFT = 1

def rotate(obj, axe, angle, keyframe):
    #angle en rad
    obj.rotation_euler[axe] += angle
    obj.keyframe_insert(data_path="rotation_euler", frame=keyframe)


def turn(obj, axe1, angle, direction, keyframe, frame_rate, vit, rayon=12):
    turn_direction = RIGHT  if angle<0 else LEFT

    angle = np.abs(angle)
    dist = angle * rayon
    frame_total = round(dist/vit)
    angle_delta = angle/frame_total
    
    # rotation
    obj.keyframe_insert(data_path="rotation_euler", frame=keyframe-frame_rate)
    old_rot = obj.rotation_euler
    rotation_array = []
    
    for i in range(frame_total):
        new_rot = copy.deepcopy(old_rot)
        
        if turn_direction == LEFT:
            new_rot[Z_AXE] += (i+1)*angle_delta
        else:
            new_rot[Z_AXE] -= (i+1)*angle_delta
            
        rotation_array.append(new_rot)
            

    # movement
    axe2 = toggle_axe(axe1)
    old_pos = obj.location
    new_pos = copy.deepcopy(old_pos)
    position_array = []

    for i in range(int(frame_total)):
        x = np.cos((i+1)*angle_delta) * rayon
        y = np.sin((i+1)*angle_delta) * rayon
        
        new_pos[axe1] = old_pos[axe1] + y*direction
        
        if ((axe1 == Y_AXE) and (turn_direction == LEFT)) or ((axe1 == X_AXE) and (turn_direction == RIGHT)):
            new_pos[axe2] = old_pos[axe2] - (rayon-x)*direction
        
        else:
            new_pos[axe2] = old_pos[axe2] + (rayon-x)*direction

        position_array.append(copy.deepcopy(new_pos))
            
    return position_array, rotation_array

def movement_turn(car_obj, position,rotation, curr_frame):
    car_obj.location = position[x]
    car_obj.rotation_euler = rotation[x]
    car_obj.keyframe_insert(data_path="location", frame=curr_frame)
    car_obj.keyframe_insert(data_path="rotation_euler", frame=curr_frame)
    

def move(obj, dist, angle,keyframe):
    new_pos = obj.location 
    new_pos[X_AXE] += dist*np.cos(angle)
    new_pos[Y_AXE] += dist*np.sin(angle)
    obj.location = new_pos[:]
    obj.keyframe_insert(data_path="location", frame=keyframe)


def frames_to_seconds(frames):
    # 24 frames per second
    return frames/24


def toggle_axe(curr_axe):
    return Y_AXE if curr_axe == X_AXE else X_AXE


def toggle_direction(curr_dir):
    return -1 if curr_dir == 1 else 1


def detectLine(sensor, trajectoire):
        
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

def line_status(sensor1, sensor2, sensor3, sensor4, sensor5, trajectoire):
    status = [0, 0, 0, 0, 0]
    if detectLine(sensor1, trajectoire):
        status[0] = 1
    if detectLine(sensor2, trajectoire):
        status[1] = 1
    if detectLine(sensor3, trajectoire):
        status[2] = 1
    if detectLine(sensor4, trajectoire):
        status[3] = 1
    if detectLine(sensor5, trajectoire):
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
    
if __name__ == '__main__':
    sensor1_obj = bpy.data.objects["sensor1"]
    sensor2_obj = bpy.data.objects["sensor2"]
    sensor3_obj = bpy.data.objects["sensor3"]
    sensor4_obj = bpy.data.objects["sensor4"]
    sensor5_obj = bpy.data.objects["sensor5"]

    trajectoire = bpy.data.objects["NurbsPath"]
    car_obj = bpy.data.objects["car"]
    lt_status_now=line_status(sensor1_obj, sensor2_obj, sensor3_obj,sensor4_obj, sensor5_obj, trajectoire )
    #variable
    move_dist = 2
    count = 0
    front_axe = X_AXE
    rotation_axe = Z_AXE
    frame_rate = 2
    curr_frame = 0
    direction = 1
    total_turning = 0
    vit_max = 1.237

    while True:
        rotate(car_obj, rotation_axe, 0, curr_frame)
        bpy.context.scene.frame_set(curr_frame)

        print("curr_frame: ", curr_frame)
        lt_status_now = line_status(sensor1_obj, sensor2_obj, sensor3_obj,sensor4_obj, sensor5_obj, trajectoire )
        if lt_status_now == [1, 1, 1, 1, 1]:
            break
        turning_angle = line_follow_angle(lt_status_now)
        
        position, rotation = turn(car_obj, front_axe, turning_angle, direction, curr_frame, frame_rate,vit_max)
        tmp_angle = turning_angle
        for x in range(len(position)):
            rotate(car_obj, rotation_axe, 0, curr_frame)
            bpy.context.scene.frame_set(curr_frame)
            lt_status_now = line_status(sensor1_obj, sensor2_obj, sensor3_obj,sensor4_obj, sensor5_obj, trajectoire )
            turning_angle = line_follow_angle(lt_status_now)
            print(lt_status_now, "status ", turning_angle, "en rad")
            if tmp_angle != turning_angle:
                print("doit break", tmp_angle, "! = ", turning_angle)
                break
            movement_turn(car_obj, position, rotation, curr_frame)
            curr_frame += frame_rate
        
        
        total_turning += turning_angle
        if ((total_turning < -np.pi) and (total_turning > -np.pi*3/2)) or ((total_turning < -np.pi and total_turning > -np.pi*3/2) and (total_turning > np.pi and total_turning < np.pi*3/2)):
            front_axe = X_AXE
            direction = -1
        elif ((total_turning < -np.pi/2) and (total_turning > -np.pi)) or ((total_turning > np.pi*3/2) and (total_turning < np.pi*2)):
            front_axe = Y_AXE
            direction = -1
        elif ((total_turning > np.pi/2) and (total_turning > np.pi)) or ((total_turning < -np.pi*3/2) and (total_turning > -np.pi*2)):
             front_axe = Y_AXE
             direction = 1
        else: 
            front_axe = X_AXE
            direction = 1
        if total_turning > 2*np.pi or total_turning < -2*np.pi:
            total_turning = total_turning - (2*np.pi)
        elif total_turning < -2*np.pi:
            total_turning = total_turning + (2*np.pi)
        
        move(car_obj, vit_max, total_turning,  curr_frame)
        curr_frame += frame_rate 

        count += 1
        
        if count > 60:
            # Trajectory end
            break
