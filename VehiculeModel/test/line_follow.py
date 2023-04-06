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
    turn_direction = RIGHT if angle<0 else LEFT

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


def move(obj, dist, axe, direction, keyframe):
    new_pos = obj.location 
    new_pos[axe] += dist*direction
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
#    
#    if intersections:
#        print('MESHES INTERSECTS')
#        print(intersections)
#    else:
#        print('NO INTERSECTIONS')
#    
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


def line_follow(off_track_count):
    #sensor1_obj = bpy.data.objects["Cube"]
    #sensor2_obj = bpy.data.objects["Cube.002"]
    #sensor3_obj = bpy.data.objects["Cube.003"]
    #sensor4_obj = bpy.data.objects["Cube.004"]
    #sensor5_obj = bpy.data.objects["Cube.005"]
#
    #trajectoire = bpy.data.objects["Cube.001"
    #call avancer
    #while True:
    step = 0
    turning_angle = 0
    lt_status_now = line_status(sensor1_obj, sensor2_obj, sensor3_obj,sensor4_obj, sensor5_obj, trajectoire )
    print(lt_status_now)
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
        
    # Direction calculate
    if	lt_status_now == [0,0,1,0,0]:
        off_track_count = 0

    # turn right
    elif lt_status_now in ([0,1,1,0,0],[0,1,0,0,0],[1,1,0,0,0],[1,0,0,0,0]):
        off_track_count = 0
        turning_angle = int(90 - step)
    # turn left
    elif lt_status_now in ([0,0,1,1,0],[0,0,0,1,0],[0,0,0,1,1],[0,0,0,0,1]):
        off_track_count = 0
        turning_angle = int(90 + step)
    elif lt_status_now == [0,0,0,0,0]:
        off_track_count += 1
        print(off_track_count)
    step_angle = (turning_angle * np.pi *2 )/ 360
    print(step_angle, "en rad")
    #turn(car_obj, rotation_axe, step_angle, direction, keyframe, frame_rate, vit)
    #rotate(car_obj, rotation_axe, step_angle, curr_frame)
    
    return off_track_count
    

if __name__ == '__main__':
    collection = bpy.data.collections['car']
    sensor1_obj = bpy.data.objects["sensor1"]
    sensor2_obj = bpy.data.objects["sensor2"]
    sensor3_obj = bpy.data.objects["sensor3"]
    sensor4_obj = bpy.data.objects["sensor4"]
    sensor5_obj = bpy.data.objects["sensor5"]

    trajectoire = bpy.data.objects["NurbsPath"]
    
    #print(detectLine(sensor4_obj, trajectoire))
    lt_status_now=line_status(sensor1_obj, sensor2_obj, sensor3_obj,sensor4_obj, sensor5_obj, trajectoire )

    car_obj = bpy.data.objects["car"]
    
    move_dist = 2
    count = 0
    front_axe = Y_AXE
    rotation_axe = 2
    frame_rate = 2
    curr_frame = 0
    direction = 1
    step = 0
    turning_angle = 0
    a_step = 1
    b_step = 5
    c_step = 10 #degré
    d_step = 25 #degré
    step_angle = 0 # axe y
    off_track_count = 0

    while True:
        rotate(car_obj, rotation_axe, 0, curr_frame)
        bpy.context.scene.frame_set(curr_frame)

        print("curr_frame: ", curr_frame)
        lt_status_now = line_status(sensor1_obj, sensor2_obj, sensor3_obj,sensor4_obj, sensor5_obj, trajectoire )
        print(lt_status_now, " line status ")
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
        
        # Direction calculate
        if	lt_status_now == [0,0,1,0,0]:
            off_track_count = 0

    # turn right
        elif lt_status_now in ([0,1,1,0,0],[0,1,0,0,0],[1,1,0,0,0],[1,0,0,0,0],[1,1,1,0,0]):
            off_track_count = 0
            turning_angle = -step
    # turn left
        elif lt_status_now in ([0,0,1,1,0],[0,0,0,1,0],[0,0,0,1,1],[0,0,0,0,1],[0,0,1,1,1]):
            off_track_count = 0
            turning_angle = step
        elif lt_status_now == [0,0,0,0,0]:
            off_track_count += 1
            turning_angle = 0
        step_angle = np.radians(turning_angle)#(turning_angle * np.pi *2 )/ 360
        print(step_angle, "en rad")
        turn(car_obj, rotation_axe, step_angle, direction, curr_frame, frame_rate,1.2375)
        #rotate(car_obj, rotation_axe, step_angle, curr_frame)
        
        #if off_track_count == 0:
        move(car_obj, move_dist, front_axe, direction, curr_frame)
        curr_frame += frame_rate 

        count += 1
        
        if count > 30:
            # Trajectory end
            break
