import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree
import time
import numpy as np


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
    rotate(car_obj, rotation_axe, step_angle, curr_frame)
    
    return off_track_count
    

if __name__ == '__main__':
    collection = bpy.data.collections['car']
    sensor1_obj = bpy.data.objects["sensor1"]
    sensor2_obj = bpy.data.objects["sensor2"]
    sensor3_obj = bpy.data.objects["sensor3"]
    sensor4_obj = bpy.data.objects["sensor4"]
    sensor5_obj = bpy.data.objects["sensor5"]

    trajectoire = bpy.data.objects["road"]
    
    #print(detectLine(sensor4_obj, trajectoire))
    line_status(sensor1_obj, sensor2_obj, sensor3_obj,sensor4_obj, sensor5_obj, trajectoire )

    car_obj = bpy.data.objects["car"]
    
    move_dist = 2
    count = 0
    front_axe = Y_AXE
    rotation_axe = 2
    frame_rate = 2
    curr_frame = 0
    direction = 1
        
    a_step = 3
    b_step = 10
    c_step = 20 #degré
    d_step = 45 #degré
    step_angle = 0 # axe y
    off_track_count = 0

    while True:
        bpy.context.scene.frame_set(curr_frame)

        print(curr_frame)
        off_track_count = line_follow(off_track_count)
        
        if off_track_count == 0:
            move(car_obj, move_dist, front_axe, direction, curr_frame)
        curr_frame += frame_rate 

        count += 1
        
        if count > 120:
            # Trajectory end
            break

    
def calibrating():
    #code example pour system reel
    REFERENCES = [200, 200, 200, 200, 200]
    #calibrate = True
    calibrate = False

    delay = 0.0005

    references = [0, 0, 0, 0, 0]
    print("cali for module:\n  first put all sensors on white, then put all sensors on black")
    mount = 100
    #fw.turn(70)
    print("\n cali white")
    time.sleep(4)
    #fw.turn(90)
    #white_references = lf.get_average(mount)
    #fw.turn(95)
    time.sleep(0.5)
    #fw.turn(85)
    time.sleep(0.5)
    #fw.turn(90)
    time.sleep(1)

    #fw.turn(110)
    print("\n cali black")
    time.sleep(4)
    #fw.turn(90)
    #black_references = lf.get_average(mount)
    #fw.turn(95)
    time.sleep(0.5)
    #fw.turn(85)
    time.sleep(0.5)
    #fw.turn(90)
    time.sleep(1)

    #for i in range(0, 5):
    #	references[i] = (white_references[i] + black_references[i]) / 2
    #lf.references = references
    print("Middle references =", references)
    time.sleep(1)