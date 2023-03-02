import bpy
import math 
#https://www.youtube.com/watch?v=ssHiWpVuxTk&ab_channel=Olav3DTutorials
def move_forward(object, frame_num, starting_position, positions):
    for position in positions:
        bpy.context.scene.frame_set(frame_num)
        object.location = position
        object.keyframe_insert(data_path="location", index=-1)
        frame_num += 20 

#https://www.youtube.com/watch?v=97CfID1UaOc
def rotate(object,frame_num, rotations):
    for rotation in rotations:
        bpy.context.scene.frame_set(frame_num)
        object.rotation_euler = rotation
        object.keyframe_insert(data_path="rotation_euler", index=-1)
        frame_num += 20 


if __name__ == '__main__':
    ob = bpy.data.objects["car"]
    start_pos = (0,0,1.5)
    positions = (0,0,1.5),(0,4,1.5),(0,6,1.5),(0,10,1.5),(0,5,1.5),(0,2,1.5)
    frame_num = 0
    move_forward(ob, frame_num, start_pos, positions) 
    
    rotations = (0,0,0),(0,0,3),
    rotate(ob,120, rotations)
    po=(0,0,1.5),(0,-2,1.5)
    move_forward(ob, 140, start_pos,po ) 