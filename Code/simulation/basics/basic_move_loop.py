import bpy
    

def move(obj, new_pos, keyframe):
    obj.location = new_pos[:]
    obj.keyframe_insert(data_path="location", frame=keyframe, index=-1)
    

if __name__ == '__main__':
    
    car_obj = bpy.data.objects["car"]
    
    move_dist = 2
    front_axe = 1
    i = 1
    frame_rate = 2
    curr_frame = 0
    
    while True:
        bpy.context.scene.frame_set(curr_frame)
        curr_pos = car_obj.location 
        curr_pos[front_axe] += move_dist
        move(car_obj, curr_pos, curr_frame)
        curr_frame += frame_rate 
        i += 1
        
        if i > 120:
            break