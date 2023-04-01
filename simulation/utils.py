import constants as const

def frames_to_seconds(frames):
    # 24 frames per second
    return frames/24


def toggle_axe(curr_axe):
    return const.Y_AXE if curr_axe == const.X_AXE else const.X_AXE


def toggle_direction(curr_dir):
    return -1 if curr_dir == 1 else 1


def get_child_obj_location(obj):
    return obj.matrix_world.to_translation()


def check_obs_direction(obj, obs, front_axe, direction):
    if direction == 1:
        return obs.location[front_axe] > get_child_obj_location(obj)[front_axe] 
    else:
        return obs.location[front_axe] < get_child_obj_location(obj)[front_axe] 