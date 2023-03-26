def frames_to_seconds(frames):
    # 24 frames per second
    return frames/24


def toggle_axe(curr_axe):
    return Y_AXE if curr_axe == X_AXE else X_AXE


def toggle_direction(curr_dir):
    return -1 if curr_dir == 1 else 1


def get_child_obj_location(obj):
    return obj.matrix_world.to_translation()


def check_obs_direction(obj, obs, front_axe):
    print(f"obs {obs.location[front_axe]} obj {obj.matrix_world.to_translation()[front_axe]} front_axe {front_axe}")
    return obs.location[front_axe] > get_child_obj_location(obj)[front_axe] 