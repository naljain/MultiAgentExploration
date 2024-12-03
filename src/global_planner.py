"""
for loop:
    1) generate global goals to feed into path planning to generate reference trajectory/waypoints
       from the most up-to-date map
    2) send reference traj to controller
    3) get updated maps at regular frequency from agents
    4) merge maps together to get most up-to-date map
"""

def merge_maps():
    pass

def send_ref_traj():
    # calls on path planning to get waypoints
    # sends to agents
    pass

def generate_goals():
    pass

def find_frontier():
    pass

def find_uknown_clusters():
    pass

def find_centroid():
    pass

def check_collision():
    pass

def reassign_goal():
    pass