import numbers

def create_empty_request(num_timesteps, joint_target, manipulator_name):
    request = {
        "basic_info" : {
            "n_steps" : num_timesteps,
            "manip" : manipulator_name, # see below for valid values
            "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
        },
        "constraints" : [
        {
            "type" : "joint", # joint-space target
            "params" : {"vals" : joint_target } # length of vals = # dofs of manip
        }
        ],
        "init_info" : {
            "type" : "straight_line", # straight line in joint space.
            "endpoint" : joint_target
        }
    }
    return request


def set_init_traj(request, init_traj):
    request["init_info"]["type"] = "given_traj"
    request["init_info"]["data"] = init_traj
    return request


def add_distance_cost(request, human_poses_mean, human_poses_var, coeffs, n_human_joints, links):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "distance_cost":
            print("ERROR distance cost already present in request")
            return request
    dist_cost = {"type" : "distance_cost", "params" : {}}
    # list of 3-vectors of size n_human_joints * num_timesteps
    dist_cost["params"]["human_poses_mean"] = human_poses_mean
    # list of 3-vectors of size n_human_joints * num_timesteps
    dist_cost["params"]["human_poses_var"] = human_poses_var
    # list of size num_timesteps
    dist_cost["params"]["coeffs"] = coeffs
    # number of human joints
    dist_cost["params"]["n_human_joints"] = n_human_joints
    dist_cost["params"]["links"] = links
    request["costs"].append(dist_cost)
    return request



def add_velocity_cost(request, human_poses_mean, human_poses_var, coeffs, n_human_joints, links):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "velocity_cost":
            print("ERROR velocity cost already present in request")
            return request
    vel_cost = {"type" : "velocity_cost", "params" : {}}
    vel_cost["params"]["human_poses_mean"] = human_poses_mean
    vel_cost["params"]["human_poses_var"] = human_poses_var
    vel_cost["params"]["coeffs"] = coeffs
    vel_cost["params"]["n_human_joints"] = n_human_joints
    vel_cost["params"]["links"] = links
    request["costs"].append(vel_cost)
    return request


def add_visibility_cost(request, head_pos_mean, head_pos_var, coeffs, object_pose, link):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "visibility_cost":
            print("ERROR visibility cost already present in request")
            return request
    vis_cost = {"type" : "visibility_cost", "params" : {}}
    vis_cost["params"]["head_pos_mean"] = head_pos_mean
    vis_cost["params"]["head_pos_var"] = head_pos_var
    vis_cost["params"]["obj_pos"] = object_pose
    vis_cost["params"]["coeffs"] = coeffs
    vis_cost["params"]["link"] = link
    request["costs"].append(vis_cost)
    return request


def add_legibility_cost(request, coeffs, link):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "legibility_cost":
            print("ERROR: Legibility cost already present in request")
            return request
    leg_cost = {"type" : "legibility_cost", "params" : {"link" : link, "coeffs" : coeffs}}
    request["costs"].append(leg_cost)
    return request


def add_optimal_trajectory_cost(request, target_eef_traj, link, num_timesteps, coeffs):
    """
    The "optimal" trajectory may actually be the default trajectory here
    """
    if "costs" not in request:
        request["costs"] = []
    for t in range(num_timesteps):
        pose_cost = {"type" : "pose", "params" : {}}
        pose_cost["params"]["timestep"] = t
        pose_cost["params"]["xyz"] = [target_eef_traj[t, 0], target_eef_traj[t, 1], target_eef_traj[t, 2]]
        pose_cost["params"]["wxyz"] = [1, 0, 0, 0]
        pose_cost["params"]["pos_coeffs"] = [coeffs, coeffs, coeffs]
        pose_cost["params"]["rot_coeffs"] = [0, 0, 0]
        pose_cost["params"]["link"] = link
        request["costs"].append(pose_cost)
    return request


def add_distance_baseline_cost(request, head_pos, torso_pos, feet_pos, link, num_timesteps, coeffs):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "distance_baseline_cost":
            print("ERROR: Distance baseline cost already present in request")
            return request
    dist_baseline_cost = {"type" : "distance_baseline_cost", "params" : {}}
    dist_baseline_cost["params"]["head_pos"] = head_pos
    dist_baseline_cost["params"]["torso_pos"] = torso_pos
    dist_baseline_cost["params"]["feet_pos"] = feet_pos
    dist_baseline_cost["params"]["link"] = link
    dist_baseline_cost["params"]["coeffs"] = coeffs
    request["costs"].append(dist_baseline_cost)
    return request

def add_visibility_baseline_cost(request, head_pos, obj_pos, link, num_timesteps, coeffs):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "visibility_baseline_cost":
            print("ERROR: Visibility baseline cost already present in request")
            return request
    dist_baseline_cost = {"type" : "visibility_baseline_cost", "params" : {}}
    dist_baseline_cost["params"]["head_pos"] = head_pos
    dist_baseline_cost["params"]["obj_pos"] = obj_pos
    dist_baseline_cost["params"]["link"] = link
    dist_baseline_cost["params"]["coeffs"] = coeffs
    request["costs"].append(dist_baseline_cost)
    return request


def add_regularize_cost(request, coeffs, link):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "regularize_cost":
            print("ERROR: Regularize cost already present in request")
            return request
    reg_cost = {"type" : "regularize_cost", "params" : {"link" : link, "coeffs" : coeffs}}
    request["costs"].append(reg_cost)
    return request

def add_smoothing_cost(request, coeff, type):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "smoothing_cost":
            print("ERROR: Smoothing cost already present in request")
            return request
    smooth_cost = {"type" : "smoothing_cost", "params" : {"coeffs" : coeff, "type" : type}}
    request["costs"].append(smooth_cost)
    return request


def add_collision_cost(request, coeffs, dist_pen):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "collision":
            print("ERROR: Collision cost already present in request")
            return request
    collision_cost = {"type" : "collision" ,"params" : {"coeffs" : coeffs, "dist_pen" : dist_pen}}
    request["costs"].append(collision_cost)
    return request


def add_cart_vel_cnst(request, max_displacement, first_step, last_step, link):
    if "constraints" not in request:
        request["constraints"] = []
    cart_vel_cnst = {"type" : "cart_vel", "name" : "cart_vel", "params" : {}}
    cart_vel_cnst["params"]["max_displacement"] = max_displacement
    cart_vel_cnst["params"]["first_step"] = first_step
    cart_vel_cnst["params"]["last_step"] = last_step
    cart_vel_cnst["params"]["link"] = link
    request["constraints"].append(cart_vel_cnst)
    return request

def add_joint_vel_cost(request, coeffs):
    if "costs" not in request:
        request["costs"] = []
    for cost in request["costs"]:
        if cost["type"] == "joint_vel":
            print("ERROR: Joint velocity cost already present in request")
            return request
    joint_vel_cost = {"type" : "joint_vel", "params" : {}}
    if isinstance(coeffs, numbers.Number):
        joint_vel_cost["params"]["coeffs"] = [coeffs]
    else:
        joint_vel_cost["params"]["coeffs"] = coeffs