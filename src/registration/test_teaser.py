import copy
import time
import numpy as np
import open3d as o3d
import teaserpp_python
from sklearn.neighbors import KDTree
import tensorflow as tf
tf.compat.v1.disable_eager_execution()
import os
from src.descriptor.network import NetworkBuilder
from src.descriptor import config
from timeit import default_timer as timer
import pandas as pd

NOISE_BOUND = 0.05
N_OUTLIERS = 100
OUTLIER_TRANSLATION_LB = 5
OUTLIER_TRANSLATION_UB = 10

VISUALIZE = True
NOISE_BOUND = 0.05
FRAG1_COLOR = [1, 0.3, 0.05]
FRAG2_COLOR = [0, 0.629, 0.9]
GT_COLOR = [1,1,0]
SPHERE_COLOR = [0,1,0.1]
SPHERE_COLOR_2 = [0.5,1,0.1]

VOXEL_SIZE = 0.05 # [m]

def iss_keypoints_to_indices(pcd: o3d.geometry.PointCloud,
                             iss_kp: o3d.geometry.PointCloud) -> np.ndarray:
    """
    Map ISS keypoint XYZ locations to nearest-neighbor indices in the original pcd.
    Returns int array of shape [K].
    """
    kpts = np.asarray(iss_kp.points)
    if kpts.size == 0:
        raise ValueError("No ISS keypoints found. Tune ISS parameters or check the point cloud.")

    kdtree = o3d.geometry.KDTreeFlann(pcd)

    idxs = np.empty((kpts.shape[0],), dtype=np.int64)
    for i, xyz in enumerate(kpts):
        _, nn_idx, _ = kdtree.search_knn_vector_3d(xyz, 1)
        idxs[i] = nn_idx[0]

    # remove duplicates (ISS points can map to same nearest input point)
    idxs = np.unique(idxs)
    return idxs

def compose_mat4_from_teaserpp_solution(solution):
    """
    Compose a 4-by-4 matrix from teaserpp solution
    """
    s = solution.scale
    rotR = solution.rotation
    t = solution.translation
    T = np.eye(4)
    T[0:3, 3] = t
    R = np.eye(4)
    R[0:3, 0:3] = rotR
    M = T.dot(R)

    if s == 1:
        M = T.dot(R)
    else:
        S = np.eye(4)
        S[0:3, 0:3] = np.diag([s, s, s])
        M = T.dot(R).dot(S)

    return M


def custom_draw_geometry_load_option(pcds, width=640, height=480):

    vis = o3d.visualization.Visualizer()
    vis.create_window(width=width, height=height)
    for pcd in pcds:
        vis.add_geometry(pcd)
    vis.get_render_option().load_from_json("./render_option.json")
    vis.run()
    vis.destroy_window()


def get_angular_error(R_exp, R_est):
    """
    Calculate angular error
    """
    return abs(np.arccos(min(max(((np.matmul(R_exp.T, R_est)).trace() - 1) / 2, -1.0), 1.0)));


def find_mutually_nn_keypoints(ref_key, test_key, ref, test):
    """
    Use kdtree to find mutually closest keypoints 

    ref_key: reference keypoints (source)
    test_key: test keypoints (target)
    ref: reference feature (source feature)
    test: test feature (target feature)
    """
    ref_features = ref.data.T
    test_features = test.data.T
    ref_keypoints = np.asarray(ref_key.points)
    test_keypoints = np.asarray(test_key.points)
    n_samples = test_features.shape[0]

    ref_tree = KDTree(ref_features)
    test_tree = KDTree(test.data.T)
    test_NN_idx = ref_tree.query(test_features, return_distance=False)
    ref_NN_idx = test_tree.query(ref_features, return_distance=False)

    # find mutually closest points
    ref_match_idx = np.nonzero(
        np.arange(n_samples) == np.squeeze(test_NN_idx[ref_NN_idx])
    )[0]
    ref_matched_keypoints = ref_keypoints[ref_match_idx]
    test_matched_keypoints = test_keypoints[ref_NN_idx[ref_match_idx]]

    return np.transpose(ref_matched_keypoints), np.transpose(test_matched_keypoints)

def execute_teaser_global_registration(source, target):
    """
    Use TEASER++ to perform global registration
    """
    # Prepare TEASER++ Solver
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1
    solver_params.noise_bound = NOISE_BOUND
    solver_params.estimate_scaling = False
    solver_params.rotation_estimation_algorithm = (
        teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    )
    solver_params.rotation_gnc_factor = 1.4
    solver_params.rotation_max_iterations = 100
    solver_params.rotation_cost_threshold = 1e-12
    print("TEASER++ Parameters are:", solver_params)
    teaserpp_solver = teaserpp_python.RobustRegistrationSolver(solver_params)

    # Solve with TEASER++
    start = timer()
    teaserpp_solver.solve(source, target)
    end = timer()
    est_solution = teaserpp_solver.getSolution()
    print(est_solution)
    est_mat = compose_mat4_from_teaserpp_solution(est_solution)
    max_clique = teaserpp_solver.getTranslationInliersMap()
    print("Max clique size:", len(max_clique))
    final_inliers = teaserpp_solver.getTranslationInliers()
    return est_mat, max_clique, end - start


if __name__ == "__main__":
    print("==================================================")
    print("        TEASER++ Python registration example      ")
    print("==================================================")
    # Parse configuration
    config_arguments, unparsed_arguments = config.get_config()

    # If we have unparsed arguments, print usage and exit
    if len(unparsed_arguments) > 0:
        config.print_usage()
        exit(1)

    _dir_path = os.path.dirname(os.path.realpath(__file__))
    creality_path = os.path.join(_dir_path, "..", "..", "Scans", "Creality")
    zivid_path = os.path.join(_dir_path, "..", "..", "Scans", "Zivid")
    bunny_path = os.path.join(_dir_path, "..", "..", "bunny")

    pcd1_name = "cloud_bin_0"
    pcd2_name = "cloud_bin_1"

    pcd1_dir = os.path.join(zivid_path, pcd1_name) + ".ply"
    pcd2_dir = os.path.join(zivid_path, pcd2_name) + ".ply"

    # Load ply files
    pcd1 = o3d.io.read_point_cloud(pcd1_dir)
    pcd2 = o3d.io.read_point_cloud(pcd2_dir)

    # Compute key points
    # tic = time.time()
    # pcd1_key = o3d.geometry.keypoint.compute_iss_keypoints(pcd1)
    # pcd2_key = o3d.geometry.keypoint.compute_iss_keypoints(pcd2)
    # toc = 1000 * (time.time() - tic)
    # print("ISS Computation took {:.0f} [ms]".format(toc))

    # pcd1_key_idx = iss_keypoints_to_indices(pcd1, pcd1_key)
    # pcd2_key_idx = iss_keypoints_to_indices(pcd2, pcd2_key)

    # with open(pcd1_dir + '_keypoints.txt', 'w') as f:
    #     f.write('\n'.join(str(idx) for idx in pcd1_key_idx))
    # with open(pcd2_dir + '_keypoints.txt', 'w') as f:
    #     f.write('\n'.join(str(idx) for idx in pcd2_key_idx))
    # load keypoints
    pcd1_key_idx = np.genfromtxt(
        os.path.join(zivid_path, pcd1_name) + "_keypoints.txt"
    )
    pcd2_key_idx = np.genfromtxt(
        os.path.join(zivid_path, pcd2_name) + "_keypoints.txt"
    )

    pcd1_keypoints = np.asarray(pcd1.points)[pcd1_key_idx.astype(int), :]
    pcd2_keypoints = np.asarray(pcd2.points)[pcd2_key_idx.astype(int), :]

    # Save as open3d point clouds
    pcd1_key = o3d.geometry.PointCloud()
    pcd1_key.points = o3d.utility.Vector3dVector(pcd1_keypoints)

    pcd2_key = o3d.geometry.PointCloud()
    pcd2_key.points = o3d.utility.Vector3dVector(pcd2_keypoints)

    # load descriptors
    frag1_desc_file = os.path.join(
        pcd1_dir + "_0.150000_16_1.750000_3DSmoothNet.npz"
    )
    frag1_desc = np.load(frag1_desc_file)
    pcd1_desc = frag1_desc["data"]

    frag2_desc_file = os.path.join(
        pcd2_dir + "_0.150000_16_1.750000_3DSmoothNet.npz"
    )
    frag2_desc = np.load(frag2_desc_file)
    pcd2_desc = frag2_desc["data"]

    # save as o3d feature
    frag1 = o3d.pipelines.registration.Feature()
    frag1.data = pcd1_desc.T

    frag2 = o3d.pipelines.registration.Feature()
    frag2.data = pcd2_desc.T

    ref_matched_key, test_matched_key = find_mutually_nn_keypoints(
        pcd2_key, pcd1_key, frag2, frag1 
    )
    ref_matched_key = np.squeeze(ref_matched_key)
    test_matched_key = np.squeeze(test_matched_key)

    print("Descriptors")
    print(frag1)
    print(frag2)
    print("Key points")
    print(pcd1_key)
    print(pcd2_key)
    print("Mathced keys")
    print(ref_matched_key.shape)
    print(test_matched_key.shape)

    # est_mat, max_clique, time = execute_teaser_global_registration(ref_matched_key, test_matched_key)
    # print(est_mat)
    # # ## Visualization
    # pcd1_viz = copy.deepcopy(pcd1)
    # pcd2_viz = copy.deepcopy(pcd2)

    # pcd1_viz.estimate_normals()
    # pcd2_viz.estimate_normals()

    # pcd1_viz.paint_uniform_color(FRAG1_COLOR)
    # pcd2_viz.paint_uniform_color(FRAG2_COLOR)

    # vis_list = [pcd2_viz]
    # print(pcd2_viz)
    # custom_draw_geometry_load_option(vis_list, width=680, height=480)

    # pcd2_viz.transform(est_mat)
    # print(pcd2_viz)
    # vis_list = [pcd2_viz]

    # custom_draw_geometry_load_option(vis_list, width=680, height=480)

