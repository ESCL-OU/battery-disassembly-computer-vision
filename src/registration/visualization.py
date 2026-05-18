import os
import copy
import time
import line_mesh
import numpy as np
import open3d as o3d
from sklearn.neighbors import KDTree
from timeit import default_timer as timer

import bench_utils
import teaserpp_python

VISUALIZE = True
NOISE_BOUND = 0.05
FRAG1_COLOR = [1, 0.3, 0.05]
FRAG2_COLOR = [0, 0.629, 0.9]
GT_COLOR = [1,1,0]
SPHERE_COLOR = [0,1,0.1]
SPHERE_COLOR_2 = [0.5,1,0.1]

def draw_registration_result(target_corrs_points, source_corrs_points, frag1, frag2, transformation, max_clique, gt=None, gt_inliers=None):

    frag1_temp = copy.deepcopy(frag1)
    frag2_temp = copy.deepcopy(frag2)

    frag1_temp.paint_uniform_color(FRAG1_COLOR)
    frag2_temp.paint_uniform_color(FRAG2_COLOR)

    frag1_temp.estimate_normals()
    frag2_temp.estimate_normals()

    frag2_temp.transform(transformation)

    inlier_spheres = []
    if max_clique is None:
        inlier_spheres = []
        target_inlier_points = np.zeros([0, 3])
    else:
        target_inlier_points = np.zeros([len(max_clique), 3])
        inlier_count = 0
        for i in range(target_corrs_points.shape[0]):
            if i in max_clique:
                target_inlier_points[inlier_count,:] = target_corrs_points[i,:]
                inlier_count+=1
        inlier_spheres = create_spheres(target_inlier_points, radius=0.3)

    vis_list = [frag1_temp, frag2_temp]
    if gt is not None:
        frag2_gt_temp = copy.deepcopy(frag2)
        frag2_gt_temp.paint_uniform_color(GT_COLOR)
        frag2_gt_temp.transform(gt)
        frag2_gt_temp.estimate_normals()

        gt_vis_list = [frag1_temp, frag2_gt_temp]
        # add gt inliers
        if gt_inliers is not None:
            gt_inlier_set = []
            gt_target_inlier_points = np.zeros([len(gt_inliers), 3])
            inlier_count = 0
            for i in range(target_corrs_points.shape[0]):
                if i in gt_inliers:
                    gt_inlier_set.append((i,i))
                    gt_target_inlier_points[inlier_count,:] = target_corrs_points[i,:]
                    inlier_count+=1

            gt_spheres = create_spheres(gt_target_inlier_points, radius=0.05)
            gt_vis_list.extend(gt_spheres)

        # ground truth alignment
        #print("Now showing ground truth alignment ...")
        #custom_draw_geometry_load_option(gt_vis_list)
        
        # TEASER++ alignment
        print("Now showing TEASER++ alignment ...")
        tpp_inlier_spheres = create_spheres(target_inlier_points, radius=0.04)
        vis_list.extend(tpp_inlier_spheres)
        custom_draw_geometry_load_option(vis_list, width=680, height=480)

        # together
        #print("Now showing GT & TEASER++ alignments ...")
        total_vis_list = vis_list
        total_vis_list.extend(gt_vis_list) 
        #custom_draw_geometry_load_option(total_vis_list)
