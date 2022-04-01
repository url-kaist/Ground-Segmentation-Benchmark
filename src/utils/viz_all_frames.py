import numpy as np
import open3d as o3d
import os
import time

######################## Set parameters #########################
alg = "patchwork"
seq = "04"
kittiraw_dir = "/data/SemanticKITTI/"       #"/media/jeewon/Elements/semantic_kitti_raw/"
label_csv_dir = "/data/"                     #"/media/jeewon/Elements/data/"
#################################################################

kittiraw_dir = kittiraw_dir+seq+"/velodyne/"
velo_list = os.listdir(kittiraw_dir)

frame_num_list = []
for file in velo_list:
    if file.count(".") == 1:
        name = file.split('.')[0]
        frame_num_list.append(name)
    else:
        for k in range(len(file)-1,0,-1):
            if file[k]=='.':
                frame_num_list.append(file[:k])
                break
frame_num_list = sorted(frame_num_list)

viz = o3d.visualization.Visualizer()
pc = o3d.geometry.PointCloud()
viz.create_window()
viz.add_geometry(pc)

for frame_num in frame_num_list:
    velo_path=kittiraw_dir+frame_num+".bin"
    label_path = label_csv_dir+alg+"_ground_labels/"+seq+"/"+frame_num+".csv"

    pcd = ((np.fromfile(velo_path, dtype=np.float32)).reshape(-1, 4))[:, 0:3]
    points = np.asarray(pcd)
    labels = np.loadtxt(label_path,dtype=np.int)

    colors = []
    for i in range(len(labels)):
        if labels[i] == 1:
            colors.append([0,1,0])
        else:
            colors.append([0,0,0])

    pc.points=o3d.utility.Vector3dVector(points)
    pc.colors=o3d.utility.Vector3dVector(colors)
    viz.add_geometry(pc)
    viz.update_geometry(pc)
    viz.poll_events()
    viz.update_renderer()
    time.sleep(0.1)

viz.destroy_window()