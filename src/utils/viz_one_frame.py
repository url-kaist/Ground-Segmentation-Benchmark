import numpy as np
import open3d as o3d
import os.path

cwd = os.getcwd()

######################## Set parameters #########################
alg = "patchwork"
seq = "04"
kittiraw_dir = "/home/user/data/SemanticKITTI/"       # absolute path of KITTI dataset folder
label_csv_dir = cwd + "/ground_labels_04/"            # modify to absolute path of data if you use other seq
frame_num ="000010"
#################################################################

velo_path=kittiraw_dir+seq+"/velodyne/"+frame_num+".bin"
label_path = label_csv_dir+alg+"_ground_labels/"+seq+"/"+frame_num+".csv"

pc = o3d.geometry.PointCloud()
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

o3d.visualization.draw_geometries([pc])

