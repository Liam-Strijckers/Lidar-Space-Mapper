#Liam Strijckers 400179278
#python 3.6.0, open3d 0.9, numpy 1.18.2
import open3d as o3d
import numpy as np

lines = []

print("Testing IO for point cloud ...")
pcd = o3d.io.read_point_cloud("data.xyz",format='xyz')#reads the xyz file and create a point cloud out of the data in the file

print(pcd)
temp = 0

for x in range(20):#creates array to so that the line set can connect the point next to each other in the yz plane and the point in the next yz plane over
    for i in range(512):
        lines.append([i + temp, i + 1 + temp])
    temp += 512

temp = 0
temp2 = 512

for x in range(19):
    for i in range(512):
        lines.append([i + temp, i + temp2 + temp])
    temp += 512


line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))#creates the line sets to connect the planes
o3d.visualization.draw_geometries([line_set])#visualizes the data from line set
#downpcd = pcd.voxel_down_sample(voxel_size=0.05)
#o3d.visualization.draw_geometries([downpcd])
