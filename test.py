from pathlib import Path
import open3d as o3d 
import numpy as np

def pcl2png(infilename, outfilename, point_size=2, wand=True):
    "write file with pointcloud to png"
    axis = True
    _black_background = False
    _text = True
    #_wand = True
    if _black_background:
        pointcloud_color = [1, 1, 1]
        background_color = [0, 0, 0]
    else:
        pointcloud_color = [0, 0, 0]
        background_color = [1,1,1]

    camera_zoom = 0.45
    #use('Agg')
    pcd = o3d.io.read_point_cloud(str(infilename))
    #pcd.paint_uniform_color(pointcloud_color)
    axis_p = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

    o3d.visualization.draw_geometries([axis_p],'navn', 1500, 1500, point_show_normal=True, lookat=np.asarray([0,0,20]), up=[0,1,0], front=[0,0,0], zoom=1)# , point_show_normal=True, lookat=[0,0,20], up=[0,1,0],front=[0,0,0],zoom=0.5)



Infile = Path(__file__).parent / "testdata/1/pointcloud_mirror.ply"

pcl2png(Infile, "out.png")
