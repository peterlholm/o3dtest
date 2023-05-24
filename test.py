"test picture generations"
from typing import Union
from pathlib import Path
import open3d as o3d
import numpy as np

_DEBUG = True


def pcl_info(pcl_s):
    "Write information about stl file"
    if isinstance(pcl_s, Union[Path, str]):
        if not Path(pcl_s).exists():
            print("File not found", pcl_info)
            return
        print("PCL_INFO: File", pcl_s)
        pcl = o3d.io.read_point_cloud(pcl_s)
    else:
        pcl = pcl_s
        print("PCL_INFO")
    print("Number of points", len(pcl.points))
    print("get_axis_aligned_bounding_box", pcl.get_axis_aligned_bounding_box())
    print("get_center", pcl.get_center())
    print("get_max_bound", pcl.get_max_bound())
    print("get_oriented_bounding_box", pcl.get_oriented_bounding_box())
    print("has_colors", pcl.has_colors())
    print("has_normals", pcl.has_normals())


def pcl2png(infilename):
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

    o3d.visualization.draw_geometries([pcd,axis_p],'navn', 500, 500, point_show_normal=True, lookat=np.asarray([0,0,0]), up=[-1,0,0], front=[0.5,0.5,-5], zoom=1)# , point_show_normal=True, lookat=[0,0,20], up=[0,1,0],front=[0,0,0],zoom=0.5)


def pcl2pic(pcd, outfile):
    "Make a jpg file from pcl"
    zoom = 1.0
    axis = True
    obj_center = pcd.get_center()
    if _DEBUG:
        pcl_info(pcd)
        arr = np.asarray(pcd.points)
        #print ("shape", arr.shape)
        amin = np.min(arr, axis=0)
        amax = np.max(arr, axis=0)
        print("Object limits min, max, center", amin, amax, obj_center)
    cam_position = obj_center.copy()
    cam_position[2] = CAM_POSITION[2]   #lodret
    # camera position
    diff = 10
    cam_position[0] -= diff

    # if cam=='n':
    #     cam_position[0] += diff
    # elif cam=='e':
    #     cam_position[1] -= diff
    # elif cam=='w':
    #     cam_position[1] += diff
    # elif cam=='s':
    #     cam_position[0] -= diff
    # else:
    #     print("Error in pcl to jpg position")
    #     cam_position[0] += diff

    vis = o3d.visualization.Visualizer()
    res = vis.create_window(visible = _DEBUG, width=PICTURE_SIZE, height=PICTURE_SIZE)
    if not res:
        print("create window result", res)
    vis.add_geometry(pcd)
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
        vis.add_geometry(axis_pcd)

    ctr = vis.get_view_control()
    if ctr is None:
        print("pcl2jpg cant get view_control", vis)
    # fix position
    obj_center =OBJ_CENTER
    cam_position=CAM_POSITION
    if _DEBUG:
        print('object center', obj_center, "cam position:", cam_position, "zoom", zoom)
    ctr.set_zoom(zoom)
    ctr.set_front(cam_position)
    ctr.set_lookat(obj_center)
    ctr.set_up([+10.0, 0, 0])
    opt = vis.get_render_option()
    opt.point_size = 2.0
    #opt.point_color_option.Color = 1
    if _DEBUG:
        vis.run()
    vis.capture_screen_image(str(outfile), do_render=True)
    # if _DEBUG:
    #     img = vis.capture_screen_float_buffer(True)
    #     plt.imshow(np.asarray(img))
    #if not _TEXT:


Infile = Path(__file__).parent / "testdata/1/pointcloud_mirror.ply"

pcl2png(Infile, "out.png")
