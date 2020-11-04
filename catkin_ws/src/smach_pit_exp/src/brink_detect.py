import numpy as np
import std_msgs
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs.point_cloud2 as pcl2
import rospy
import ros_numpy
# import utils
import pyvista as pv
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import KMeans
import copy
from smach_pit_exp.msg import euler_list

def get_mesh(cloud_array):
        pv_cloud = pv.PolyData(cloud_array).clean(tolerance=0.033, absolute=False)
        return pv_cloud.delaunay_2d()

def get_cell_centers(mesh):                                 # Note: Confirming if mesh.n_faces = cell_centers.shape[0] is a nice sanity check
    '''
    Input:  Pyvista triangle mesh object
    Output: Unordered numpy array of containing the positions of cell centers
    '''
    vtk_list = mesh.faces
    vtk_list = vtk_list.reshape(-1,4)[:,1:4].ravel()
    cell_verts = np.array(mesh.points[vtk_list])
    cell_verts_list = np.split(cell_verts, cell_verts.shape[0]/3, axis = 0)
    cell_centers = [np.mean(x, axis = 1) for x in cell_verts_list]
    return cell_centers


def get_mesh_center(mesh):
    '''
    Input:  Pyvista triangle mesh
    Output: Cooridinates of the center of mesh
    '''
    return mesh.center_of_mass()

def extract_boundary(mesh):
    '''
    Input:  Pyvista triangle mesh object
    Output: Edges forming the mesh boundary
            Vertices of the boundary edges
    '''
    edges = mesh.extract_feature_edges(boundary_edges=True,
                           feature_edges=False,
                           manifold_edges=False)
    boundary_edges_verts = edges.points
    return edges, boundary_edges_verts

def visualize_mesh(mesh, points = None, edges = None, color_pt = None):
    '''
    Function for non-blocking visualization of a triangle mesh

    Input:  Pyvista triangle mesh object
            Points: Numpy array containing points to be visualized
            Edges: Pyvista object containing edge information (obtained from extract_edges)
    '''
    p.add_mesh(mesh, show_edges=True, color=True, name = 'mesh')
    if edges is not None:
        p.add_mesh(edges, color="blue", line_width=5, name = 'edges')
    if points is not None:
        p.add_points(np.array(points), color = color_pt, name = 'points', point_size = 10)
    p.show(auto_close = False, interactive_update = True)
    p.update()
    return

# def get_distance(boundary_points):
#     pts = np.array(boundary_points)
#     pts = pts[pts[:,1].argsort()]
#     print(pts)
#     bins = np.array_split(pts, 10, axis = 0)
#     print(bins)
#     return

def get_farthest_points(boundary_points):
    points = boundary_points[boundary_points[:,2].argsort()]
    # print(points)
    points = points[:points.shape[0]//2 + 5]
    return points


class CloudSubscriber:
    def __init__(self, tvec, rvec):
        self.cloud_sub = rospy.Subscriber('/points', PointCloud2, self.cloud_sub_callback)
        self.imu_sub = rospy.Subscriber('/apnapioneer3at/inertial_unit/roll_pitch_yaw', Imu, self.imu_sub_cb)
        self.cloud_pub = rospy.Publisher('/processed_cloud_py3', PointCloud2, queue_size = 10)
        self.imu_euler_pub = rospy.Publisher('/imu_euler_angles', euler_list, queue_size = 10)
        self.alert_pub = rospy.Publisher('/edge_alert', Bool, queue_size = 10)

        self.alert_bool = False
        self.cloud_data = None
        self.imu_euler_object = euler_list()
        self.h = std_msgs.msg.Header()
        self.tvec = tvec
        self.rvec = rvec
        self.imu_quat = None
        self.H = self.get_transformation_matrix(tvec, rvec)
        return

    def cloud_sub_callback(self, msg):
        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
        xyz = xyz[xyz[:,2] > 0.5]
        xyz = xyz[xyz[:,2] < 1]
        print('One Step')
        self.H = self.get_transformation_matrix(self.tvec, self.rvec)
        self.cloud_data = self.transform_cloud(xyz, self.H)

        self.mesh = self.get_mesh(self.cloud_data)
        edges, boundary_points = extract_boundary(self.mesh)
        pts = get_farthest_points(boundary_points)
        print(np.mean(pts[:,2]))
        if (np.mean(pts[:,2]) > -0.9*1.4):
            c = 'red'
        else:
            c = 'green'
        visualize_mesh(self.mesh, points = pts, edges = edges, color_pt = c)
        if self.cloud_data.shape[0] < 250000:
            self.alert_bool = True
        else:
            self.alert_bool = False

        # if self.near_edge(self.get_mesh(self.cloud_data)):
        self.publish_stop_signal()

        # grid = self.quantize(self.cloud_data.copy())
        # temp = self.compute_averages(grid)
        # y = temp[:,1].copy()
        # z = temp[:,2].copy()
        # temp[:,1] = z
        # temp[:,2] = y
        self.publish_processed_cloud(self.cloud_data)
        return
        
    def get_mesh(self, cloud_array):
        pv_cloud = pv.PolyData(cloud_array).clean(tolerance=0.033, absolute=False)
        
        return pv_cloud.delaunay_2d()

    def near_edge(self, surf):
        # Check how many cells in front of rover are untraversable
        # TODO: Don't hard-code closeness threshold here
        # print(surf.points)
        # close_mesh = surf.clip('-z', (0, 0, 0.2)) # 0.1 is 2x twist magnitude, plus 0.1 for near edge of point cloud
        unsafe_cells = [abs(surf.cell_normals[:,1]) < 0.93]
        danger_rating = np.sum(unsafe_cells) / surf.n_points
        return danger_rating > 0.2 


    def publish_processed_cloud(self, cloud_array):
        '''
        Publishes pointcloud w.r.t base_link
        '''
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        cloud = pcl2.create_cloud_xyz32(header,cloud_array)
        self.cloud_pub.publish(cloud)
        return

    def publish_stop_signal(self):
        self.alert_pub.publish(self.alert_bool)
        return

    def get_transformation_matrix(self, tvec, quat):
        T = np.ones((4,4)).astype('float')
        r = R.from_quat(quat)
        if self.imu_quat is not None:
            # print('Adding rotation')
            # print()
            r_imu = R.from_euler('zyx', [[-self.imu_euler_object.roll, 0, 180 + self.imu_euler_object.pitch]], degrees = 'True')
            # r_imu = R.from_quat(self.imu_quat)
            R_MAT = r_imu.as_matrix()
            T[:3, :3] = R_MAT@r.as_matrix()
            # print('yes', R_MAT)
        else:
            T[:3, :3] = r.as_matrix()
        T[:3, -1] = tvec
        return T

    def transform_cloud(self, cloud_array, transformation_matrix = np.eye(4)):
        '''
        Transforms all points in a cloud given a transformation matrix
        Input: Nx3 array of points in cloud
        Output: Nx3 array of transformed points
        '''
        if cloud_array.shape[0] > cloud_array.shape[1]:
            homogenious_coordinates = np.vstack((cloud_array.T, np.ones(cloud_array.shape[0])))
        else:
            homogenious_coordinates = np.vstack((cloud_array, np.ones(cloud_array.shape[1])))

        transformed_cloud = np.matmul(transformation_matrix, homogenious_coordinates)
        transformed_xyz = transformed_cloud[:3,:].T
        return transformed_xyz

    def imu_sub_cb(self, msg):
        self.imu_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = R.from_quat(self.imu_quat)
        angles = r.as_euler('zyx', degrees=True)
        self.h.stamp = rospy.Time.now()
        self.imu_euler_object.header = self.h
        self.imu_euler_object.roll = angles[1]
        self.imu_euler_object.pitch = angles[2]
        self.imu_euler_object.yaw = angles[0]
        self.imu_euler_pub.publish(self.imu_euler_object)
        return

# def get_farthest_points(edges, boundary_points):
#     edge_array = np.zeros((edges.n_cells*2, 3)).astype('float')
#     # edge_list = np.array([np.array(edges.extract_cells(i).points) for i in range(edges.n_cells)])
#     for i in range(edges.n_cells):
#         edge_array[2*i:2*i+2,:] = np.array(edges.extract_cells(i).points)
    
#     print(edge_array)

#     max_pts = boundary_points[boundary_points[:,1] == np.amax(boundary_points[:,1])]
#     min_pts = boundary_points[boundary_points[:,0] == np.amin(boundary_points[:,0])]
#     left_corner = min_pts[min_pts[:,1] == np.amax(min_pts[:,1])][0]
#     right_corner = max_pts[max_pts[:,1] == np.amax(max_pts[:,1])][0]
#     # print(left_corner.shape)
#     not_found = True
#     far_edges = [left_corner]
#     while(not_found):
#         idx = (edge_array == far_edges[-1]).all(axis = 1).nonzero()[0]
#         if idx[0]%2 == 0:
#             neighbour1 = edge_array[idx+1]
#         else:
#             neighbour1 = edge_array[idx-1]
#         if idx[1]%2 == 0:
#             neighbour2 = edge_array[idx+1]
#         else:
#             neighbour2 = edge_array[idx-1]



#     return

# def get_best_nbr(pt, n1, n2):
#     s1 = n1[2] - n[1]
#     return



if __name__ == '__main__':
    rospy.init_node('Cloud_Processor')
    p = pv.Plotter()
    translation = [0, 0.13, -0.23]
    rotation = [-0.3583641, 0, 0, 0.9335819]
    cs = CloudSubscriber(translation, rotation)
    rospy.spin()

    # for i in range(1):
    #     x = np.arange(40)
    #     y = np.arange(40)
    #     xv, yv = np.meshgrid(x, y, sparse=False, indexing='ij')
    #     # points = np.vstack((xv.flatten(), yv.flatten(), np.random.randint(0,4, size = xv.flatten().shape[0]))).T
    #     points = np.vstack((xv.flatten(), yv.flatten(), np.zeros((xv.flatten().shape[0])))).T

    #     mesh = get_mesh(points)
    #     get_cell_centers(mesh)    
    #     edges, boundary_points = extract_boundary(mesh)
    #     # import ipdb; ipdb.set_trace()
    #     pts = get_farthest_points(boundary_points)
    #     visualize_mesh(mesh, points = pts, edges = edges)








    # edge_dict = {}
    # for i in range(edges.n_cells):
    #     pts = np.array(edges.extract_cells(i).points)
        
    #     if pts[0] in edge_dict.keys():
    #         if pts[1] in edge_dict[pts[0]]:
    #             pass
    #         else:
    #             edge_dict[pts[0]] = edge_dict[pts[0]].append(list(pts[1]))
    #     else:
    #         edge_dict[pts[0]] = list(pts[1])
    
    # print(edge_dict)
