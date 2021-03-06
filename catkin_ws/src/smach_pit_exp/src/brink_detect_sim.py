import numpy as np
import std_msgs
from std_msgs.msg import Bool, Int8
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


min_depth = -0.58
max_depth = -0.25

max_width = 0.54
min_width = -0.54


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
    cell_centers = [np.mean(x, axis = 0) for x in cell_verts_list]
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

def check_position_bool(array, low_bound, high_bound):
    
    return

def split_mesh(mesh):
    '''
    Input:  Pyvista triangle mesh object
    Output: Array of nine Pyvista triangle mesh objects that are subsets of the input mesh, divided by location in XZ plane
    '''
    centers = np.array(get_cell_centers(mesh))

    # TODO: Refactor so this isn't copying the code from get_cell_centers
    vtk_list = mesh.faces
    vtk_list = vtk_list.reshape(-1,4)[:,1:4].ravel()
    cell_verts = np.array(mesh.points[vtk_list])
    cell_verts_list = np.array(np.split(cell_verts, cell_verts.shape[0]/3, axis = 0))


    sorted_args = np.argsort(centers[:,2])
    
    cell_verts_list = cell_verts_list[sorted_args]      # cell vertices sorted along the z-axis
    centers = centers[sorted_args]                      # cell centers sorted along the z-axis

    z_band_1 = cell_verts_list[centers[:,2] > max_depth + (min_depth - max_depth)/3]
    cell_verts_list = cell_verts_list[centers[:,2] <= max_depth + (min_depth - max_depth)/3]

    c1 = centers[centers[:,2] > max_depth + (min_depth - max_depth)/3]
    centers = centers[centers[:,2] <= max_depth + (min_depth - max_depth)/3]
    c2 = centers[centers[:,2] > max_depth + 2*(min_depth - max_depth)/3]
    c3 = centers[centers[:,2] <= max_depth + 2*(min_depth - max_depth)/3]
    
    z_band_2 = cell_verts_list[centers[:,2] > max_depth + 2*(min_depth - max_depth)/3]
    z_band_3 = cell_verts_list[centers[:,2] <= max_depth + 2*(min_depth - max_depth)/3]

    z_bands = [z_band_1, z_band_2, z_band_3]
    cen = [c1, c2, c3]

    grid = []
    for idx, band in enumerate(z_bands):
        centers = cen[idx]
        grid.append(band[centers[:,0] < min_width + (max_width-min_width)/3])
        band = band[centers[:,0] >= min_width + (max_width-min_width)/3]
        centers = centers[centers[:,0] >= min_width + (max_width-min_width)/3]
        grid.append(band[centers[:,0] < min_width + 2*(max_width-min_width)/3])
        grid.append(band[centers[:,0] >= min_width + 2*(max_width-min_width)/3])
    return grid

def visualize_mesh(mesh, points = None, edges = None, color_pt = None):
    '''
    Function for non-blocking visualization of a triangle mesh

    Input:  Pyvista triangle mesh object
            Points: Numpy array containing points to be visualized
            Edges: Pyvista object containing edge information (obtained from extract_edges)
    '''
    p.add_mesh(mesh, scalars = abs(mesh.cell_normals[:,1]) < 0.93, show_edges=True, color=True, name = 'mesh')
    if edges is not None:
        p.add_mesh(edges, color="blue", line_width=5, name = 'edges')
    if points is not None:
        p.add_points(np.array(points), color = color_pt, name = 'points', point_size = 10)
    p.show(auto_close = False, interactive_update = True)
    p.update()
    return

def visualize_grid(mesh, grid = None, color_pt = None):
    p.clear()
    p.add_mesh(mesh, show_edges=True, color=True, name = 'mesh')
    color_pt = ['red', 'green', 'blue']
    if grid is not None:
        for idx, points in enumerate(grid):
            points = points.reshape((-1,3))
            if points.shape[0] == 0:
                print('Missing Index = {}'.format(idx))
                pass
            else:
                p.add_points(np.array(points), name = 'points' + str(idx), color = color_pt[idx%3], point_size = 2*(idx+1))
    p.show(auto_close = False, interactive_update = True)
    p.update()
    return

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
        # self.alert_pub = rospy.Publisher('/edge_alert', Bool, queue_size = 10)                            # USING A DIFFERENT METHOD
        self.alert_pub = rospy.Publisher('/edge_alert', Int8, queue_size = 10)

        # self.alert_bool = False                                                                           # USING A DIFFERENT METHOD
        self.alert_status = 0
        self.cloud_data = None
        self.imu_euler_object = euler_list()
        self.h = std_msgs.msg.Header()
        self.tvec = tvec
        self.rvec = rvec
        self.imu_quat = None
        self.H = self.get_transformation_matrix(tvec, rvec)
        self.flush = True
        return

    def cloud_sub_callback(self, msg):
        self.flush = not self.flush
        self.alert_status = 0
        if self.flush:
            #print('Skipping Data')
            return
        
        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
        xyz = xyz[xyz[:,2] > 0.1]
        xyz = xyz[xyz[:,2] < 0.4]
        if xyz.shape[0] == 0:
            return
        rand_vec = np.random.randint(0, xyz.shape[0], size = xyz.shape[0]//3)
        xyz = xyz[rand_vec]
        print('\n')
        # print('One Step')
        print(xyz.shape)
        self.H = self.get_transformation_matrix(self.tvec, self.rvec)
        self.cloud_data = self.transform_cloud(xyz, self.H)

        self.mesh = self.get_mesh(self.cloud_data)
        edges, boundary_points = extract_boundary(self.mesh)
        #print("Max y val = {}".format(np.amax(boundary_points[:,1])))
        #print("Min y val = {}".format(np.amin(boundary_points[:,1])))
        #print("Min X val = {}".format(np.amin(boundary_points[:,0])))
        #print("Min X val = {}".format(np.amax(boundary_points[:,0])))
        pts = get_farthest_points(boundary_points)

        warn, stop = self.risk(self.mesh)
        if warn:
            self.alert_status = 1
        if stop:
            self.alert_status = 2

        print('Alert Status = {}'.format(self.alert_status))
        # if self.near_brink(self.mesh, 0.9*min_depth): # What is the meaning of this number?
        #     c = 'red'
        # else:
        #     c = 'green'
        # # visualize_mesh(self.mesh, points = pts, edges = edges, color_pt = c)
        
        # if self.near_unsafe_slope(self.mesh) or self.near_brink(self.mesh, 0.9*min_depth):
        #     self.alert_bool = True
        # else:
        #     self.alert_bool = False

        # if self.near_edge(self.get_mesh(self.cloud_data)):
        self.publish_stop_signal()

        self.publish_processed_cloud(self.cloud_data)
        return
        
    def get_mesh(self, cloud_array):
        pv_cloud = pv.PolyData(cloud_array).clean(tolerance=0.033, absolute=False)
        
        return pv_cloud.delaunay_2d(alpha=0.1)

    def risk(self, mesh):
        warn = False
        stop = False
        grid = split_mesh(mesh)
        band_1 = np.vstack((grid[0].reshape((-1, 3)),grid[1].reshape((-1, 3)),grid[2].reshape((-1, 3))))
        # band_2 = np.vstack((grid[3].reshape((-1, 3)),grid[4].reshape((-1, 3)),grid[5].reshape((-1, 3))))
        pv_cloud = pv.PolyData(band_1)       
        near_mesh = pv_cloud.delaunay_2d()
        grid_cell_sizes = [x.shape[0] for x in grid]
        print('Grid cell_sizes = ', grid_cell_sizes)
        
        
        far_points = sum(grid_cell_sizes[-3:])
        intermediate_points = sum(grid_cell_sizes[3:6])
        visualize_mesh(mesh)

        unsafe_cells = [abs(near_mesh.cell_normals[:,1]) < 0.93]
        danger_rating = np.sum(unsafe_cells) / near_mesh.n_cells
        print(danger_rating)
        if danger_rating > 0.7:
            stop = True
            print('Unsafe Slope Detected')
        # if far_points < 30:
        #     warn = True
        if intermediate_points < 50 or far_points < 40:
            stop = True
            print('Brink Detected')
        return warn, stop

    def near_unsafe_slope(self, surf):
        grid = split_mesh(surf)
        print('Size of grid cells = ', [x.shape for x in grid])
        band_1 = np.vstack((grid[0].reshape((-1, 3)),grid[1].reshape((-1, 3)),grid[2].reshape((-1, 3))))
        pv_cloud = pv.PolyData(band_1)       
        near_mesh = pv_cloud.delaunay_2d()
        # visualize_mesh(near_mesh)
        # visualize_grid(surf, grid)
        # Check how many cells in front of rover are untraversable
        unsafe_cells = [abs(near_mesh.cell_normals[:,1]) < 0.93]
        danger_rating = np.sum(unsafe_cells) / near_mesh.n_points
        print('Near unsafe slope : ', danger_rating > 0.2)
        return danger_rating > 0.2 

    def near_brink(self, surf, thresh):
        edges, boundary_points = extract_boundary(surf)
        pts = get_farthest_points(boundary_points)
        print('Near brink : ', np.mean(pts[:, 2]) > thresh)
        print('Mean distance = {}'.format(np.mean(pts[:, 2])))
        print('Threshold = {}'.format(thresh))
        return np.mean(pts[:, 2]) > thresh # This would be easier to understand if it was mean < thresh, but as long as the thresh is negative that won't work

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
        # self.alert_pub.publish(self.alert_bool)               # USING A DIFFERENT METHOD
        self.alert_pub.publish(self.alert_status)
        return

    def get_transformation_matrix(self, tvec, quat):
        T = np.ones((4,4)).astype('float')
        r = R.from_quat(quat)
        if self.imu_quat is not None:
            # print('Adding rotation')
            r_imu = R.from_euler('zyx', [[-self.imu_euler_object.roll, 0, 180 + self.imu_euler_object.pitch]], degrees = 'True')
            R_MAT = r_imu.as_matrix()
            T[:3, :3] = R_MAT@r.as_matrix()
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
        if cloud_array.shape[0] == 0:
            return
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
        # print('IMU Angles = ', angles)
        self.h.stamp = rospy.Time.now()
        self.imu_euler_object.header = self.h
        self.imu_euler_object.roll = angles[1]
        self.imu_euler_object.pitch = angles[2]
        self.imu_euler_object.yaw = angles[0]
        self.imu_euler_pub.publish(self.imu_euler_object)
        return

if __name__ == '__main__':
    rospy.init_node('Cloud_Processor')
    p = pv.Plotter()
    translation = [0, 0.13, -0.23]
    rotation = [-0.3583641, 0, 0, 0.9335819]
    cs = CloudSubscriber(translation, rotation)
    rospy.spin()
