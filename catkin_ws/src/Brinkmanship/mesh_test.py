import numpy as np
import std_msgs
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import rospy
import ros_numpy
# import utils
import pyvista as pv
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import KMeans
import copy
class CloudSubscriber:
    def __init__(self, tvec, rvec):
        self.cloud_sub = rospy.Subscriber('/points', PointCloud2, self.cloud_sub_callback)
        self.cloud_pub = rospy.Publisher('/processed_cloud_py3', PointCloud2, queue_size = 10)
        self.cloud_data = None
        self.H = self.get_transformation_matrix(tvec, rvec)
        return

    def cloud_sub_callback(self, msg):
        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
        xyz = xyz[xyz[:,2] > 0.1]
        print('One Step')
        self.cloud_data = self.transform_cloud(xyz, self.H)
        self.cloud_data = self.cloud_data[self.cloud_data[:,2] < 0.5]
        self.cloud_data = self.cloud_data[abs(self.cloud_data[:,0]) < 0.6]

        # grid = self.quantize(self.cloud_data.copy())
        # temp = self.compute_averages(grid)
        temp = self.cloud_data.copy()

        y = temp[:,1].copy()
        z = temp[:,2].copy()
        temp[:,1] = z
        temp[:,2] = y
        self.publish_processed_cloud(self.cloud_data)

        pv_cloud = pv.PolyData(temp).clean(tolerance=0.033, absolute=False)

        pv_cloud.plot()

        surf = pv_cloud.delaunay_2d(alpha=1.0)
        surf.plot(show_edges=True, scalars=[abs(surf.cell_normals[:, 2]) < 0.85])
        return    

    def quantize(self, points):
        n_iter = 15
        z_arr = [None]*n_iter
        grid = [[None]*n_iter for num in range(n_iter) ]

        z_max = np.amax(points[:,2])
        z_min = np.amin(points[:,2])
        z_step = (z_max - z_min)/n_iter

        for i in range(n_iter):
            z_arr[i] = points[points[:,2] < (z_min + (i+1)*z_step)]
            x_min = np.amin(z_arr[i][:,0])
            x_max = np.amax(z_arr[i][:,0])
            x_step = (x_max-x_min)/n_iter
            for j in range(n_iter):
                grid[i][j] = z_arr[i][z_arr[i][:,0] < (x_min + (j+1)*x_step)]
                z_arr[i] = z_arr[i][z_arr[i][:,0] >= (x_min + (j+1)*x_step)]
            points = points[points[:,2] >= (z_min+(i+1)*z_step)]
        return grid

    def compute_averages(self, grid):
        mesh = [np.mean(element, axis = 0) for x in grid for element in x]
        mesh = np.array(mesh)
        return mesh

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

    def get_transformation_matrix(self, tvec, quat):
        T = np.ones((4,4)).astype('float')
        r = R.from_quat(quat)
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

if __name__ == '__main__':
    translation = [0, 0.13, -0.18]
    rotation = [-0.3583641, 0, 0, 0.9335819]
    rospy.init_node('Cloud_Processor')
    cs = CloudSubscriber(translation, rotation)
    rospy.spin()