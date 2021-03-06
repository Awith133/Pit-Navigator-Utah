import numpy as np
import std_msgs
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import rospy
import tf
import ros_numpy

class CloudSubscriber:
    def __init__(self, tvec, rvec):
        self.cloud_sub = rospy.Subscriber('/points', PointCloud2, self.cloud_sub_callback)
        self.cloud_pub = rospy.Publisher('/processed_cloud', PointCloud2, queue_size = 10)
        self.alert_pub = rospy.Publisher('/edge_alert', String, queue_size = 10)
<<<<<<< HEAD
<<<<<<< HEAD
        self.alert_bool = True
=======
        self.alert_bool = False
>>>>>>> 3dc8bfe06b4db08c5f7260cd56aafa3a2dbfc1e3
=======
        self.alert_bool = False
>>>>>>> 320633c487afe1f01e0afb2cbaf1c39cc4e95fa4
        self.cloud_data = None
        self.H = self.get_transformation_matrix(tvec, rvec)
        return

    def cloud_sub_callback(self, msg):
        xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
<<<<<<< HEAD
<<<<<<< HEAD
        print('Size of point cloud = {}'.format(xyz.shape))
        print('Max Z value = {}'.format(np.max(xyz[:,2])))
        print('Max Y value = {}'.format(np.max(xyz[:,1])))
        print('Max X value = {}'.format(np.max(xyz[:,0])))
        xyz = xyz[xyz[:,2] < 1]
        self.cloud_data = self.transform_cloud(xyz, self.H)
        if self.cloud_data.shape[0] < 20000:
=======
=======
>>>>>>> 320633c487afe1f01e0afb2cbaf1c39cc4e95fa4
        xyz = xyz[xyz[:,2] < 0.4]
        # print('Size of point cloud = {}'.format(xyz.shape))
        self.cloud_data = self.transform_cloud(xyz, self.H)
        if self.cloud_data.shape[0] < 250000:
<<<<<<< HEAD
>>>>>>> 3dc8bfe06b4db08c5f7260cd56aafa3a2dbfc1e3
=======
>>>>>>> 320633c487afe1f01e0afb2cbaf1c39cc4e95fa4
            self.publish_stop_signal()
            self.alert_bool = True
        else:
            self.alert_bool = False
        self.publish_processed_cloud(self.cloud_data)
        return    

    def publish_stop_signal(self):
        if not self.alert_bool:
<<<<<<< HEAD
<<<<<<< HEAD
            self.alert_pub('Close to pit edge')
=======
            self.alert_pub.publish('Close to pit edge')
>>>>>>> 3dc8bfe06b4db08c5f7260cd56aafa3a2dbfc1e3
=======
            self.alert_pub.publish('Close to pit edge')
>>>>>>> 320633c487afe1f01e0afb2cbaf1c39cc4e95fa4
        else:
            pass
        return

    def publish_processed_cloud(self, cloud_array):
        '''
        Publishes pointcloud w.r.t base_link
        '''
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
<<<<<<< HEAD
<<<<<<< HEAD
        header.frame_id = 'apnapioneer3at/MultiSense_S21_meta_range_finder'
=======
        header.frame_id = 'base_link'
>>>>>>> 3dc8bfe06b4db08c5f7260cd56aafa3a2dbfc1e3
=======
        header.frame_id = 'base_link'
>>>>>>> 320633c487afe1f01e0afb2cbaf1c39cc4e95fa4
        cloud = pcl2.create_cloud_xyz32(header,cloud_array)
        self.cloud_pub.publish(cloud)
        return

    def get_transformation_matrix(self, tvec, quat):
        euler_angles = tf.transformations.euler_from_quaternion(quat, 'rxyz')
        T = tf.transformations.compose_matrix(angles=euler_angles, translate=tvec)
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