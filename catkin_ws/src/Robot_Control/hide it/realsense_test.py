import pyrealsense2 as rs
import numpy as np
import cv2
# from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
# import helper as hp
from mpl_toolkits.mplot3d import Axes3D
# fx = 380.16          # lense focal length
# fy = 380.16
# baseline = 50     # distance in mm between the two cameras
# disparities = 0   # num of disparities to consider
# block = 31          # block size to match
# units = 0.001       # depth units
#
# width: 640, height: 480, ppx: 321.331, ppy: 235.967, fx: 380.16, fy: 380.16, model: 4, coeffs: [0, 0, 0, 0, 0]
#
intrinsic_matrix = np.asarray([[380.16, 0 , 321.331],[0, 380.16, 235.967], [0,0,1]])
extrinsic_matrix1 = np.asarray([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
extrinsic_matrix2 = np.asarray([[1,0,0,0.0499],[0,1,0,0],[0,0,1,0]])
projection_matrix1 = np.matmul(intrinsic_matrix,extrinsic_matrix1)
projection_matrix2 = np.matmul(intrinsic_matrix,extrinsic_matrix2)
points = rs.points()
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
profile = pipeline.start(config)

def triangulate(C1, pts1, C2, pts2):
    # Replace pass by your implementation
    # print(C1.shape, C2.shape)
    A = np.zeros((4,4))
    for i in range(0,pts1.shape[0]):
        A[0,:] = C1[0,:] - pts1[i,0]*C1[2,:]
        A[1,:] = C1[1,:] - pts1[i,1]*C1[2,:]
        A[2,:] = C2[0,:] - pts2[i,0]*C2[2,:]
        A[3,:] = C2[1,:] - pts2[i,1]*C2[2,:]
        # print(A)
        # tt
        U,S,VT = np.linalg.svd(A)
        point = VT[-1,:]
        point = point/point[-1]
        # tt
        if i == 0:
            points = point
        else:
            points = np.vstack((points,point))

    # print(points)
    x1_hat = np.matmul(C1,points.T)
    x2_hat = np.matmul(C2,points.T)
    x1_hat = x1_hat/x1_hat[-1]
    x2_hat = x2_hat/x2_hat[-1]
    # print(x1_hat,x2_hat)
    error_1 = pts1 - x1_hat[:-1,:].T
    error_2 = pts2 - x2_hat[:-1,:].T
    reprojection_error = np.sum(error_1**2 + error_2**2)
    # print(reprojection_error)
    points = points[:,:-1]
    return points,reprojection_error

try:
    count = 0
    fig = plt.figure()
    ax = Axes3D(fig)
    # ax.set(xlim=(-0.5,0.5), ylim=(-0.5,0.5), zlim = (-0.5,0.5))
    plt.ion()

    while True:
        ir1 = profile.get_stream(rs.stream.infrared,1).as_video_stream_profile().get_intrinsics()
        ir2 = profile.get_stream(rs.stream.infrared,2).as_video_stream_profile()
        # print(ir1.get_extrinsics_to(ir2))
        # print(ir1)
        # print(ir2)
        frames = pipeline.wait_for_frames()
        nir_lf_frame = frames.get_infrared_frame(1)
        nir_rg_frame = frames.get_infrared_frame(2)


        if not nir_lf_frame or not nir_rg_frame:
            continue
        #
        nir_rg_image = np.asanyarray(nir_rg_frame.get_data())
        nir_lf_image = np.asanyarray(nir_lf_frame.get_data())
        # nir_rg_image = cv2.imread("right_image" + str(count) + ".jpg")
        # nir_lf_image = cv2.imread("left_image" + str(count) + ".jpg")
        orb = cv2.ORB_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = orb.detectAndCompute(nir_rg_image,None)
        kp2, des2 = orb.detectAndCompute(nir_lf_image,None)
        kp1 = np.asarray(kp1)
        kp2 = np.asarray(kp2)
        # print(dir(kp1[0]))
        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Match descriptors.
        matches = bf.match(des1,des2)
        # print(len(matches))
        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)
        # import ipdb; ipdb.set_trace()
        # matches = matches[0:2]
        points1 = []
        points2 = []
        matches = matches[:30]
        for match in matches:
            # print(kp1[match.queryIdx].pt)
            points1.append(kp1[match.queryIdx].pt)
            points2.append(kp2[match.trainIdx].pt)

        points1 = np.asarray(points1).astype('int')
        points2 = np.asarray(points2).astype('int')
        # print(points2)

        # print(idx.imgIdx, idx.queryIdx)
        # Draw first 10 matches.
        # print("---------------------")
        # for i in range(points1.shape[0]):
        #     img3 = cv2.circle(nir_lf_image, tuple(points1[i]), 2, (0,250,0),1)

        points,reprojection_error = triangulate(projection_matrix1,np.asarray(points1), projection_matrix2, np.asarray(points2))
        image=np.hstack((nir_lf_image,nir_rg_image))
        img4= cv2.drawMatches(nir_rg_image,kp1,nir_lf_image,kp2,matches, flags=2, outImg = image)
        # import ipdb; ipdb.set_trace()
        # print(len(matches))
        print(points)

        cv2.namedWindow('NIR images (left, right)', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('IR Example', img3)
        cv2.imshow('IR Example2', img4)
        ax.scatter(points[:,0],points[:,1],points[:,2], marker='.')
        plt.show()
        plt.pause(0.007)
        # plt.close()
        key = cv2.waitKey(1)
        count += 1
        # ax.clear()
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break


finally:
    pipeline.stop()
