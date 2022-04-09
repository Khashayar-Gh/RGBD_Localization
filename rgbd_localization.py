import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import pandas as pd
from tools import *
import traceback

img_folder = r'rgb'
depth_folder = r'depth'
rgb_depth_names = pd.read_csv("rgb-depth-synced.txt", header=None, sep=" ")
img_dataset = imgdataset2list(img_folder, rgb_depth_names[1].to_list())
depth_dataset = imgdataset2list(depth_folder, rgb_depth_names[3].to_list())
print('There are {} RGB images'.format(len(img_dataset)))
print('There are {} depth images'.format(len(depth_dataset)))

n = len(img_dataset)
sift = cv.SIFT_create(nfeatures=100)
img_freq = 1  # frequency of image usage
used_imges = np.arange(0, n, img_freq)
number_of_used_imgs = len(used_imges)
fx = 525.0  # focal length x
fy = 525.0  # focal length y
cx = 319.5  # optical center x
cy = 239.5  # optical center y
depth_scale = 5000
R = np.zeros((number_of_used_imgs - 1, 3, 3))
T = np.zeros((number_of_used_imgs - 1, 3, 1))
bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)

k = 0
try:
    for i, i2 in zip(used_imges, used_imges[1:]):
        # sift
        img1 = img_dataset[i]  # queryImage
        img2 = img_dataset[i2]  # trainImage
        keypoints_1, descriptors_1 = sift.detectAndCompute(img1, None)
        keypoints_2, descriptors_2 = sift.detectAndCompute(img2, None)

        matches = bf.match(descriptors_1, descriptors_2)
        matches = sorted(matches, key=lambda x: x.distance)
        good = matches
        src_pts = np.float32([keypoints_1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([keypoints_2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, mask = cv.findHomography(src_pts, dst_pts, cv.LMEDS)
        inliers = [inl for (inl, v) in zip(matches, mask) if v == [1]]
        A = np.zeros((3, len(inliers)))
        B = np.zeros((3, len(inliers)))
        no_depth_data = []
        for j in range(len(inliers)):
            c = keypoints_2[inliers[j].trainIdx].pt
            xp = int(c[0])
            yp = int(c[1])
            if depth_dataset[i2][yp, xp] == 0:
                no_depth_data.append(j)
                continue

            z = depth_dataset[i2][yp, xp] / depth_scale
            x = (xp - cx) * z / fx
            y = -1 * (yp - cy) * z / fy

            A[0, j] = x
            A[1, j] = -y
            A[2, j] = z

            c = keypoints_1[inliers[j].queryIdx].pt
            xp = int(c[0])
            yp = int(c[1])
            if depth_dataset[i][yp, xp] == 0:
                no_depth_data.append(j)
                continue

            z = depth_dataset[i][yp, xp] / depth_scale
            if z < 0:
                print('negative depth')
                print(depth_dataset[i][yp, xp])
                print(i)
            x = (xp - cx) * z / fx
            y = -1 * (yp - cy) * z / fy

            B[0, j] = x
            B[1, j] = -y
            B[2, j] = z

        A2 = np.delete(A, no_depth_data, 1)
        B2 = np.delete(B, no_depth_data, 1)
        A_bar = np.mean(A2, 1).reshape((3, 1))
        A2 = A2 - A_bar
        B_bar = np.mean(B2, 1).reshape((3, 1))
        B2 = B2 - B_bar
        u, _, vh = np.linalg.svd(A @ B.T)
        R[k, :, :] = vh.T @ u.T
        if np.linalg.det(R[k, :, :]) < 0:
            print("det(R) < R, reflection detected!, correcting for it ...")
            vh[2, :] *= -1
            R[k, :, :] = vh.T @ u.T
        T[k, :, :] = B_bar - R[k, :, :] @ A_bar
        k += 1

    print('Done without error')

except Exception as e:
    print(e)
    traceback.print_exc()
    print(i)
    print(i2)
    print(len(matches))
    print(len(inliers))
TrajectoryGT = pd.read_csv("groundtruth.txt", sep=" ", header=2)
start_ind = closest(rgb_depth_names[0][0], TrajectoryGT['timestamp'].to_numpy())
trajectory_rgbd = np.zeros((3, number_of_used_imgs))
X_ii = np.zeros((3, 1))
for level in range(number_of_used_imgs):
    X_0 = Coo_to_level_0(X_ii, level, R, T, TrajectoryGT.iloc[start_ind].to_numpy())
    trajectory_rgbd[:, level] = X_0.reshape(-1)

plt.figure(figsize=(10, 10))
plt.plot(TrajectoryGT['timestamp'], TrajectoryGT['tx'])
rgbd_ts = [float(rgbd_time) for rgbd_time in rgb_depth_names[0].to_numpy()]
rgbd_ts = [rgbd_ts[i] for i in used_imges]
plt.plot(rgbd_ts, trajectory_rgbd[0, :])
plt.legend(['GT', 'SLAM'])
plt.xlabel('Timestamp (s)')
plt.ylabel('x (m)')

plt.figure(figsize=(10, 10))
plt.plot(TrajectoryGT['timestamp'], TrajectoryGT['ty'])
plt.plot(rgbd_ts, trajectory_rgbd[1, :])
plt.legend(['GT', 'SLAM'])
plt.xlabel('Timestamp (s)')
plt.ylabel('y (m)')

plt.figure(figsize=(10, 10))
plt.plot(TrajectoryGT['timestamp'], TrajectoryGT['tz'])
plt.plot(rgbd_ts, trajectory_rgbd[2, :])
plt.legend(['GT', 'SLAM'])
plt.xlabel('Timestamp (s)')
plt.ylabel('z (m)')
plt.show()
