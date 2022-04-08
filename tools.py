import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as Ro


def imgdataset2list(img_folder, img_path_list=None):
    """
    This function would list all images in img_folder or all located at img_path_list
    @param img_folder: the folder in which the dataset is located
    @param img_path_list: addresses of all images
    @return: list of images
    """
    img_data_array = []
    if img_path_list is None:
        for dir1 in os.listdir(img_folder):
            image_path = os.path.join(img_folder, dir1)
            image = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH)
            image = np.array(image)
            img_data_array.append(image)

    else:
        for image_path in img_path_list:
            image = cv2.imread(image_path, cv2.IMREAD_ANYDEPTH)
            image = np.array(image)
            img_data_array.append(image)

    return img_data_array


def imglist2video(imglist, filename='dataset.mp4'):
    """
    Turns a list of images into a video. used for debugging.
    @param imglist: list og images
    @param filename: name of the video
    """
    height, width = imglist[0].shape[:2]
    size = (width, height)
    out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'DIVX'), 15, size)
    for image in imglist:
        out.write(image)
    out.release()


def Coo_transform(X_ii, R_i, T_i):
    R_i = R_i.reshape((3, 3))
    T_i = T_i.reshape((3, 1))

    X_ii = X_ii.reshape((3, 1))
    X_i = R_i @ X_ii + T_i
    return X_i


def Coo_to_level_0(X_ii, i, R, T, first_Coo):
    while i != 0:
        X_ii = Coo_transform(X_ii, R[i - 1, :, :], T[i - 1, :, :])
        i = i - 1

    r = Ro.from_quat(first_Coo[-4:])
    R_f = r.as_matrix()
    X_ii = Coo_transform(X_ii, R_f, first_Coo[1:4])
    return X_ii


def closest (num, arr):
    curr = arr[0]
    curr_ind = 0
    for index in range (len (arr)):
        if abs (num - arr[index]) < abs (num - curr):
            curr = arr[index]
            curr_ind = index
    return curr_ind

