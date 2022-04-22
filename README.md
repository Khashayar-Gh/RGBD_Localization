# RGBD_Localization
## How to run the script
- I suggest using Linux as your operating system.
- Clone this repo.
- Download the dataset from [here](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz).[[1]](#1)
- Place the rgb and depth folders where the repo is cloned.
- Use the following command to install the requirements.
```
$ pip install -r requirements.txt
```
- Run the rgbd_localization.py

## How it works

- By a frequency determined by ```img_freq```, 2 images are compared at each iteration.
- SIFT [[2]](#2) is used to find key points in the 2 images.
- Inliers are determined by finding Homography between matched pixel coordinates using the Least-Median robust method (LMEDS).
- Then, by using the depth images, 3D points of the matched inliers are generated.
- The rotation and translation matrices are found by the method suggested in [[3]](#3) from the 3D coordinates.
- By knowing the first ground truth positions and the relative orientation between every 2 frames, the trajectory is estimated

## Expected results

These figures show the estimated and real trajectories in x, y, and z coordinates.

![Figure_x](https://user-images.githubusercontent.com/87909120/163976733-8b5e1824-0507-4148-81f9-66926747a740.png)

![Figure_y](https://user-images.githubusercontent.com/87909120/163976745-d2d98aa4-c8bc-4335-b54a-1e400a4b3479.png)

![Figure_z](https://user-images.githubusercontent.com/87909120/163976748-9a558da5-3483-489b-8e41-b27f2d1efecc.png)

## Next Steps

- Quantifying the error of the estimated trajectory.
- Improving the accuracy by methods such as sensor fusion or AI approaches.

## References
<a id="1">[1]</a>
J. Sturm, N. Engelhard, F. Endres, W. Burgard and D. Cremers, "A benchmark for the evaluation of RGB-D SLAM systems," 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems, 2012, pp. 573-580, doi: 10.1109/IROS.2012.6385773.

<a id="2">[2]</a>
David G. Lowe. Distinctive image features from scale-invariant keypoints. Int. J. Comput. Vision, 60(2):91–110, November 2004.

<a id="3">[3]</a>
K. S. Arun, T. S. Huang and S. D. Blostein, "Least-Squares Fitting of Two 3-D Point Sets," in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. PAMI-9, no. 5, pp. 698-700, Sept. 1987, doi: 10.1109/TPAMI.1987.4767965.
