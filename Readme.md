# SFND-Kalman_Filter
Repository for the Sensor Fusion Nanodegree Project and exercises





# Useful Papers and Articles

## UKF Paper : 

https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf

## Autoware Implementation and Documentation : 

https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/blob/master/src/prediction/kalman_filter/design/kalman_filter-design.md


## Tracking Multiple Objects and Sensor Fusion
The below papers and resources concern tracking multiple objects, using Kalman Filters as well as other techniques!

### [No Blind Spots: Full-Surround Multi-Object Tracking for Autonomous Vehicles using Cameras & LiDARs by A. Rangesh and M. Trivedi](https://arxiv.org/pdf/1802.08755.pdf)

### Abstract: 

Online multi-object tracking (MOT) is extremely important for high-level spatial reasoning and path planning for autonomous and highly-automated vehicles. In this paper, we present a modular framework for tracking multiple objects (vehicles), capable of accepting object proposals from different sensor modalities (vision and range) and a variable number of sensors, to produce continuous object tracks. [...] We demonstrate that our framework is well-suited to track objects through entire maneuvers around the ego-vehicle, some of which take more than a few minutes to complete. We also leverage the modularity of our approach by comparing the effects of including/excluding different sensors, changing the total number of sensors, and the quality of object proposals on the final tracking result.

### [Multiple Sensor Fusion and Classification for Moving Object Detection and Tracking by R.O. Chavez-Garcia and O. Aycard](https://hal.archives-ouvertes.fr/hal-01241846/document)

### Abstract: 
[...] We believe that by including the objects classification from multiple sensors detections as a key component of the object’s representation and the perception process, we can improve the perceived model of the environment. First, we define a composite object representation to include class information in the core object’s description. Second, we propose a complete perception fusion architecture based on the Evidential framework to solve the Detection and Tracking of Moving Objects (DATMO) problem by integrating the composite representation and uncertainty management. Finally, we integrate our fusion approach in a real-time application inside a vehicle demonstrator from the interactIVe IP European project which includes three main sensors: radar, lidar and camera. [...]

## Stereo cameras

The below papers cover various methods of using stereo camera set-ups for object detection and tracking.

### [Robust 3-D Motion Tracking from Stereo Images: A Model-less Method by Y.K. Yu, et. al.](https://hal.archives-ouvertes.fr/hal-01241846/document)

### Abstract: 
Traditional vision-based 3-D motion estimation algorithms require given or calculated 3-D models while the motion is being tracked. We propose a high-speed extended Kalman filter-based approach that recovers camera position and orientation from stereo image sequences without prior knowledge as well as the procedure for the reconstruction of 3-D structures. [...] The proposed method has been applied to recover the motion from stereo image sequences taken by a robot and a hand-held stereo rig. The results are accurate compared to the ground truths. It is shown in the experiment that our algorithm is not susceptible to outlying point features with the application of a validation gate.

### [Vehicle Tracking and Motion Estimation Based on Stereo Vision Sequences by A. Barth (long read)](http://hss.ulb.uni-bonn.de/2010/2356/2356.pdf)
Abstract: In this dissertation, a novel approach for estimating trajectories of road vehicles such as cars, vans, or motorbikes, based on stereo image sequences is presented. Moving objects are detected and reliably tracked in real-time from within a moving car. [...] The focus of this contribution is on oncoming traffic, while most existing work in the literature addresses tracking the lead vehicle. The overall approach is generic and scalable to a variety of traffic scenes including inner city, country road, and highway scenarios. [...] The key idea is to derive these parameters from a set of tracked 3D points on the object’s surface, which are registered to a time-consistent object coordinate system, by means of an extended Kalman filter. Combining the rigid 3D point cloud model with the dynamic model of a vehicle is one main contribution of this thesis. [...] The experimental results show the proposed system is able to accurately estimate the object pose and motion parameters in a variety of challenging situations, including night scenes, quick turn maneuvers, and partial occlusions.

## Deep Learning-based approaches

The below papers include various deep learning-based approaches to 3D object detection and tracking.

### [Fast and Furious: Real Time End-to-End 3D Detection, Tracking and Motion Forecasting with a Single Convolutional Net by W. Luo, et. al.](http://openaccess.thecvf.com/content_cvpr_2018/papers/Luo_Fast_and_Furious_CVPR_2018_paper.pdf)
### Abstract: In this paper we propose a novel deep neural network that is able to jointly reason about 3D detection, tracking and motion forecasting given data captured by a 3D sensor. By jointly reasoning about these tasks, our holistic approach is more robust to occlusion as well as sparse data at range. Our approach performs 3D convolutions across space and time over a bird’s eye view representation of the 3D world, which is very efficient in terms of both memory and computation. Our experiments on a new very large scale dataset captured in several north american cities, show that we can outperform the state-of-the-art by a large margin. Importantly, by sharing computation we can perform all tasks in as little as 30 ms.

### [VoxelNet: End-to-End Learning for Point Cloud Based 3D Object Detection by Y. Zhou and O. Tuzel](https://arxiv.org/abs/1711.06396)

### Abstract: 

Accurate detection of objects in 3D point clouds is a central problem in many applications, such as autonomous navigation, housekeeping robots, and augmented/virtual reality. To interface a highly sparse LiDAR point cloud with a region proposal network (RPN), most existing efforts have focused on hand-crafted feature representations, for example, a bird's eye view projection. In this work, we remove the need of manual feature engineering for 3D point clouds and propose VoxelNet, a generic 3D detection network that unifies feature extraction and bounding box prediction into a single stage, end-to-end trainable deep network. [...] Experiments on the KITTI car detection benchmark show that VoxelNet outperforms the state-of-the-art LiDAR based 3D detection methods by a large margin. Furthermore, our network learns an effective discriminative representation of objects with various geometries, leading to encouraging results in 3D detection of pedestrians and cyclists, based on only LiDAR.

### Other papers on Tracking Multiple Objects and Sensor Fusion

The below papers and resources concern tracking multiple objects, using Kalman Filters as well as other techniques! We have not included the abstracts here for brevity, but you should check those out first to see which of these you want to take a look at.

### [Multiple Object Tracking using Kalman Filter and Optical Flow by S. Shantaiya, et. al.](http://www.ejaet.com/PDF/2-2/EJAET-2-2-34-39.pdf)
### [Kalman Filter Based Multiple Objects Detection-Tracking Algorithm Robust to Occlusion by J-M Jeong, et. al.](https://pdfs.semanticscholar.org/f5a2/bf3df3126d2923a617b977ec2b4e1c829a08.pdf)
### [Tracking Multiple Moving Objects Using Unscented Kalman Filtering Techniques by X. Chen, et. al.](https://arxiv.org/pdf/1802.01235.pdf)
### [LIDAR-based 3D Object Perception by M. Himmelsbach, et. al](https://velodynelidar.com/lidar/hdlpressroom/pdf/Articles/LIDAR-based%203D%20Object%20Perception.pdf)
### [Fast multiple objects detection and tracking fusing color camera and 3D LIDAR for intelligent vehicles by S. Hwang, et. al.](https://www.researchgate.net/publication/309503024_Fast_multiple_objects_detection_and_tracking_fusing_color_camera_and_3D_LIDAR_for_intelligent_vehicles)
### [3D-LIDAR Multi Object Tracking for Autonomous Driving by A.S. Rachman (long read)](https://repository.tudelft.nl/islandora/object/uuid%3Af536b829-42ae-41d5-968d-13bbaa4ec736)
