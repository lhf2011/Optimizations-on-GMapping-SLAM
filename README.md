# Optimizations-on-GMapping-SLAM
 Optimize openslam-GMapping in three aspects: motion model, sensor data preprocessing, and mapping

This project is based on an open source project named GMapping SLAM, download the base codes in the following link:
git clone https://github.com/ros-perception/slam_gmapping
git clone https://github.com/ros-perception/openslam_gmapping

Updated the following 5 files:
md5sum                            filename
c5feac28c3352a10d30071b6463264ab  motionmodel.cpp
eaa6cadab7ebc8f048813443668aa265  scanmatcher.cpp
19ab88587855df1db135a37b42aa339b  slam_gmapping.cpp
38871799bbb95273958afac922b5af44  slam_gmapping.h
4662d2ff7f5a251576215c317ce7a491  smmap.h