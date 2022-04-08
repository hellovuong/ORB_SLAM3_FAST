Script for extract data from bag file to image and txt file in format that we can you in stereo_inertial_zed.cc
1. Download datasets
2. run
``python3 export_data_new.py /path/to/bag/file/exp_04.bag  --image_topic /alphasense/cam1/image_raw --right_image_topic /alphasense/cam0/image_raw --side_left /alphasense/cam4/image_raw --side_right /alphasense/cam3/image_raw --imu_topic /alphasense/imu ``