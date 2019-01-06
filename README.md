# Mono-Odometry

This repository is a basic MATLAB implementation of monocular visual odometry.

The code has been tested on [MATLAB R2018a](https://in.mathworks.com/?s_tid=gn_logo) and depends on the following toolboxes:
* Parallel Processing Toolbox
* Computer Vision Toolbox

## How to run the repository?

1. Clone the repository using the following command:
```
git clone https://github.com/Mayankm96/Mono-Odometry.git
```

2. Import the dataset to the folder [`data`](data). In case you wish to use the [KITTI](http://www.cvlibs.net/datasets/kitti/) Dataset, such as the [Residential dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php?type=residential), the following command might be useful:
```bash
cd PATH/TO/Mono-Odometry
## For Reseidential Sequence: 61 (2011_09_46_drive_0061)
# synced+rectified data
wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0009/2011_09_26_drive_0009_sync.zip -P data
# calib.txt
wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip -P data
```

3. Change the corresponding paramters in the configuration file [`configFile.m`](code/config/configFile.m) according to your need

4. Run the script [`main.m`](code/main.m) to get a plot of the estimated odometry

## Proposed Implementation of the Algorithm

Please find attached the accompanied report.
