# Mono-Odometry

This repository is a basic MATLAB implementation of monocular visual odometry. The code has been tested on [MATLAB R2018a](https://in.mathworks.com/?s_tid=gn_logo).

The project has been done by the following:
* Hao-Chih Lin
* Mayank Mittal
* Chen-Yen Kao

Demonstration video is available [https://youtu.be/trbBh8Rjc4s](https://youtu.be/trbBh8Rjc4s).

## System Configuration

```
CPU:                          Intel(R) Core(TM) i7-8750H CPU @ 2.20GHz
Memory:                       16 GB @ 1333 MHz
                               - 8192 MB, DDR4-2667, Samsung M471A1K43CB1-CTD    
                               - 8192 MB, DDR4-2667, Samsung M471A1K43CB1-CTD    
Graphics:                     NVIDIA GeForce GTX 1070 with Max-Q Design, 8192 MB
Graphics:                     Intel(R) UHD Graphics 630, 1024 MB
```

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
