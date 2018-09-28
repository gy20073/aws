# GTA Hacking Program

[![Build Status](https://travis-ci.com/ucbdrive/gta.svg?token=9QKS6inVmkjyhrWUHjqT&branch=master)](https://travis-ci.com/ucbdrive/gta)

This is a hacking program to fetch data from GTA V game. The hack consists of 2 parts: 

1. injecting the render buffer (credit to Philipp Krahenbuhl)
2. using the naive functions through [ScriptHookV](http://www.dev-c.com/gtav/scripthookv/)

Through the program we can fetch RGD frame, depth, 2D/3D bounding boxes, etc. from the GTA game through socket.

## Structure

```
.
├── gamewrap                # The server client injected into the game
├── gameclient              # The client code to receive the data from the game
├── hybrid_client           # The outer wrapper. Only need this to collect data
├── LICENSE
└── README.md
```

## Usage

#### Data Collection

To collect the data, you need to first install the GTA V game as described in [DeepGTAV](https://github.com/ai-tor/DeepGTAV]). Another special requirement is to install in Windows 10, which is the only system where the code is tested through (There is known bug running in Windows8).

Compile the solution in `gamewrap` folder to produces 4 files: `dxgi.dll` , `fetch_info.asi` , `gta5.hk` , `server.hk` (you may need to modify the path of After Build Events in Visual Studio). Copy the compiled files to the root folder of GTA V game.

To start the game collection procedure, locate to the `hybrid_client` foler and run:

```
python collect_data.py --name=xxx --path=xxx
```

#### Data Visualization
To visualize the collected data:
For map view:
```
python get_map_vid.py -p your_filepath
```
For video with object detection bounding boxes and control signal outputs:
```
python show_demo.py -p your_filepath
```
For birdview demo:
```
python show_birdview.py -p your_filepath
```
See more configurations you can modify in the code

#### Data Post-processing (Optional)

The post-processing is a very important part of getting the final data, as the intermediate result is not complete.

The main code for post-processing is `hybrid_client/utils/proc_data_utils.py`(actual implementation) and `hybrid_client/data_proc_jobs.py` (outer wrapper).

After collecting data, your file structure will look like this:

```
base_dir
├── data1
├── data2
├── data3
└── ...
```

To process the data, you need to run some of the jobs in `hybrid_client/data_proc_jobs.py` one-by-one.  You need to change the base folder in the code to `base_dir`.

1. First transform the format of the data. Run job `parse_data` .
2. Then filter out the invalid data not needed for your task, and split the data into train/test. If you want to get data for detection, run `filter_for_detection_and_split`; if you want to get data for tracking, run `filter_for_tracking_and_split`.

*SPECIAL NOTICE: If you find the IDs of objects in segmentation map inconsistent across frames, it does not matter. The ID here is used only for classification (vehicle, pedestrian, misc). We will assign the right tracking ID from native APIs in post-processing.**



## Tested System 

1. Windows 10 Home
2. Visual Studio 2017 Community
3. GTA V
4. Intel(R) Core(TM) i7-6700 CPU
5. 16 GB memory
6. GTX 1070
7. Python 3.6.2 (Anaconda)

## Before committing

run `sh scripts/setup.sh`

## Contributors

[Github statistics](https://github.com/ucbdrive/gta/graphs/contributors)
