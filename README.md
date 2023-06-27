# perception_squad

Github documentation to run the node and bag file

## Creating directories

To save the output files and bags, first few directories need to be created (appologies for this strenuous request).   

Inside the `/perception_squad/src` folder, create 2 directories:
- `output` to store output files (.PLY files and .txt file)
- `bags` where the test_data folder needs to be copied

After creating the directories, copy the `test_data` folder from https://siaroboticsolutions.sharepoint.com/:u:/s/Robot/EZfyh6hzdhhPkoi7R8yrN4IBlX5oab_xKMWEc5FhGorWGA?email=sth.anish%40hotmail.com&e=4%3a5PDsP6&at=9   

Once this is done we are almost ready to get started.   

## Building the package

From the root of the workspace (i.e., `/perception_squad`), run the following command:   
```
colcon build
```
This will build the package

Don't forget to source the setup.   
```
source install/setup.bash
```

After this the package will be ready to run.   


## Running the node
From the root of the catkin workspace, `/perception_squad`, run the following command:   
```
ros2 run perception_task extrinsic_calibration src/perception_task/output/ src/perception_task/bags/test_data/rosbag2_2023_06_19-18_17_14/rosbag2_2023_06_19-18_17_14_0.db3
``` 

Here, we pass on the output directory and the path to the bag file which will be passed on to the parameter server as parameters for this ros2 node.   


Once the node is running you will see something like this:   
![Screenshot from 2023-06-27 16-58-07](https://github.com/AnishShr/perception_squad/assets/62991158/05b91b9c-b2ac-42ec-849b-310cccec6d6d)

This prints the output directory and path to bag files.   

## Running a launch file
Once the node is running, it subscribes to the point cloud messages for appropriate topic names for lidar and depth camera. So now we run the launch file which plays the bag file.   

In another terminal, run the following command from the root of the catkin workspace (i.e., `/perception_squad`)

```
ros2 launch perception_task bag_launch.py bag_file:=src/perception_task/bags/test_data/rosbag2_2023_06_19-18_17_14/rosbag2_2023_06_19-18_17_14_0.db3
```

The path to bag file can be copied from the previous terminal.

once the bag file is played, the node for calibrating extrinsics start to accumulate point clouds and start performing the ICP. Subsequently, the required point clouds are saved into `/perception_squad/src/perception_task/ouput` directory.

![Screenshot from 2023-06-27 17-09-04](https://github.com/AnishShr/perception_squad/assets/62991158/1b78525b-ed1e-4927-adfd-bb4631eaae1d)
Once it is complete, you can see something like this in the first terminal.

###
Just make sure that the folder structure is like this:
- src
    - perception_task
        - output
            - files
        - bag
            - test_data
                - rosbag_2023.06.19-18_17_14


Once everything is done, the `/perception_squad/src/perception_task/output` directory will have 5 files: 4 point clouds and one .txt file.