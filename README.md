# perception_squad

Github documentation to run the node and bag file

## Creating directories

To save the output files and bags, first a directory need to be created (appologies for this strenuous request).   

Inside the `/perception_squad/src` folder, create a directory:
- `bags` where the test_data folder needs to be copied

After creating the directory, copy the `test_data` folder (or extract)from https://siaroboticsolutions.sharepoint.com/:u:/s/Robot/EZfyh6hzdhhPkoi7R8yrN4IBlX5oab_xKMWEc5FhGorWGA?email=sth.anish%40hotmail.com&e=4%3a5PDsP6&at=9   

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
ros2 run perception_task extrinsic_calibration --ros-args -p bag_file:="./src/perception_task/bags/test_data/rosbag2_2023_06_19-18_17_14/" -p pointcloud_out_dir:="./src/perception_task/output/"
``` 

Here, we pass on the output directory and the path to the bag file which will be passed on to the parameter server as parameters for this ros2 node.   


Once the node is running you will see something like this:   
![screen_node](https://github.com/AnishShr/perception_squad/assets/62991158/5f2bfcf5-3022-4672-8927-2b78d26698f2)

This prints the output directory and path to bag files.   

## Running a launch file
Once the node is running, it subscribes to the point cloud messages for appropriate topic names for lidar and depth camera. So now we run the launch file which plays the bag file.   

In another terminal, run the following command from the root of the catkin workspace (i.e., `/perception_squad`)

```
ros2 launch perception_task bag_launch.py bag_file:="./src/perception_task/bags/test_data/rosbag2_2023_06_19-18_17_14/"
```

The path to bag file can be copied from the previous terminal.

once the bag file is played, the node for calibrating extrinsics start to accumulate point clouds and start performing the ICP. Subsequently, the required point clouds are saved into `/perception_squad/src/perception_task/ouput` directory.

![screen_bag](https://github.com/AnishShr/perception_squad/assets/62991158/4a9e8443-4f26-40da-a384-50c64768df3c)

Once it is launched, you can see something like this in the terminal.

On the other teminal where the extrinsic calibration is carried out and which outputs the files to the output directory, you should see something like this:   

![screen_ongoing](https://github.com/AnishShr/perception_squad/assets/62991158/f2bc670c-3c04-413b-bdc2-64f81dfef15f)
   

Once the ICP converges and gives a transformation between the 2 pointclouds, the terminal window will have the following loggers printed:   

![screen_icp_done](https://github.com/AnishShr/perception_squad/assets/62991158/61d647dd-6c27-47be-ae3e-752887504361)



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

The 4 output pointclouds would look something like this, when placed on top of each other:   

![snapshot00](https://github.com/AnishShr/perception_squad/assets/62991158/ea33333c-7171-4b94-a7da-6f9d6a1142c7)


