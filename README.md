# baxter_cubic
Robot solves rubic repo

## Launch instruction

You should run robot connection script in all shells, which you use for robot programms.
```bash
cd catkin_ws
./baxter.sh sim
```

* Then to start simulation run:
```bash
roslaunch baxter_gazebo baxter_world.launch
```

* After that to enable robot and than open head_camera
```bash
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools camera_control.py -o head_camera -r 1280x80
```

* At last run the cubic solver
```bash
rosrun project rsm_kubic.py
```
Or optionally you can create a directory for images:
```bash
mkdir cube_images
cd cube_images
pwd
rosrun project rsm_kubic.py -d <!!!! here you copy what you got from pwd!!!!
```
Then check if the images are in the folder:
```bash
ls
```


Then follow the instructions from programm.
