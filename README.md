# geotagging_ros2
### The geotagging contains ros2 package
* clone the ros2 package into src of your workspace and run
```
# This launches the package
ros2 launch geotagging geotag.launch.xml
```
---
>** create a folder named bash_scripts in home directory and copy _ros2_launch.sh_, _dds_launch.sh_, _micro_ros.sh_, _startup_handler.sh_ into the directory**
```
# Run the commands to create bash_scripts folder in the home directory 
clear
cd
mkdir bash_scripts
```
* Now copy the text in profile.txt and paste it at the end of ./profile in linux
```
  if [ -z "$SSH_TTY" ] && [ -z "$DISPLAY" ]; then
    source ~/bash_scripts/startup_handler.sh
fi
```
* This enables the script to run on startup and it launches all the necessary packages __But make sure to enable autologin on the raspberry pi so the scripts start__ 
