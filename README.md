
# UI Development & Socket Programming
## TASK DESCRIPTION
* SOCKET PROGRAMMING
Use pre-made applications(from the play store/app store) to send your phones IMU data to your laptop using socket programming. The data should be published onto a rostopic and should be visualised on RViz.

* UI DEVELOPMENT
Create either a desktop GUI application capable of displaying real-time dynamic data. Display the following data from ROS info streams -

1. IMU data retrieved from the phone
2. Video feed from the 4 wheeled gazebo bot.
3. GPS data from the gazebo bot
4. Visualise a way to display whether an obstacle has been detected or if the path is clear
Create a toggle button which will change the IMU data representation switch between Euler angle and quaternions using ros services.

Create a screenshot button to save an image from the videostream and a record button to record a video and save it.

All data recorded should be stored in either CSV or XLSX format.

## TASK APPROACH
* A server node is made to establish connection with phone and receive IMU data using the app **HyperIMU** over TCP. This data is then published on */phone_imu* topic to visualise it in RViz.
* GUI is made using **PyQt** which displays this IMU data, GPS data, Obstacle data and Camera feed from the four wheeled rover by subscribing to respective topics.
* GUI includes switches and widgets like :
1. IMU representation toggle switch to toggle values from Quaternions to Euler Angles.
2. Record button to record camera feed.
3. Screenshot button to take a screenshot of the camera feed.
4. Data record button to record IMU, GPS, Obstacle dataand store it in a CSV file using **Pandas** library.
5. Obstacle Widget that turns green when the path is clear and red when obstacle is detected.

## RELAVENT ROS TOPIC LIST
```
/camera/rgb/image_raw
/gps/fix
/laserscan
/phone_imu
```

## TASK IMAGES
### Terminal after launching. Waiting for client to connect.
![server listening](https://github.com/MRM-AIA-TP-2024/MRM_PrateekMhatre/assets/117933472/b696e7e4-e1b9-4487-b221-c7da84bc0dab) 

### Receiving IMU data from phone in terminal.
![receiver IMU terminal](https://github.com/MRM-AIA-TP-2024/MRM_PrateekMhatre/assets/117933472/2cb36004-b966-4347-80d2-b2fd7868996d)

### Phone's IMU orientation visualised in RViz. Arrow moves as phone's orientation is changed.
![rviz imu](https://github.com/MRM-AIA-TP-2024/MRM_PrateekMhatre/assets/117933472/593c9be9-7f80-43c4-869d-b737163313d2)

### GUI displaying phone's IMU data and GPS,Video feed and obstacle data of the Rover in Gazebo.
![GUI](https://github.com/MRM-AIA-TP-2024/MRM_PrateekMhatre/assets/117933472/63d27d7a-02ce-4c29-8145-c01b0227395c)

### GUI when obstacle is detected by the Rover. Widget color changes from green to red.
![obstacle detected](https://github.com/MRM-AIA-TP-2024/MRM_PrateekMhatre/assets/117933472/6fcbd598-ad34-4dd1-a65d-9337e555fc95)

### Pop-Up window after any data is saved with timestamp.
![save gui](https://github.com/MRM-AIA-TP-2024/MRM_PrateekMhatre/assets/117933472/93782344-670e-400e-b519-85ce47f998b0)

### RQT Graph
![RQT](https://github.com/MRM-AIA-TP-2024/MRM_PrateekMhatre/assets/117933472/dd8e9a6f-643d-47fe-aade-d7c0508cdd6a)










