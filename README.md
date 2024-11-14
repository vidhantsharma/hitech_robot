# How to run #

1. unzip the file shared
2. make sure docker is installed in your PC
3. go to hitech_robot_ws/
4. run this command "sudo docker build --no-cache -t hitech_robot_workspace .". This will build the docker container. if you face network related build fail, then just build again.
5. once docker is success, then run it using this command - 
    "
    sudo docker run -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --privileged \
    hitech_robot_workspace
    "
6. above command will open the docker container. Enter your linux password if required.
7. now open 4 more terminals (use tilix or something similar), and find the container id using this command – "sudo docker ps"
8. then in all of the 4 terminals run this command - "sudo docker exec -it <container_id> /bin/bash". This will open the ssame docker instance in all of the 5 terminals
9. Now run these launch files in sequence
    a. roslaunch hitech_robot_description robot_gazebo.launch [to open gazebo]
    b. roslaunch hitech_robot_controller robot_controller.launch [to launch robot controls]
    c. roslaunch hitech_robot_description robot_display.launch [to display robot in rviz]
        note:-here obstacle can be added in rviz, by going to add->by topic->marker-	>visualization marker, in case it is not added already.
    d. roslaunch hitech_robot_controller keyboard_teleop.launch [for keyboard teleoepration]
    e. (for object detection)
        rosrun hitech_robot_functionality object_detection_node.py
    e. (for going towards the object if it is visible in camera)
        rosrun hitech_robot_functionality object_follow_node.py

# Results #

(A) From object detection node, we get 2 informations -

1. Relative position of object w.r.t. camera frame
[INFO] [1731604147.136420, 191.345000]: Relative position: [-2.31579685e+00 -2.08910893e-03 -3.13366340e-02]
2. Global position of object w.r.t. world frame
[INFO] [1731604147.138612, 191.348000]: Object position in world frame: x: -0.02488881256646973
y: -2.99691498751568
z: 0.11866336601768616


(B) From object follow node, the robot will go towards the object and stop at about ~0.5 m before it. It is visible in gazebo, as well as rviz after running the object_follow_node.