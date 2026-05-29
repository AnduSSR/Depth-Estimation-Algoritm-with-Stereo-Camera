# STEPS TO RUN THE STEREO CAMERA SIMULATION
1. ros2 run ros_gz_bridge parameter_bridge "\"
  /stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image "\"
  /stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo "\"
  /stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image "\"
  /stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo

2. gz sim -v 4 stereo_world.sdf //this runs the simulation with advanced debugging
3. gz sim stereo_world.sdf // this runs the simulation without debbuger
4. ros2 run ros_gz_sim create -file stereo_camers.urdf -name stereo_cube -z 0.5 // to run the camera URDF
5. ros2 topic hz /stereo_camera/left/image_raw // to check topic status
6. ros2 run rqt_image_view rqt_image_view //view image data
