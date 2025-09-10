Node:

an executable, responsible for a single (modular) purpose.

For example: Sensor, effector, etc...

The communicate via:

- Topics (simple pub-sub)
- Services (request-response)
- Actions
  - Goal service
  - Result service
  - Feedback topic

How to play rosbag?

example: ros2 bag play ~/bags/my_bag --topics /camera/image /odom

For viewing:

ros2 run rqt_image_view rqt_image_view

ros2 run rqt_plot rqt_plot

ros2 run rqt_tf_tree rqt_tf_tree
