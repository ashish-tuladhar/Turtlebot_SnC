# Turtlebot_SnC
Robot following a straight line by observing a square using Turtlebot (Aut 22 - Group 9) 

video link - https://youtu.be/VaupTq8zg6Q
README
This project integrates MATLAB and ROS environment to simulate TurtleBot Waffle to detect a square follow in a straight line. 
MATLAB Code
Initialization
    • Connect MATLAB to the TurtleBot environment using its IP address
    • Define necessary ROS subscribers and publishers

Image Processing
    • Load original image to be detected in the environment
    • Detect and extract SURF features of the original image and rgb image obtained from the TurtleBot camera
    • Find matched features between the two images
    • Rotate the TurtleBot until enough (50 for our case) matched features are found
    • Once adequate matched features are found, find the geometric transform between the original image and the detected image
    • Find the pixel coordinates (inlier Points) of the matched feature points

Coordinate Transformation
    • (Transform pixel coordinates to global coordinates)

Finding Object Centre and Object Plane
    • Find object centre by taking the mean of all inlier matched feature points (in global coordinate frame)
    • Find object plane by fitting a plane to a point cloud given by all inlier matched feature points (in global coordinate frame)

Finding Intersection Points
    • Find a line with direction normal to the object plane and passing through the object centre
    • Find another line with direction parallel to the object plane and passing through the current TurtleBot position
    • The intersection of the two lines gives the intersection point (that the TurtleBot first travels to)

Finding Final Points
    • The final point is on the line normal to the object plane with 0.5m positive distance from the object centre  

TurtleBot Movement
    • Receive target position x, y
    • Rotate to face target 
    • Move forward to target
    • During move to target, keep receiving yaw of robot, and adjust angle to face target straghtly

Contribution to the code:
| Student ID | Student Name   |Percentage Contribution |
|------------|---------------_|------------------------|
|13319470    |Ashish Tuladhar |35%|
|13054705|Mina Jang|35%|
|12912667|Jiahui Huang|30%|
  
