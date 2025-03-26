# Assignment 3: Rocky Times Challenge - Search, Map, & Analyze

The Rocky Times Challenge involved the Drone identifying two Cylinders placed across various locations and then idnetifying them ,measuring them and then landing on the biggest cylinder there is. 
the script that i have utilised is in src/terrain_mapping_drone_control/terrain_mapping_drone_control/Cylinder_landing_complete.py

# Methodology:

the System was intended to follow the path of where it would produce a Lawnmower pattern and identify the coordinates using the Aruco marker and the Drones own Odometry after which the Drone will continue finishing the survey and return to both the circles once it has returned to it , the drone is intended to fly at an offset and mark the Cylinders height and radius , but in the code i have provided even though it recongizes the circle its seems to fail to navigate to the highest point because it cannot draw an ellispe on the cylinder itself , this causes the Drone to crash once it finishes the Survey. because it cannot recognize the height the only thing i have been able to achieve is successfully executing a lawn mower pattern using the drone.


![Screenshot from 2025-03-25 22-49-11](https://github.com/user-attachments/assets/d34ea5f7-9ab8-4434-be12-99acc9b2ff60)


# Result:

The drone crashes once it finishes the survey however through the use of Rviz the drone recongises both the aruco markers and the circle on top of the cylinders but it cannot recognize the pillars itself as a cylinder unfortunately.
