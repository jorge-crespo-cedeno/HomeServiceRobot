# HomeServiceRobot

This project implements a turtlebot used for picking up and dropping off things. The pick up action is simulated at a pickup zone. Similarly, the drop off action is simulated in the drop off zone as well. Both actions are simulated by placing or removing cubes in the zones.

Initially, a cube is placed in the pick up zone. Then the robot navigates to the pickup zone and once it reaches this zone, the cube dissappears, simulating that the robot has picked the object up. Then the robot navigates to the drop off zone, and once it reaches it, the cube appears there, simulating that the robot drop the object off in such zone.
