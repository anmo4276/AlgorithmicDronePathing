```
Algorithmic Drone Pathing
Andres Montealegre, Alexis Acevedo, Ilya Zinyakin
April 4th, 2021
```

**Channel 7 Drone Abstract**

Upper Division Software Project meant to capture the "tight interplay between mechanisms, sensors, and control in the design of intelligent systems" (Correll). Utilizing OpenStreetMaps API, DJI Mavic 2 Pro Drone Inverse Kinematics, and a custom Rapidly Exploring Random Trees Algorithm (RRT), this project is meant to optimize news reporting methods while minimizing risk associated with danger zones. Our Channel 7 DJI Mavic 2 Pro Drone incorporates each of these features to traffic news events happening around a city, removing the need for helicopters, delivering fast data on the target location, and in case of hazard areas can prevent the risk of human interaction for a more reliable news coverage. Using Lidar on the drone, we map the cities generated from OpenStreetMaps. These maps are then passed to our RRT algorithm, along with starting and target locations. The RRT algorithm uses these points and tree nodes to generate an optimal set of waypoints that resemble a path. This path is then passed to the drone, which utilizes inverse kinematics, to traverse through the real-life cities, while live streaming vide (which would be reported back to news rooms in theory).

**Equipment:**
1. DJI Mavic 2 Pro
2. Lidar
3. DJI Camera
4. OpenStreetMaps Implementation

**Deliverables and Implementation Plan**

1. OpenStreetMap (Webots) - Lead: Alex Deadline: 4/
    a. For this project to stay true to our story, we need to create a world that will stay true to a
       real life city. Thus, we need to create a complex setting that will be mapped for the RRT.
       We plan to use OpenStreetMap to generate real-life worlds which our drone must
       maneuver.
2. Working DJI Mavic Mini - Lead: Andres Deadline: 4/
    a. Ability to successfully avoid obstacles and move around complex spaces
    b. Ability to apply RRT implementation when a new address(location) is sent.
    c. Ability to use DJI’s built in camera feature to live stream video
    d. Ability to land when needed
3. RRT Implementation - Lead: Ilya Deadline: 4/
    a. Map world using Lidar
    b. Steer the Drone around obstacles
    c. Get random points on the map and have the points a certain distance away from each
       other in a bigger map
    d. Avoid collision and make it step far enough from the target goal to take a picture
 
1. OpenStreetMap (Webots) - Lead: Alex
    a. OpenStreetMap is a Webots feature that allows users to generate new worlds based on
       real life environments. Alex, the OpenStreetMaps team lead, will have to first become
       familiar with the feature and generate basic maps. As we become more comfortable with
       OpenStreetMaps, we hope to generate maps that will be useful to our project (inclusion of
       tall buildings, trees or exclusion of unnecessary objects). After creating a few worlds, we
       would cycle between them and run our DJI in each.
2. DJI Mavic Mini - Lead: Andres
    a. Andres will lead the implementation of the autonomous drone with the RRT and the map
       provided by first mapping the location, setting the goal on the map, finding the start and
       use the individual rotors to find its way around the obstacles. The drone will have to start
       out in a location and fly to its goal and around 10meters away take a picture of the target
       area that will be used to send to Channel 7 HQ for TV news coverage.
3. RRT Implementation - Lead: Ilya
    a. First thing used to map the world is to use lidar with tiago. Once worlds/obstacles are
       mapped, Ilya, the RRT team lead, will create an RRT method. This RRT method will
       have to be applied to the drone, allowing it to maneuver through the complex worlds.
    b. The map will be uploaded to JupiterLab to use the custom path finding RRT.
    c. The RRT has a few extra implementations from before that let the obstacles be mapped
       out on a larger scale and print out the path nodes that were converted to points.
    d. The points will be put back into webots for the drone to use as a path.

**Demo**

In order to demonstrate our final project with it’s various components we will first show off the worlds
generated via OpenStreetMap in which our DJI would run through. This will give our audience a sense of
each environment and how our DJI would be able to traffic news events no matter the obstacles. Then we
will go through the simulation in each world whileshowing its mapping using Lidar and having our DJI
successfully maneuver through obstacles to find the fastest approach to take pictures of the target.

**Results**

The Channel 7 Drone successfully completed our goals. First, Alex was able to use
OpenStreetMaps and convert the OSM files into wbt files. These worlds were extremely accurate and
created the “real-life” scenario that we were aiming for. During this time, Ilya created a test world and
used Tiago’s Lidar to map the world. We then used this test world to create a path using an custom RRT
Algorithm that would take in a starting point and target location. Andres then changed the DJI drone
controller, which was initially controlled manually, into an autonomous drone that would follow the
waypoints used from Ilya’s path. The maps generated can be seen below:


This path was successful when used to fly and film on Ilya’s test map. We then used this autonomous
drone to fly around a map of Dubai, that was generated by Alex using OpenStreetMaps. Unfortunately,
the waypoints used to fly around the map of Dubai were not generated from the RRT Algorithm, due to
Webot’s lidar limitations, which we talk about in problems encountered in the conclusion below.

**Conclusion**

This project was a great learning experience for each of us, as we were able to get both a broad
experience with the DJI Drone and a specific understanding of each of our features. We all enjoyed being
able to apply and combine the elements we had learned in class into one project that we were excited and
passionate about. Although we were able to complete our major goals, we faced several issues during our
process. Alex, when working within OpenStreetMaps, initially had trouble converting OSM files into
WBT as they often had errors. However, when working with both Ilya and Andres to resolve them, we
found that we were able to remove error nodes by removing them in the OSM importer. After removing
them, we were able to successfully generate accurate and real-life worlds. Ilya faced troubles when the
pathing would cross boundaries. He initially thought it was due to lack of points enabling small pathing to
squeeze through but learned it was because the thickness of the obstacles were too small. An example on
the right and the one below is the fixed version:


Another issue we faced was that the mapping would not work with the OpenStreetMap, which is why we
were not able to use RRT on the Dubai map. Unfortunately, due to the building properties generated by
the OSM files, Tiago’s lidar was unable to sense the buildings. Without the map, we were unable to use
RRT on the OpenStreetMap maps. However, to show our product’s functionality, we used the RRT on the
test world and created our own waypoints for the Dubai map. Lastly, Andres faced several issues when
transforming the manual drone into an autonomous one. After some time, we found that our test world
and the OpenStreetMaps worlds had very different physical properties, forcing us to change our turning,
altitude, and distance thresholds. For example, on the Dubai map, we needed much higher thresholds for
turning, or else it would overshoot the turn. However ,the test world was more accurate and allowed us to
have very small thresholds. We also have some trouble determining how much to turn from one waypoint
to the next. Initially, we were using the _atan_ function to calculate the turn. However, thanks to Professor
Correll, we were advised to use _atan2_ instead. By using _atan2_ , we were able to determine the quadrant to
which we had to turn to. Overall, we were able to solve all issues except for the mapping of the
OpenStreetMaps map. Despite this, we are extremely excited with what we were able to accomplish and
learn!



