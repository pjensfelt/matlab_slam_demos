# matlab_slam_demos
MATLAB demo scripts to illustrate SLAM

## Localization
* Reset the similator
* Turn on one of the landmarks and click update to make it converge (you might have to reduce sensor noise, increase again after)
* Turn off the landmark
* Move the robot which will make uncertainty increase, turn on landmark and click update.
* This will reduce noise -> localization

## Correlation between landmark and robot
* Reset the similator
* Disturb the robot position (so that the true position changes)
* Inject noise to reflect the uncertainty. 
* Activate a landmark and let it coverge  by clicking update repeatedly. This creates a strong correlation between the robot position and the landmark. 
* Note that as the robot was uncertain about its positin when the landmark was added that uncertainty is also in the landmark. We see that the landmark has large horisontal uncertainty (robot position uncertainty) but small vertically (no uncertainty from robot here)
* Click SuperGPS to generate a very priecise measurement of the robot's position.
* Note how both the robot and the landmark position moves together. This is thanks to the correlation.

## Loop closure
* Reset the simulator
* Turn on one landmark from start pose. Click update to let its coverge
* Turn off that landmark.
* Move around and acculumate uncertainty. 
* Also disturb the pose so that it gets a bit lost but be sure to inject enough noise if you do that to account for that uncertainty.
* Add another landmark (NOT THE SAME AS BEFORE). 
* This landmark will now "inherit" the uncertainty from the robot.
* Click update many times to ensure that the correlation builds up between the robot position and the second landmark.
* Note that the second landmark is estimated to be in the wrong position because the robot was not in the right position when the landmark was added.
* Turn on first landmark withness LOOP CLOSURE! You will see that
    * the uncertainy shrinks in both the robot position and the that of the second landmark.
    * the position of the robot and the second landmark are adjusted to be closer to their true values thanks to the information contained in the map via the first landmark. 
