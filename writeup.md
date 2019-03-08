# Path Planning Project

This is my writeup for the Path Planning Project in Term 3 of Udacity's Self Driving Car Nanodegree. I will go through each step of the algorithm I created to successfully drive the car around the track. All of the code can be found in 'main.cpp' from lines 95 to lines 406. I will reference the relevant code by line number.

My implementation is similar to the finite state machine discussed in the classroom lectures, but much simpler. Each time the algorithm is run, there are 3 possible actions: keep the lane, change lane left, and change lane right. The default decision is to keep the lane at the reference velocity. Since the speed limit is 50 mph, I set the reference velocity to 49.5 mph to get close with a margin for error. If there is car in front of us going too slowly, we then try and change lanes. 2 things are considered when changing lanes: is it safe and is the lane clear.

## Step 1. Check for cars
The first step is to see if there is a car in front of us or not. Lines 116 to 147 check to see if a car is in our lane and within 20 meters of us. If so, the change_lanes flag is triggered. If not, we skip this code and move on to generate a keep_lane trajectory.

## Step 2. Change lanes
Lines 150 to 286 contain the code to try and change lanes. I used a switch statement which checks which lane we are currently in and goes from there. In case we check the potential lane we will change into to see if there are any cars within 20 meters. If so, it is not safe and we will keep our lane. If it is determined to be safe, we then check if the lane is clear. If there are no cars in the potential lane within 50 meters ahead of us, it makes sense to change to that lane.

## Step 3. Create spline
The code here is from lines 316 to 360. Now that we have our desired lane, we generate a list of 5 points from the car in 30 meter increments. This is done in Frenet coordinates to more easily follow the desired lane. If there is a previous path available, we start our spline points from the end of that. Lastly, we transform the points to vehicle coordinates and fit the spline.

## Step 4. Sample spline to create path
Lines 367 to 406 are the final step. Here I sample the spline at either the same distance or an increasing/decreasing distance based on whether or not we need to slow down to follow a vehicle or speed up to get back up to the reference velocity. I transform the sampled points back to global coordinates and send them to the simulator.



