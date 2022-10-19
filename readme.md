# First Place Solution in Summer 2022 ROAR S1/S2 Series

Written by Daniel Chuang, this paper describes his team's experience and thought process for their first place solution in the Summer 2022 ROAR S1/S2 Series Competition with teammates Lucy Wang and Chris Wang. Iterating off of Aaron Xie's previous winning solution, the team reprogrammed the car's controller to be more aggressive, and developed a suite of tools to optimize testing and waypoint tuning.

The code for the solution can be found here: [https://github.com/daniel-chuang/ROAR](https://github.com/daniel-chuang/ROAR)

A video of the solution doing a full runthrough of the map (without shortcuts) can be found here: [https://www.youtube.com/watch?v=2viwByhYhuc&ab_channel=DanielChuang](https://www.youtube.com/watch?v=2viwByhYhuc&ab_channel=DanielChuang)

## Introduction

The Summer 2022 ROAR S1/S2 Series Challenge is an autonomous racing competition held by the University of California, Berkeley. Competitors use Python to automate a simulated Tesla Model 3 in the virtual Carla research environment, which was developed for the testing of self-driving car algorithms. The objective was to programmatically control a car to complete a lap around the Berkeley campus and its vicinity as fast as possible.

In this paper, I will discuss how our team improved upon the previous solution developed by Aaron Xie, resulting in a lap time 69.25 seconds faster than the previous record, for a ~12.4% faster speed.

1. Firstly, we developed a suite of tools to test the car efficiently, which is open-sourced for future competitors to use.

2. Secondly, we reprogrammed and tuned the lateral controller to be more efficient and to change the car's transmission.

3. Thirdly, we made a new set of waypoints that followed more optimal racing lines, to minimize lost velocity during turns.

4. Finally, we found a major shortcut in the map that we cut in order to further reduce our time, in complete accordance with the competition's official rules. However, even without this shortcut, our solution would've placed first.

## Efficient Testing

### Minimap

One of the immediate challenges that we encountered in this competition was the difficulty of testing changes. The lap is extremely long, yet competitors can only monitor the immediate field of view of the car. Given the miniscule proportion that the field of view is compared to the entire race course, and the high speed that the car is traveling at, this lack of information leads to inefficient testing of algorithms, slowing down development time. Moreover, this lack of information also means that only the current state of the car can be monitored. In order to benchmark the car's progress with the status quo, our team members had to watch over the simulation throughout endless full runs, which was very problematic as it consumed lots of time.

In response to this issue, we created a program that generates a live map of the race course that displays the car's current position, as well its past positions. By doing so, we were able to constantly run the algorithm, keeping track of locations at which the car would crash without needing a team member to constantly monitor it. Every time the car crashed, the program would save an image of the map, so we were able to see exactly what happened during the collision.

In order to create the map, we downloaded the occupancy map of the race course [https://roar.berkeley.edu/berkeley-major-map/] and imported it as a 2-dimensional Numpy array. Then, we created 12 checkpoints, dividing the map into smaller sections. Using this data, we programmed a function that takes the coordinates of the car using the Carla API, and records it onto a birds-eye view of the map during a run by modifying the Numpy array and displaying it with OpenCV, with different colors scaling off of speed and magnitude of acceleration.

### Checkpoint by Checkpoint Timing

Another challenge with measuring the efficiency of our algorithms was timing the immense duration of the lap. Since the goal was to achieve maximum speed and minimal time, the status quo for observing improvements was to finish a complete lap, and to compare that to previous lap times.

However, a full lap takes hundreds of seconds to complete, so between every change of the algorithm our team would have to wait for a long time before recieving feedback as to whether or not to keep that change, which was extremely inefficient when making minor changes in small sections of the race course. Additionally, new changes can cause the car to crash, which prevents us from recieving any feedback at all, as the lap will not be completed.

In order to address these issues, we created a time benchmarking system that would report the elapsed time from checkpoint to checkpoint. This way, we were able to identify specific portions of the map that needed more tuning. For example, the race course begins with a massive straight followed by a sharp right – by using this new time benchmark, we were able to increase our car's time for that section by 4 seconds.

## Reprogramming the Controller

A controller is a programmatic system that takes inputs and returns an output for how our car should act. The inputs for our controller was a set of waypoints that were manually set by the team, and the output were values by which we controlled the car autonomously.

Our solution functions off of two controllers: a Proportional-Integral-Derivative (PID) controller for the steering, and a conditional sensor-based controller for the acceleration, braking, and transmission.

The conditional sensor-based controller was our primary focus.

### Section Specific Tuning

In the process of developing the mapping tools explained in the previous section for testing, we realized that we could directly use the map in our actual controller's algorithm as well. By using the mapping script's ability to keep track of which section of the map we were at, we were able to tune the controller at specific points of the race. For example, at parts with big hills, the car will leave the ground and crash if going too fast, so the controller uses the map's checkpoint feature to slow down at that specific section of the map. This allows for us to pursue aggressive speeds while avoiding crashes by slowing down when the car is in danger of crashing. This is one of the most important features that we included in our solution.

### Transmission

The previous solution kept the car at the default gearing. However, our controller toggled the transmission to be manual rather than automatic, changing the gear throughout the lap through a formula arrived upon after plenty of testing:

- <img src="https://latex.codecogs.com/gif.latex?\text{gear} = \lceil\frac{\text{speed} - 2\cdot \text{pitch}}{60}\rceil" />

Since higher gears are smaller in radius, there is less inertia rotating the wheel, but a faster angular velocity. This means that higher gears are more efficient when the car is moving at a high speed, and doesn't need much more acceleration.

On the other hand, lower gears have bigger radii, meaning that there is more inertia, but a slower angular velocity due to the larger radius. This makes low gears very efficient for when the car needs to accelerate quickly, such as at the beginning of the race and after sharp turns.

It is for this reason that speed is in the numerator of our formula for the controller: when speed increases, our gear should also increase.

The only other factor in the formula is pitch. When the car is facing an uphill, we opt for a lower gear because the force generated by the wheels needs to not only increase the car's velocity, but also counteract the force of gravity. The greater the uphill, the more force is needed, and therefore lower gearing. This is why pitch is in the denomenator of our equation.

### PID Controller

None of the code of the PID controller was changed from the previous record-holding solution. However, some of the parameters for the coefficients were subtly tuned in order to account for the changes that we made in the conditional sensor-based controller.

## Tuning Racing Lines

Since our solution follows waypoints, it is important that those waypoints are optimized. By applying theories of racing lines onto our waypoint generation, we were able to decrease our lap time.

We used our map in order to create new racing lines for each part of the race. In general, it is most optimal for the car to enter a turn from the outer lane, touch the very inner point of the inner lane during the apex of the turn, and then exit back into the outer lane to complete the turn. With this strategy the car loses minimal speed, as the turning angle is minimized.

Additionally, we considered future turns when planning our racing lines. For example, if entering a turn that was then continued by a long straight, we opted to start turning early, sacrificing entering speed for exiting speed. Because the turn is then proceeded by a long straight, that exit speed is well used over the course of time that it takes to elapse the long straight.

On the other hand, when entering a turn that then continues into another turn, we chose to instead turn late, as the car needs to slow down in order to complete the second turn. This means that having a high exiting speed from the first turn is not useful, as the car must soon be slowed down regardless. Instead, it is better to make the most out of the entering speed, and start turning late.

## Shortcut

We discovered throughout the map that there were many off-road routes that we could rulefully take in order to reduce the total distance traveled, and therefore our lap time. In order to use these new shortcuts, we created new sets of waypoints for our controllers to follow. Most of these shortcuts resulted in only marginal differences in lap time, but there was one major reroute between Checkpoint 5 and Checkpoint 6 that we estimate to have reduced our lap time by 24 seconds. However, even without using these shortcuts, our solution would've won first place.

## Conclusion

Creating this solution was an incredibly difficult endeavor given the impressive record set by Aaron Xie in his Spring 2022 solution. However, by combining our understanding of physics to optimize our car's controllers and racing lines, as well as creating a suite of tools to better benchmark our algorithmic tuning and changes, our team was able to significantly improve upon the previous record. We look forwards to what future competitors can do to further push the boundaries of what is possible in this autonomous racing challenge, and hope that our open-source toolset and solution helps them create even better solutions.
