# Push PyBullet
Contains code for our simulated push primitive in PyBullet.

The world for this simple simulation includes a plane, a floating gripper, and a cube as the object to be pushed.

## MODEL DEVELOPMENT

### Iteration 1: Get it to work
In our first working example, the floating gripper would randomly choose an angle to approach the cube from (at a fixed distance), and apply a push in a straight line for a fixed duration of iterations. The only parameter for `push` at this point was the angle (relative to the world frame) that the gripper would push from. 

![First Iteration](https://github.com/ivanjut/push-pybullet/blob/master/demos/first_iteration.gif)

### Iteration 2: Parametrize
Next we wanted to come up with a few parametrizations for `push`. In this iteration, we still included the `angle` parameter, but instead chose to make the `distance` a parameter instead of being fixed, as well as the `iters`, the duration of iterations that the push would take. Importantly, we also added the orientation of the gripper as a parameter. This parameter is constrained to a reasonable pose for the purpose of not messing up PyBullet. We were then able to simulate lots of pushes with randomized combinations of these parameters.

![Second Iteration](https://github.com/ivanjut/push-pybullet/blob/master/demos/second_iteration.gif)

### Iteration 3: Loss function
We then started thinking about possible loss functions to develop a learning model for this simulation. Our basic idea is as follows: at every iteration, calculate the perpendicular distance of the cube's position to the desired straight line trajectory, and use that squared distance as the loss for that iteration. For a single push, we sum up all those losses to get a total loss for a push.

![Third Iteration](https://github.com/ivanjut/push-pybullet/blob/master/demos/third_iteration.gif)

We implemented a total straight-line loss, a mean straight-line loss (total divided by number of iterations), and an angular loss function. The angular loss is the angle between the desired straight-line trajectory and the line to the cube's end position.

<img src="https://github.com/ivanjut/push-pybullet/blob/master/demos/initial_losses.png" width="300" height="300">

### Iteration 4: New constraint
We incorporated the idea that the gripper could only push the block from an angle that is normal to one of it's faces. We also incorporated a second gripper to simulate the PR2 better.

## DATA COLLECTION
