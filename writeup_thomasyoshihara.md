## Project: Kinematics Pick & Place

---


**Steps to complete the project:**


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/schemata.jpg
[imaget23]: ./misc_images/transform_2_3.png
[imagetht]: ./misc_images/ht_components.png
[imaget]: ./misc_images/homogeneoustransform.jpg
[image_wc]: ./misc_images/image-4.png
[image2]: ./misc_images/ik_angles.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

Here we go.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The following figure describes the link assignments and joint rotations as well as the positions of the non zero link lengths and offsets.

![alt text][image1]

The DH parameter table configuration is set to:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

Following the course content, i chose d1 to intersect with the next joint to minimize the non zero parameters.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
The homogeneous transform matrix consists of the following components:

![alt text][imagetht]

After multiplication, it can be build for e.g. in python for the individual joints as this.

```python
def get_tf_matrix(alpha, a, d, q):
    return Matrix([
            [              cos(q),             -sin(q),            0,               a],
            [   sin(q)*cos(alpha),   cos(q)*cos(alpha),  -sin(alpha),   -sin(alpha)*d],
            [   sin(q)*sin(alpha),   cos(q)*sin(alpha),   cos(alpha),    cos(alpha)*d],
            [                   0,                   0,            0,               1]
        ])
# Create individual transformation matrices
T0_1 = get_tf_matrix(alpha0, a0, d1, q1).subs(s)
T1_2 = get_tf_matrix(alpha1, a1, d2, q2).subs(s)
T2_3 = get_tf_matrix(alpha2, a2, d3, q3).subs(s)
T3_4 = get_tf_matrix(alpha3, a3, d4, q4).subs(s)
T4_5 = get_tf_matrix(alpha4, a4, d5, q5).subs(s)
T5_6 = get_tf_matrix(alpha5, a5, d6, q6).subs(s)
T6_E = get_tf_matrix(alpha6, a6, d7, q7).subs(s)
```

As a concrete example, the transformation matrix of Joint3 in respect of Joint2 would be:

![alt text][imaget23]



The homomogeneous transform between the `base_link` and the `gripper_link` can be build as a multiplication of all the individual transformations in order:

```python
T0_E = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
Since the last three revolute joints axes of the kuka robot (and many other serial robots as well) intersect at a single point and thus can be combined as a sperical joint, the end effectors position and orientation of this robot can be decoupled.

We first obtain the cartesian coordinates of the wrist center.
Location of the endeffector relative to the the base is calculated by

![alt text][image_wc]

The postition and roll pitch yaw angles of the end effector are given by the ros request.

From there, we first obtain a combined rotation matrix by substituting the values for roll (x-axis rotation), pitch (y-axis rotation) and yaw (z-axis rotation) into the regarding rotation matrices and multiplying them together.
We also have to correct for angles and translation of the gripper. Looking at our DH parameter table, we have two rotations (-pi/2) along the z-axis and one (pi/2) along the y-axis.
The gripper is also translated on the z-axis (0.303).


Now that we know the position and rotation of the end effector, we move on to derive the angles of the first three joints via trigonomic functions:

```
theta1 = atan2(WC[1],WC[0])
theta2 = pi/2. - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi/2. - (angle_b + 0.036)
```
![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

IK＿Server initializes as a ROS Node and declares the service calculate_ik.
Our function handle_calculate_IK from line 22 onwards receives endeffector poses from the motion planning to calculate the according IKResponses.

We first prepare our FK routines by declaring symbols for our Denavit Hartenberg Parameters anda dictionary to hold the actual values.

Based on these parameters we can obtain individual transformation matrices for all the joints and the homogeneous transform to the end effector wrt the base by matrix multiplication.

For every pose in the request object, we extract the endeffector position and then first calculate the thetas for orientation and finally the positioning joints following the steps described.

To skip unnecessary steps, i decided add only the joint angles to the trajectory list.
This can be undone by simply indenting lines 151 and 152 one more to the right so that they are included in the loop.

Finally this trajectory list is returned synchroneously to the requesting ros node.


![alt text][image3]


