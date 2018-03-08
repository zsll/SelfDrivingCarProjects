[//]: # (Image References)

[image1]: state_update_equations.png "state_update_equations"

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

# Compile and Run Instructions

There are two ways to build and run the code:

### Manual Build
1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./mpc`. 

### Using Xcode
A Xcode project "MPC.xcodeproj" is generated. Just run it in Xcode. Click on the "ALL_BUILD" Scheme on the top left. Dropdown with Schemes will open. Select 'mpc' then hit 'Run'. 

## Reflection

#### The Model

The MPC is based on a kinematic model that composes coordinate of the car (x and y), orientation angle (psi), speed v, cross-track-error cte and error of psi (epsi). Along with actuator parameters acceleration a and steering angle delta of previous timestamp, the model develops state of current timestamp using the following update equations. These equations appears in section 9 of Lesson 19 MPC.
![Before][image1]

#### Timestep Length and Elapsed Duration (N & dt)
To handle latency of 1s, N and dt are chosen to be 10 and 0.1. Thus Horizon T equals 1s, which predicts the trajectory of the next 1s duration. When using smaller dt like 0.05s and N as 20, the car oscillate a lot though Horizon has been the same. Seems that frequent actuator intervention tends to introduce overshooting and mitigate stability of the vehicle. I also tried to use large dt like 0.25s while N equals 4. The car quickly falls out out boundary, which indicates a timely actuating will be vital as well. In the firstly place I thought given huge N like 100 would result in slow performance as mentioned in lecture. It turns out the car was behaving bizarre like trying to back up. Ipopt may have resolve so much data.


#### Polynomial Fitting and MPC Preprocessing
A set of 3rd order polynomial coefficients is fitted using waypoints using polyfit() function (line 127 in main.cpp). Note that based on https://github.com/udacity/CarND-MPC-Project/blob/master/DATA.md, ptsx and ptsy are global positions of car. I used convertToLocalCoordinates() (line 124 in main.cpp) to convert them to local car coordinate vectors. The local system conversion also implies the car is at (0, 0) and zero orientation angle, which simplifies the state update equations above for predicting the state after 100ms (line 145 - 150 in main.cpp). The generated state out of the model and the coefficients from poly fitting are passed into MPC::Solve() method to resolve current control output for current timestamp.

#### Model Predictive Control with Latency
As mentioned above, before the state is passed into MPS resolver, predicted state after 100ms is calculated using the kinematic model (line 145 - 150 in main.cpp).

The cost functions are tuned using cost weights (line 67 - 73 in MPC.cpp) as below

    const int cte_cost_weight = 1000;
	
    const int epsi_cost_weight = 1000;
	
    const int v_cost_weight = 1;
	
    const int delta_cost_weight = 10;
	
    const int a_cost_weight = 10;
	
    const int delta_change_cost_weight = 100;
	
    const int a_change_cost_weight = 10;
	
In this way the model will focus on minimizing the cross track error and psi error, which avoids drastic change in steering angle as instructed by section 10 of Lesson 19 MPC, Tuning MPC.


