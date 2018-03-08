# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

# Compile and Run Instructions

There are two ways to build and run the code:

### Manual Build
1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./pid`. 

### Using Xcode
A Xcode project "PID.xcodeproj" is generated. Just run it in Xcode. Click on the "ALL_BUILD" Scheme on the top left. Dropdown with Schemes will open. Select 'pid' then hit 'Run'. 

## Reflection

#### Describe the effect each of the P, I, D components

1. P component: This is the intuitive part and core to PID controller. Even without other components it's possible to solely rely on it to keep the car on track, though it will sway back and forth around the reference. In case of larger throttle (adjusted in line 74 in main.cpp), it will [easily go over the curb due to such zigzag movement] (https://youtu.be/2XNduAeRZAg).

2. D component: Introduced to solves the sway issue. Cannot work independent of P component since it only measures CTE difference. The video of [only using D component](https://youtu.be/HJtFK0TRHBE) (when setting Kp and Ki to zero in initialization) shows it can't be used for reference tracking without Kp. However, in addition to Kp, a PD controller renders smooth lane tracking performance. This should be sufficient for a car with perfect wheel alignment, while a drift (introduced in line 72,73 in main.cpp) would cause [a sustained drift] (https://youtu.be/S-ouFKXJ02E). The car in the video has consistent bias to the right of the track.

3.  I component: The PID controller [eliminates the offset in case of drift] (https://youtu.be/edQcNYFBwgU) (final code version), while it's OK to set it to zero in non-drift scenario. PD controller will even converge faster in such case.

#### Parameter Selection
 The parameters are chosen using a manual process: firstly I try to build a PD controller in non-drift case. Then larger kp will cause more severe oscillation, so it's easy to firstly lock it around 0.1. Then since CTE difference between two frames are usually small, need a larger kd to impose some influence on the final result. Increasing kd will also cause the stability to degrade. After the PD is working, I added drift into the model and started tuning ki. This efficient needs to be dramatically small or else it will navigate the car out of track immediately. In the start of the video the car controlled by PID might still say for seconds. A twiddle process will help it quickly converge to the reference. Note that in case of different throttle (speed), the parameters should be different to satisfy varied response time requirement. In reality, it might require a car to travel on many terrains at divided speed levels to acquire parameter sets for multiple PID controllers.



