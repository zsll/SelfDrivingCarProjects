//
//  consts.h
//  Path_Planning
//
//  Created by Yangzi Liu on 1/31/18.
//

#ifndef consts_h
#define consts_h


#define MAX_SPEED 49.5  // Same with project walk through Q&A
#define SIDE_SAFE_DISTANCE 5
#define FRONT_SAFE_DISTANCE 3 // Kind of aggressive
#define BACK_SAFE_DISTANCE 10
#define MAX_ACCELERATION 4.5 * 0.02 * 2.24 // 10m/s^3 is max jerk, so in 0.02s, it can speed up to 0.02s * 10 * 2.24 miles/s^3
#define MAX_DECELERATION -4.5 * 0.02 * 2.24  // Here MAX_ACCELERATION - MAX_DECELERATION = 9 * 0.02 * 2.24 so it's not over max jerk. Note, this does not include jerk on d direction


#endif /* consts_h */
