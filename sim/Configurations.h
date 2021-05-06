#ifndef __SSC_CONFIGURATIONS_H__
#define __SSC_CONFIGURATIONS_H__

#define JOINT_DAMPING 0.05
#define TIME_STEP 0.0333333

// terminal condition
#define TERMINAL_ITERATION 3
#define TERMINAL_ROOT_DIFF_THRESHOLD 0.8
#define TERMINAL_ROOT_DIFF_ANGLE_THRESHOLD 0.4*M_PI
#define TERMINAL_ROOT_HEIGHT_LOWER_LIMIT 0.1
#define TERMINAL_ROOT_HEIGHT_UPPER_LIMIT 4.0

// character
#define CHARACTER_TYPE "skel_mxm"
#define RightHand "RightHand"
#define RightToe "RightToeBase"
#define RightFoot "RightFoot"
#define LeftHand "LeftHand"
#define LeftToe "LeftToeBase"
#define LeftFoot "LeftFoot"
#define Head "Head"
#define Root "Hips"
#define Spine "Spine"
#define Spine1 "Spine1"
#define Spine2 "Spine2"

#endif
