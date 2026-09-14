#include "bsp_tim.hpp"
#include "mathetics.hpp"
#include "roboarm.hpp"
#include "roboarm_kinematics.hpp"

/* 
          上电机械零位      上电关节角
j1 8009p  -0.21044         
j2 8009p -2.38221
j3 4340p 0.99863
j4 4310P -2.80770
J5 4310P 0.58819
J6 4310 -1.448012
J7 3507 -1.28081
*/
/* place all variables in one file to avoid initialization order issues */

Timebase_Timer micro_base;

/* Match MX_FDCAN3_Init(): standard-ID classic CAN frames for DAMIAO motors. */
Bsp_CAN abc_can3(&hfdcan3, false, true, true);

constexpr Motor_DM_N::param_factor_s dm3507 = Motor_DM_N::FacZoom(3.14159f, 50.f, 5.f);
constexpr Motor_DM_N::param_factor_s dm4340p = Motor_DM_N::FacZoom(3.14159f, 10.f, 28.f);
constexpr Motor_DM_N::param_factor_s dm4310 = Motor_DM_N::FacZoom(3.14159f, 30.f, 10.f);
constexpr Motor_DM_N::param_factor_s dm4310p = Motor_DM_N::FacZoom(3.14159f, 30.f, 10.f);
constexpr Motor_DM_N::param_factor_s dm8009p = Motor_DM_N::FacZoom(3.14159f, 45.f, 54.f);

__attribute__((used)) Joint j1_left(0x11, 0x01, dm8009p, abc_can3,
	{1.44042873f}, {Math_N::two_pi - 2.8f, -1.7f}, 
	-0.14122, RoboArm_N::mdh_left[0],  
	{0, 0, 0.f, 0.f, 0.f, 0.f, 0.f},
	{6.f, 2.f, 0.f, 1.44042873f});

__attribute__((used)) Joint j2_left(0x12, 0x02, dm8009p, abc_can3,
	{1.8f}, {Math_N::two_pi - 2.4f, 1.75f}, 
	-2.38192f, RoboArm_N::mdh_left[1],  
	{0, 0, 0.f, 0.f, 0.f, 0.f, 0.f},
	{6.f, 2.f, 0.f, 1.8f});

__attribute__((used)) Joint j3_left(0x13, 0x03, dm4340p, abc_can3,
	{-0.0510056764f}, {0.99f, -Math_N::pi}, 
	1.00439f, RoboArm_N::mdh_left[2], 
	{0, 0, 0.f, 0.f, 0.f, 0.f, 0.f},
	{6.f, 2.f, 0.f, -0.0510056764f});

__attribute__((used)) Joint j4_left(0x14, 0x04, dm4310p, abc_can3,
	{Math_N::two_pi - 1.64398444f}, {Math_N::two_pi - 0.15f, 0.8f},
	-2.80760, RoboArm_N::mdh_left[3], 
	{0, 0, 0.2f, 5.f, 2.f, 0.5f, 0.f},
	{6.f, 2.f, 0.f, Math_N::two_pi - 1.14398444f});

__attribute__((used)) Joint j5_left(0x15, 0x05, dm4310, abc_can3,
	{0.0774672702f}, {1.f, -2.6f}, 
	0.62223f, RoboArm_N::mdh_left[4],  
	{0, 0, 0.2f, 2.f, 3.2f, 0.2f, 0.f},
	{6.f, 2.f, 0.f, 0.0774672702f});

__attribute__((used)) Joint j6_left(0x16, 0x06, dm4310, abc_can3,
	{-0.00671127299f}, {1.8f, 2.85f - Math_N::two_pi}, 
	-0.77208, RoboArm_N::mdh_left[5],  
	{0, 0, 0.2f, 2.f, 3.2f, 0.2f, 0.f},
	{6.f, 2.f, 0.f, -0.00671127299f});

__attribute__((used)) Joint j7_left(0x17, 0x07, dm3507, abc_can3,
	{-0.00671127299f}, {3.f, -3.f}, 
	3.11051f, RoboArm_N::mdh_left[6],  
	{0, 0, 0.2f, 2.f, 3.2f, 0.2f, 0.f},
	{6.f, 2.f, 0.f, -0.00671127299f});

/* 7 DAMIAO joint motors on FDCAN3 (motor_id 0x01..0x06, host_id 0x11..0x16).
   Add more Joint instances here (e.g. j7: 0x17/0x07) if the arm grows. */

/* 机械臂运动学解算：由各关节角度解算转轴相对基座 z 轴的倾角，写入 Joint::angle.base */
RoboArmJointAngles robo_arm_kin({j1_left, j2_left, j3_left, j4_left, j5_left, j6_left, j7_left});
