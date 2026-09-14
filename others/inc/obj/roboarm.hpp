#pragma once

#include <array>
#include <functional>
#include "motor_dm.hpp"


namespace Joint_N
{
	struct angle_limit_s
	{
		float pos_max;
		float pos_min;
	};
	struct target_s
	{
		float pos;
	};
	struct plan_s
	{
		float accel_max;
		float vel_max;
		float vel;
		float pos;
	};
	struct param_s
	{
		uint16_t kp_pos;
		uint16_t kp_vel;
		float ki_pos;
		float int_max;
		float sep_vel;
		float sep_pos;
		float integral;
	};

	/* 统一存放关节的角度信息 */
	struct angle_s
	{
		float theta;  // 关节角 θ = mdh.theta_offset + (编码器连续角 - offset)
		float base;   // 连杆在基座系下与基座 z 轴的倾角（由运动学解算写入）
	};
}

namespace Gripper_N
{
	enum class command_e : int8_t  // outward positive
	{
		outward = 1,
		inward = -1,
		hold = 0
	};

	inline constexpr float ctrl_dt = 0.002f;  // 500 Hz
	inline constexpr uint16_t kp = 88;
	inline constexpr uint16_t kd = 240;
	inline constexpr float speed_abs = 3.0f;
	inline constexpr float torque_block_thres = 0.6f;
	inline constexpr float vel_block_thres = 0.1f;
	inline constexpr uint8_t block_cnt_thres = 100;  // consecutive control cycles

	// inline constexpr float radius_gripper = 1.f;  // unnecessary
}

namespace RoboArm_N
{
	struct mdh_param_s
	{
		float alpha;
		float a;  // i - 1
		float theta_offset;
		float d;  // i
		float axis[3];  // 该关节对应连杆在自身局部坐标系中的方向向量（单位向量：x/y/z）
	};

	inline constexpr std::array<mdh_param_s, 7> mdh_left
	{{
		{0.f, 0.f, 0.f, 0.f, 
			{0.f, 0.f, 1.f}},             
		{1.57080f, 0.f, 2.65289f, 0.f, 
			{1.f, 0.f, 0.f}},    
		{3.14159f, 0.f, 4.22348f, 0.f, 
			{0.f, -1.f, 0.f}},    
		{1.57080f, 0.f, 3.14159f, 0.f, 
			{0.f, 0.f, 1.f}},        
		{1.57080f, 0.f, 4.57276f, 0.f, 
			{0.f, -1.f, 0.f}},   
		{1.57080f, 0.f, 0.f, 0.f, 
			{0.f, 0.f, 1.f}},         
		{0.f, 0.f, 0.f, 0.f, 
			{0.f, 0.f, 1.f}}               
	 }};
}

class Joint : public Motor_DM
{
	public:
	explicit Joint(const uint8_t host_id_, const uint8_t motor_id_, const Motor_DM_N::param_factor_s& facs_,
		Bsp_CAN& hcan_, const Joint_N::target_s& target_, const Joint_N::angle_limit_s& angle_limit_,
		const float offset_, const RoboArm_N::mdh_param_s& mdh_,
		const Joint_N::param_s& pid_param_, const Joint_N::plan_s& plan_)
		: Motor_DM(host_id_, motor_id_, facs_, hcan_), angle{}, target(target_), angle_limit(angle_limit_),
		offset(offset_), mdh(mdh_), pos_wrapped_last(0.f), pos_unwrapped(0.f),
		param{pid_param_}, plan(plan_), inited(false)
	{}
	void UpdateInfo(const uint8_t* data_);
	void LoadTarget();

	Joint_N::angle_s angle;                                    // 角度信息：theta + base
	const RoboArm_N::mdh_param_s& Mdh() const { return mdh; }  // 改进 DH 参数

	private:
	Joint_N::target_s target;
	const Joint_N::angle_limit_s angle_limit;
	const float offset;
	const RoboArm_N::mdh_param_s mdh;
	float pos_wrapped_last;
	float pos_unwrapped;
	Joint_N::param_s param;
	Joint_N::plan_s plan;
	bool inited;
};

class Gripper : public Motor_DM
{
	public:
	explicit Gripper(const uint8_t host_id_, const uint8_t motor_id_, const Motor_DM_N::param_factor_s& facs_,
		Bsp_CAN& hcan_)
		: Motor_DM(host_id_, motor_id_, facs_, hcan_), hold_pos(0.f), cmd(Gripper_N::command_e::hold),
		stall_cnt(0), inited(false), clamped(false)
	{}
	void UpdateInfo(const uint8_t* data_);
	void SetCommand(Gripper_N::command_e cmd_);
	void LoadTarget();

	private:
	bool IsBlocked();
	void Move(float target_pos, float target_vel) const;
	void Hold() const;
	void MoveTillBlock();

	float hold_pos;
	Gripper_N::command_e cmd;
	uint8_t stall_cnt;
	bool inited;
	bool clamped;
};

class RoboArm
{
	public:
	explicit RoboArm(const std::array<std::reference_wrapper<Joint>, 7>& joints_,
		const std::array<RoboArm_N::mdh_param_s, 7>& mdh_params_)
		: joints(joints_), mdh_params(mdh_params_)
	{}
	std::array<std::reference_wrapper<Joint>, 7> joints;
	std::array<RoboArm_N::mdh_param_s, 7> mdh_params;
};
