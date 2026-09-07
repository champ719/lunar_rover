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
	};

	inline constexpr std::array<mdh_param_s, 7> mdh_left
	{{
		{0.f, 0.f, 0.f, 0.f},
		{0.f, 0.f, 0.f, 0.f},
		{0.f, 0.f, 0.f, 0.f},
		{0.f, 0.f, 0.f, 0.f},
		{0.f, 0.f, 0.f, 0.f},
		{0.f, 0.f, 0.f, 0.f},
		{0.f, 0.f, 0.f, 0.f}
	}};
}

class Joint : public Motor_DM
{
	public:
	explicit Joint(const uint8_t host_id_, const uint8_t motor_id_, const Motor_DM_N::param_factor_s& facs_,
		Bsp_CAN& hcan_, const Joint_N::target_s& target_, const Joint_N::angle_limit_s& angle_limit_,
		const float offset_, const Joint_N::param_s& pid_param_, const Joint_N::plan_s& plan_)
		: Motor_DM(host_id_, motor_id_, facs_, hcan_), target(target_), angle(angle_limit_), offset(offset_),
		pos_relative(0.f), param{pid_param_}, plan(plan_), inited(false)
	{}
	void UpdateInfo(const uint8_t* data_);
	void UpdateTarget(float delta_pos_);
	// void UpdateTarget(bool polarity_);
	void LoadTarget();
	void Follow() const;
	void ReturnHomePos();

	private:
	Joint_N::target_s target;
	const Joint_N::angle_limit_s angle;
	const float offset;
	float pos_relative;  // relative to offset
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
