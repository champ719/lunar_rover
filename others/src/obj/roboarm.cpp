#include "roboarm.hpp"
#include "mathetics.hpp"


/* update joint info from CAN data */
void Joint::UpdateInfo(const uint8_t* data_)
{
	Update(data_);
	if (!inited)
	{
		pos_unwrapped = UnwrapRadNear(pos, offset);
		pos_wrapped_last = pos;
		LimitSelf(target.pos, angle_limit.pos_max, angle_limit.pos_min);
		plan.vel = 0.f;
		plan.pos = pos_unwrapped;
		inited = true;
	}
	else
	{
		pos_unwrapped += RadDiff(pos, pos_wrapped_last);
		pos_wrapped_last = pos;
	}
	angle.theta = mdh.theta_offset + pos_unwrapped - offset;
}

/* load target in roboarm ctrl task every 2ms */
void Joint::LoadTarget()
{
	if (!inited)
	{
		Ctrl(Motor_DM_N::order_byte_e::enable);
		return;
	}
	const float pos_diff = DeadZone(target.pos - plan.pos, 0.01f);
	const float diff_abs = Abs(pos_diff);
	if (diff_abs == 0.f && Abs(plan.vel) < 0.02f)
	{
		plan.vel = 0.f;
		plan.pos = target.pos;
		Ctrl(WrapRad(plan.pos), plan.vel, param.kp_pos, param.kp_vel, 0);
		return;
	}
	const float polarity = Sign(pos_diff);
	const float vel_planed = Limit(polarity * Sqrt(2 * plan.accel_max * diff_abs), plan.vel_max);
	plan.vel += Limit(vel_planed - plan.vel, plan.accel_max * 0.002f);  // about 500Hz
	LimitSelf(plan.vel, plan.vel_max);
	plan.pos = Limit(plan.pos + plan.vel * 0.002f, pos_unwrapped + 0.1f, pos_unwrapped - 0.1f);
	LimitSelf(plan.pos, angle_limit.pos_max, angle_limit.pos_min);
	// if (diff_abs < param.sep_pos && Abs(vel) < param.sep_vel)
	// {
	// 	param.int_pos += param.ki_pos * pos_diff;
	// }
	// param.int_pos = Limit(param.int_pos, param.int_max);
	Ctrl(WrapRad(plan.pos), plan.vel, param.kp_pos, param.kp_vel, 0);
}

void Gripper::UpdateInfo(const uint8_t* data_)
{
	Update(data_);
	if (!inited)
	{
		hold_pos = pos;
		inited = true;
	}
}

void Gripper::SetCommand(const Gripper_N::command_e cmd_)
{
	cmd = cmd_;
	stall_cnt = 0;
	if (cmd_ == Gripper_N::command_e::hold) hold_pos = pos;
}

bool Gripper::IsBlocked()
{
	const bool torque_over = Abs(torque) > Gripper_N::torque_block_thres;
	const bool vel_low = Abs(vel) < Gripper_N::vel_block_thres;
	if (torque_over && vel_low) ++stall_cnt;
	else stall_cnt = 0;
	return stall_cnt >= Gripper_N::block_cnt_thres;
}

void Gripper::Move(const float target_pos, const float target_vel) const
{
	Ctrl(target_pos, target_vel, Gripper_N::kp, Gripper_N::kd, 0.f);
}

void Gripper::Hold() const
{
	const float tq = clamped ? 0.5f : 0.f;
	Ctrl(hold_pos, 0.f, Gripper_N::kp, Gripper_N::kd, tq);
}

void Gripper::MoveTillBlock()
{
	const float cmd_vel = Gripper_N::speed_abs * static_cast<float>(static_cast<int8_t>(cmd));
	if (IsBlocked())
	{
		hold_pos = pos;
		stall_cnt = 0;
		if (cmd == Gripper_N::command_e::inward) clamped = true;
		cmd = Gripper_N::command_e::hold;  // auto switch to hold after stall
		Hold();
		return;
	}
	const float target_pos = pos + cmd_vel * Gripper_N::ctrl_dt;
	Move(target_pos, cmd_vel);
}

void Gripper::LoadTarget()
{
	if (!inited)
	{
		Ctrl(Motor_DM_N::order_byte_e::enable);
		return;
	}
	switch (cmd)
	{
	case Gripper_N::command_e::outward:
		clamped = false;
	case Gripper_N::command_e::inward:
		MoveTillBlock();
		break;
	case Gripper_N::command_e::hold:
	default:
		Hold();
		break;
	}
}
