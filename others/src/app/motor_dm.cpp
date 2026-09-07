#include <cstring>
#include "motor_dm.hpp"


using namespace Motor_DM_N;

/* update motor info from rx_data */
void Motor_DM::Update(const uint8_t* const data_)
{
	if (__builtin_expect(data_ == nullptr, 0)) return;
	// if (__builtin_expect(id_ != host_id, 0)) return;
	if (__builtin_expect(data_[0] == motor_id && data_[2] == 0x33, 0))
	{
		std::memcpy(&param, data_ + 4, 4);
	}
	else
	{
		error = static_cast<error_state_e>(data_[0] >> 4);
		pos = (static_cast<float>(data_[1] << 8 | data_[2]) - 32767.f) * m_facs.bit_to_pos;  // 0 -> 32767(.5) -> 0.f
		vel = (static_cast<float>(data_[3] << 4 | data_[4] >> 4) - 2047.f) * m_facs.bit_to_vel;
		torque = (static_cast<float>((data_[4] & 0xF) << 8 | data_[5]) - 2047.f) * m_facs.bit_to_torque;
		temperature_mos = static_cast<int8_t>(data_[6]);
		temperature_rotor = static_cast<int8_t>(data_[7]);
	}
	++m_rx_cnt;
}

/*
 * mit protocol command
 * _pos, _vel and _torque_ff range set by the host
 * _kp range [0, 500], _kd range[0, 5]
 */
void Motor_DM::Ctrl(const float pos_, const float vel_, const uint16_t kp_, const uint16_t kd_,
	const float torque_ff_) const
{
	const auto angle = static_cast<uint16_t>(pos_ * m_facs.pos_to_bit + 32767.5f);
	const auto speed = static_cast<uint16_t>(vel_ * m_facs.vel_to_bit + 2047.5f);
	const auto torque_ff = static_cast<uint16_t>(torque_ff_ * m_facs.torque_to_bit + 2047.5f);
	const uint8_t tx_data[8] {static_cast<uint8_t>(angle >> 8), static_cast<uint8_t>(angle),
        static_cast<uint8_t>(speed >> 4), static_cast<uint8_t>((speed & 0xF) << 4 | (kp_ >> 8 & 0xF)),
        static_cast<uint8_t>(kp_), static_cast<uint8_t>(kd_ >> 4),
        static_cast<uint8_t>((kd_ & 0xF) << 4 | (torque_ff >> 8 & 0xF)), static_cast<uint8_t>(torque_ff)};
	m_hcan.Transmit(motor_id, tx_data);
	++m_tx_cnt;
}

/*
 * pos-vel protocol command
 * trapezoidal vel control
 */
void Motor_DM::Ctrl(const float pos_, const float vel_) const
{
	const auto pos_buf = reinterpret_cast<const uint8_t*>(&pos_);
	const auto vel_buf = reinterpret_cast<const uint8_t*>(&vel_);
	const uint8_t tx_data[8] {*pos_buf, *(pos_buf + 1), *(pos_buf + 2), *(pos_buf + 3),
		*vel_buf, *(vel_buf + 1), *(vel_buf + 2), *(vel_buf + 3)};
	m_hcan.Transmit(motor_id & POS_VEL_MODE_OFFSET, tx_data);
	++m_tx_cnt;
}

/*
 * vel protocol command
 * data length 4 bytes
 */
void Motor_DM::Ctrl(const float vel_) const
{
	const auto vel_buf = reinterpret_cast<const uint8_t*>(&vel_);
	const uint8_t tx_data[4] {*vel_buf, *(vel_buf + 1), *(vel_buf + 2), *(vel_buf + 3)};
	m_hcan.Transmit(motor_id & VEL_MODE_OFFSET, tx_data, 4);
	++m_tx_cnt;
}

/* misc command */
void Motor_DM::Ctrl(const order_byte_e order_) const
{
    const uint8_t tx_data[8] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, static_cast<uint8_t>(order_)};
    m_hcan.Transmit(motor_id, tx_data);
	++m_tx_cnt;
}

/* get params from motor registers */
void Motor_DM::GetParam(const param_index_e index_) const
{
	const uint8_t tx_data[4] {motor_id, 0, 0x33, static_cast<uint8_t>(index_)};
	m_hcan.Transmit(0x7FF, tx_data, 4);
	++m_tx_cnt;
}

/* set unsigned int param */
void Motor_DM::SetParam(const param_index_e index_, const uint32_t param_) const
{
	const uint8_t tx_data[8] {motor_id, 0, 0x55, static_cast<uint8_t>(index_), static_cast<uint8_t>(param_),
		static_cast<uint8_t>(param_ >> 8), static_cast<uint8_t>(param_ >> 16), static_cast<uint8_t>(param_ >> 24)};
	m_hcan.Transmit(0x7FF, tx_data, 8);
	++m_tx_cnt;
}

/* set float param */
void Motor_DM::SetParam(const param_index_e index_, const float param_) const
{
	uint8_t tx_data[8] {motor_id, 0, 0x55, static_cast<uint8_t>(index_),};
	std::memcpy(tx_data, &param_, 4);
	m_hcan.Transmit(0x7FF, tx_data, 8);
	++m_tx_cnt;
}

/* let motor save all current params, taking about 30ms */
void Motor_DM::SaveParam() const
{
	const uint8_t tx_data[4] {motor_id, 0, 0xAA, 0x01};
	m_hcan.Transmit(0x7FF, tx_data, 4);
	++m_tx_cnt;
}
