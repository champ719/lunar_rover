#pragma once

#include "bsp_can.hpp"

#define POS_VEL_MODE_OFFSET 0x100
#define VEL_MODE_OFFSET 0x200


namespace Motor_DM_N
{
	enum class error_state_e : uint8_t
	{
		disabled = 0x00,
		enabled = 0x01,
		over_voltage = 0x08,
		under_voltage = 0x09,
		over_current = 0x0A,
		over_temperature_mos = 0x0B,
		over_temperature_rotor = 0x0C,
		comms_lost = 0x0D,
		overload = 0x0E
	};

	enum class order_byte_e : uint8_t
	{
		clear_error = 0xFB,
		enable = 0xFC,
		disable = 0xFD,
		set_zero = 0xFE
	};

	enum class param_index_e : uint8_t
	{
		uv_value = 0x00,
		kt_value = 0x01,
		ot_value = 0x02,
		oc_value = 0x03,
		acc = 0x04,
		dec = 0x05,
		max_spd = 0x06,
		mst_id = 0x07,
		esc_id = 0x08,
		timeout = 0x09,
		ctrl_mode = 0x0A,
		damp = 0x0B,
		inertia = 0x0C,
		sw_ver = 0x0E,
		npp = 0x10,
		rs = 0x11,
		ls = 0x12,
		flux = 0x13,
		gr = 0x14,
		pmax = 0x15,
		vmax = 0x16,
		tmax = 0x17,
		i_bw = 0x18,
		kp_asr = 0x19,
		ki_asr = 0x1A,
		kp_apr = 0x1B,
		ki_apr = 0x1C,
		ov_value = 0x1D,
		gref = 0x1E,
		deta = 0x1F,
		v_bw = 0x20,
		iq_c1 = 0x21,
		vl_c1 = 0x22,
		can_br = 0x23,  // 4 for 1M, 9 for 5M
		sub_ver = 0x24,
		boot_ver = 0x25,
		dir = 0x37,
		m_off = 0x38,
		imax = 0x3B,
		vbus = 0x3C,
		tpcb = 0x3D,
		tmtr = 0x3E,
		iu_off = 0x3F,
		iv_off = 0x40,
		iw_off = 0x41,
		p_m = 0x50,
		xout = 0x51
	};

	struct param_factor_s
	{
		const float bit_to_pos;
		const float bit_to_vel;
		const float bit_to_torque;
		const float pos_to_bit;
		const float vel_to_bit;
		const float torque_to_bit;
	};

	constexpr param_factor_s FacZoom(const float _pos_max, const float _vel_max, const float _torque_max)
	{
		return {_pos_max / 32767.5f, _vel_max / 2047.5f,
			_torque_max / 2047.5f, 32767.5f / _pos_max,
			2047.5f / _vel_max, 2047.5f / _torque_max};
	}
}

class Motor_DM
{
	public:
	explicit Motor_DM(const uint8_t host_id_, const uint8_t motor_id_,
		const Motor_DM_N::param_factor_s& facs_, Bsp_CAN& hcan_)
		: host_id(host_id_), motor_id(motor_id_), error(Motor_DM_N::error_state_e::disabled), pos(0.f), vel(0.f),
		torque(0.f), temperature_mos(0), temperature_rotor(0), m_facs(facs_), m_hcan(hcan_), m_tx_cnt(0), m_rx_cnt(0)
	{}
	void Update(const uint8_t* data_);
	void Ctrl(float pos_, float vel_, uint16_t kp_, uint16_t kd_, float torque_ff_) const;
	void Ctrl(float pos_, float vel_) const;
	void Ctrl(float vel_) const;
	void Ctrl(Motor_DM_N::order_byte_e order_) const;
	void GetParam(Motor_DM_N::param_index_e index_) const;
	void SetParam(Motor_DM_N::param_index_e index_, uint32_t param_) const;
	void SetParam(Motor_DM_N::param_index_e index_, float param_) const;
	void SaveParam() const;
	friend void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs);
	uint32_t param = 0;

	protected:
	const uint8_t host_id;
	const uint8_t motor_id;
	Motor_DM_N::error_state_e error;
	float pos;  // rad
	float vel;  // rad/s
	float torque;  // Nm
	int8_t temperature_mos;
	int8_t temperature_rotor;
	const Motor_DM_N::param_factor_s& m_facs;
	Bsp_CAN& m_hcan;
	mutable uint16_t m_tx_cnt;  // can be modified in const methods
	uint16_t m_rx_cnt;
};
