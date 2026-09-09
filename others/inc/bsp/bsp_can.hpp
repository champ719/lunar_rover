#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"

#ifdef __cplusplus
}
#endif


struct can_err_s
{
	FDCAN_HandleTypeDef* err_hcan;  // initialized as nullptr in C++ 11 and later
	uint32_t err_bits;
	uint32_t hal_error;
	uint32_t last_protocol_error;
	uint32_t tx_error_cnt;
	uint32_t rx_error_cnt;
	uint32_t bus_off;
};

/* always check CubeMX to ensure the FDCAN initialization settings match */
class Bsp_CAN
{
	public:
	explicit Bsp_CAN(FDCAN_HandleTypeDef* _hcan, const bool _is_extended = false,
		const bool _is_brs = false, const bool _is_fd = false)
	: m_handle(_hcan), m_tx_header {0, _is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
		FDCAN_DATA_FRAME, 8, FDCAN_ESI_ACTIVE,
		_is_brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF, _is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN,
		FDCAN_NO_TX_EVENTS, 0}, err_cnt(0), err{}
	{}

	void FilterConfig(bool is_extended_ = false) const;
	void ConfigTDC(uint32_t tdc_offset_, uint32_t tdc_filter_) const;

	void Start() const
	{
		if (HAL_FDCAN_Start(m_handle) != HAL_OK) Error_Handler();
		HAL_FDCAN_ActivateNotification(m_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF |
			FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0);
	}
	void Reset() const
	{
		if (HAL_FDCAN_Stop(m_handle) == HAL_OK)
		{
			if (HAL_FDCAN_Start(m_handle) != HAL_OK) Error_Handler();
			HAL_FDCAN_ActivateNotification(m_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF |
			FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0);
		}
	}

	void Transmit(uint32_t id_, const uint8_t* data_, uint8_t length_ = 8);
	void Receive() { HAL_FDCAN_GetRxMessage(m_handle, FDCAN_RX_FIFO0, &m_rx_header, m_rx_buffer); }

	FDCAN_RxHeaderTypeDef* GetRxHeader() { return &m_rx_header; }
	uint8_t* GetRxData() { return m_rx_buffer; }
	[[nodiscard]] FDCAN_HandleTypeDef* GetHandle() const { return m_handle; }

	friend void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs);
	// friend void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs);
	friend void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs);

	private:
	FDCAN_HandleTypeDef* m_handle;
	FDCAN_TxHeaderTypeDef m_tx_header;
	FDCAN_RxHeaderTypeDef m_rx_header {};
	uint8_t m_rx_buffer[8] {};
	uint8_t err_cnt;
	can_err_s err;
};
