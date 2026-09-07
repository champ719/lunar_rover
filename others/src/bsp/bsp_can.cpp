#include "bsp_can.hpp"


/* CAN filter configuration */
void Bsp_CAN::FilterConfig(const bool is_extended_) const
{
	const FDCAN_FilterTypeDef filter_cfg_t {is_extended_ ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
		0, FDCAN_FILTER_MASK, FDCAN_FILTER_TO_RXFIFO0,
		0x00000000, 0x00000000, 0, 0};
	if (HAL_FDCAN_ConfigFilter(m_handle, &filter_cfg_t) != HAL_OK) Error_Handler();
	HAL_FDCAN_ConfigGlobalFilter(m_handle, FDCAN_REJECT, FDCAN_REJECT,
		FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(m_handle, FDCAN_CFG_RX_FIFO0, 1);
}

/* config TxDelayCompensation in FD mode with BRS */
void Bsp_CAN::ConfigTDC(const uint32_t tdc_offset_, const uint32_t tdc_filter_) const
{
	if (!m_tx_header.BitRateSwitch || ! m_tx_header.FDFormat) { return; }
	if (HAL_FDCAN_ConfigTxDelayCompensation(m_handle, tdc_offset_, tdc_filter_) != HAL_OK) { Error_Handler(); }
	if (HAL_FDCAN_EnableTxDelayCompensation(m_handle) != HAL_OK) { Error_Handler(); }
}

/* CAN message transmission */
void Bsp_CAN::Transmit(const uint32_t id_, const uint8_t* data_, const uint8_t length_)
{
	m_tx_header.Identifier = id_;
	m_tx_header.DataLength = length_;
	if (HAL_FDCAN_AddMessageToTxFifoQ(m_handle, &m_tx_header, data_) != HAL_OK) {}
}
