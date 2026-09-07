#include "roboarm.hpp"


extern Bsp_CAN abc_can3;

extern Joint j1_left;
extern Joint j2_left;
extern Joint j3_left;
extern Joint j4_left;
extern Joint j5_left;
extern Joint j6_left;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	UNUSED(RxFifo0ITs);
	if (hfdcan == abc_can3.m_handle)  // friendly
	{
		abc_can3.Receive();
		switch (abc_can3.m_rx_header.Identifier)
		{
		case 0x11:
			j1_left.UpdateInfo(abc_can3.m_rx_buffer);
			break;
		case 0x12:
			j2_left.UpdateInfo(abc_can3.m_rx_buffer);
			break;
		case 0x13:
			j3_left.UpdateInfo(abc_can3.m_rx_buffer);
			break;
		case 0x14:
			j4_left.UpdateInfo(abc_can3.m_rx_buffer);
			break;
		case 0x15:
			j5_left.UpdateInfo(abc_can3.m_rx_buffer);
			break;
		case 0x16:
			j6_left.UpdateInfo(abc_can3.m_rx_buffer);
			break;
		default:
			break;
		}
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if (hfdcan == abc_can3.m_handle)
	{
		abc_can3.err.err_hcan = hfdcan;
		abc_can3.err.err_bits = ErrorStatusITs;
		++abc_can3.err_cnt;
	}
}
