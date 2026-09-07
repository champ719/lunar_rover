#include "threads.hpp"
#include "roboarm.hpp"

#ifdef __cplusplus
extern "C" {
#endif


extern osThreadId_t ErrorHandle;
extern osThreadId_t roboARMHandle;


#ifdef __cplusplus
}
#endif


extern Joint j1_left;
extern Joint j2_left;
extern Joint j3_left;
extern Joint j4_left;
extern Joint j5_left;
extern Joint j6_left;

/* implements the CubeMX "Error" thread (weak stub in freertos.c) */
void Error_tasks(void *argument)
{
	UNUSED(argument);
	using namespace Motor_DM_N;
	uint32_t notif_val = 0;
	bool suspended = false;  // mark the first time getting in
	for (;;)
	{
		if (!suspended) { notif_val = osThreadFlagsWait(0x00000003, osFlagsNoClear, osWaitForever); }
		else { notif_val = osThreadFlagsWait(0x00000003, osFlagsNoClear, 0); }
		if (notif_val & 0x00000001)
		{
			if (!suspended)
			{
				osThreadSuspend(roboARMHandle);
				suspended = true;
			}
			j1_left.Ctrl(order_byte_e::disable);
			j2_left.Ctrl(order_byte_e::disable);
			j3_left.Ctrl(order_byte_e::disable);
			j4_left.Ctrl(order_byte_e::disable);
			osDelay(1);
			j5_left.Ctrl(order_byte_e::disable);
			j6_left.Ctrl(order_byte_e::disable);
			osDelay(1);
		}
		if (notif_val == 0x00000003)  // will cause reset when getting into error
		{
			HAL_NVIC_SystemReset();
		}
	}
}
