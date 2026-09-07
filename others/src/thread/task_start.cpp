#include "threads.hpp"
#include "bsp_can.hpp"
#include "bsp_tim.hpp"


#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId_t defaultTaskHandle;

#ifdef __cplusplus
}
#endif


extern Timebase_Timer micro_base;

extern Bsp_CAN abc_can3;

/* implements the CubeMX "defaultTask" thread (weak stub in freertos.c) */
void StartDefaultTask(void *argument)
{
	UNUSED(argument);
	micro_base.Init();
	abc_can3.FilterConfig();
	abc_can3.ConfigTDC(31, 1);
	abc_can3.Start();
	osThreadTerminate(defaultTaskHandle);  // obtained by osThreadNew or osThreadGetId
}
