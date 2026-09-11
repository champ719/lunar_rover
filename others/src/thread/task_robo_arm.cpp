#include "threads.hpp"
#include "roboarm.hpp"


extern Joint j1_left;
extern Joint j2_left;
extern Joint j3_left;
extern Joint j4_left;
extern Joint j5_left;
extern Joint j6_left;
extern Joint j7_left;


/* implements the CubeMX "roboARM" thread (weak stub in freertos.c) */
void roboARM_task(void *argument)
{
	UNUSED(argument);
	using namespace Motor_DM_N;
	j1_left.Ctrl(order_byte_e::disable);
	j2_left.Ctrl(order_byte_e::disable);
	j3_left.Ctrl(order_byte_e::disable);
	j4_left.Ctrl(order_byte_e::disable);
	osDelay(1);
	j5_left.Ctrl(order_byte_e::disable);
	j6_left.Ctrl(order_byte_e::disable);
	j7_left.Ctrl(order_byte_e::disable);
	osDelay(1);
	j1_left.Ctrl(order_byte_e::clear_error);
	j2_left.Ctrl(order_byte_e::clear_error);
	j3_left.Ctrl(order_byte_e::clear_error);
	j4_left.Ctrl(order_byte_e::clear_error);
	osDelay(1);
	j5_left.Ctrl(order_byte_e::clear_error);
	j6_left.Ctrl(order_byte_e::clear_error);
	j7_left.Ctrl(order_byte_e::clear_error);
	osDelay(1);
	j1_left.Ctrl(order_byte_e::enable);
	j2_left.Ctrl(order_byte_e::enable);
	j3_left.Ctrl(order_byte_e::enable);
	j4_left.Ctrl(order_byte_e::enable);
	osDelay(1);
	j5_left.Ctrl(order_byte_e::enable);
	j6_left.Ctrl(order_byte_e::enable);
	j7_left.Ctrl(order_byte_e::enable);
	osDelay(1);
	for (;;)
	{
		j1_left.LoadTarget();
		j2_left.LoadTarget();
		j3_left.LoadTarget();
		j4_left.LoadTarget();
		osDelay(1);
		j5_left.LoadTarget();
		j6_left.LoadTarget();
		j7_left.LoadTarget();
		osDelay(1);
	}
}
