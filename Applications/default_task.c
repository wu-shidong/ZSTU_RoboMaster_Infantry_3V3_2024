#include "default_task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "bsp_servo_pwm.h"
/* USER CODE BEGIN Header */
/**
* @brief Function implementing the Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
void default_task(void const * argument)
{
  /* USER CODE BEGIN  */
  /* Infinite loop */
	while(1)
	{
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
		osDelay(500);
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
		osDelay(500);
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		osDelay(500);
//		CAN_cmd_chassis(200,200,200,-200);
//		CAN_cmd_chassis(0,0,0,0);
		osDelay(10);
	}
  /* USER CODE END */
}



