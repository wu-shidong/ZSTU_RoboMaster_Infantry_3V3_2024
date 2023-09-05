#include "bsp_buzzer.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
/**
 * @brief  蜂鸣器初始化
 * @param  None
 * @retval None
 */
void buzzer_init(void)
{
	//start tim
	//开启定时器
	HAL_TIM_Base_Start(&htim4);
	//start pwm channel
	//开启PWM通道
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
/**
 * @brief  蜂鸣器开
 * @param  psc: 预分频值
 * @param  pwm: PWM值
 * @retval None
 */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
/**
 * @brief  蜂鸣器关
 * @param  None
 * @retval None
 */
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
/**
 * @brief  蜂鸣器高音
 * @param  None
 * @retval None
 */
void buzzer_high(void)
{
	  __HAL_TIM_PRESCALER(&htim4, 1);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
}
/**
 * @brief  蜂鸣器中音
 * @param  None
 * @retval None
 */
void buzzer_mid(void)
{
	  __HAL_TIM_PRESCALER(&htim4, 7);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
}
/**
 * @brief  蜂鸣器低音
 * @param  None
 * @retval None
 */
void buzzer_low(void)
{
	  __HAL_TIM_PRESCALER(&htim4, 20);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
}
