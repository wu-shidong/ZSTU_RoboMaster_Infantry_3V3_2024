#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/**
 * @brief  CAN过滤器初始化 
 * @param  None
*/
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;//CAN滤波器结构体
    can_filter_st.FilterActivation = ENABLE;//使能滤波器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//ID屏蔽模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//32位滤波器
    can_filter_st.FilterIdHigh = 0x0000;//32位ID
    can_filter_st.FilterIdLow = 0x0000;//32位ID
    can_filter_st.FilterMaskIdHigh = 0x0000;//32位屏蔽ID
    can_filter_st.FilterMaskIdLow = 0x0000;//32位屏蔽ID
    can_filter_st.FilterBank = 0;//过滤器组0
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//过滤器组0关联到FIFO0
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);//配置过滤器
    HAL_CAN_Start(&hcan1);//开启CAN1
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//开启中断


    can_filter_st.SlaveStartFilterBank = 14;//从过滤器组14开始
    can_filter_st.FilterBank = 14;//过滤器组14
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);//配置过滤器
    HAL_CAN_Start(&hcan2);//开启CAN2
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//开启中断



}
