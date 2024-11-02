#include "motor_can.h"
#include "main.h"

CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8];
uint8_t TxData[8];
uint32_t TxMailbox;
CAN_FilterTypeDef canfilterconfig;
SpeedControllerFeedback feedback;
uint8_t _flag;
uint16_t _current;
DIRECTION _dir=COUNTER_CLOCKWISE;
float rotor_angle;
float velocity_outshaft;
extern CAN_HandleTypeDef hcan1; // Khai báo CAN_HandleTypeDef
bool offset = 0;
void send_speed_controller_command(int16_t current1, int16_t current2)
{
    TxHeader.StdId = 0x200;
    TxHeader.DLC = 4;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    TxData[0] = (current1 >> 8) & 0xFF;
    TxData[1] = current1 & 0xFF;
    TxData[2] = (current2 >> 8) & 0xFF;
    TxData[3] = current2 & 0xFF;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        _flag = 1;
        Error_Handler();
    }
    else
    {
        _flag = 3;
    }
}
void get_moto_offset(SpeedControllerFeedback *ptr, uint8_t *RxData)
{
	ptr->angle_deg = (RxData[0] << 8) | RxData[1];
	ptr->offset_angle = ptr->angle_deg;
}
void update_round_count(SpeedControllerFeedback *feedback) {
    if (feedback->angle_deg - feedback->last_angle_deg < -OFFSET_THRESHOLD) {
        feedback->round_cnt++;
    } else if (feedback->angle_deg - feedback->last_angle_deg > OFFSET_THRESHOLD) {
        feedback->round_cnt--;
    }
}

void parse_speed_controller_feedback(uint16_t can_id, uint8_t *RxData, SpeedControllerFeedback *feedback)
{
    if (can_id >= 0x201 && can_id <= 0x208)
    {
        _flag = 5;
        feedback->last_angle_deg = feedback->angle_deg;
        feedback->controller_id = can_id - 0x200;
        feedback->angle_deg = (int16_t)(RxData[0] << 8) | RxData[1];
        feedback->speed_rpm = (int16_t)((RxData[2] << 8) | RxData[3]);
        velocity_outshaft = (float)feedback->speed_rpm / 36.0f;
        feedback->current_mA = (int16_t)((RxData[4] << 8) | RxData[5]);
        rotor_angle = (float)feedback->angle_deg * 360.0f / 8191.0f;

        update_round_count(feedback); // Call the helper function

        feedback->total_angle = feedback->round_cnt * 8192 + feedback->angle_deg - feedback->offset_angle;
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        Error_Handler();
    }
    if(!offset)
    {
       get_moto_offset(&feedback,RxData);
       offset=1;
    }
    parse_speed_controller_feedback(RxHeader.StdId, RxData, &feedback);

    _flag = 2;
}

void CAN_FilterConfig(void)
{
    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 0;
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterIdHigh = 0x0000;
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
    {
        Error_Handler();
    }
}

//void do_motor(DIRECTION dir, float throttle)
//{
//    if (throttle < 0) throttle = 0;
//    if (throttle > 100) throttle = 100;
//
//    switch (dir)
//    {
//        case CLOCKWISE:
//            _current = MOTOR_THR_STOP - (int)(throttle * (MOTOR_THR_STOP - MOTOR_THR_MIN) / 100);
//            break;
//        case COUNTER_CLOCKWISE:
//            _current = MOTOR_THR_STOP + (int)(throttle * (MOTOR_THR_MAX - MOTOR_THR_STOP) / 100);
//            break;
//        case STOP:
//            _current = MOTOR_THR_STOP;
//            break;
//        default:
//            break;
//    }
//    send_speed_controller_command(0, _current);
//}
