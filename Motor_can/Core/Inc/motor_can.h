
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_CAN_H
#define __MOTOR_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "stm32f4xx_hal.h"
/* ------------------------------------------------------------------ */
#define MOTOR_THR_STOP 0
#define MOTOR_THR_MAX 1000
#define MOTOR_THR_MIN -1000
#define OFFSET_THRESHOLD 4096
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

typedef struct {
    uint16_t last_angle_deg;  // range:[0-8191]
    uint16_t angle_deg;       // range:[0-8191]
    uint16_t offset_angle;    // range:[0-8191]
    int16_t speed_rpm;
    int16_t current_mA;
    uint8_t controller_id;
    int32_t round_cnt;
    int32_t total_angle;
} SpeedControllerFeedback;
typedef enum DIRECTION
{
	COUNTER_CLOCKWISE,
	CLOCKWISE,
	STOP
}DIRECTION;


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void CAN_FilterConfig(void);
void parse_speed_controller_feedback(uint16_t can_id, uint8_t *RxData,SpeedControllerFeedback *f);
void send_speed_controller_command(int16_t current1, int16_t current2);
void get_moto_offset(SpeedControllerFeedback *ptr, uint8_t *RxData);
//void wrap_around(SpeedControllerFeedback *f);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif
