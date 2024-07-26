/*
 * timestamp.c
 *
 *  Created on: Jun 17, 2024
 *      Author: vichu
 */


#include "timestamp.h"

#include "main.h"

//extern TIM_HandleTypeDef htim2;

 uint32_t milliseconds;
 uint32_t seconds ;

 uint8_t running=0;
 uint32_t total_seconds;


 uint32_t milliseconds_display;


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim->Instance == TIM2) {
//         if (running) {
//             milliseconds++;
//             if (milliseconds >= 1000) {
//                 milliseconds = 0;
//                 seconds++;
//             }
//         }
//     }
//     milliseconds_display = milliseconds % 1000;
// }
//

void Start_Stopwatch(void) {
    running = 1;
}

void Stop_Stopwatch(void) {
    running = 0;
}

void Reset_Stopwatch(void) {
    running = 0;
    milliseconds = 0;
    seconds = 0;
}

//void Display_Time(void) {
//    char buffer[30];
//    sprintf(buffer, "%02lu:%02lu:%02lu:%03lu", hours, minutes, seconds, milliseconds);
//    // Implement your display method here, e.g., UART, LCD, etc.
////    printf("%s\n", buffer);
//}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//    if (htim->Instance == TIM2) {
//        if (running) {
//            milliseconds++;
//            if (milliseconds >= 1000) {
//                milliseconds = 0;
//                seconds++;
//                if (seconds >= 60) {
//                    seconds = 0;
//                    minutes++;
//                    if (minutes >= 60) {
//                        minutes = 0;
//                        hours++;
//                    }
//                }
//            }
//        }
//    }
//    total_seconds = seconds + minutes * 60 + hours * 3600;
//}

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	 HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//
//     if (htim->Instance == TIM2) {
//         if (running) {
//             milliseconds++;
//             // Update display every timer interrupt if needed
//         }
//     }
// }
