/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "debug.h"

#include "stm32f10x.h"
			

void led_init(void)
{
	GPIO_InitTypeDef gpioInitStrcut;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_10;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpioInitStrcut);

  GPIO_WriteBit(GPIOC, GPIO_Pin_7, 1);
  GPIO_WriteBit(GPIOC, GPIO_Pin_8, 1);
  GPIO_WriteBit(GPIOC, GPIO_Pin_10, 1);

}


#if 1
char buffer[512];
void task_stat(void)
{
  memset(buffer, 0, sizeof(buffer));

  strcat((char *)buffer, "\nName\t\tState\tPriority\tStack\tNum\n" );

  strcat((char *)buffer, "--------------------------------------------------------\n");
  vTaskList((unsigned char *)(buffer + strlen(buffer)));
  DEBUG_PRINT("%s", buffer);
}
#endif



void led_task(void *arg)
{
  int ctr = 0;

	led_init();

	while (1) {

    if (ctr++ % 4) {
//      task_stat();
    }

    GPIO_WriteBit(GPIOC, GPIO_Pin_7, 0);
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, 0);
		vTaskDelay(pdMS_TO_TICKS(300));

    GPIO_WriteBit(GPIOC, GPIO_Pin_7, 1);
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, 1);
		vTaskDelay(pdMS_TO_TICKS(300));

	}
}
