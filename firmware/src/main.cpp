/***
 * Control Turntable and Led Lights
 *
 * Uses FreeRTOS Task
 * Jon Durrant
 * 15-Aug-2022
 */

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "hardware/uart.h"

#include "BlinkAgent.h"

#include "uRosBridge.h"
#include "PubEntities.h"

#include "MotorsAgent.h"
#include "DDD.h"
#include "HCSR04Agent.h"

extern "C"
{
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_uart.h"
}

#ifndef ENABLE_DEBUG_HEARTBEAT
#define ENABLE_DEBUG_HEARTBEAT 1
#endif

#ifndef DEBUG_HEARTBEAT_INTERVAL_MS
#define DEBUG_HEARTBEAT_INTERVAL_MS 1000
#endif

// Standard Task priority
#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

// LED PAD to use
#define BLINK_LED_PAD 2
#define CONN_LED_PAD 3

// Left Motor
#define LEFT_PWR_CW 9
#define LEFT_PWR_CCW 8
#define LEFT_ROTENC_A 14
#define LEFT_ROTENV_B 15

// Right Motor
#define RIGHT_PWR_CW 6
#define RIGHT_PWR_CCW 7
#define RIGHT_ROTENC_A 12
#define RIGHT_ROTENV_B 13

// PID
#define KP 0.55
#define KI 0.019
#define KD 0.24

char ROBOT_NAME[] = "ddd";

#if ENABLE_DEBUG_HEARTBEAT
static void debugHeartbeatTask(void *params)
{
	(void)params;
	const TickType_t delay_ticks = pdMS_TO_TICKS(DEBUG_HEARTBEAT_INTERVAL_MS);
	for (;;)
	{
		uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
		char buf[64];
		int n = snprintf(buf, sizeof(buf), "ALIVE uptime=%lu ms\r\n", (unsigned long)uptime_ms);
		if (n > 0)
		{
			uart_write_blocking(uart0, (const uint8_t *)buf, (size_t)n);
		}
		vTaskDelay(delay_ticks);
	}
}
#endif

/***
 * Debug function to look at Task Stats
 */
void runTimeStats()
{
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime;

	// Get number of takss
	uxArraySize = uxTaskGetNumberOfTasks();
	printf("Number of tasks %d\n", uxArraySize);

	// Allocate a TaskStatus_t structure for each task.
	pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

	if (pxTaskStatusArray != NULL)
	{
		// Generate raw status information about each task.
		uxArraySize = uxTaskGetSystemState(pxTaskStatusArray,
										   uxArraySize,
										   &ulTotalRunTime);

		// Print stats
		for (x = 0; x < uxArraySize; x++)
		{
			printf("Task: %d \t cPri:%d \t bPri:%d \t hw:%d \t%s\n",
				   pxTaskStatusArray[x].xTaskNumber,
				   pxTaskStatusArray[x].uxCurrentPriority,
				   pxTaskStatusArray[x].uxBasePriority,
				   pxTaskStatusArray[x].usStackHighWaterMark,
				   pxTaskStatusArray[x].pcTaskName);
		}

		// Free array
		vPortFree(pxTaskStatusArray);
	}
	else
	{
		printf("Failed to allocate space for stats\n");
	}

	// Get heap allocation information
	HeapStats_t heapStats;
	vPortGetHeapStats(&heapStats);
	printf("HEAP avl: %d, blocks %d, alloc: %d, free: %d\n",
		   heapStats.xAvailableHeapSpaceInBytes,
		   heapStats.xNumberOfFreeBlocks,
		   heapStats.xNumberOfSuccessfulAllocations,
		   heapStats.xNumberOfSuccessfulFrees);
}

/***
 * Main task to boot the other Agents
 * @param params - unused
 */
void mainTask(void *params)
{
	BlinkAgent blink(BLINK_LED_PAD);

	printf("Boot task started for %s\n", ROBOT_NAME);

	blink.start("Blink", TASK_PRIORITY);

	MotorsAgent motors;
	motors.addMotor(0, LEFT_PWR_CW, LEFT_PWR_CCW,
					LEFT_ROTENC_A, LEFT_ROTENV_B);
	motors.addMotor(1, RIGHT_PWR_CW, RIGHT_PWR_CCW,
					RIGHT_ROTENC_A, RIGHT_ROTENV_B);
	motors.configAllPID(KP, KI, KD);
	motors.start("Motors", TASK_PRIORITY);

	HCSR04Agent range;
	range.addSensor(0, "range_front");
	range.addSensor(18, "range_back");
	range.start("Range", TASK_PRIORITY);

	DDD ddd;
	ddd.setMotorsAgent(&motors);
	ddd.setHCSR04Agent(&range);
	ddd.start("DDD", TASK_PRIORITY);

	// Start up a uROS Bridge
	uRosBridge *bridge = uRosBridge::getInstance();

	// PubEntities entities;

	// bridge->setuRosEntities(&entities);
	// bridge->setuRosEntities(&motors);
	bridge->setuRosEntities(&ddd);
	bridge->setLed(CONN_LED_PAD);
	bridge->start("Bridge", TASK_PRIORITY + 2);
	// entities.start("PubSub", TASK_PRIORITY);

	bool cw = false;
	float rpm = 20.0;
	float rps = 1.0;
	for (;;)
	{
		vTaskDelay(10000);
	}

	for (;;)
	{
		printf("Set Speed %f dir %d\n", rpm, cw);
		/*
		motors.setSpeedRPM(0, rpm, cw);
		motors.setSpeedRPM(1, rpm, !cw);
		rpm += 20.0;
		if (rpm > 200.0){
			rpm = 100.0;
		}
		*/

		motors.setSpeedRadPS(0, rps, cw);
		motors.setSpeedRadPS(1, rps, !cw);
		rps += 1.0;
		if (rps > 6)
		{
			rps = 1.0;
		}

		cw = !cw;
		vTaskDelay(10000);
	}
}

/***
 * Launch the tasks and scheduler
 */
void vLaunch(void)
{

	// Start blink task
	TaskHandle_t task;
	xTaskCreate(mainTask, "MainThread", 500, NULL, TASK_PRIORITY, &task);

#if ENABLE_DEBUG_HEARTBEAT
	xTaskCreate(debugHeartbeatTask, "DbgBeat", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
#endif

	/* Start the tasks and timer running. */
	vTaskStartScheduler();
}

/***
 * Main
 * @return
 */
int main(void)
{
	// Setup serial over UART and give a few seconds to settle before we start
	stdio_init_all();
	sleep_ms(2000);
	printf("GO\n");
	fflush(stdout);

	// Start tasks and scheduler
	const char *rtos_name = "FreeRTOS";
	printf("Starting %s on core 0:\n", rtos_name);
	vLaunch();

	return 0;
}
