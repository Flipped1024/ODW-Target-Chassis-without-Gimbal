/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
#include "includes.h"
#include "chassis_task.h"

/* Private variables ---------------------------------------------------------*/
extern uint32_t resetCount; // 引用来自 detect_task.c 的复位计数器

/* Task Handles --------------------------------------------------------------*/
osThreadId INSTaskHandle;
osThreadId DetectTaskHandle;
osThreadId ChassisTaskHandle;

/* Private function prototypes -----------------------------------------------*/
void StartINSTask(void const * argument);     /* 姿态解算任务 (用于无头模式补偿) */
void StartDetectTask(void const * argument);  /* 离线检测看门狗任务 */
void StartChassisTask(void const * argument); /* 底盘核心控制任务 */

void MX_FREERTOS_Init(void); 

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* 创建传感器姿态解算任务 (优先级: 高) */
  osThreadDef(INSTask, StartINSTask, osPriorityAboveNormal, 0, 512);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

  /* 创建离线检测任务 (优先级: 低) */
  osThreadDef(DetectTask, StartDetectTask, osPriorityBelowNormal, 0, 512);
  DetectTaskHandle = osThreadCreate(osThread(DetectTask), NULL);

  /* 创建底盘控制主任务 (优先级: 高) */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityAboveNormal, 0, 512);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);
}

/**
* @brief 姿态解算任务实现 (用于获取底盘相对Yaw轴实现无头平移)
*/
void StartINSTask(void const * argument)
{
  INS_Init();
  for (;;)
  {
     // 操作手长按 R + E 键可手动硬重启单片机 (保留靶车急救功能)
     if ((remote_control.key_code & Key_R) && (remote_control.key_code & Key_E))
     {
       resetCount++;
       if (resetCount * INS_TASK_PERIOD > 1500)
       {
         __set_FAULTMASK(1);
         HAL_NVIC_SystemReset();
       }
     }
     else
       resetCount = 0;
    
    INS_Task();
    osDelay(INS_TASK_PERIOD); // 默认 1ms
  }
}

/**
* @brief 离线检测与看门狗任务实现
*/
void StartDetectTask(void const * argument) 
{
  Detect_Task_Init();
  for (;;)
  {
    HAL_IWDG_Refresh(&hiwdg); // 喂狗，防止硬件复位
    
    // 操作手长按 F + E 键硬重启
    if ((remote_control.key_code & Key_F) && (remote_control.key_code & Key_E))
    {
      resetCount++;
      if (resetCount * DETECT_TASK_PERIOD > 1500)
      {
        __set_FAULTMASK(1);
        HAL_NVIC_SystemReset();
      }
    }
    else
      resetCount = 0;

    Detect_Task();
    osDelay(DETECT_TASK_PERIOD); // 默认 10ms
  }
}

/**
* @brief 底盘核心控制任务实现
*/
void StartChassisTask(void const * argument)
{
  osDelay(1000); // 等待电调与传感器初始化稳定
  Chassis_Init();
  
  for (;;)
  {
    Chassis_Control();
    
    // 靶车无云台，直接移除原版代码中与 hcan2 通讯的 Send_Robot_Info
    
    osDelay(CHASSIS_TASK_PERIOD); // 默认 2ms，保证 500Hz 控制频率
  }
}