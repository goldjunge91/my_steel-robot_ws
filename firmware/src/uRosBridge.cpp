/*
 * uRosBridge.cpp
 *
 * Bridge singleton object to manage all uRos comms
 *
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#include "uRosBridge.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/time.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

/***
 * Agent states
 */
enum states
{
  WAITING_AGENT,     /**< WAITING_AGENT */
  AGENT_AVAILABLE,   /**< AGENT_AVAILABLE */
  AGENT_CONNECTED,   /**< AGENT_CONNECTED */
  AGENT_DISCONNECTED /**< AGENT_DISCONNECTED */
};

/***
 * Structure for the Publish queue
 */
struct PubCmd
{
  rcl_publisher_t* publisher;
  void* msg;
  uRosEntities* entities;
  void* args;
};
typedef struct PubCmd PubCmd_t;

uRosBridge* uRosBridge::pSingleton = NULL;

static void uart_log(const char* format, ...)
{
  char buf[160];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  if (len <= 0)
  {
    return;
  }
  if (len > static_cast<int>(sizeof(buf)))
  {
    len = sizeof(buf);
  }
  static bool uart_initialized = false;
  if (!uart_initialized)
  {
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    uart_initialized = true;
  }
  uart_write_blocking(uart0, reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(len));
}

/***
 * Get the uRos Bridge object
 * @return
 */
uRosBridge* uRosBridge::getInstance()
{
  if (pSingleton == NULL)
  {
    pSingleton = new uRosBridge();
  }
  return pSingleton;
}

uRosBridge::uRosBridge()
{
  xPubQ = xQueueCreate(PUB_Q_LEN, sizeof(PubCmd_t));
  xPubQLock = xSemaphoreCreateMutex();

  if (xPubQ == NULL)
  {
    uart_log("ERROR uRosBridge create Queue failed\r\n");
  }
}

uRosBridge::~uRosBridge()
{
  if (xPubQ != NULL)
  {
    vQueueDelete(xPubQ);
  }
  if (xPubQLock != NULL)
  {
    vSemaphoreDelete(xPubQLock);
  }
}

/***
 * Run loop for the agent.
 */
void uRosBridge::run()
{
  PubCmd_t cmd;
  uint readCount;

  uRosInit();
  xAllocator = rcl_get_default_allocator();
  states state = WAITING_AGENT;
  absolute_time_t nextPing = make_timeout_time_ms(1000);
  xMsg.data = 0;

  for (;;)
  {
    // State change logging with descriptive names
    static states last_state = WAITING_AGENT;
    if (state != last_state)
    {
      const char* state_names[] = { "WAITING_AGENT", "AGENT_AVAILABLE", "AGENT_CONNECTED", "AGENT_DISCONNECTED" };
      uart_log("\n[STATE] 🔄 %s -> %s\n\n", state_names[last_state], state_names[state]);
      last_state = state;
    }

    switch (state)
    {
      case WAITING_AGENT:
        xSessionReady = false;
        if (time_reached(nextPing) && pingAgent())
        {
          xPingFailures = 0;
          state = AGENT_AVAILABLE;
        }
        if (time_reached(nextPing))
        {
          nextPing = make_timeout_time_ms(1000);
        }
        break;
      case AGENT_AVAILABLE:
        if (createEntities())
        {
          xPingFailures = 0;
          xSessionReady = true;
          state = AGENT_CONNECTED;
        }
        else
        {
          xSessionReady = false;
          state = WAITING_AGENT;
          nextPing = make_timeout_time_ms(1000);
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        break;
      case AGENT_CONNECTED: {
        xSessionReady = true;

        // light-weight loop diagnostics
        static uint32_t loop_iter = 0;
        loop_iter++;
        if ((loop_iter % 100) == 0)
        {
          if (xPubQ != NULL)
          {
            UBaseType_t q = uxQueueMessagesWaiting(xPubQ);
            uart_log("[LOOP] iter=%u queued=%u\r\n", (unsigned)loop_iter, (unsigned)q);
          }
        }

        // Periodic diagnostics every 5s
        static uint32_t last_diag = 0;
        uint32_t now_diag = to_ms_since_boot(get_absolute_time());
        if ((now_diag - last_diag) > 5000)
        {
          if (xPubQ != NULL)
          {
            UBaseType_t queued = uxQueueMessagesWaiting(xPubQ);
            UBaseType_t spaces = uxQueueSpacesAvailable(xPubQ);
            uart_log("[DIAG] State=CONNECTED, Queue: %u/%u, SessionReady=%d\r\n", (unsigned)queued,
                     (unsigned)(queued + spaces), xSessionReady);
          }
          last_diag = now_diag;
        }

        if (time_reached(nextPing))
        {
          if (!pingAgent())
          {
            if (xPingFailures < UROS_MAX_PING_FAILURES)
            {
              xPingFailures++;
              uart_log("Ping timeout %u/%u\r\n", (unsigned)xPingFailures, (unsigned)UROS_MAX_PING_FAILURES);
            }
            else
            {
              xPingFailures = 0;
              state = AGENT_DISCONNECTED;
              xSessionReady = false;
              uart_log("micro-ROS disconnected (ping failed)\r\n");
            }
          }
          else
          {
            xPingFailures = 0;
          }
          nextPing = make_timeout_time_ms(1000);
        }

        rclc_executor_spin_some(&xExecutor, RCL_MS_TO_NS(10));

        // Handle Pub queue
        if (xPubQ != NULL)
        {
          BaseType_t res = pdTRUE;
          readCount = 0;
          while (res == pdTRUE)
          {
            res = xQueueReceive(xPubQ, &cmd, 0);

            if (res == pdTRUE)
            {
              // Diagnostic: track dequeues processed
              dequeue_count++;
              if ((dequeue_count % 50) == 0)
              {
                uart_log("[DIAG] processed %u dequeues\r\n", (unsigned)dequeue_count);
              }

              // Validate publisher before attempting to publish
              if (cmd.publisher == NULL || !xSessionReady)
              {
                uart_log("[PUB] ❌ FAILED: Publisher=%p Session=%d\r\n", cmd.publisher, xSessionReady);
                cmd.entities->pubComplete(cmd.msg, cmd.args, PubFailed);
                continue;
              }

              // Attempt to publish (only log failures)
              rcl_ret_t pubRet = rcl_publish(cmd.publisher, cmd.msg, NULL);
              if (RCL_RET_OK != pubRet)
              {
                const char* err = rcl_get_error_string().str;
                uart_log("[PUB] ❌ PUBLISH FAILED (ret=%d): %s\r\n", pubRet, err != NULL ? err : "(null)");
                rcl_reset_error();

                xPubFailures++;
                uart_log("[PUB] ⚠️  Consecutive failures: %u/%u\r\n", (unsigned)xPubFailures,
                         (unsigned)UROS_MAX_PUB_FAILURES);

                // Only disconnect after multiple consecutive failures
                if (xPubFailures >= UROS_MAX_PUB_FAILURES)
                {
                  uart_log("[PUB] 🔴 TOO MANY FAILURES - DISCONNECTING\r\n");
                  state = AGENT_DISCONNECTED;
                  xSessionReady = false;
                  xPubFailures = 0;
                }

                cmd.entities->pubComplete(cmd.msg, cmd.args, PubFailed);
                // Yield briefly to let transport recover and continue
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
              }
              else
              {
                // Reset failure counter on successful publish
                // Only log success every 100th publish to reduce spam
                static uint32_t success_count = 0;
                success_count++;
                if ((success_count % 100) == 0)
                {
                  uart_log("[PUB] ✅ Success count: %u\r\n", (unsigned)success_count);
                }
                xPubFailures = 0;
                cmd.entities->pubComplete(cmd.msg, cmd.args, PubOK);
              }

              readCount++;
              if (readCount > UROS_MAX_PUB_MSGS)
              {
                break;
              }
            }
          }
        }
        break;
      }
      case AGENT_DISCONNECTED:
        xSessionReady = false;
        destroyEntities();
        xPingFailures = 0;
        state = WAITING_AGENT;
        break;
      default:
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE uRosBridge::getMaxStackSize()
{
  return 1024;
}

/***
 * Initialise uROS by setting up allocator and transport
 */
void uRosBridge::uRosInit()
{
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = __freertos_allocate;
  freeRTOS_allocator.deallocate = __freertos_deallocate;
  freeRTOS_allocator.reallocate = __freertos_reallocate;
  freeRTOS_allocator.zero_allocate = __freertos_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))
  {
    uart_log("Error on default allocators (line %d)\r\n", __LINE__);
    return;
  }

  rmw_uros_set_custom_transport(true, NULL, pico_usb_transport_open, pico_usb_transport_close, pico_usb_transport_write,
                                pico_usb_transport_read);
}

/***
 * Ping the uROS Agent
 * @return
 */
bool uRosBridge::pingAgent()
{
  const int timeout_ms = 100;
  const uint8_t attempts = 3;

  rmw_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
  if (ret == RMW_RET_OK)
  {
    if (xLedPad != 0)
    {
      gpio_put(xLedPad, 1);
    }
    return true;
  }
  else
  {
    if (xLedPad != 0)
    {
      gpio_put(xLedPad, 0);
    }
    return false;
  }
}

/***
 * Set the Pad to use as a status LED
 * @param pad - GPIO PAD
 */
void uRosBridge::setLed(uint8_t pad)
{
  xLedPad = pad;
  gpio_init(xLedPad);
  gpio_set_dir(xLedPad, GPIO_OUT);
  gpio_put(xLedPad, 0);
}

/***
 * Create the Entities (Publishers)
 */
bool uRosBridge::createEntities()
{
  PubCmd_t cmd;
  xAllocator = rcl_get_default_allocator();

  rclc_support_init(&xSupport, 0, NULL, &xAllocator);

  rclc_node_init_default(&xNode, "pico_node", "", &xSupport);
  rclc_publisher_init_default(&xPublisher, &xNode, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "pico_count");

  rclc_timer_init_default(&xTimer, &xSupport, RCL_MS_TO_NS(1000), uRosBridge::timerCallback);

  uint handles = 1;
  uint count = 2;

  uart_log("\n========================================\n");
  uart_log("[ENTITIES] Creating ROS2 entities...\n");
  uart_log("[ENTITIES] Base: pico_count publisher + timer\n");

  if (pURosEntities != NULL)
  {
    pURosEntities->createEntities(&xNode, &xSupport);
    handles += pURosEntities->getHandles();
    count += pURosEntities->getCount();
    uart_log("[ENTITIES] Additional entities: %d\n", count - 2);
  }

  uart_log("[ENTITIES] ✅ Total: %d entities, %d handles\n", count, handles);

  // Log agent connection status
  uart_log("[ENTITIES] Agent ping: %s\n", pingAgent() ? "✅ OK" : "❌ FAILED");
  uart_log("========================================\n\n");

  rclc_executor_init(&xExecutor, &xSupport.context, handles, &xAllocator);
  rclc_executor_add_timer(&xExecutor, &xTimer);

  if (pURosEntities != NULL)
  {
    pURosEntities->addToExecutor(&xExecutor);
  }

  // Sync Time
  rmw_ret_t sync_ret = rmw_uros_sync_session(5000);
  if (RMW_RET_OK != sync_ret)
  {
    if (sync_ret == RMW_RET_TIMEOUT)
    {
      uart_log("TIME SYNC timeout (agent busy) – continuing without sync\r\n");
    }
    else if (sync_ret == RMW_RET_ERROR)
    {
      uart_log("TIME SYNC error (agent not ready) – continuing without sync\r\n");
    }
    else
    {
      uart_log("TIME SYNC failed (ret=%d) – continuing without sync\r\n", sync_ret);
    }
  }
  else
  {
    uart_log("TIME SYNC successful\r\n");
  }

  // Empty Queue as publishers have been regenerated
  if (xPubQ != NULL)
  {
    BaseType_t res = pdTRUE;
    while (res == pdTRUE)
    {
      res = xQueueReceive(xPubQ, &cmd, 0);

      if (res == pdTRUE)
      {
        cmd.entities->pubComplete(cmd.msg, cmd.args, PubCleared);
      }
    }
  }

  // Give agent time to fully set up communication channels
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Verify agent is still responsive after entity creation
  if (!pingAgent())
  {
    uart_log("Agent not responsive after entity creation - retrying\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (!pingAgent())
    {
      uart_log("Agent still not responsive - continuing anyway\r\n");
    }
  }

  uart_log("Entities Created\r\n");
  return true;
}

/***
 * Destroy the entities
 */
void uRosBridge::destroyEntities()
{
  uart_log("\n========================================\n");
  uart_log("[ENTITIES] Destroying ROS2 entities...\n");

  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&xSupport.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&xPublisher, &xNode);
  rcl_timer_fini(&xTimer);

  if (pURosEntities != NULL)
  {
    pURosEntities->destroyEntities(&xNode, &xSupport);
  }

  rclc_executor_fini(&xExecutor);
  rcl_node_fini(&xNode);
  rclc_support_fini(&xSupport);

  uart_log("[ENTITIES] ✅ All entities destroyed\n");
  uart_log("========================================\n\n");

  uart_log("Entities Destroyed\r\n");
}

/***
 * Timer call back used for the pico_count
 * @param timer
 * @param last_call_time
 */
void uRosBridge::timerCallback(rcl_timer_t* timer, int64_t last_call_time)
{
  uRosBridge::getInstance()->pubCount();
}

/***
 * Publish the pico_count topic
 */
void uRosBridge::pubCount()
{
  rcl_ret_t ret = rcl_publish(&xPublisher, &xMsg, NULL);
  // Update diagnostic timer counter and log occasionally
  timer_pub_count++;
  if ((timer_pub_count % 5) == 0)
  {
    uart_log("[TIMER PUB] pico_count=%d publish_ret=%d count=%u\r\n", xMsg.data, (int)ret, (unsigned)timer_pub_count);
  }
  // printf("Published %d\n", xMsg.data);
  xMsg.data++;
}

/***
 * set Ros Entities object. This is used to create and destroy publishers
 * @param mgr: pointer to the object managing the publishers
 */
void uRosBridge::setuRosEntities(uRosEntities* mgr)
{
  pURosEntities = mgr;
}

/***
 * Publish the msg using the publisher.
 * The msg should remain in scope until entities->pubComplete is called
 * @param publisher - ROS Publisher
 * @param msg - ROS Msg
 * @param entities - Entities object to get call back of pubComplete
 * @param arg - arguments to provide back to the pubComplete function
 * @return True if publish is queued ok
 */
// bool uRosBridge::publish(rcl_publisher_t *publisher,
// 						 void *msg,
// 						 uRosEntities *entities,
// 						 void *args)
// {
// 	PubCmd_t cmd;

// 	if (xPubQ == NULL)
// 	{
// 		return false;
// 	}

// 	if (!xSessionReady)
// 	{
// 		if (entities != NULL)
// 		{
// 			entities->pubComplete(msg, args, PubCleared);
// 		}
// 		return true;
// 	}

// 	cmd.publisher = publisher;
// 	cmd.msg = msg;
// 	cmd.entities = entities;
// 	cmd.args = args;

// 	// Try to enqueue the publish command. Use a short block time so that
// 	// transient backpressure can clear instead of immediately failing.
// 	const TickType_t enqueue_timeout = pdMS_TO_TICKS(100);
// 	BaseType_t res = xQueueSendToBack(
// 		xPubQ,
// 		&cmd,
// 		enqueue_timeout);

// 	if (res != pdTRUE)
// 	{
// 		// Provide diagnostic information when the queue is full or send
// fails. 		UBaseType_t queued = uxQueueMessagesWaiting(xPubQ);
// UBaseType_t spaces = uxQueueSpacesAvailable(xPubQ);
// uart_log("uRosBridge::publish Failed to send to buffer (queued=%u,
// spaces=%u)\r\n", (unsigned)queued, (unsigned)spaces); 		if
// (entities
// != NULL)
// 		{
// 			entities->pubComplete(
// 				msg,
// 				args,
// 				PubFailed);
// 		}
// 		return false;
// 	}
// 	return true;
// }
bool uRosBridge::publish(rcl_publisher_t* publisher, void* msg, uRosEntities* entities, void* args)
{
  PubCmd_t cmd;

  if (xPubQ == NULL)
  {
    uart_log("[uRosBridge] ERROR: Queue is NULL\r\n");
    return false;
  }

  if (!xSessionReady)
  {
    static bool warned_once = false;
    if (!warned_once)
    {
      uart_log("[uRosBridge] ⚠️  Session not ready - messages will be cleared until connection established\r\n");
      warned_once = true;
    }
    if (entities != NULL)
    {
      entities->pubComplete(msg, args, PubCleared);
    }
    return true;
  }

  cmd.publisher = publisher;
  cmd.msg = msg;
  cmd.entities = entities;
  cmd.args = args;

  // First try a short blocking enqueue to allow transient consumer progress
  const TickType_t initial_timeout = pdMS_TO_TICKS(10);
  BaseType_t res = xQueueSendToBack(xPubQ, &cmd, initial_timeout);
  if (res == pdTRUE)
  {
    return true;
  }

  // Queue still full. Try to make space by dropping the oldest queued message
  // Do the dequeue+enqueue pair in a critical section to avoid races with
  // other producers on the same core.
  uart_log("[uRosBridge] WARN: Queue full, attempting drop-old to make room\r\n");

  UBaseType_t queued = uxQueueMessagesWaiting(xPubQ);
  UBaseType_t spaces = uxQueueSpacesAvailable(xPubQ);
  uart_log("[uRosBridge] INFO: before drop queued=%u, spaces=%u\r\n", (unsigned)queued, (unsigned)spaces);

  PubCmd_t oldest;

  taskENTER_CRITICAL();
  BaseType_t r = xQueueReceive(xPubQ, &oldest, 0);
  if (r == pdTRUE)
  {
    // Inform owner that message was dropped due to overflow
    if (oldest.entities != NULL)
    {
      oldest.entities->pubComplete(oldest.msg, oldest.args, PubFailed);
    }

    // Try enqueue again immediately while in critical section
    BaseType_t r2 = xQueueSendToBack(xPubQ, &cmd, 0);
    if (r2 == pdTRUE)
    {
      taskEXIT_CRITICAL();
      queued = uxQueueMessagesWaiting(xPubQ);
      spaces = uxQueueSpacesAvailable(xPubQ);
      uart_log("[uRosBridge] INFO: after drop queued=%u, spaces=%u\r\n", (unsigned)queued, (unsigned)spaces);
      return true;
    }
    // Could not enqueue inside critical section; release and fallthrough to
    // try a short blocking enqueue outside critical region.
  }
  taskEXIT_CRITICAL();

  // As a last attempt, try a slightly longer blocking enqueue to let the
  // consumer catch up.
  const TickType_t fallback_timeout = pdMS_TO_TICKS(50);
  BaseType_t r3 = xQueueSendToBack(xPubQ, &cmd, fallback_timeout);
  if (r3 == pdTRUE)
  {
    queued = uxQueueMessagesWaiting(xPubQ);
    spaces = uxQueueSpacesAvailable(xPubQ);
    uart_log("[uRosBridge] INFO: after fallback enqueue queued=%u, spaces=%u\r\n", (unsigned)queued, (unsigned)spaces);
    return true;
  }

  // All attempts failed - report and notify caller
  uart_log("[uRosBridge] ERROR: Unable to enqueue message after drop-old attempts\r\n");
  if (entities != NULL)
  {
    entities->pubComplete(msg, args, PubFailed);
  }
  return false;
}

/***
 * Returns the ROS2 support pointer
 * @return NULL if not connected
 */
rclc_support_t* uRosBridge::getSupport()
{
  return &xSupport;
}
