/******************************************************************************
 *  File Name:
 *    freertos_thread.cpp
 *
 *  Description:
 *    FreeRTOS based threading implementation
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include "mbedutils/drivers/threading/lock.hpp"
#include "mbedutils/drivers/threading/thread.hpp"
#include <FreeRTOS.h>
#include <etl/pool.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/threading.hpp>
#include <task.h>

/*-----------------------------------------------------------------------------
Validate user configuration parameters from in the project's FreeRTOSConfig.h
-----------------------------------------------------------------------------*/

#if !defined( configMBEDUTILS_MAX_NUM_TASKS ) || ( configMBEDUTILS_MAX_NUM_TASKS <= 0 )
#error "configMBEDUTILS_MAX_NUM_TASKS must be defined as > 0 in FreeRTOSConfig.h"
#endif

// static_assert( configMAX_PRIORITIES >= std::numeric_limits<mb::thread::TaskPriority>::max(),
//                "configMAX_PRIORITIES incorrectly sized" );

namespace mb::thread
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct FreeRtosTaskMeta
  {
    TaskId        id;     /**< System identifier for the thread */
    TaskHandle_t  handle; /**< FreeRTOS handle to the task */
    StaticTask_t *task;   /**< (Optional) Static allocation parameters */
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/
  static mb::osal::mb_mutex_t                                           s_task_mutex;
  static etl::pool<StaticTask_t, configMBEDUTILS_MAX_NUM_TASKS>         s_task_pool;
  static etl::pool<FreeRtosTaskMeta, configMBEDUTILS_MAX_NUM_TASKS>     s_task_meta_pool;
  static etl::vector<FreeRtosTaskMeta *, configMBEDUTILS_MAX_NUM_TASKS> s_task_meta_map;
  static size_t                                                         s_module_ready = ~DRIVER_INITIALIZED_KEY;

  /*---------------------------------------------------------------------------
  Private Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Looks up a task in the internal map based on the handle.
   *
   * @param handle Handle generated from the create_task() function
   * @return std::unordered_map<TaskId, TaskData>::iterator
   */
  static TaskHandle_t find_task( const TaskHandle handle )
  {
    for( auto task = s_task_meta_map.begin(); task != s_task_meta_map.end(); ++task )
    {
      if( ( *task )->id == handle )
      {
        return ( *task )->handle;
      }
    }

    return nullptr;
  }

  /*---------------------------------------------------------------------------
  Classes
  ---------------------------------------------------------------------------*/

  Task::Task() noexcept : mId( TASK_ID_INVALID ), mName( "" ), mHandle( 0 ), pImpl( nullptr )
  {
  }


  Task::~Task()
  {
  }


  Task &Task::operator=( Task &&other ) noexcept
  {
    this->mId     = other.mId;
    this->mHandle = other.mHandle;
    this->pImpl   = other.pImpl;

    return *this;
  }


  void Task::start()
  {
    mb::thread::LockGuard _lock( s_task_mutex );
    pImpl = reinterpret_cast<void *>( find_task( mHandle ) );
    if( pImpl == nullptr )
    {
      mbed_assert_always();
      return;
    }
  }


  void Task::kill()
  {
    auto tsk_data = reinterpret_cast<TaskHandle_t>( pImpl );
    if( tsk_data )
    {
      vTaskDelete( tsk_data );
    }
  }


  bool Task::killPending()
  {
    return false;
  }

  void Task::join()
  {
    this->kill();
  }


  bool Task::joinable()
  {
    return true;
  }


  TaskId Task::id() const
  {
    return mId;
  }


  TaskName Task::name() const
  {
    return mName;
  }


  TaskHandle Task::implementation() const
  {
    return mHandle;
  }

  namespace this_thread
  {
    TaskName get_name()
    {
      TaskHandle_t currentTaskHandle = xTaskGetCurrentTaskHandle();
      const char  *taskName          = pcTaskGetName( currentTaskHandle );
      TaskName     name;
      name.assign( taskName );
      return name;
    }


    void sleep_for( const size_t timeout )
    {
      vTaskDelay( pdMS_TO_TICKS( timeout ) );
    }


    void sleep_until( const size_t wakeup )
    {
      TickType_t xLastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( wakeup ) );
    }


    void yield()
    {
      taskYIELD();
    }


    TaskId id()
    {
      mb::thread::LockGuard _lock( s_task_mutex );

      auto handle = xTaskGetCurrentTaskHandle();

      for( auto task = s_task_meta_map.begin(); task != s_task_meta_map.end(); ++task )
      {
        if( ( *task )->handle == handle )
        {
          return ( *task )->id;
        }
      }

      return TASK_ID_INVALID;
    }
  }    // namespace this_thread
}    // namespace mb::thread

namespace mb::thread::intf
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void driver_setup()
  {
    mbed_assert( mb::osal::buildMutexStrategy( s_task_mutex ) );
  }


  void driver_teardown()
  {
    mbed_assert( s_task_mutex != nullptr );
    mb::osal::destroyMutexStrategy( s_task_mutex );
  }


  TaskHandle create_task( Task::Config &cfg )
  {
    /*---------------------------------------------------------------------------
    Input validation
    ---------------------------------------------------------------------------*/
    if( ( cfg.func == nullptr ) || ( cfg.name.empty() ) )
    {
      return 0;
    }

    /*-------------------------------------------------------------------------
    Allocate a new task meta structure
    -------------------------------------------------------------------------*/
    mbed_dbg_assert( s_task_mutex != nullptr );
    mb::thread::RecursiveLockGuard _lock( s_task_mutex );

    FreeRtosTaskMeta *meta = s_task_meta_pool.allocate();
    if( meta == nullptr )
    {
      mbed_assert_continue_msg( false, "FreeRTOS task meta pool is full" );
      return 0;
    }

    meta->id     = cfg.id;
    meta->handle = 0;
    meta->task   = nullptr;

    /*-------------------------------------------------------------------------
    Ensure the affinity mask is valid. FreeRTOS expects the affinity to be a
    bitmask of the cores to run on.
    -------------------------------------------------------------------------*/
#if( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 )
    constexpr size_t AffinityMask = ( 1 << configNUMBER_OF_CORES ) - 1;
    UBaseType_t      affinity     = cfg.affinity & AffinityMask;
#endif /* ( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 ) */

    /*-------------------------------------------------------------------------
    Create a dynamic task
    -------------------------------------------------------------------------*/
    if constexpr( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
    {
      if( cfg.stack_buf == nullptr )
      {
        if( configUSE_CORE_AFFINITY != 1 )
        {
          auto result = xTaskCreate( cfg.func, cfg.name.data(), cfg.stack_size, cfg.user_data, cfg.priority, &meta->handle );
          if( result != pdPASS )
          {
            meta->handle = nullptr;
          }
        }
#if( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 )
        else if( affinity != 0 )
        {
          auto result = xTaskCreateAffinitySet( cfg.func, cfg.name.data(), cfg.stack_size, cfg.user_data, cfg.priority,
                                                affinity, &meta->handle );
          if( result != pdPASS )
          {
            meta->handle = nullptr;
          }
        }
#endif /* ( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 ) */
      }
    }

    /*-------------------------------------------------------------------------
    Create a static task
    -------------------------------------------------------------------------*/
    if constexpr( configSUPPORT_STATIC_ALLOCATION == 1 )
    {
      if( ( cfg.stack_buf != nullptr ) && ( cfg.stack_size > 0 ) )
      {
        meta->task = s_task_pool.allocate();

        if( configUSE_CORE_AFFINITY != 1 )
        {
          meta->handle = xTaskCreateStatic( cfg.func, cfg.name.data(), cfg.stack_size, cfg.user_data, cfg.priority,
                                            cfg.stack_buf, meta->task );
        }
#if( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 )
        else if( affinity != 0 )
        {
          meta->handle = xTaskCreateStaticAffinitySet( cfg.func, cfg.name.data(), cfg.stack_size, cfg.user_data, cfg.priority,
                                                       cfg.stack_buf, meta->task, affinity );
        }
#endif /* ( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 ) */
      }
    }

    /*-------------------------------------------------------------------------
    Clean up allocated resources should the task fail to create for some reason
    -------------------------------------------------------------------------*/
    if( meta->handle == 0 )
    {
      if( meta->task && s_task_pool.is_in_pool( meta->task ) )
      {
        s_task_pool.release( meta->task );
      }

      s_task_meta_pool.release( meta );
      return 0;
    }

    return cfg.id;
  }


  void destroy_task( TaskHandle task )
  {
    /*-------------------------------------------------------------------------
    Input validation
    -------------------------------------------------------------------------*/
    if( task == 0 )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Ensure exclusive access to the task pools
    -------------------------------------------------------------------------*/
    mbed_dbg_assert( s_task_mutex != nullptr );
    mb::thread::RecursiveLockGuard _lock( s_task_mutex );

    /*-------------------------------------------------------------------------
    All task resources are stored in the meta structure, so we can just
    delete the task from the FreeRTOS perspective and then release the
    meta structure back to the pool.
    -------------------------------------------------------------------------*/
    auto meta = reinterpret_cast<FreeRtosTaskMeta *>( task );
    if( s_task_meta_pool.is_in_pool( meta ) )
    {
      /* FreeRTOS task deletion */
      if( meta->handle != nullptr )
      {
        vTaskDelete( meta->handle );
      }

      /* If the task was statically allocated, this will need to be released as well */
      if( meta->task && s_task_pool.is_in_pool( meta->task ) )
      {
        s_task_pool.release( meta->task );
      }

      s_task_meta_pool.release( meta );
    }
  }


  void set_affinity( TaskHandle task, size_t coreId )
  {
#if configUSE_CORE_AFFINITY == 1
    /*-------------------------------------------------------------------------
    Input validation
    -------------------------------------------------------------------------*/
    if( task == nullptr )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Set the core affinity for the task
    -------------------------------------------------------------------------*/
    auto meta = reinterpret_cast<FreeRtosTaskMeta *>( task );
    if( meta->handle != nullptr )
    {
      vTaskCoreAffinitySet( meta->handle, coreId );
    }
#endif /* configUSE_CORE_AFFINITY */
  }


  void start_scheduler()
  {
    vTaskStartScheduler();
  }


  __attribute__( ( weak ) ) void on_stack_overflow()
  {
    mbed_assert_always();
  }


  __attribute__( ( weak ) ) void on_malloc_failed()
  {
    mbed_assert_always();
  }

  __attribute__( ( weak ) ) void on_idle()
  {
    // Default implementation does nothing
  }


  __attribute__( ( weak ) ) void on_tick()
  {
    // Default implementation does nothing
  }
}    // namespace mb::thread::intf


/*-----------------------------------------------------------------------------
FreeRTOS Application Hooks
-----------------------------------------------------------------------------*/

extern "C"
{
#if( configCHECK_FOR_STACK_OVERFLOW != 0 )
  void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
  {
    mbed_assert_continue_msg( false, "Stack overflow detected on task: %s", pcTaskName );
    mb::thread::intf::on_stack_overflow();
  }
#endif /* configCHECK_FOR_STACK_OVERFLOW */

#if( configUSE_MALLOC_FAILED_HOOK == 1 )
  void vApplicationMallocFailedHook( void )
  {
    mb::thread::intf::on_malloc_failed();
  }
#endif /* configUSE_MALLOC_FAILED_HOOK */

#if( configUSE_IDLE_HOOK == 1 )
  __attribute__( ( weak ) ) void vApplicationIdleHook( void )
  {
    mb::thread::intf::on_idle();
  }
#endif /* configUSE_IDLE_HOOK */

#if( configUSE_TICK_HOOK != 0 )
  __attribute__( ( weak ) ) void vApplicationTickHook( void )
  {
    mb::thread::intf::on_tick();
  }
#endif /* configUSE_TICK_HOOK */

#if( configENABLE_HEAP_PROTECTOR == 1 )
  void vApplicationGetRandomHeapCanary( portPOINTER_SIZE_TYPE *pxHeapCanary )
  {
    *pxHeapCanary = 0x12345678;
  }
#endif /* configENABLE_HEAP_PROTECTOR */

#if( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configKERNEL_PROVIDED_STATIC_MEMORY == 0 )
  void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                      configSTACK_DEPTH_TYPE *puxIdleTaskStackSize )
  {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t  uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *puxIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
  }

#if defined( configTIMER_TASK_STACK_DEPTH )
  void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer,
                                       uint32_t *pulTimerTaskStackSize )
  {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t  uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
  }
#endif /* configTIMER_TASK_STACK_DEPTH */

#if( configNUMBER_OF_CORES > 1 )
  void vApplicationGetPassiveIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                             configSTACK_DEPTH_TYPE *puxIdleTaskStackSize, BaseType_t xPassiveIdleTaskIndex )
  {
    static StaticTask_t xIdleTaskTCB[ configNUMBER_OF_CORES - 1 ];
    static StackType_t  uxIdleTaskStack[ configNUMBER_OF_CORES - 1 ][ configMINIMAL_STACK_SIZE ];

    mbed_assert( xPassiveIdleTaskIndex < configNUMBER_OF_CORES );

    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB[ xPassiveIdleTaskIndex ];
    *ppxIdleTaskStackBuffer = uxIdleTaskStack[ xPassiveIdleTaskIndex ];
    *puxIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
  }
#endif /* configNUMBER_OF_CORES > 1 */
#endif /* configSUPPORT_STATIC_ALLOCATION */
}
