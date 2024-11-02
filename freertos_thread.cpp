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
#include "mbedutils/drivers/threading/thread.hpp"
#include <etl/pool.h>
#include <mbedutils/assert.hpp>
#include <mbedutils/interfaces/thread_intf.hpp>
#include <mbedutils/threading.hpp>

#include <FreeRTOS.h>
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
    TaskId        id;                        /**< System identifier for the thread */
    TaskFunction  func;                      /**< Function to run as the task */
    void         *args;                      /**< (Optional) User data to pass to the thread */
    TaskHandle_t  freertos_handle;           /**< FreeRTOS handle to the task */
    StaticTask_t *freertos_static_task_data; /**< (Optional) Static allocation parameters */
    bool          kill_request;              /**< Flag to signal the task to terminate */
    bool          block_on_start;            /**< Flag to block the task until start() is called */
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
   * @brief Looks up a task in the internal map based on the ID.
   *
   * @param id ID returned from the create_task() function
   * @return std::unordered_map<TaskId, TaskData>::iterator
   */
  static FreeRtosTaskMeta * find_task_meta( const TaskId id )
  {
    for( auto task = s_task_meta_map.begin(); task != s_task_meta_map.end(); ++task )
    {
      if( ( *task )->id == id )
      {
        return *task;
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


  Task::Task( Task &&other ) noexcept
  {
    this->mId     = other.mId;
    this->mName   = other.mName;
    this->mHandle = other.mHandle;
    this->pImpl   = other.pImpl;
  }

  Task &Task::operator=( Task &&other ) noexcept
  {
    this->mId     = other.mId;
    this->mName   = other.mName;
    this->mHandle = other.mHandle;
    this->pImpl   = other.pImpl;

    return *this;
  }


  void Task::start()
  {
    mb::thread::LockGuard _lock( s_task_mutex );

    /*-------------------------------------------------------------------------
    Find the task meta structure. This allows us to access the FreeRTOS task
    handle and start the task.
    -------------------------------------------------------------------------*/
    pImpl = reinterpret_cast<void *>( find_task_meta( mId ) );
    if( pImpl == nullptr )
    {
      mbed_assert_always();
      return;
    }

    /*-------------------------------------------------------------------------
    Check if the task scheduler is running. This is a prerequisite for starting
    up the task.
    -------------------------------------------------------------------------*/
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      mbed_assert_always();
      return;
    }

    /*-------------------------------------------------------------------------
    Signal the task to begin execution.
    -------------------------------------------------------------------------*/
    auto tsk_meta = reinterpret_cast<FreeRtosTaskMeta *>( pImpl );
    if( tsk_meta->block_on_start )
    {
      tsk_meta->kill_request = false;
      vTaskResume( tsk_meta->freertos_handle );
    }
    else
    {
      auto state = eTaskGetState( tsk_meta->freertos_handle );
      mbed_assert_continue_msg( state < eDeleted, "Start called on already running task: %s", mName.c_str() );
    }
  }


  void Task::kill()
  {
    auto tsk_meta = reinterpret_cast<FreeRtosTaskMeta *>( pImpl );
    if( tsk_meta )
    {
      tsk_meta->kill_request = true;
    }
  }


  bool Task::killPending()
  {
    auto tsk_meta = reinterpret_cast<FreeRtosTaskMeta *>( pImpl );
    if( tsk_meta )
    {
      return tsk_meta->kill_request;
    }

    return false;
  }


  void Task::join()
  {
    auto tsk_meta = reinterpret_cast<FreeRtosTaskMeta *>( pImpl );
    if( !tsk_meta )
    {
      return;
    }

    /*-------------------------------------------------------------------------
    Wait for the task to finish executing
    -------------------------------------------------------------------------*/
    eTaskState state = eTaskGetState( tsk_meta->freertos_handle );
    while( state != eDeleted )
    {
      vTaskDelay( pdMS_TO_TICKS( 5 ) );
      state = eTaskGetState( tsk_meta->freertos_handle );
    }

    /*-------------------------------------------------------------------------
    Reverse the create() function
    -------------------------------------------------------------------------*/
    mb::thread::destroy( this );
  }


  bool Task::joinable()
  {
    auto tsk_meta = reinterpret_cast<FreeRtosTaskMeta *>( pImpl );
    if( !tsk_meta )
    {
      return false;
    }

    auto state = eTaskGetState( tsk_meta->freertos_handle );
    return state < eDeleted;
  }


  TaskId Task::id() const
  {
    return mId;
  }


  TaskName Task::name() const
  {
    return mName;
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
        if( ( *task )->freertos_handle == handle )
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
  Private Functions
  ---------------------------------------------------------------------------*/

  /**
   * @brief Wrapper to ensure controlled entry/exit from FreeRTOS tasking.
   *
   * @param arg Pointer to a FreeRtosTaskMeta structure
   */
  static void thread_entry_point( void *arg )
  {
    FreeRtosTaskMeta *meta = reinterpret_cast<FreeRtosTaskMeta *>( arg );

    /*-------------------------------------------------------------------------
    Wait for a signal to start the task. This ensures control flow is not
    passed to the user's thread function until the start() method is called.
    -------------------------------------------------------------------------*/
    if( meta->block_on_start )
    {
      vTaskSuspend( nullptr );
    }

    /*-------------------------------------------------------------------------
    Run the user's thread function
    -------------------------------------------------------------------------*/
    if( meta && meta->func )
    {
      meta->func( meta->args );
    }

    /*-------------------------------------------------------------------------
    Clean up the task before exiting
    -------------------------------------------------------------------------*/
    vTaskDelete( nullptr );
  }

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


  TaskId create_task( Task::Config &cfg )
  {
    /*---------------------------------------------------------------------------
    Input validation
    ---------------------------------------------------------------------------*/
    if( ( cfg.func == nullptr ) || ( cfg.name.empty() ) )
    {
      return TASK_ID_INVALID;
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
      return TASK_ID_INVALID;
    }

    meta->id                        = cfg.id;
    meta->freertos_handle           = nullptr;
    meta->freertos_static_task_data = nullptr;
    meta->func                      = cfg.func;
    meta->args                      = cfg.user_data;
    meta->block_on_start            = cfg.block_on_start;

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
          auto result = xTaskCreate( thread_entry_point, cfg.name.data(), cfg.stack_size, meta, cfg.priority,
                                     &meta->freertos_handle );
          if( result != pdPASS )
          {
            meta->freertos_handle = nullptr;
          }
        }
#if( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 )
        else if( affinity != 0 )
        {
          auto result = xTaskCreateAffinitySet( thread_entry_point, cfg.name.data(), cfg.stack_size, meta, cfg.priority,
                                                affinity, &meta->freertos_handle );
          if( result != pdPASS )
          {
            meta->freertos_handle = nullptr;
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
        meta->freertos_static_task_data = s_task_pool.allocate();

        if( configUSE_CORE_AFFINITY != 1 )
        {
          meta->freertos_handle = xTaskCreateStatic( thread_entry_point, cfg.name.data(), cfg.stack_size, meta,
                                                     cfg.priority, cfg.stack_buf, meta->freertos_static_task_data );
        }
#if( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 )
        else if( affinity != 0 )
        {
          meta->freertos_handle =
              xTaskCreateStaticAffinitySet( thread_entry_point, cfg.name.data(), cfg.stack_size, meta, cfg.priority,
                                            cfg.stack_buf, meta->freertos_static_task_data, affinity );
        }
#endif /* ( configNUMBER_OF_CORES > 1 ) && ( configUSE_CORE_AFFINITY == 1 ) */
      }
    }

    /*-------------------------------------------------------------------------
    Clean up allocated resources should the task fail to create for some reason
    -------------------------------------------------------------------------*/
    if( meta->freertos_handle == nullptr )
    {
      if( meta->freertos_static_task_data && s_task_pool.is_in_pool( meta->freertos_static_task_data ) )
      {
        s_task_pool.release( meta->freertos_static_task_data );
      }

      s_task_meta_pool.release( meta );
      return TASK_ID_INVALID;
    }

    /*-------------------------------------------------------------------------
    Add the task to the internal map for later lookup
    -------------------------------------------------------------------------*/
    s_task_meta_map.push_back( meta );

    /*-------------------------------------------------------------------------
    Don't return until we know the thread has started. This ensures proper
    control flow with the start() method and thread_entry_point().
    -------------------------------------------------------------------------*/
    if( meta->block_on_start )
    {
      while( eTaskGetState( meta->freertos_handle ) != eSuspended )
      {
        vPortYield();
      }
    }

    return cfg.id;
  }


  void destroy_task( TaskId task )
  {
    /*-------------------------------------------------------------------------
    Input validation
    -------------------------------------------------------------------------*/
    if( task == TASK_ID_INVALID )
    {
      return;
    }

    mbed_dbg_assert( s_task_mutex != nullptr );
    mb::thread::RecursiveLockGuard _lock( s_task_mutex );

    /*-------------------------------------------------------------------------
    Find the meta structure associated with this task
    -------------------------------------------------------------------------*/
    FreeRtosTaskMeta *meta = nullptr;
    for( auto &taskMeta : s_task_meta_map )
    {
      if( taskMeta->id == task )
      {
        meta = taskMeta;
        break;
      }
    }

    if( meta == nullptr )
    {
      return;
    }

    mbed_dbg_assert( s_task_meta_pool.is_in_pool( meta ) );

    /*-----------------------------------------------------------------------
    Delete the FreeRTOS task
    -----------------------------------------------------------------------*/
    if( meta->freertos_handle != nullptr )
    {
      /*---------------------------------------------------------------------
      It's highly likely the task is already in the process of being deleted,
      but just in case, we'll check and delete it if necessary.
      ---------------------------------------------------------------------*/
      if( eTaskGetState( meta->freertos_handle ) < eDeleted )
      {
        vTaskDelete( meta->freertos_handle );
      }

      /*---------------------------------------------------------------------
      Yield back to the scheduler to give the system a chance to run the
      idle task and clean up the task resources. If we don't do this, the
      pool cleanup operations will corrupt the handle and cause a hard fault.
      This was found as a result of integration testing.

      Unfortunately, the FreeRTOS architecture expects task deletion to be
      associated with a "power off" type event, so it doesn't really have a
      concept of "waiting for a task to finish". This is the best we can do.

      This delay to allow the idle task a chance to run will likely only fail
      on highly loaded systems where the idle task doesn't even get execution
      time. But if that's the case, you also probably aren't deleting tasks
      rapidly either.
      ---------------------------------------------------------------------*/
      vTaskDelay( pdMS_TO_TICKS( 25 ) );
      meta->freertos_handle = nullptr;
    }

    /*-----------------------------------------------------------------------
    Deallocate the static task if it was used
    -----------------------------------------------------------------------*/
    if( meta->freertos_static_task_data != nullptr )
    {
      mbed_dbg_assert( s_task_pool.is_in_pool( meta->freertos_static_task_data ) );
      s_task_pool.release( meta->freertos_static_task_data );
    }

    /*-----------------------------------------------------------------------
    Deregister and release the meta structure back to the pool
    -----------------------------------------------------------------------*/
    s_task_meta_pool.release( meta );
    s_task_meta_map.erase( etl::remove( s_task_meta_map.begin(), s_task_meta_map.end(), meta ), s_task_meta_map.end() );
  }


  void set_affinity( TaskId task, size_t coreId )
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
    if( meta->freertos_handle != nullptr )
    {
      vTaskCoreAffinitySet( meta->freertos_handle, coreId );
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
