/******************************************************************************
 *  File Name:
 *    freertos_mutex.cpp
 *
 *  Description:
 *    FreeRTOS implementation of the mutex interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <etl/array.h>
#include <limits>
#include <mbedutils/assert.hpp>
#include <mbedutils/config.hpp>
#include <mbedutils/interfaces/smphr_intf.hpp>

#include <FreeRTOS.h>
#include "semphr.h"

namespace mb::osal
{
  /*---------------------------------------------------------------------------
  Structures
  ---------------------------------------------------------------------------*/

  struct StaticSemaphore
  {
    StaticSemaphore_t data;
    SemaphoreHandle_t handle;
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

#if MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0
  using smphr_array = etl::array<StaticSemaphore, MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE>;
  static smphr_array        s_smphr_pool;
  static size_t             s_smphr_pool_index;
  static StaticSemaphore    s_smphr_pool_cs;
#endif    // MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initSmphrDriver()
  {
#if MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0
    s_smphr_pool_index = 0;
    s_smphr_pool_cs.handle = xSemaphoreCreateMutexStatic( &s_smphr_pool_cs.data );

    // Initialize the pool of semaphores
    for( auto &sem : s_smphr_pool )
    {
      sem.handle = nullptr;
    }
#endif    // MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0
  }


  bool createSmphr( mb_smphr_t &s, const size_t maxCount, const size_t initialCount )
  {
    mbed_dbg_assert( s == nullptr );
    mbed_dbg_assert( maxCount < std::numeric_limits<int16_t>::max() );
    mbed_dbg_assert( initialCount < std::numeric_limits<int16_t>::max() );

    SemaphoreHandle_t tmp = xSemaphoreCreateCounting( maxCount, initialCount );
    if( tmp == nullptr )
    {
      mbed_assert_continue_msg( false, "Failed to allocate semaphore" );
      return false;
    }

    s = reinterpret_cast<mb_smphr_t>( tmp );
    return true;
  }


  void destroySmphr( mb_smphr_t &s )
  {
    mbed_dbg_assert( s != nullptr );
    vSemaphoreDelete( reinterpret_cast<SemaphoreHandle_t>( s ) );
    s = nullptr;
  }


  bool allocateSmphr( mb_smphr_t &s, const size_t maxCount, const size_t initialCount )
  {
    bool allocated = false;
    mbed_dbg_assert( s == nullptr );
    mbed_dbg_assert( maxCount < std::numeric_limits<int16_t>::max() );
    mbed_dbg_assert( initialCount < std::numeric_limits<int16_t>::max() );

#if MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0
    xSemaphoreTake( s_smphr_pool_cs.handle, portMAX_DELAY );
    {
      if( s_smphr_pool_index < s_smphr_pool.size() )
      {
        StaticSemaphore *sem = &s_smphr_pool[ s_smphr_pool_index ];
        sem->handle = xSemaphoreCreateCountingStatic( maxCount, initialCount, &sem->data );
        s_smphr_pool_index++;
        s = reinterpret_cast<mb_smphr_t>( sem->handle );
        allocated = true;
      }
    }
    xSemaphoreGive( s_smphr_pool_cs.handle );
#endif    // MBEDUTILS_OSAL_SEMAPHORE_POOL_SIZE > 0

    return allocated;
  }


  void deallocateSemaphore( mb_smphr_t &s )
  {
    // TODO: Implement when/if needed. Likely need to change to a pool allocator.
    mbed_dbg_assert( s != nullptr );
    mbed_assert_always();
  }


  size_t getSmphrAvailable( mb_smphr_t &s )
  {
    mbed_dbg_assert( s != nullptr );
    return static_cast<size_t>( uxSemaphoreGetCount( static_cast<SemaphoreHandle_t>( s ) ) );
  }


  void releaseSmphr( mb_smphr_t &s )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return;
    }

    mbed_dbg_assert( s != nullptr );
    xSemaphoreGive( static_cast<SemaphoreHandle_t>( s ) );
  }


  void releaseSmphrFromISR( mb_smphr_t &s )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return;
    }

    /* Pico doesn't need to do anything fancy for this */
    mbed_dbg_assert( s != nullptr );
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( static_cast<SemaphoreHandle_t>( s ), &higherPriorityTaskWoken );
    portYIELD_FROM_ISR( higherPriorityTaskWoken );
  }


  void acquireSmphr( mb_smphr_t &s )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return;
    }

    mbed_dbg_assert( s != nullptr );
    xSemaphoreTake( static_cast<SemaphoreHandle_t>( s ), portMAX_DELAY );
  }


  bool tryAcquireSmphr( mb_smphr_t &s )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return true;
    }

    mbed_dbg_assert( s != nullptr );
    return xSemaphoreTake( static_cast<SemaphoreHandle_t>( s ), 0 ) == pdTRUE;
  }


  bool tryAcquireSmphr( mb_smphr_t &s, const size_t timeout )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return true;
    }

    mbed_dbg_assert( s != nullptr );
    mbed_dbg_assert( timeout < std::numeric_limits<uint32_t>::max() );
    return xSemaphoreTake( static_cast<SemaphoreHandle_t>( s ), pdMS_TO_TICKS( timeout ) ) == pdTRUE;
  }
}  // namespace mb::osal
