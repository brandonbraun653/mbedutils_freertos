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
#include <mbedutils/interfaces/mutex_intf.hpp>

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

#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
  using mt_array = etl::array<StaticSemaphore, MBEDUTILS_OSAL_MUTEX_POOL_SIZE>;

  static mt_array        s_mutex_pool;
  static size_t          s_mutex_pool_index;
  static StaticSemaphore s_mutex_pool_cs;
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
  using rmt_array = etl::array<StaticSemaphore, MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE>;

  static rmt_array       s_r_mutex_pool;
  static size_t          s_r_mtx_pool_idx;
  static StaticSemaphore s_r_mtx_pool_cs;
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initMutexDriver()
  {
#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
    s_mutex_pool_index     = 0;
    s_mutex_pool_cs.handle = xSemaphoreCreateMutexStatic( &s_mutex_pool_cs.data );

    // Initialize the mutex pool
    for( auto &mtx : s_mutex_pool )
    {
      mtx.handle = xSemaphoreCreateMutexStatic( &mtx.data );
    }
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
    s_r_mtx_pool_idx       = 0;
    s_r_mtx_pool_cs.handle = xSemaphoreCreateMutexStatic( &s_r_mtx_pool_cs.data );

    // Initialize the recursive mutex pool
    for( auto &rmtx : s_r_mutex_pool )
    {
      rmtx.handle = xSemaphoreCreateRecursiveMutexStatic( &rmtx.data );
    }
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
  }


  bool createMutex( mb_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex == nullptr );
    SemaphoreHandle_t tmp = xSemaphoreCreateMutex();
    if( tmp == nullptr )
    {
      mbed_assert_continue_msg( false, "Failed to allocate mutex" );
      return false;
    }

    mutex = reinterpret_cast<mb_mutex_t>( tmp );
    return true;
  }


  void destroyMutex( mb_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    vSemaphoreDelete( reinterpret_cast<SemaphoreHandle_t>( mutex ) );
    mutex = nullptr;
  }


  bool allocateMutex( mb_mutex_t &mutex )
  {
    bool allocated = false;
    mbed_dbg_assert( mutex == nullptr );

#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
    xSemaphoreTake( s_mutex_pool_cs.handle, portMAX_DELAY );
    {
      if( s_mutex_pool_index < s_mutex_pool.size() )
      {
        mutex = reinterpret_cast<mb_mutex_t>( &s_mutex_pool[ s_mutex_pool_index ].handle );
        s_mutex_pool_index++;
        allocated = true;
      }
    }
    xSemaphoreGive( s_mutex_pool_cs.handle );
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

    return allocated;
  }


  void deallocateMutex( mb_mutex_t &mutex )
  {
    // TODO: Implement when/if needed. Likely need to change to a pool allocator.
    mbed_dbg_assert( mutex != nullptr );
    mbed_assert_always();
  }


  void lockMutex( mb_mutex_t mutex )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return;
    }

    mbed_dbg_assert( mutex != nullptr );
    xSemaphoreTake( static_cast<SemaphoreHandle_t>( mutex ), portMAX_DELAY );
  }


  bool tryLockMutex( mb_mutex_t mutex )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return true;
    }

    mbed_dbg_assert( mutex != nullptr );
    return xSemaphoreTake( static_cast<SemaphoreHandle_t>( mutex ), 0 ) == pdTRUE;
  }


  bool tryLockMutex( mb_mutex_t mutex, const size_t timeout )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return true;
    }

    mbed_dbg_assert( mutex != nullptr );
    mbed_dbg_assert( timeout < std::numeric_limits<uint32_t>::max() );
    return xSemaphoreTake( static_cast<SemaphoreHandle_t>( mutex ), pdMS_TO_TICKS( timeout ) ) == pdTRUE;
  }


  void unlockMutex( mb_mutex_t mutex )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return;
    }

    mbed_dbg_assert( mutex != nullptr );
    xSemaphoreGive( static_cast<SemaphoreHandle_t>( mutex ) );
  }


  bool createRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex == nullptr );
    SemaphoreHandle_t tmp = xSemaphoreCreateRecursiveMutex();
    if( tmp == nullptr )
    {
      mbed_assert_continue_msg( false, "Failed to allocate recursive mutex" );
      return false;
    }

    mutex = reinterpret_cast<mb_recursive_mutex_t>( tmp );
    return true;
  }


  void destroyRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
    vSemaphoreDelete( reinterpret_cast<SemaphoreHandle_t>( mutex ) );
    mutex = nullptr;
  }


  bool allocateRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    bool allocated = false;
    mbed_dbg_assert( mutex == nullptr );

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
    xSemaphoreTake( s_r_mtx_pool_cs.handle, portMAX_DELAY );
    {
      if( s_r_mtx_pool_idx < s_r_mutex_pool.size() )
      {
        mutex = reinterpret_cast<mb_recursive_mutex_t>( &s_r_mutex_pool[ s_r_mtx_pool_idx ].handle );
        s_r_mtx_pool_idx++;
        allocated = true;
      }
    }
    xSemaphoreGive( s_r_mtx_pool_cs.handle );
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0

    return allocated;
  }


  void deallocateRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    // TODO: Implement when/if needed. Likely need to change to a pool allocator.
    mbed_dbg_assert( mutex != nullptr );
    mbed_assert_always();
  }

  void lockRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return;
    }

    mbed_dbg_assert( mutex != nullptr );
    xSemaphoreTakeRecursive( static_cast<SemaphoreHandle_t>( mutex ), portMAX_DELAY );
  }


  bool tryLockRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return true;
    }

    mbed_dbg_assert( mutex != nullptr );
    return xSemaphoreTakeRecursive( static_cast<SemaphoreHandle_t>( mutex ), 0 ) == pdTRUE;
  }


  bool tryLockRecursiveMutex( mb_recursive_mutex_t mutex, const size_t timeout )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return true;
    }

    mbed_dbg_assert( mutex != nullptr );
    mbed_dbg_assert( timeout < std::numeric_limits<uint32_t>::max() );
    return xSemaphoreTakeRecursive( static_cast<SemaphoreHandle_t>( mutex ), pdMS_TO_TICKS( timeout ) ) == pdTRUE;
  }


  void unlockRecursiveMutex( mb_recursive_mutex_t mutex )
  {
    if( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )
    {
      return;
    }

    mbed_dbg_assert( mutex != nullptr );
    xSemaphoreGiveRecursive( static_cast<SemaphoreHandle_t>( mutex ) );
  }
}  // namespace mb::osal
