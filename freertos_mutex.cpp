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
    bool in_use;
  };

  /*---------------------------------------------------------------------------
  Static Data
  ---------------------------------------------------------------------------*/

#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
  using mt_array = etl::array<StaticSemaphore, MBEDUTILS_OSAL_MUTEX_POOL_SIZE>;

  static mt_array        s_mutex_pool;
  static StaticSemaphore s_mutex_pool_cs;
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
  using rmt_array = etl::array<StaticSemaphore, MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE>;

  static rmt_array       s_r_mutex_pool;
  static StaticSemaphore s_r_mtx_pool_cs;
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0

  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  void initMutexDriver()
  {
#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
    s_mutex_pool_cs.handle = xSemaphoreCreateMutexStatic( &s_mutex_pool_cs.data );

    // Initialize the mutex pool
    for( auto &mtx : s_mutex_pool )
    {
      mtx.handle = xSemaphoreCreateMutexStatic( &mtx.data );
      mtx.in_use = false;
    }
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
    s_r_mtx_pool_cs.handle = xSemaphoreCreateMutexStatic( &s_r_mtx_pool_cs.data );

    // Initialize the recursive mutex pool
    for( auto &rmtx : s_r_mutex_pool )
    {
      rmtx.handle = xSemaphoreCreateRecursiveMutexStatic( &rmtx.data );
      rmtx.in_use = false;
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
      for( auto &mtx : s_mutex_pool )
      {
        if( !mtx.in_use )
        {
          mutex = reinterpret_cast<mb_mutex_t>( &mtx.handle );
          mtx.in_use = true;
          allocated = true;
          break;
        }
      }
    }
    xSemaphoreGive( s_mutex_pool_cs.handle );
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0

    return allocated;
  }


  void deallocateMutex( mb_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex != nullptr );
#if MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
    xSemaphoreTake( s_mutex_pool_cs.handle, portMAX_DELAY );
    {
      for( auto &mtx : s_mutex_pool )
      {
        if( &mtx.handle == mutex )
        {
          mtx.in_use = false;
          mutex      = nullptr;
          break;
        }
      }
    }
    xSemaphoreGive( s_mutex_pool_cs.handle );
#endif    // MBEDUTILS_OSAL_MUTEX_POOL_SIZE > 0
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
      for( auto &rmtx : s_r_mutex_pool )
      {
        if( !rmtx.in_use )
        {
          mutex = reinterpret_cast<mb_recursive_mutex_t>( &rmtx.handle );
          rmtx.in_use = true;
          allocated = true;
          break;
        }
      }
    }
    xSemaphoreGive( s_r_mtx_pool_cs.handle );
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0

    return allocated;
  }


  void deallocateRecursiveMutex( mb_recursive_mutex_t &mutex )
  {
    mbed_dbg_assert( mutex != nullptr );

#if MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
    xSemaphoreTake( s_r_mtx_pool_cs.handle, portMAX_DELAY );
    {
      for( auto &rmtx : s_r_mutex_pool )
      {
        if( &rmtx.handle == mutex )
        {
          rmtx.in_use = false;
          mutex       = nullptr;
          break;
        }
      }
    }
    xSemaphoreGive( s_r_mtx_pool_cs.handle );
#endif    // MBEDUTILS_OSAL_RECURSIVE_MUTEX_POOL_SIZE > 0
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
