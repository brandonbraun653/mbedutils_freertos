/******************************************************************************
 *  File Name:
 *    pico_time.cpp
 *
 *  Description:
 *    RPI Pico implementation of the mb time interface
 *
 *  2024 | Brandon Braun | brandonbraun653@protonmail.com
 *****************************************************************************/

/*-----------------------------------------------------------------------------
Includes
-----------------------------------------------------------------------------*/
#include <mbedutils/interfaces/time_intf.hpp>
#include <FreeRTOS.h>
#include <task.h>

namespace mb::time
{
  /*---------------------------------------------------------------------------
  Public Functions
  ---------------------------------------------------------------------------*/

  size_t millis()
  {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
  }


  size_t micros()
  {
    return millis() * 1000;
  }


  void delayMilliseconds( const size_t val )
  {
    vTaskDelay( pdMS_TO_TICKS( val ) );
  }


  void delayMicroseconds( const size_t val )
  {
    vTaskDelay( pdMS_TO_TICKS( val / 1000 ) );
  }

}  // namespace mb::time
