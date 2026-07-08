/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/**
 * @file  vl53l1_types.h
 * @brief VL53L1 types definition
 */

/** @defgroup porting_type  Basic type definition
 *  @ingroup  api_platform
 *
 *  @brief  file vl53l1_types.h files hold basic type definition that may requires porting
 *
 *  contains type that must be defined for the platform\n
 *  when target platform and compiler provide stdint.h and stddef.h it is enough to include it.\n
 *  If stdint.h is not available review and adapt all signed and unsigned 8/16/32 bits basic types. \n
 *  If stddef.h is not available review and adapt NULL definition .
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>







/** use where fractional values are expected
 *
 * Given a floating point value f it's .16 bit point is (int)(f*(1<<16))*/
typedef uint32_t FixPoint1616_t;


