// typedef struct for message decoding
// Example! Replace this file with the c-coderdbc output!
#pragma once
#include <stdint.h>
typedef struct __attribute__((packed, aligned(1))) Frame_1
{
  uint16_t Acceleration_X : 16;                //      Bits=16 Offset= -327.680000        Factor= 0.040000        Unit:'Unit_MeterPerSeconSquar'
  uint16_t Acceleration_Y : 16;                //      Bits=16 Offset= -327.680000        Factor= 0.040000        Unit:'Unit_MeterPerSeconSquar'
  uint16_t Rate_Z : 16;                        //      Bits=16 Offset= -163.840000        Factor= 0.020000        Unit:'Unit_DegreOfArcPerSecon'
} Frame_1;