// typedef struct for calculation of physical values
// Example! Replace this file with the c-coderdbc output!

#include <stdint.h>
#include <stdlib.h>
#include "decode.h"
#include <ixxat_gw/Frame_1.h>


ixxat_gw::Frame_1* Frame_1(Frame_1 *pkt)
{
ixxat_gw::Frame_1 *pAppMsg = (ixxat_gw::Frame_1 *) calloc(1,sizeof(ixxat_gw::Frame_1));
pAppMsg->header.stamp = ros::Time::now();
pAppMsg->header.seq = 0;
  pAppMsg->Acceleration_X = pkt->Acceleration_X * 0.040000 + -327.680000;
  pAppMsg->Acceleration_Y = pkt->Acceleration_Y * 0.040000 + -327.680000;
  pAppMsg->Rate_Z = pkt->Rate_Z * 0.020000 + -163.840000;  
return pAppMsg;
}

