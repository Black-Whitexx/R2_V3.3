/**
  ******************************************************************************
  * @file           : DT35.c
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/16
  ******************************************************************************
  */
#include "DT35.h"
#include "Chassis.h"
#include "PID.h"

DT35_Struct DT35_Data;
PID_t DT35_Run;
PointStruct DT32_Points;
/** 用于存储比赛5个放球点 **/
PointStruct DT32_AimPoints[5]= {
        {.x = 48.41f,.y = 41.00f,.angle = 90.0f,.num = 0},
        {.x = 123.51f,.y = 41.32f,.angle = 90.0f,.num = 0},
        {.x = 198.09f,.y = 41.32f,.angle = 90.0f,.num = 0},
        {.x = 274.81f,.y = 41.32f,.angle = 90.0f,.num = 0},
        {.x = 350.02f,.y = 41.32f,.angle = 90.0f,.num = 0}
};

void DT35_Rec(uint8_t *data,DT35_Struct *DT35_data)
{
    DT35_data->forward = (float)( data[1] << 24 | data[2] << 16 | data[3] << 8 | data[4] ) / 10000;
    DT35_data->Left = (float)( data[5] << 24 | data[6] << 16 | data[7] << 8 | data[8] ) / 10000;
    DT35_data->Right = (float)( data[9] << 24 | data[10] << 16 | data[11] << 8 | data[12] ) / 10000;
//    printf("%f,%f,%f\n",DT35_data->forward,DT35_data->Left,DT35_data->Right);
}