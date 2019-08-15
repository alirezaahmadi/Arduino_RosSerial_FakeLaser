/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include <ros.h>
#include <Wire.h>

#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
       
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

#include <ros/time.h>
#include "OctoSonar.h"
#define SONAR_ADDR 0x20
#define SONAR_INT 19
#define ACTIVE_SONARS 0x7777
#define OUT_SERIAL              // uncomment for serial output
#define LEDPin 13 // Onboard LED

OctoSonarX2 myOcto(SONAR_ADDR, SONAR_INT);

ros::NodeHandle  nh;
sensor_msgs::LaserScan scan;
ros::Publisher pub_range( "scan", &scan);
float ranges[60];
char frameid[] = "/base_link";

     
void setup()
{
  myOcto.begin(ACTIVE_SONARS);   // initialize bus, pins etc
  
  nh.initNode();
  delay(1000);
  nh.advertise(pub_range);
  scan.header.frame_id = "base_footprint";
  scan.header.stamp=nh.now();
  scan.angle_min=0;
  scan.angle_max=0.6;
  scan.time_increment=0;
  scan.angle_increment=0.052359877559833*2;  //(0.26179938779)*2;
  scan.range_min=0.03;
  scan.range_max=5.0;
  scan.ranges_length=60;
  scan.intensities_length=10;
  scan.ranges= ranges;
} 

uint32_t last_print = 0;
uint16_t Distance[16] = {0};
long range_time = 0;
int r =0;
long count = 0;
int32_t i=0;

void loop()
{
//  if (last_print + 999 < millis()) {   // serial output every 200ms
//    last_print = millis();
//    i=count;
//    count =0;
//  }
//  scan.ranges[15]=i; 
//  count++;
  
  OctoSonar::doSonar();  // call every cycle, OctoSonar handles the spacing  
  
  for (uint8_t i = 0; i < 16; i++) {
    Distance[i] = myOcto.read(i);
  }

  scan.ranges[0]=(float)Distance[1]/100;
  for(uint8_t i = 0; i <= 10; i++)  scan.ranges[i]=scan.ranges[0];
  
  scan.ranges[6]=(float)Distance[2]/100;
  for(uint8_t i = 6; i <= 10; i++)  scan.ranges[i]=scan.ranges[6];
  
  scan.ranges[11]=(float)Distance[0]/100;
  for(uint8_t i = 11; i <= 15; i++)  scan.ranges[i]=scan.ranges[11];
  
  scan.ranges[16]=(float)Distance[5]/100;
  for(uint8_t i = 16; i <= 20; i++)  scan.ranges[i]=scan.ranges[16];
  
  scan.ranges[21]=(float)Distance[6]/100;
  for(uint8_t i = 21; i <= 25; i++)  scan.ranges[i]=scan.ranges[21];
  
  scan.ranges[26]=(float)Distance[4]/100;
  for(uint8_t i = 26; i <= 30; i++)  scan.ranges[i]=scan.ranges[26];
  
  scan.ranges[31]=(float)Distance[9]/100;
  for(uint8_t i = 31; i <= 35; i++)  scan.ranges[i]=scan.ranges[31];
  
  scan.ranges[36]=(float)Distance[10]/100;
  for(uint8_t i = 36; i <= 40; i++)  scan.ranges[i]=scan.ranges[36];
  
  scan.ranges[41]=(float)Distance[8]/100;
  for(uint8_t i = 41; i <= 45; i++)  scan.ranges[i]=scan.ranges[41];
  
  scan.ranges[46]=(float)Distance[13]/100;
  for(uint8_t i = 46; i <= 50; i++) scan.ranges[i]=scan.ranges[46];
  
  scan.ranges[51]=(float)Distance[14]/100;
  for(uint8_t i = 51; i <= 55; i++)  scan.ranges[i]=scan.ranges[51];
  
  scan.ranges[56]=(float)Distance[12]/100;
  for(uint8_t i = 56; i < 60; i++)  scan.ranges[i]=scan.ranges[565];

  
  pub_range.publish(&scan);
  nh.spinOnce();
  delay(35);
}
