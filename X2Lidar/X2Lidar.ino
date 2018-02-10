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
float ranges[12];
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
  scan.angle_increment=(0.26179938779)*2;
  scan.range_min=0.03;
  scan.range_max=5.0;
  scan.ranges_length=12;
  scan.intensities_length=12;
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
  scan.ranges[1]=(float)Distance[2]/100;
  scan.ranges[2]=(float)Distance[0]/100;
  scan.ranges[3]=(float)Distance[5]/100;
  scan.ranges[4]=(float)Distance[6]/100;
  scan.ranges[5]=(float)Distance[4]/100;
  scan.ranges[6]=(float)Distance[9]/100;
  scan.ranges[7]=(float)Distance[10]/100;
  scan.ranges[8]=(float)Distance[8]/100;
  scan.ranges[9]=(float)Distance[13]/100;
  scan.ranges[10]=(float)Distance[14]/100;
  scan.ranges[11]=(float)Distance[12]/100;

  pub_range.publish(&scan);
  nh.spinOnce();
  delay(35);
}
