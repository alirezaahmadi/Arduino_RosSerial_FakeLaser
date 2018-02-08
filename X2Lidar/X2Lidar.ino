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
float ranges[16];
char frameid[] = "/base_link";

     
void setup()
{
  myOcto.begin(ACTIVE_SONARS);   // initialize bus, pins etc
  
  nh.initNode();
  delay(1000);
  nh.advertise(pub_range);
  scan.header.stamp=nh.now();
  scan.ranges_length=16;
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
  if (last_print + 999 < millis()) {   // serial output every 200ms
    last_print = millis();
    i=count;
    count =0;
  }
  
  OctoSonar::doSonar();  // call every cycle, OctoSonar handles the spacing  
  
  for (uint8_t i = 0; i < 16; i++) {
    Distance[i] = myOcto.read(i);
  }
  
  for(int j=0;j<16;j++){
    scan.ranges[j] =Distance[j];
    
  }
  
  scan.ranges[i]=i; 
  count++;
  
  pub_range.publish(&scan);
  nh.spinOnce();
  delay(50);
}
