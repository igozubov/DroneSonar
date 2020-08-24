#include <Arduino.h>
#include <mavlink.h>
#include "mavlink_msg_distance_sensor.h"

void Mav_Request_Data();
void comm_receive();
void command_distance(int8_t orient ,uint16_t rngDist);
void command_heartbeat();
//void command_distance();

#define RxPin0 9
#define TxPin0 10

#define SOFT_SERIAL_DEBUGGING

#ifdef SOFT_SERIAL_DEBUGGING
  #include <SoftwareSerial.h>

  int request = 0;
  int receive = 0;

  SoftwareSerial pxSerial = SoftwareSerial(RxPin0, TxPin0);  // RX, TX || UNO - PX4
#endif

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

int trigger_pin = 2;

int echo_pin = 3;

//nt buzzer_pin = 10; 

int time;

int distance; 

void setup() {
  pinMode(RxPin0, INPUT);
  pinMode(TxPin0, OUTPUT);

  Serial.begin(57600);

  #ifdef SOFT_SERIAL_DEBUGGING
    Serial.begin(57600);
    pxSerial.begin(57600);
    Serial.println("MAVLink starting.");
  #endif
}

void loop() {
/*
  // MAVLink

  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_FIXED_WING;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    previousMillisMAVLink = currentMillisMAVLink;

    #ifdef SOFT_SERIAL_DEBUGGING
      pxSerial.write(buf, len);
    #else
      Serial.write(buf, len);
    #endif

    num_hbs_pasados++;
    if (num_hbs_pasados >= num_hbs) {
      
      // Request streams from Pixhawk
      #ifdef SOFT_SERIAL_DEBUGGING
        Serial.println("Streams requested!");
      #endif
      
      Mav_Request_Data();
      
      num_hbs_pasados = 0;
    }

  }

  */
command_heartbeat();
  // Check reception buffer
  comm_receive();

//uint16_t distance = 10; 
//uint8_t orient = 1;
// distance= ((uint16_t)((byteLow) + (byteHigh*256))/scale);

 //command_distance(orient ,distance);
}

void command_heartbeat() {
  int sysid = 100;                            //< ID 1 for this system               
  int compid = MAV_COMP_ID_PATHPLANNER;       //< The component sending the message.
  uint8_t system_type =MAV_TYPE_GCS;         // Define the system type, in this case ground control station
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = 0; 
  uint32_t custom_mode = 0;                
  uint8_t system_state = 0;
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message 
  //Serial.write(buf, len);
}

void Mav_Request_Data()
{
  request = request + 1;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02, 0x05};


  for (int i = 0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    #ifdef SOFT_SERIAL_DEBUGGING
      pxSerial.write(buf, len);
    #else
      Serial.write(buf, len);
    #endif
  }
}


void comm_receive() {

  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 30; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 900; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = 10; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 1; /*< Onboard ID of the sensor*/
  uint8_t orientation = 0; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class
  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  pxSerial.write(buf, len);

//------------------------------


 
 // receive = receive + 1;

 // mavlink_message_t msg;
 // mavlink_status_t status;

  // Echo for manual debugging
  // Serial.println("---Start---");
/*
  #ifdef SOFT_SERIAL_DEBUGGING
    while (pxSerial.available() > 0) {
      uint8_t c = pxSerial.read();
  #else
    while (Serial.available() > 0) {
      uint8_t c = Serial.read();
  #endif


    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
            #ifdef SOFT_SERIAL_DEBUGGING
              Serial.println("PX HB");
            #endif
          }
          break;

        case MAVLINK_MSG_ID_GPS_RAW_INT: //24
          {
      
            mavlink_gps_raw_int_t packet;
            mavlink_msg_gps_raw_int_decode(&msg, &packet);

            #ifdef SOFT_SERIAL_DEBUGGING
              Serial.print("\nGPS Fix: ");Serial.println(packet.fix_type);
              Serial.print("GPS Latitude: ");Serial.println(packet.lat);
              Serial.print("GPS Longitude: ");Serial.println(packet.lon);
              Serial.print("GPS Speed: ");Serial.println(packet.vel);
              Serial.print("Sats Visible: ");Serial.println(packet.satellites_visible);
            #endif
          }

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            
            #ifdef SOFT_SERIAL_DEBUGGING
             // Serial.println("PX ATTITUDE1");
             // Serial.println(attitude.roll);
              //Serial.println("PX ATTITUDE2");
              //Serial.println(attitude.pitch);
            //            if(attitude.roll>1) leds_modo = 0;
            //            else if(attitude.roll<-1) leds_modo = 2;
            //            else leds_modo=1;
            #endif
          }

        default:
          break; 
      } 
   }
  }
 */
}


void command_distance(uint8_t orient ,uint16_t rngDist) {
  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 30; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 900; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = rngDist; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 1; /*< Onboard ID of the sensor*/
  uint8_t orientation = orient; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class
  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  Serial.write(buf, len);
 // Serial.print (orientation);
  //Serial.print (" - ");
  //Serial.println (current_distance);
}