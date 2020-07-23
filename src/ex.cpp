/*
#include <common/mavlink.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

void request_datastream();
void MavLink_receive();

#define RXpin 9
#define TXpin 11
SoftwareSerial SerialMAV(RXpin, TXpin); // sets up serial communication on pins 3 and 2

void setup() {
  SerialMAV.begin(57600); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  Serial.begin(57600); //Main serial port for console output

request_datastream();

}

void loop() {

MavLink_receive();

}

//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
void MavLink_receive()
  { 
  mavlink_message_t msg;
  mavlink_status_t status;

  while(SerialMAV.available())
  {
    uint8_t c= SerialMAV.read();

    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {

    //Handle new message from autopilot
      switch(msg.msgid)
      {

        case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        mavlink_gps_raw_int_t packet;
        mavlink_msg_gps_raw_int_decode(&msg, &packet);
        
        Serial.print("\nGPS Fix: ");Serial.println(packet.fix_type);
        Serial.print("GPS Latitude: ");Serial.println(packet.lat);
        Serial.print("GPS Longitude: ");Serial.println(packet.lon);
        Serial.print("GPS Speed: ");Serial.println(packet.vel);
        Serial.print("Sats Visible: ");Serial.println(packet.satellites_visible);
       
      }
      break;

      }
    }
  }
}

void request_datastream() {
//Request Data from Pixhawk
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  uint8_t _start_stop = 1; //1 = start, 0 = stop

// STREAMS that can be requested


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  SerialMAV.write(buf, len); //Write data to serial port
}
*/