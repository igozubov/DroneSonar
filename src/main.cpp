#include <Arduino.h>
#include <mavlink.h>

void Mav_Request_Data();
void comm_receive();

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
  /*
          Serial.begin (57600); 

        pinMode (trigger_pin, OUTPUT); 

        pinMode (echo_pin, INPUT);

          pinMode(LED_BUILTIN, OUTPUT);

*/


  pinMode(RxPin0, INPUT);
  pinMode(TxPin0, OUTPUT);

  // MAVLink interface start
   Serial.begin(57600);

  #ifdef SOFT_SERIAL_DEBUGGING
    // [DEB] Soft serial port start
    Serial.begin(57600);
    pxSerial.begin(57600);
    Serial.println("MAVLink starting.");
  #endif
 //Mav_Request_Data();
}

void loop() {
  /*
   digitalWrite (trigger_pin, HIGH);

    delayMicroseconds (10);

    digitalWrite (trigger_pin, LOW);

    time = pulseIn (echo_pin, HIGH);

    distance = ((time * 0.034) / 2)+1;

    

  //if (distance <= 10) 

    //    {

     //   Serial.println (" Door Open ");

      //  Serial.print (" Distance= ");              

        Serial.println (distance);        

        //digitalWrite (buzzer_pin, HIGH);

       // delay (500);

          digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
*/
//------------------------------------------------



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

    //Mav_Request_Data();
    //Serial.println(num_hbs_pasados);
      //Serial.println(request);
     //  Serial.println(receive);
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

  // Check reception buffer
  comm_receive();

}

void Mav_Request_Data()
{

  request = request + 1;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // STREAMS that can be requested
  /*
     Definitions are in common.h: enum MAV_DATA_STREAM

     MAV_DATA_STREAM_ALL=0, // Enable all data streams
     MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
     MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
     MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
     MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
     MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
     MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
     MAV_DATA_STREAM_ENUM_END=13,

     Data in PixHawk available in:
      - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
      - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
  */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02, 0x05};


  for (int i = 0; i < maxStreams; i++) {
    /*
       mavlink_msg_request_data_stream_pack(system_id, component_id,
          &msg,
          target_system, target_component,
          MAV_DATA_STREAM_POSITION, 10000000, 1);

       mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id,
          mavlink_message_t* msg,
          uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
          uint16_t req_message_rate, uint8_t start_stop)

    */
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

  receive = receive + 1;

  mavlink_message_t msg;
  mavlink_status_t status;

  // Echo for manual debugging
  // Serial.println("---Start---");

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
      
            /*
            mavlink_gps_raw_int_t packet;
            mavlink_msg_gps_raw_int_decode(&msg, &packet);
            
            #ifdef SOFT_SERIAL_DEBUGGING
              Serial.print("\nGPS Fix: ");Serial.println(packet.fix_type);
              Serial.print("GPS Latitude: ");Serial.println(packet.lat);
              Serial.print("GPS Longitude: ");Serial.println(packet.lon);
              Serial.print("GPS Speed: ");Serial.println(packet.vel);
              Serial.print("Sats Visible: ");Serial.println(packet.satellites_visible);
            #endif
            */
          }

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
                  mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
            */
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
 // delay(500);
}