/*
  Proximity Sensor
*/

#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>
#include <SparkFunLSM9DS1.h>

#define sensorIR_1 14               //Must be an analog pin
#define sensorIR_2 15     
#define sensorIR_3 16     
#define sensorIR_4 17     
float sensorValue, inches, cm;    //Must be of type float for pow()

#define CYCLE_TIME 500 //ms

//Networking
int status = WL_IDLE_STATUS;
char ssid[] = "A_Notable_Abundance_of_Otters";
char pass[] = "ohmamathoseotterscoming";
int keyIndex = 0;
unsigned int localPort = 2390;
char packetBuffer[256];                 //buffer for incoming packet
char  ReplyBuffer[] = "default";   //buffer for outgoing packet
WiFiUDP Udp;

//IMU
LSM9DS1 imu;
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

float roll;
float pitch;
float heading;
float distance;

void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

void setup() {
  Serial.begin(9600);
  Serial.println("Setting up Arduino");
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while(!Serial){
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE){
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION){
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  print_wifi_status();

  Serial.println("Starting server");
  Udp.begin(localPort);

  Wire.begin();
  if (imu.begin() == false) { // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }

  int packetSize = Udp.parsePacket();
  if (packetSize){
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);
  }
}

void loop() {             
  //Timing code
  unsigned long begin_time = millis();
  unsigned long end_time = 0;

  read_imu();
  char distance_string[32];
  char sensor_id[16];
  
  distance = get_sensor_value(sensorIR_1);
  memset(distance_string, 0, sizeof(distance_string));
  dtostrf(distance, 4, 1, distance_string); //4 is mininum width, 3 is precision; float value is copied onto buff
  set_reply_buffer();
  strcat(ReplyBuffer, distance_string);
  strcat(ReplyBuffer, ",");
  itoa(1, sensor_id, 10);
  strcat(ReplyBuffer, sensor_id);
  send_packet();
  delay(75);
  
  distance = get_sensor_value(sensorIR_2);
  memset(distance_string, 0, sizeof(distance_string));
  dtostrf(distance, 4, 1, distance_string); //4 is mininum width, 3 is precision; float value is copied onto buff
  set_reply_buffer();
  strcat(ReplyBuffer, distance_string);
  strcat(ReplyBuffer, ",");
  itoa(2, sensor_id, 10);
  strcat(ReplyBuffer, sensor_id);
  send_packet();
  delay(75);

  distance = get_sensor_value(sensorIR_3);
  memset(distance_string, 0, sizeof(distance_string));
  dtostrf(distance, 4, 1, distance_string); //4 is mininum width, 3 is precision; float value is copied onto buff
  set_reply_buffer();
  strcat(ReplyBuffer, distance_string);
  strcat(ReplyBuffer, ",");
  itoa(3, sensor_id, 10);
  strcat(ReplyBuffer, sensor_id);
  send_packet();
  delay(75);

  distance = get_sensor_value(sensorIR_4);
  memset(distance_string, 0, sizeof(distance_string));
  dtostrf(distance, 4, 1, distance_string); //4 is mininum width, 3 is precision; float value is copied onto buff
  set_reply_buffer();
  strcat(ReplyBuffer, distance_string);
  strcat(ReplyBuffer, ",");
  itoa(4, sensor_id, 10);
  strcat(ReplyBuffer, sensor_id);
  send_packet();

  //More timing code
  end_time = millis();
  if (end_time - begin_time < 1000*CYCLE_TIME)
    delay(CYCLE_TIME - (end_time - begin_time)); // each loop takes CYCLE_TIME ms
  else
    delay(10); // Accept some error
}

float get_sensor_value(int pin){
  sensorValue = analogRead(pin);
  cm = 10650.08 * pow(sensorValue,-0.935) - 10;
  return cm;
}

void set_reply_buffer(){
  char roll_string[32];
  dtostrf(roll, 4, 3, roll_string);
  char pitch_string[32];
  dtostrf(pitch, 4, 3, pitch_string);
  char heading_string[32];
  dtostrf(heading, 4, 3, heading_string);

  //put buffer string together
  memset(ReplyBuffer, 0, sizeof(ReplyBuffer));
  strcat(ReplyBuffer, roll_string);
  strcat(ReplyBuffer, ",");
  strcat(ReplyBuffer, pitch_string);
  strcat(ReplyBuffer, ",");
  strcat(ReplyBuffer, heading_string);
  strcat(ReplyBuffer, ",");
}
  
void send_packet(){
  Serial.print("Sending Packet: ");
  Serial.println(ReplyBuffer);
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(ReplyBuffer);
  Udp.endPacket();
}

void print_wifi_status(){

  
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void read_imu(){
  if ( imu.gyroAvailable() ){
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() ){
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() ){
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

  if ((lastPrint + PRINT_SPEED) < millis()){
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    printAttitude(imu.ax, imu.ay, imu.az,
                  -imu.my, -imu.mx, imu.mz);
    Serial.println();

    lastPrint = millis(); // Update lastPrint time
  }
}

void printGyro(){
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel(){
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag(){
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz){
  roll = atan2(ay, az);
  pitch = atan2(-ax, sqrt(ay * ay + az * az));

  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
