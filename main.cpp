/* ========================================
*
* Copyright Tonkin and Taylor, 2020
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : (Eddie?) and Nathanaël Esnault
* Verified by   : N/A
* Creation date : 2020-11-20
* Version       : 1.0 (finished on ...)
* Modifications :
* Known bugs    :
*
*
* Possible Improvements
*
* Associated files .c and .h: 
* ----> parameters.h
*
*
* Notes
*
*
* ========================================
*/

// MAC address (ESP8266): 60:01:94:79:74:3E



//---------------------- Libraries (12) ----------------------  
#include <Arduino.h>              // Only when using Visual Studio Code, if using Arduino IDE then comment out with "//" or Ctrl + /

#include <Wire.h>                 // Used for I2C protocol communication: INA219 current sensor + IMU
#include <Adafruit_INA219.h>      // For voltage and current consumtion

// IMU: FXO and FXA
#include <Adafruit_FXOS8700.h>   // used for the Mag+Acc
#include <Adafruit_FXAS21002C.h> // used for the Gyr
#include <Adafruit_Sensor.h>     // Unified libraries for most Adafruit sensors

#include <ESP8266WiFi.h>         // Wifi client (+NTP +led server)

#include <WiFiUdp.h>            // NTP client: to retrieve the time via internet
#include <NTPClient.h> 

#include <WiFiClient.h>         //static IP (+NTP)
#include <ESP8266WebServer.h>   //static IP

#include <Ticker.h>             // Watchdog + Timer interrupt (ISR) for logging IMU data

#include <parameters.h>         // The paramters chosen for this code, put in a separate file

//---------------------- Define ----------------------  
// Pins
#define PROTO_SHIELD_LED    D4

// Data needed to find and connect to the 4G module (Access Point)
#ifndef STASSID
  #define STASSID       "Robots"      // Name of the WiFi network
  #define STAPSK        "0800838383"  // Password for the WiFi
#endif

// NTP: for getting time via internet
#define NTP_TIMEOFFSET      46800
#define NTP_UPDATEINTERVAL  60000

// Watchdog
#define WDG_TIMEOUT_S       20
#define WDG_DISPLAY_COUNTER 0
#define WDG_PERIOD_S        0.1

#define WEBS_PERIOD_S       4.2

// Data for the ISR to determine at what frequency to log the IMU data (the argument is the period in [s], ie 1/freq)
#define READANDLOG_PERIOD_S 0.25

// Some debugging tools
#define USE_NTP

//---------------------- Global Variables ----------------------

// INA219 current sensor
Adafruit_INA219 ina219;

// IMU
// Assign a unique ID to this sensor at the same time
Adafruit_FXOS8700   accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro     = Adafruit_FXAS21002C(0x0021002C);

// NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", NTP_TIMEOFFSET, NTP_UPDATEINTERVAL); // This specifies when to try to reconnect
// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

// Webserver for diaplaying voltage/current
WiFiServer server(80);          // This webserver will use the port 80
//Ticker     handleWbRequests;    // A hardware timer to increase timing accuracy

// Watchdog
Ticker secondTick;
volatile int watchdogCount = 0;
int value = 0;

// Logs
Ticker           readAndLog;         // A hardware timer to increase timing accuracy
volatile int32_t messageCounter = 0; // used as an index to check if we missed any values in the log
volatile bool    toggleLed      = 0; // used to toggle the LEDs at each loop  

// Wifi connection (static IP, NTP and webserver)
const char* ssid     = STASSID;
const char* password = STAPSK;

// Static IP address configuration
IPAddress staticIP(192, 168, 1, 200); //ESP static ip
IPAddress gateway(192, 168, 1, 1);    //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);   //Subnet mask
IPAddress dns(192, 168, 1, 1);        //DNS
const char* deviceName = "IMU_Logger"; // Used in the static IP example, can be anything, wii,be displayed in the "Client Name" in the DHCP


/* Example on your router/AP/4G module:
+----+-------------+-------------------+-----------------+------------+
| ID | Client Name |    MAC Address    |   Assigned IP   | Lease Time |
+----+-------------+-------------------+-----------------+------------+
|  1 |  IMU_Logger | 60:01:94:79:74:3E | 192.168.1.151   |  00:00:44  |
+----+-------------+-------------------+-----------------+------------+
*/

//---------------------- Function declaration (6) ----------------------  
void displayWebServerPage (float displayBusVoltage, float displayCurrent);
void ISRwatchdog          (void); // ISR
void displaySensorDetails (void); // ISR
void esp_output           (void);
void readAndLogFunction   (void);
void ISRwebserver         (void); // ISR


//---------------------- Setup ----------------------  
void setup(){

  //system_update_cpu_freq(160); //Set CPU frequency to 160MHz


  delay(3000); // Wait 1s before starting the program 
  Serial.begin(115200);
  delay(1000);
  Serial.print("Startup reason: ");
  Serial.println(ESP.getResetReason());
  esp_output();  // Display some information about the ESP microcontroller

  // Set up pins
  // builtin LED is the blue LED on the ESP8266 metal RF shield
  pinMode(LED_BUILTIN, OUTPUT);
  // use the external red LED on the proto shield
  pinMode(PROTO_SHIELD_LED, OUTPUT); 


  //----
  // Start the I2C (INA219 and IMU)
  Wire.begin();
  //----
  // IMU
  if (!gyro.begin()) 
  {
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1)
      ;
  }
  if (!accelmag.begin(ACCEL_RANGE_4G)) 
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1)
      ;
  }
  // Display some basic information about the IMU
  displaySensorDetails();
  
  // Wifi Webserver
  ESP.eraseConfig(); // <--- use this before begin
  server.begin();  // Start the Server
  Serial.println("Server started");
  Serial.print("To read the battery voltage and the  WiFi, use the following URL: https://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  //----
  //  Start by connecting to a WiFi network
  WiFi.disconnect();              //Prevent connecting to wifi based on previous configuration
  WiFi.hostname(deviceName);      // DHCP Hostname (useful for finding device for static lease)
  WiFi.config(staticIP, subnet, gateway, dns);
  WiFi.begin(ssid, password);
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
    would try to act as both a client and an access-point and could cause
    network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA); // WiFi mode STAtion (ie connect to wifi router only)
  // Wait for connection to the 4G module (Access Point) (STATIC IP + NTP)
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());

  //----
  timeClient.begin();
  //----
    if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  ina219.setCalibration_16V_400mA();
  //----
  //create CSV headers in the SD card console logger
  Serial.println("Index, Watchdog[s], Acc_X[m/s^2], Acc_Y[m/s^2], Acc_Z[m/s^2], Mag_X[uT], Mag_Y[uT], Mag_Z[uT], Gyr_X[rad/s], Gyr_Y[rad/s], Gyr_Z[rad/s]");


  //----
  //Serial.print("Starting webserver overwatch every: ");
  //Serial.print(WEBS_PERIOD_S);
  //Serial.println("s");
  //handleWbRequests.attach(WEBS_PERIOD_S, ISRwebserver); // Every WEBS_PERIOD_S seconds, check if web requests
  //----
  // Call the IMUlogging every interval (as a delay in s) with a hardware timer interrupt
  Serial.print("Starting IMU read and log ISR every: ");
  Serial.print(READANDLOG_PERIOD_S);
  Serial.println("s");
  readAndLog.attach(READANDLOG_PERIOD_S, readAndLogFunction);
  //----
  Serial.print("Starting Watchdog every: ");
  Serial.print(WDG_PERIOD_S);
  Serial.println("s");
  secondTick.attach(WDG_PERIOD_S, ISRwatchdog); // Every WDG_PERIOD_S seconds, check watchdog

}









//---------------------- Loop ---------------------- 
void loop() 
{

  /*
  There is very few code in the loop because the IMU data retreiving and
  logging is done in an Timer ISR to be time accurate: readAndLogFunction
  */

   //  Get battery voltage and current for HTML page
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();

  // Webserver for displaying information
  displayWebServerPage(busvoltage, current_mA);
  
}

//---------------------- Function Definition (5) ---------------------- 

void ISRwatchdog(void) 
{
  watchdogCount++;
  if (WDG_DISPLAY_COUNTER)
  {
      Serial.print("Watchdog count: ");
      Serial.println(watchdogCount);
  }

  if ( watchdogCount > WDG_TIMEOUT_S ) 
  {
     Serial.print("Watchdog triggered: ");
     Serial.println(watchdogCount);
     ESP.reset();
  }
}



void displayWebServerPage (float displayBusVoltage, float displayCurrent)
{
  
  // Webserver: Check if a client has connected
    WiFiClient client = server.available();
    if (client) 
    {
      readAndLog.detach();

      // Wait until the client sends some data
      //Serial.println("new client");
      if(!client.available())
      {
      //  while(!client.available()){
        delay(1);
      }
      // Read the first line of the request
      String request = client.readStringUntil('\r');
      //Serial.println(request);
      client.flush();
    
      // Return the response (HTML code)
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println(""); //  do not forget this one
      client.println("<!DOCTYPE HTML>");
      client.println("<html>");
      
      client.println("<head>");
      client.println("<meta http-equiv=""refresh"" content=""5"">");
      client.println("</head>");
    

/*
    // Simple lines
    client.println("");  
    // Voltage
    client.print(displayBusVoltage);
    client.println("V <br>");
  client.println("<br><br>");
  // Current
      client.print(displayCurrent);
    client.println("mA <br>");
  client.println("<br><br>"); 
  */



    // Table w/ CSS
    client.println("<style type=""text/css"">");
    client.println(".tg  {border-collapse:collapse;border-color:#9ABAD9;border-spacing:0;}");
    client.println(".tg td{background-color:#EBF5FF;border-color:#9ABAD9;border-style:solid;border-width:1px;color:#444;");
    client.println("  font-family:Arial, sans-serif;font-size:14px;overflow:hidden;padding:10px 5px;word-break:normal;}");
    client.println(".tg th{background-color:#409cff;border-color:#9ABAD9;border-style:solid;border-width:1px;color:#fff;");
    client.println("  font-family:Arial, sans-serif;font-size:14px;font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}");
    client.println(".tg .tg-bzci{font-size:20px;text-align:center;vertical-align:top}");
    client.println(".tg .tg-7hap{font-size:20px;font-weight:bold;text-align:left;vertical-align:top}");
    client.println(".tg .tg-60hs{font-size:20px;text-align:left;vertical-align:top}");
    client.println("</style>");
    client.println("<table class=""tg"">");
    client.println("<thead>");
    client.println("  <tr>");
    client.println("    <th class=""tg-7hap"">Parameter</th>");
    client.println("    <th class=""tg-7hap"">Value</th>");
    client.println("    <th class=""tg-7hap"">Unit</th>");
    client.println("  </tr>");
    client.println("</thead>");
    client.println("<tbody>");
    client.println("  <tr>");
    client.println("    <td class=""tg-60hs"">Battery Voltage</td>");
    client.print("    <td class=""tg-bzci"">");
    client.print(displayBusVoltage);
    client.println("</td>");
    client.println("    <td class=""tg-60hs"">V</td>");
    client.println("  </tr>");
    client.println("  <tr>");
    client.println("    <td class=""tg-60hs"">Current Consumption</td>");
    client.print("    <td class=""tg-bzci"">");
    client.print(displayCurrent);
    client.println("</td>");
    client.println("    <td class=""tg-60hs"">mA</td>");
    client.println("  </tr>");
    client.println("</tbody>");
    client.println("</table>");

/*
    // Table w/o CSS
    client.println("<table>");
    client.println("<thead>");
    client.println("<tr>");
    client.println("<th>Parameter</th>");
    client.println("<th>Value</th>");
    client.println("<th>Unit</th>");
    client.println("</tr>");
    client.println("</thead>");
    client.println("<tbody>");
    client.println("<tr>");
    client.println("<td>BatteryVoltage</td>");
    client.println("<td>99999</td>");
    client.println("<td>V</td>");
    client.println("</tr>");
    client.println("<tr>");
    client.println("<td>CurrentConsumption</td>");
    client.println("<td>88888</td>");
    client.println("<td>mA</td>");
    client.println("</tr>");
    client.println("</tbody>");
    client.println("</table>");

    */

    client.println("</html>"); 
    delay(1);
    //Serial.println("Client disconnected");
    //Serial.println(""); 


    // Reset watchdog counter
    watchdogCount = 0; 

    readAndLog.attach(READANDLOG_PERIOD_S, readAndLogFunction);

  }
  
}


void displaySensorDetails(void) {
  //Allocating
  sensor_t accel, mag, gyr;
  accelmag.getSensor(&accel, &mag);
  gyro.getSensor(&gyr);

  //Display
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(accel.name);
  Serial.print("Driver Ver:   ");
  Serial.println(accel.version);
  Serial.print("Unique ID:    0x");
  Serial.println(accel.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(accel.max_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(accel.min_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(accel.resolution, 8);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(mag.name);
  Serial.print("Driver Ver:   ");
  Serial.println(mag.version);
  Serial.print("Unique ID:    0x");
  Serial.println(mag.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(mag.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(mag.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(mag.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("Gyroscope");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(gyr.name);
  Serial.print("Driver Ver:   ");
  Serial.println(gyr.version);
  Serial.print("Unique ID:    0x");
  Serial.println(gyr.sensor_id, HEX);
  Serial.print("Max Value:    ");
  Serial.print(gyr.max_value);
  Serial.println(" rad/s");
  Serial.print("Min Value:    ");
  Serial.print(gyr.min_value);
  Serial.println(" rad/s");
  Serial.print("Resolution:   ");
  Serial.print(gyr.resolution);
  Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


void esp_output() 
{
  Serial.println();

  Serial.printf("SDK version %s\n", ESP.getSdkVersion());
#ifndef ESP32
  Serial.printf("Arduino Core version %s\n", ESP.getCoreVersion().c_str());
  Serial.printf("VCC %d\n", ESP.getVcc());
  Serial.printf("Chip ID %d\n", ESP.getChipId());
  Serial.printf("Flash Chip ID %d\n", ESP.getFlashChipId());
#endif
  Serial.printf("Flash Chip Speed %d\n", ESP.getFlashChipSpeed());
#ifndef ESP32
  Serial.printf("Sketch Size %d\n", ESP.getSketchSize());
  Serial.printf("Free heap %d\n", ESP.getFreeHeap());
  Serial.printf("Free sketch space %d\n", ESP.getFreeSketchSpace());
  Serial.printf("Reset reason %s\n", ESP.getResetReason().c_str());
#endif

  Serial.printf("Maximum TCP connections %d\n", MEMP_NUM_TCP_PCB);

  Serial.println();
  Serial.println();
}



void readAndLogFunction(void)
{

    // --- Toggle proto shield led for activity ---
  if (toggleLed)
  {
    toggleLed = 0;
    digitalWrite(PROTO_SHIELD_LED, LOW);
  }
  else
  {
    toggleLed = 1;
    digitalWrite(PROTO_SHIELD_LED, HIGH);
  }


  if (messageCounter > 2147483646) //avoid overflow when doing long aquisitions
  {
    messageCounter = 0; // reset the counter
  }
  else
  {
    messageCounter++; 
  }


  #ifdef USE_NTP
    //-- NTP

   // Current time is calculated based on previous NTP time and time passed since last update
    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime(); 
    String formattedTime = timeClient.getFormattedTime();
    //Get a time structure
    struct tm *ptm = gmtime ((time_t *)&epochTime); 
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon+1;
    int currentYear = ptm->tm_year+1900;
    //Print complete date:
    String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay) + "--" + formattedTime;
    Serial.print(currentDate);
    Serial.print(",");
  #endif

  // IMU
  sensors_event_t aevent, mevent, gevent;
  accelmag.getEvent(&aevent, &mevent);
  gyro.getEvent(&gevent);
  

  // Display all data in 1 line (CSV)
  Serial.print(messageCounter);
  Serial.print(",");
  Serial.print(watchdogCount);
  Serial.print("/");
  Serial.print(WDG_TIMEOUT_S);
  Serial.print(",");  
  /*Serial.print(ina219.getShuntVoltage_mV());
  Serial.print(",");
  Serial.print(ina219.getBusVoltage_V());
  Serial.print(",");
  Serial.print(ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000));
  Serial.print(",");
  Serial.print(ina219.getCurrent_mA());
  Serial.print(",");
  Serial.print(ina219.getPower_mW());
  Serial.print(","); */
  // Display the accel results (acceleration is measured in m/s^2)
  Serial.print(aevent.acceleration.x, 4);
  Serial.print(",");
  Serial.print(aevent.acceleration.y, 4);
  Serial.print(",");
  Serial.print(aevent.acceleration.z, 4);
  Serial.print(",");
  // Display the mag results (mag data is in uTesla)
  Serial.print(mevent.magnetic.x, 1);
  Serial.print(",");
  Serial.print(mevent.magnetic.y, 1);
  Serial.print(",");
  Serial.print(mevent.magnetic.z, 1);
  Serial.print(",");
  // Display the gyr results (gyr data is in rad/s)
  Serial.print(gevent.gyro.x);
  Serial.print(",");
  Serial.print(gevent.gyro.y);
  Serial.print(",");
  Serial.print(gevent.gyro.z);
  Serial.println("");

  // Reset watchdog counter
  watchdogCount = 0;
  
}

void ISRwebserver (void)
{

  //  Get battery voltage and current for HTML page
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();

  // Webserver for displaying information
  displayWebServerPage(busvoltage, current_mA);

}








// END OF FILE //