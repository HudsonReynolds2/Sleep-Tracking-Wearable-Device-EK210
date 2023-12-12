// This is arduino code. It is running on an esp32c3.
#include <Vector.h>

//HEART RATE CODE
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// Modem Control
#include "esp_bt.h"

// Flash memory read/write
#include <EEPROM.h>
// define the number of bytes you want to access
#define EEPROM_SIZE 5

//WIFI CODE
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#define READ_INTERVAL 20000  // Sets read interval for 10 seconds

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

const char *ssid = "SleepRateTrackingWatch";  // Set these to your desired credentials.
const char *password = "pineapple";

WiFiServer server(80);


MAX30105 particleSensor;  // MORE HEART RATE CODE

const byte RATE_SIZE = 7; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// BUTTON AND LED CODE
const int greenPin = 4;
const int redPin = 5;//was 3
const int bluePin = 3; // was 4
const int buttonPin = 2;
unsigned long buttonPressStartTime = 0;
static bool ledOn = true;
static bool wasChanged = false;  // Can remove this with only an efficiency penalty (the last switch statement)
static bool isReadyWifi = false;

// Recording:
static bool isReading = false;
static long readIntervalStart = 0;
static int blipCount = 0;
static int blipValue = 0;
static int minutesRecorded = 0;
static int runningSum = 0;

static double printScore;

static bool readyToChill = false;

enum ButtonState {
  IDLE,
  PRESS_DETECTED,
  HELD
};

enum deviceMode {
  RECORD,
  TRANSFER,
  CHILL
};

static ButtonState currentButtonState = IDLE;
static deviceMode currentMode = CHILL;  // Start out not recording

// Temp vector code
static int storage_array[250];
static Vector<int> vector(storage_array);

void setColor(int redValue, int greenValue, int blueValue){
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

double scoreCalculator( ){
  double minutesRecorded = EEPROM.read(0);  // This value is basically N in the RMSSD equation (it's number of deltas not number of beats)
  double sleepScore = EEPROM.read(1);  // Get runningSum

  if( minutesRecorded != 0 )
  {
    sleepScore /= minutesRecorded;
    if( sleepScore > 0 )
    {
      sleepScore = sqrt( sleepScore );
    }
    else
    {
      return 0;
    }
  }
  else{
    return 0;
  }
  return sleepScore;  // This is now RMSSD
}

void myWifiWakeup(){  // This function is problematic. It never leaves the function.
  //esp_wifi_bt_power_domain_on();
  esp_wifi_start();
  // You can remove the password parameter if you want the AP to be open.
  // a valid password must have more than 7 characters
  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while(1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("Server started");
  isReadyWifi = true;
}

void myWifiShutdown(){
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  //esp_wifi_bt_power_domain_off();
  Serial.println("WiFi off");
  isReadyWifi = false;
}

void modeHandler( deviceMode passedMode )  // Handles the switching of the modes
{
  switch( passedMode ) {
    case RECORD:
      // If the device is now in recording mode, wake up the sensor
      myWifiShutdown();
      break; 
    case TRANSFER:  // Transfer
      particleSensor.shutDown();
      break;
    case CHILL:  // Chill
      myWifiShutdown();
      particleSensor.shutDown();
      break;
    default:
      myWifiShutdown();
      particleSensor.shutDown();
      break;
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor
  esp_wifi_bt_power_domain_on();
  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);  // 6 minutes, 45 seconds WHAT IF THE DEVICE IS ASLEEP WHEN THE USER WAKES UP

  // Flash memory alteration
  EEPROM.begin(EEPROM_SIZE);

  

  wasChanged = false;  // Can remove this with only an efficiency penalty (the last switch statement)
  
  isReading = false;
  readIntervalStart = 0;
  blipCount = 0;
  blipValue = 0;

  // HEART RATE, LED, AND BUTTON CODE:
  Wire.begin(7, 6, 100000);  // Set the ESP32's I2C SDA/SCL pins
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 
  pinMode(buttonPin, INPUT_PULLUP); // Using internal pull-up resistor
  
  if( minutesRecorded == 0 )
  {
    runningSum = 0;
    ledOn = true;
  }
  else{
    //currentMode = EEPROM.read(2);
    //ledOn = EEPROM.read(3);
  }

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    Serial.flush();
    while (1);
  }
  // Sensor lights
  particleSensor.setup();  //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  //Turn off Green LED

  modeHandler( currentMode );
}



void loop(){
  // BUTTON & LED CODE, MODE SWAPPER
  //buttonHandler();  // Move button code out of loop 
  int buttonState = digitalRead(buttonPin);

  if( currentButtonState == IDLE ){
    if (buttonState == LOW) {
      // Button is pressed
      buttonPressStartTime = millis();
      currentButtonState = PRESS_DETECTED;
    }
  }

  if( currentButtonState == PRESS_DETECTED ){
    if (buttonState == HIGH) {
      // Button was released
      if (millis() - buttonPressStartTime < 1000){
        // Button TAPPED for less than 1 second: TOGGLE LED
        Serial.println("Tapped for less than 1 second");
        if( ledOn ){
           ledOn = false; // Turn off LED
           setColor( 0, 0, 0 );   
        }
        else{
          ledOn = true;  // Turn on LED
          wasChanged = true;
        }
        if( currentMode == TRANSFER )
        {
          // How to exit transfer with the button? The button hold seems to expire after second
          if( readyToChill == true )
          {
            currentMode = CHILL;
            modeHandler( currentMode );
            readyToChill = false;
          }
        }
      } 
      else if( (millis() - buttonPressStartTime < 3000) && (millis() - buttonPressStartTime > 1000)){
        // BUTTON HELD for between 1 and 3 seconds
        wasChanged = true;
        
        if( currentMode == CHILL ) // TOGGLE RECORD:
        {
          Serial.println("Changing to record mode from chill mode");
          currentMode = RECORD;
        }
        else if( currentMode == RECORD ){
          Serial.println("Changing to chill mode from record mode");
          currentMode = CHILL;
        }
        modeHandler( currentMode );
      }
      else {  // TOGGLE TRANSFER
        // Button held for 3 seconds or more
        if( currentMode == TRANSFER ){
          currentMode = CHILL;
          Serial.println("Changing to chill mode from transfer mode");
        }
        else{
          currentMode = TRANSFER;
          Serial.println("Transfer mode activated");
        }
        wasChanged = true;
        modeHandler( currentMode );
      }
      currentButtonState = IDLE;  // Reset button
      if( wasChanged ){
        wasChanged = false;
        if( ledOn ){
          switch ( currentMode ){
            case RECORD:
              setColor( 255, 0, 0 );  // Red for recording
              break;
            case TRANSFER:
              setColor( 0, 0, 255 );  // Blue for transfering
              break;
            case CHILL:
              setColor( 0, 255, 0 );  // Green for chillin
              break;
            }   // Can add more colors/modes
        }  
      }
    }
  }
  delay(5);  // Debouncing delay
  
  if( currentMode == TRANSFER )  // having trouble exiting transfer mode
  {
    if( isReadyWifi == false )
    {
      //myWifiWakeup();
      //esp_wifi_bt_power_domain_on();
      esp_wifi_start();
      // You can remove the password parameter if you want the AP to be open.
      // a valid password must have more than 7 characters
      if (!WiFi.softAP(ssid, password)) {
        log_e("Soft AP creation failed.");
        while(1);
      }
      IPAddress myIP = WiFi.softAPIP();
      Serial.print("AP IP address: ");
      Serial.println(myIP);
      server.begin();
      Serial.println("Server started");
      isReadyWifi = true;
      printScore = scoreCalculator();
      Serial.print("The score of the last night of sleep is: ");
      Serial.println(printScore);
    }
    
    WiFiClient client = server.available();   // listen for incoming clients
    if (client) {                             // if you get a client,
      Serial.println("New Client.");           // print a message out the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          if (c == '\n') {                    // if the byte is a newline character

            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();

              // the content of the HTTP response follows the header:
              client.print("Click <a href=\"/H\">here</a> to turn ON the LED.<br>");
              client.print("Click <a href=\"/L\">here</a> to turn OFF the LED.<br>");
              client.print("Click <a href=\"/R\">here</a> to shut down the WiFi network.<br>");
              client.print("Click <a href=\"/P\">here</a> to clear all of the sleep data from the flash.<br>");
              
              printScore = scoreCalculator();
              client.print("Your sleep score from last night is: ");
              if( printScore < 100 ){
                client.print(printScore);
              }
              else
              {
                client.print("100");
              }
              client.print("/100.");
              client.println();
              client.println();  // The HTTP response ends with another blank line
              // break out of the while loop:
              break;
            } else {    // if you got a newline, then clear currentLine:
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }

          // Check to see if the client request was "GET /H" or "GET /L":
          if (currentLine.endsWith("GET /H")) {
            ledOn = true;
            setColor( 0, 0, 255 );  // GET /H turns the LED on
          }
          if (currentLine.endsWith("GET /L")) {
            setColor(0,0,0);  // GET /L turns the LED off
          }
          if (currentLine.endsWith("GET /R")) {  // Honoroary comment: I spent so long debugging why this would not work, all just cuz I had a 0 instead of an O.UGH. Changed to R to avoid confusion.
            readyToChill = true;
            modeHandler( CHILL );
            Serial.flush();
            setColor(0,0,0);
          }
          if (currentLine.endsWith("GET /P")) {
            EEPROM.write( 0, 0 );  // Clears the sleep data from the device
            EEPROM.write( 1, 0 );
          }
        }
      }
      client.stop();  // close the connection
      Serial.println("Client Disconnected.");
    }
  }
  else if( currentMode == RECORD )
  {
    if( !isReading )
    {
      particleSensor.wakeUp();  // Wake up the sensor from standby
      readIntervalStart = millis();
      isReading = true;
      long irValue = particleSensor.getIR();
    }
    else  // HEART RATE CODE:
    {
      long irValue = particleSensor.getIR();
      if (checkForBeat(irValue) == true)
      {
        long delta;
        if( vector.size() < 2 )
        {
          lastBeat = millis();
          delta = 0;
        }
        else{
          delta = millis() - lastBeat;  // We sensed a beat!
        }
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        Serial.print("DELTA=");
        Serial.println(delta);

        vector.push_back(delta);  // Save the time between the this heart beat and the last one in the RAM

        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute;  // Store this reading in the temporary array
          rateSpot %= RATE_SIZE;  // Wrap variable

          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)  //Take average of readings
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
        Serial.print(", Avg BPM=");
        Serial.println(beatAvg);
      }

      if (irValue < 50000){
        Serial.print(" No finger?");
        Serial.println();
      }

      long recordingProgress = millis() - readIntervalStart;
      if( recordingProgress > READ_INTERVAL )  // Change this to stop when mode is changed FIX THIS
      {
        isReading = false;
        particleSensor.shutDown();  // Put the sensor to sleep
         //  Do RMSSD EQUATION HERE FIX THIS

        static int ctr = 0;
        blipValue = 0;
        for( int i = 0; i < vector.size(); i++ )
        {
          if( vector[i] != 0 )
          {
            blipValue += vector[i];
            ctr++;
          }
          blipValue += vector[i];
        }
        blipValue /= ctr;  // Now blipValue is the average of the values in vector
        vector.clear();

        if( minutesRecorded == 0 )  // End of the first reading interval
        {
          runningSum = 0;
          minutesRecorded = 1;
        }
        else
        {
          runningSum = EEPROM.read(1);
          minutesRecorded = EEPROM.read(0);
          minutesRecorded++;
        }
        runningSum += (blipValue * blipValue);  // Square for RMSSD calculation
        
        EEPROM.write( 0, minutesRecorded );
        EEPROM.write( 1, runningSum );
        EEPROM.commit();
        delay(5);
        
        blipValue = 0;
        Serial.println("Done reading for this interval.");
        Serial.flush();
        
        //esp_light_sleep_start();  // Go to sleep for the next interval to complete one cycle
      }
    }
  }
  else  // if( currentMode == CHILL )
  {

  }    
}

