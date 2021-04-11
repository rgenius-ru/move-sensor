/*  Accesspoint - station communication without router
 *  see: https://github.com/esp8266/Arduino/blob/master/doc/esp8266wifi/station-class.rst
 *  Works with: accesspoint_bare_01.ino
 */

/* Connection table

FTDI    ESP8266, ESP-01
----    ---------------
3V3     3V3 (VCC)
3V3     EN (CH_PD)
TX      RX 
RX      TX 
GND     IO0 (GPIO_0)
GND     GND

GPIO_0 is Grounded to enable the programming mode of ESP8266.
*/

#include <ESP8266WiFi.h>
#include <Wire.h>

// ****************** MPU6050 **********************
/*
Accelerometer readings from Adafruit MPU6050

ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu(0, 2); //sda, scl pins


//byte ledPin = 2;
char ssid[] = "Game-Event";           // SSID of your AP
char pass[] = "Event-Game";         // password of your AP

IPAddress server(192,168,4,15);     // IP address of the AP
WiFiClient client;

const String sensor_side = "r"; // "l" or "r"

const uint8_t amplitude_gain = 25;
const uint8_t amplitude_min = 0;
const uint8_t amplitude_max = 255;
// ****************************************************


// ****************** Neopixel **********************
#include <Adafruit_NeoPixel.h>

#define PIN       3 //3 - Rx pin // Which pin on the Arduino is connected to the NeoPixels?
#define NUMPIXELS 1  // How many NeoPixels are attached to the Arduino?

/*
When setting up the NeoPixel library, we tell it how many pixels,
and which pin to use to send signals. Note that for older NeoPixel
strips you might need to change the third parameter -- see the
strandtest example for more information on possible values.
*/
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
// ****************************************************


float absolute(float x){
  if (x < 0){
    return -x;
  }
  
  return x;
}

int get_amplitude(float gyro_x, float gyro_y, float gyro_z){
  int amplitude = int(absolute(gyro_x) + absolute(gyro_y) + absolute(gyro_z));
  amplitude *= amplitude_gain;
  amplitude = constrain(amplitude, amplitude_min, amplitude_max);

  return amplitude;
}


void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);           // connects to the WiFi AP

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'

  Serial.println();
  Serial.println("Connection to the AP");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");

    pixels.setPixelColor(0, pixels.Color(35, 35, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay(250);

    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay(250);
  }
  Serial.println();
  Serial.println("Connected");
  Serial.println("move-sensor - main.cpp");
  Serial.print("LocalIP:"); Serial.println(WiFi.localIP());
  Serial.println("MAC:" + WiFi.macAddress());
  Serial.print("Gateway:"); Serial.println(WiFi.gatewayIP());
  Serial.print("AP MAC:"); Serial.println(WiFi.BSSIDstr());
  
  //pinMode(ledPin, OUTPUT);

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      pixels.setPixelColor(0, pixels.Color(35, 0, 0));
      pixels.show();   // Send the updated pixel colors to the hardware.
      delay(300);

      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();   // Send the updated pixel colors to the hardware.
      delay(700);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  if (client.connect(server, 80)){
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    String value_str = String(get_amplitude(g.gyro.x, g.gyro.y, g.gyro.z));
    value_str = sensor_side + value_str + "\r";

    int size = client.print(value_str);
    Serial.println(value_str); //Byte sent to the AP
    //Serial.println(size); //Byte sent to the AP

    //String answer = client.readStringUntil('\r');
    //Serial.println("From the AP: " + answer);
    
    //digitalWrite(ledPin, LOW);

    pixels.setPixelColor(0, pixels.Color(0, 35, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.

    client.flush();
    //digitalWrite(ledPin, HIGH);
    
    client.stop();
  }
  else{
    Serial.println("Can't connect to the base station..");
    
    pixels.setPixelColor(0, pixels.Color(35, 0, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
  }

  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.
  
  delay(200);
}

