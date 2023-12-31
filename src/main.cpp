#include <Arduino.h>
#include <esp_adc_cal.h>
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"
/// --------- /////
#include "Adafruit_SGP30.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "DHT.h"
#include "MQ135.h"
#include <Arduino.h>
#include "SharpGP2Y10.h"
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WiFiManager.h> 
#include <DNSServer.h>

#define buzzer 13

// set nguong
int thresholdTOV = 600;
int thresholdCO2 = 1200;
int thresholdMQ135 = 70;
int thresholdDust = 10;
// define sensor
#define DHT_Pin 26
#define DHT_Type DHT11
// dust sensor
#define voPin 36
#define ledPin 34
// ky040
const int CLK_PIN = 14;  // KY-040 CLK pin
const int DT_PIN = 27;   // KY-040 DT pin
const int SW_PIN = 16;   // KY-040 SW pin
volatile bool aState;
volatile bool bState;
volatile bool buttonState;
volatile bool lastButtonState = HIGH;
volatile long encoderValue = 0;
int selectedThreshSensor = 0;
// MQ135
#define MQ135_pin 39
double R0;
// oled
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET 4         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // 0x3D for 128x64, 0x3C for 128x32
// ssid and passwork
const char* host = "esp32";

WebServer server(80);
/*
 * Login page
 */
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

/*
 * setup function
 */
// firebase
#define FIREBASE_HOST "https://air-minotoring-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "AIzaSyDSnijCeDSsrXzgTRYTi70El68oJb_0p-Q"
//  call obj
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht(DHT_Pin, DHT_Type);
Adafruit_SGP30 sgp;
//MQ135 mq135 = MQ135(MQ135_pin);
SharpGP2Y10 dustSensor(voPin, ledPin);
/*firebase*/
FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;
unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

// put function declarations here:
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));  // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                 // [mg/m^3]
  return absoluteHumidityScaled;
}
// Calibration function
void caliberate(int mq135_apin, double& R0) {
  int m = 0;
  double sensor_volt, RS_air;
  double sensor_value = 0.0;

  for (int x = 0; x < 5000; ++x) {
    int analog_read = analogRead(mq135_apin);
    if (m != 500) {
      if (analog_read != 0 && analog_read <= 1023) {
        sensor_value += analog_read;
        ++m;
      } else if (analog_read > 1023) {
        analog_read = 1023;
        sensor_value += analog_read;
        ++m;
      }
    } else if (m == 500) {
      sensor_value /= 500.0;
      sensor_volt = sensor_value * (5.0 / 1023.0);
      RS_air = ((5.0 * 20) / sensor_volt) - 10.0;

      // Set appropriate ratio_air for each sensor based on the desired gas
      double ratio_air = 3.7;  // Assuming MQ135 for ammonia (NH3)

      R0 = RS_air / ratio_air;
      delay(500);
      break;
    }
  }

  Serial.println("Calibration Done");
}
// Function to calculate NH3 (ammonia) concentration
double mq135_Calc(int mq135_apin, double R0) {
  int loadRes = 20;  // Assuming loadRes for MQ135 only
  double m = -0.318;
  double b = 1.13;
  int sensorVal = analogRead(mq135_apin);

  if (sensorVal != 0 && sensorVal <= 1023) {
    double sensor_volt = sensorVal * (5.0 / 1023.0);
    double RS_gas = ((5.0 * loadRes) / sensor_volt) - 10;
    double ratio = RS_gas / R0;

    if (ratio > 0) {
      double ppm_log = (log10(ratio) - b) / m;
      double ppm = pow(10, ppm_log);
      double percentage = ppm / 10000;
      return ppm;
    }
  } else if (sensorVal > 1023) {
    sensorVal = 1023;
    double sensor_volt = sensorVal * (5.0 / 1023.0);
    double RS_gas = ((5.0 * loadRes) / sensor_volt) - 10;
    double ratio = RS_gas / R0;

    if (ratio > 0) {
      double ppm_log = (log10(ratio) - b) / m;
      double ppm = pow(10, ppm_log);
      double percentage = ppm / 10000;
      return ppm;
    }
  }

  return 0.0;  // If sensorVal == 0 or other conditions not met, return 0.0
}

void SendSensorData() {
#pragma region GetDataSensor
  /*DHT11*/
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read DHT");
    return;
  }
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.println(t);
  /* SGP30 */
  sgp.setHumidity(getAbsoluteHumidity(t, h));
  if (!sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  Serial.print("TVOC ");
  Serial.print(sgp.TVOC);
  Serial.print(" ppb\t");
  Serial.print("eCO2 ");
  Serial.print(sgp.eCO2);
  Serial.println(" ppm");
  /* MQ135 */
  float nh3_ppm = mq135_Calc(MQ135_pin, R0);

  // Display the result on the Serial Monitor
  Serial.print("NH3 Concentration (ppm): ");
  Serial.println(nh3_ppm);
  /* Dust Sensor */
  float dustDensity = dustSensor.getDustDensity();
  Serial.println(dustDensity);
#pragma endregion
/* Display oled */
#pragma region Display Oled
  /*DHT11*/
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temperature: ");
  display.print(t);
  display.print(" ");
  display.print("C");
  display.setCursor(0, 10);
  display.print("Humidity: ");
  display.setCursor(62, 10);
  display.print(h);
  display.print(" %");
  /*SGP30*/
  display.setCursor(0, 20);
  display.setTextColor(SH110X_WHITE);
  display.print("TVOC:");
  display.print(sgp.TVOC);
  display.setTextSize(0.5);
  display.print(" ppb");
  display.setCursor(0, 30);  //oled display
  display.setTextColor(SH110X_WHITE);
  display.print("CO2:");
  display.print(sgp.eCO2);
  display.setTextSize(0.5);
  display.print(" ppm");
  /*MQ135*/
  display.setCursor(0, 40);  //oled display
  display.setTextColor(SH110X_WHITE);
  display.print("NH3:");
  display.print(nh3_ppm);
  display.setTextSize(0.5);
  display.print(" ppm");
  
  /*Dust Sensor*/
  display.setCursor(0, 50);  //oled display
  display.setTextColor(SH110X_WHITE);
  display.print("Dust: ");
  display.print(dustDensity);
  display.setTextSize(0.5);
  display.print(" ppm");
  display.display();
  
#pragma endregion
/* Send data to firebase */
#pragma region SendDataToFirebase
  /*DHT11*/
  if (Firebase.setFloat(fbdo, "/DHT11/temp", t)) {
    Serial.println("Upload success");
  } else {
    Serial.println("Upload fail");
  }
  if (Firebase.setFloat(fbdo, "/DHT11/hum", h)) {
    Serial.println("Upload success");
  } else {
    Serial.println("Upload fail");
  }
  /* SGP30 */
  if (Firebase.setFloat(fbdo, "/SGP30_SenSor/Data/TVOC", sgp.TVOC))
    Serial.println("Upload success");
  else
    Serial.println("Upload fail");
  if (Firebase.setFloat(fbdo, "/SGP30_SenSor/Data/eCO2", sgp.eCO2))
    Serial.println("Upload success");
  else
    Serial.println("Upload fail");

  /* MQ135 */
  if (Firebase.setFloat(fbdo, "/MQ135/Data", nh3_ppm))
    Serial.println("Upload success");
  else
    Serial.println("Upload fail");

  if (Firebase.setFloat(fbdo, "/DustSenSor/Data", dustDensity))
    Serial.println("Upload success");
  else
    Serial.println("Upload fail");
  /*set nguong*/
  if (Firebase.setInt(fbdo, "/SGP30_SenSor/Thresh_Hold/TVOC", thresholdTOV)) {
    Serial.println("TVOC Threshold updated successfully");
  } else {
    Serial.println("Failed to update TVOC Threshold");
  }

  if (Firebase.setInt(fbdo, "/SGP30_SenSor/Thresh_Hold/eCO2", thresholdCO2)) {
    Serial.println("eCO2 Threshold updated successfully");
  } else {
    Serial.println("Failed to update eCO2 Threshold");
  }

  if (Firebase.setInt(fbdo, "/MQ135/Thresh_Hold", thresholdMQ135)) {
    Serial.println("MQ135 Threshold updated successfully");
  } else {
    Serial.println("Failed to update MQ135 Threshold");
  }

  if (Firebase.setInt(fbdo, "/DustSenSor/Thresh_Hold", thresholdDust)) {
    Serial.println("Dust Threshold updated successfully");
  } else {
    Serial.println("Failed to update Dust Threshold");
  }

#pragma endregion
#pragma region canh bao
if(nh3_ppm > thresholdMQ135){
  digitalWrite(buzzer,1);
  delay(5000);
  digitalWrite(buzzer,0);
}
if(sgp.TVOC > thresholdTOV){
  digitalWrite(buzzer,1);
  delay(5000);
  digitalWrite(buzzer,0);
}
if(sgp.eCO2 > thresholdCO2){
  digitalWrite(buzzer,1);
  delay(5000);
  digitalWrite(buzzer,0);
}
if(dustDensity > thresholdDust){
  digitalWrite(buzzer,1);
  delay(5000);
  digitalWrite(buzzer,0);
}
#pragma endregion
}
void handleVariableChange(int& variable) {
  noInterrupts();
  long val = encoderValue;
  interrupts();

  if (val != 0) {
    variable += (val > 0) ? 1 : -1;
    noInterrupts();
    encoderValue = 0;
    interrupts();
  }

  if (buttonState != lastButtonState) {
    if (!buttonState) {
      selectedThreshSensor++;
      if (selectedThreshSensor >= 4) {
        selectedThreshSensor = 0;
      }
      Serial.print("Selected Sensor: ");
      Serial.println(selectedThreshSensor + 1);
    }
    lastButtonState = buttonState;
  }
}

void handleEncoder() {
  aState = digitalRead(CLK_PIN);
  bState = digitalRead(DT_PIN);

  if (aState == bState) {
    encoderValue++;
  } else {
    encoderValue--;
  }
}
void handleButton() {
  buttonState = digitalRead(SW_PIN);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFiManager wifiManager;
  wifiManager.autoConnect("DoAn2","password");;
  Serial.println("Connected.");
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(buzzer,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_PIN), handleButton, CHANGE);
  
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
  config.api_key = FIREBASE_AUTH;
  config.database_url = FIREBASE_HOST;
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Firebase.setReadTimeout(fbdo, 1000 * 60);
  Firebase.setwriteSizeLimit(fbdo, "tiny");
  dht.begin();
  Wire.begin();
  display.begin(SCREEN_ADDRESS, true);
  display.display();
  if (!sgp.begin()) {
    Serial.println("Sensor not found :(");
    Firebase.setString(fbdo, "/SGP30_SenSor", "Sensor not found :(");
    display.setCursor(0, 5);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.println("Sensor not found :(");
    display.display();
  }
  display.clearDisplay();
  caliberate(MQ135_pin, R0);
}

void loop() {
  if (selectedThreshSensor == 0) {
    handleVariableChange(thresholdTOV);
  }
  if (selectedThreshSensor == 1) {
    handleVariableChange(thresholdCO2);
  }
  if (selectedThreshSensor == 2) {
    handleVariableChange(thresholdMQ135);
  }
  if (selectedThreshSensor == 3) {
    handleVariableChange(thresholdDust);
  }
  // put your main code here, to run repeatedly:
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 4000 || sendDataPrevMillis == 0)) {

    SendSensorData();
    sendDataPrevMillis = millis();
  }
  server.handleClient();
  delay(1);
}