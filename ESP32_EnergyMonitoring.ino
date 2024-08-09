#include "UbidotsEsp32Mqtt.h"
#include <PZEM004Tv30.h>
#include <WiFiManager.h>
#include <Arduino.h>
TaskHandle_t monitoringTask;
TaskHandle_t safetyTask;

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define PZEM_SERIAL Serial2
#define TRIGGER_PIN 34

PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);

WiFiManager wm;
WiFiManagerParameter control;
bool wm_nonblocking = false;

static bool buttonPressed = false;
static unsigned long buttonPressTime = 0;

#define From_comparator 35
#define Relay 14
#define Reset_pin 39
#define MOSFET_pin 32
#define LED_FaultDetected 23
#define LED_RelayStatus 21
#define LED_portalPin 19
#define LED_wifiPin 18
String RelayState = "ON";

int relay_status = 0;
int trip_flag = 0;
int testVal = 0;

bool resetEnergy = false;
bool resetEverything = false;

int loadPin = 25;

bool flaggedOn = false;
bool flaggedOff = false;

long int messageArrived = 0;
long int relaytoggledon = 0;
long int relaytoggledoff = 0;

long int timeon = 0;
long int timeoff = 0;

/****************************************
 * Define Constants
 ****************************************/
const char *APSSID = "SmartSocket";
const char *APPassword = "password";

const char *UBIDOTS_TOKEN = "BBFF-Uc8wXlPXbYKL95FHN7NETvhPYGegPY";  // Put here your Ubidots TOKEN

const char *PUBLISH_DEVICE_LABEL = "monitoring";       // Put here your Device label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL1 = "voltage";       // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL2 = "current";       // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL3 = "power";         // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL4 = "energy";        // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL5 = "frequency";     // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL6 = "power-factor";  // Put here your Variable label to which data  will be published

const char *PUBLISH_VARIABLE_LABEL7 = "relay_status";  // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL8 = "trip-flag";     // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL9 = "publishLatency";

const char *SUBSCRIBE_DEVICE_LABEL = "monitoring";
const char *SUBSCRIBE_VARIABLE_LABEL = "relay";  // Replace with the variable label to subscribe to

const int PUBLISH_FREQUENCY = 5000;  // Update rate in millisecondsx

unsigned long timer;

Ubidots ubidots(UBIDOTS_TOKEN);

void bindServerCallback() {
  wm.server->on("/custom", handleRoute);
  wm.server->on("/save", handleSave);
  wm.server->on("/erase", HTTP_POST, handleErase);  // Add the erase route
}

String getParam(String name) {
  // Read parameter from server, for custom HTML input
  String value;
  if (wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}

void handleRoute() {
  //relayState();
  if (trip_flag == 0) {
    unsigned long lastUpdate = 0;  // Variable to track the last update time
    String html = "<!DOCTYPE html><html>";
    html += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<link rel=\"icon\" href=\"data:,\">";
    html += "<meta http-equiv=\"refresh\" content=\"2\">";  // Update page every 2 seconds

    // CSS to style the toggle switch
    html += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}";
    html += ".switch { position: relative; display: inline-block; width: 60px; height: 34px;}";
    html += ".switch input { display: none; }";
    html += ".slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 34px; }";
    html += ".slider:before { position: absolute; content: ''; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; -webkit-transition: .4s; transition: .4s; border-radius: 50%; }";
    html += "input:checked + .slider { background-color: #4CAF50; }";
    html += "input:focus + .slider { box-shadow: 0 0 1px #4CAF50; }";
    html += "input:checked + .slider:before { -webkit-transform: translateX(26px); -ms-transform: translateX(26px); transform: translateX(26px); }";
    html += ".monitoring { text-align: left; margin: 0 auto; max-width: 500px; }";
    html += ".monitoring h2 { font-weight: bold; color: white; background-color: #000000; padding: 10px; border-radius: 5px; }";
    html += ".monitoring p { background-color: #f2f2f2; padding: 5px; margin: 5px; border-radius: 5px; font-weight: bold; color: #000000; }";
    html += ".erase-button { display: inline-block; background-color: #ff0000; color: white; padding: 10px; margin-top: 10px; text-decoration: none; border-radius: 5px; }";
    html += "</style></head>";

    // Web Page Heading
    html += "<body style=\"background-color: #000000;\"><h1 style=\"color: white;\">SMART SOCKET</h1>";

    // Display current state and toggle switch for GPIO 12
    html += "<h2 style=\"color: white;\"><b>SWITCH " + RelayState + "</b></h2>";
    html += "<label class=\"switch\">";
    html += "<input type=\"checkbox\" id=\"toggle\" name=\"toggle\" onchange=\"saveInput ()\"";
    if (RelayState == "ON") {
      html += " checked=\"checked\"";
    }
    html += ">";
    html += "<span class=\"slider\"></span>";
    html += "</label>";

    // JavaScript to handle saving the input
    html += "<script>";
    html += "function saveInput() {";
    html += "  var toggle = document.getElementById('toggle');";
    html += "  var status = toggle.checked ? 'ON' : 'OFF';";
    html += "  var xhttp = new XMLHttpRequest();";
    html += "  xhttp.open('POST', '/save', true);";
    html += "  xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');";
    html += "  xhttp.send('toggle=' + status);";
    html += "}";
    html += "</script>";

    // Monitoring data section
    html += "<div style=\"text-align: center; margin: 0 auto; max-width: 500px; background-color: #000000; padding: 2px;\">";
    html += "<h2 style=\"color: white;\"><b>MONITORING</b></h2>";

    unsigned long now = millis();
    if (now - lastUpdate >= 2000) {  // Refresh every 2 seconds
      lastUpdate = now;
      float voltage = pzem.voltage();
      float current = pzem.current();
      float power = pzem.power();
      float energy = pzem.energy();
      float frequency = pzem.frequency();
      float pf = pzem.pf();

      html += "<p style=\"background-color: #000000; color: #ffffff; padding: 2px; text-align: center;\">Voltage<br><span style=\"display: block; font-size: 20px; font-weight: bold; margin: 0 auto;\">" + String(voltage, 2) + " V</span></p>";
      html += "<p style=\"background-color: #000000; color: #ffffff; padding: 2px; text-align: center;\">Current<br><span style=\"display: block; font-size: 20px; font-weight: bold; margin: 0 auto;\">" + String(current, 2) + " A</span></p>";
      html += "<p style=\"background-color: #000000; color: #ffffff; padding: 2px; text-align: center;\">Power<br><span style=\"display: block; font-size: 20px; font-weight: bold; margin: 0 auto;\">" + String(power, 1) + " W</span></p>";
      html += "<p style=\"background-color: #000000; color: #ffffff; padding: 2px; text-align: center;\">Energy<br><span style=\"display: block; font-size: 20px; font-weight: bold; margin: 0 auto;\">" + String(energy, 1) + " kWh</span></p>";
      html += "<p style=\"background-color: #000000; color: #ffffff; padding: 2px; text-align: center;\">Frequency<br><span style=\"display: block; font-size: 20px; font-weight: bold; margin: 0 auto;\">" + String(frequency, 1) + " Hz</span></p>";
      html += "<p style=\"background-color: #000000; color: #ffffff; padding: 2px; text-align: center;\">Power Factor<br><span style=\"display: block; font-size: 20px; font-weight: bold; margin: 0 auto;\">" + String(pf, 1) + "</span></p>";
    }

    // Erase PZEM data button
    html += "<form action=\"/erase\" method=\"post\" style=\"margin-top: 20px;\">";
    html += "<input type=\"hidden\" name=\"action\" value=\"reset_energy\">";  // Add the hidden field
    html += "<input type=\"submit\" value=\"Erase PZEM Energy\" style=\"font-size: 16px;\">";
    html += "</form>";

    html += "</div>";
    html += "</body></html>";
    wm.server->send(200, "text/html", html);
  } else {
    String html = "<!DOCTYPE html><html>";
    html += "<html>\n";
    html += "<head>\n";
    html += "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
    html += "  <link rel=\"icon\" href=\"data:,\">";
    html += "  <meta http-equiv=\"refresh\" content=\"2\">";

    html += "  <style>\n";
    html += "    html {\n";
    html += "      font-family: Helvetica;\n";
    html += "      display: inline-block;\n";
    html += "      margin: 0px auto;\n";
    html += "      text-align: center;\n";
    html += "    }\n";
    html += "    .safety-message {\n";
    html += "      color: red;\n";
    html += "      font-size: 36px;\n";
    html += "      font-weight: bold;\n";
    html += "      animation: blink 1s infinite;\n";
    html += "    }\n";
    html += "    @keyframes blink {\n";
    html += "      0% { opacity: 1; }\n";
    html += "      50% { opacity: 0; }\n";
    html += "      100% { opacity: 1; }\n";
    html += "    }\n";
    html += "  </style>\n";
    html += "</head>\n";
    html += "<body>\n";
    html += "  <h1>SMART SOCKET</h1>\n";
    html += "  <div class=\"safety-message\">GROUND FAULT DETECTED</div>\n";
    // Add the audio element
    html += "  <audio src=\"Alarm%20sound%20effect.mp3\" autoplay></audio>\n";
    // Rest of your HTML code
    html += "</body>\n";
    html += "</html>\n";

    wm.server->send(200, "text/html", html);
  }
}

void handleErase() {
  if (wm.server->method() == HTTP_POST) {
    String action = wm.server->arg("action");

    if (action == "reset_energy") {
      // Reset energy using pzem.resetEnergy()
      pzem.resetEnergy();
      ESP.restart();
      wm.server->send(200, "text/plain", "Energy reset!");
    } else {
      wm.server->send(400, "text/plain", "Invalid action");
    }
  } else {
    wm.server->send(400, "text/plain", "Bad Request");
  }
}

void handleSave() {
  pinMode(Relay, OUTPUT);
  pinMode(LED_RelayStatus, OUTPUT);
  if (wm.server->method() == HTTP_POST) {
    String toggleValue = wm.server->arg("toggle");

    // Process and save the toggle value as needed
    if (toggleValue == "ON") {
      // Switch is ON
      // Perform corresponding actions
      RelayState = "ON";
      relay_status = 1;
      digitalWrite(LED_RelayStatus, HIGH);
      Serial.println(digitalRead(LED_RelayStatus));
      digitalWrite(Relay, HIGH);
      Serial.println(digitalRead(Relay));
      Serial.println("Relay CHARGED, LED OFF");

    } else {
      // Switch is OFF
      // Perform corresponding actions
      RelayState = "OFF";
      relay_status = 0;
      digitalWrite(Relay, LOW);
      digitalWrite(LED_RelayStatus, LOW);
      Serial.println("Relay NOT CHARGED, LED ON");
      Serial.println(digitalRead(Relay));
      Serial.println(digitalRead(LED_RelayStatus));
    }

    // Print the toggle value
    Serial.println("Toggle value: " + toggleValue);

    wm.server->send(200, "text/plain", "Input saved!");
  } else {
    wm.server->send(400, "text/plain", "Bad Request");
  }
}

void WiFi_Setup() {
  WiFi.mode(WIFI_STA);  // explicitly set mode, esp defaults to STA+AP
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(3000);
  Serial.println("\nStarting");

  pinMode(TRIGGER_PIN, INPUT);
  
  pinMode(LED_portalPin, OUTPUT);
  pinMode(LED_wifiPin, OUTPUT);

  digitalWrite(LED_portalPin, LOW);  // Turn off LED
  digitalWrite(LED_wifiPin, LOW);

  if (wm_nonblocking) wm.setConfigPortalBlocking(false);

  wm.setTitle("Smart Socket");

  const char *menuhtml = "<form action='/custom' method='get'><button>Offline Control</button></form><br/>\n";
  wm.setCustomMenuHTML(menuhtml);

  wm.setSaveParamsCallback(saveParamCallback);

  std::vector<const char *> menu = { "wifi", "info", "sep", "custom", "sep", "restart", "exit" };
  wm.setMenu(menu);

  // set dark theme
  wm.setClass("invert");
  wm.setWebServerCallback(bindServerCallback);

  digitalWrite(LED_portalPin, HIGH);  // Turn on LED
  digitalWrite(LED_wifiPin, LOW);

  bool res;

  digitalWrite(LED_portalPin, HIGH);

  res = wm.autoConnect(APSSID, APPassword);  // password protected ap

  if (!res) {
    Serial.println("Failed to connect or hit timeout");
    Serial.println("Starting config portal");

    if (!wm.startConfigPortal(APSSID, APPassword)) {
      Serial.println("Failed to connect to the WiFi or hit timeout during config portal");
      delay(3000);
      ESP.restart();
    } else {
      Serial.println("Connected to the WiFi via config portal");
      digitalWrite(LED_portalPin, LOW);
      digitalWrite(LED_wifiPin, HIGH);
    }
  } else {
    Serial.println("Connected to the WiFi");
    digitalWrite(LED_portalPin, LOW);
    digitalWrite(LED_wifiPin, HIGH);
    wm.setConfigPortalTimeout(20);
  }
}

void checkButton() {
  static bool buttonPressed = false;
  int buttonState = digitalRead(TRIGGER_PIN);

  if (buttonState == HIGH && !buttonPressed) {
    buttonPressed = true;
    Serial.println("Button Pressed");

    // Open the portal immediately
    digitalWrite(LED_portalPin, HIGH);
    delay(500);
    digitalWrite(LED_portalPin, LOW);
    delay(500);
    digitalWrite(LED_portalPin, HIGH);
    delay(500);
    digitalWrite(LED_portalPin, LOW);
    delay(500);

    Serial.println("Starting config portal");
    digitalWrite(LED_portalPin, HIGH);
    digitalWrite(LED_wifiPin, LOW);

    if (!wm.startConfigPortal(APSSID, APPassword)) {
      Serial.println("failed to connect or hit timeout");
      delay(3000);
      //ESP.restart();
    } else {
      Serial.println("connected...yay :)");
      digitalWrite(LED_portalPin, LOW);
      digitalWrite(LED_wifiPin, HIGH);
    }
    return;
  }

  buttonPressed = (buttonState == HIGH);  // Update buttonPressed state
}

void saveParamCallback() {
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM customfieldid = " + getParam("customfieldid"));
}

void callback(char *topic, byte *payload, unsigned int length) {
  pinMode(Relay, OUTPUT);
  pinMode(LED_RelayStatus, OUTPUT);
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  messageArrived = micros();
  Serial.println();
  if ((char)payload[0] == '1') {
    digitalWrite(Relay, HIGH);
    digitalWrite(LED_RelayStatus, HIGH);
    RelayState = "ON";
    relay_status = 1;
    Serial.println("Switch is ON");
    relaytoggledon = micros();
    flaggedOn = true;
  } else {
    digitalWrite(Relay, LOW);
    digitalWrite(LED_RelayStatus, LOW);
    RelayState = "OFF";
    relay_status = 0;
    Serial.println("Switch is OFF");
    relaytoggledoff = micros();
    flaggedOff = true;
  }
}

void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);

  xTaskCreatePinnedToCore(monitoringTaskFunction, "Monitoring Task", 10000, NULL, 2, &monitoringTask, 1);
  xTaskCreatePinnedToCore(safetyTaskFunction, "Safety Task", 10000, NULL, 1, &safetyTask, 0);
}

void loop() {
  vTaskDelete(nullptr);
}

void monitoringTaskFunction(void *pvParameters) {
  pinMode(LED_portalPin, OUTPUT);
  pinMode(LED_wifiPin, OUTPUT);

  digitalWrite(LED_portalPin, LOW);
  digitalWrite(LED_wifiPin, LOW);

  // relayState();
  WiFi_Setup();

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_wifiPin, HIGH);
    digitalWrite(LED_portalPin, LOW);
    ubidots.connectToWifi(WiFi.SSID().c_str(), WiFi.psk().c_str());
    ubidots.setDebug(true);  // uncomment this to make debug messages available
    ubidots.setCallback(callback);
    ubidots.setup();
    ubidots.reconnect();
    ubidots.subscribeLastValue(PUBLISH_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL);  // Insert the device and variable's Labels, respectively

    timer = millis();

  } else {
    Serial.println("No Internet");
    Serial.println("Starting Web Portal");
    digitalWrite(LED_portalPin, HIGH);
    digitalWrite(LED_wifiPin, LOW);
    wm.startConfigPortal(APSSID, APPassword);
  }

  while (1) {
    unsigned long publishStartTime = millis();  // Measure start time before publishing
    digitalWrite(LED_portalPin, LOW);
    digitalWrite(LED_wifiPin, HIGH);

    // Perform monitoring tasks here
    Serial.println("Monitoring task running");
    delay(1000);
    if (!ubidots.connected()) {
      unsigned long startTime = millis();
      if (millis() - startTime > 5000) {
        Serial.println("No Internet");
        Serial.println("Starting Web Portal");
        digitalWrite(LED_portalPin, HIGH);
        digitalWrite(LED_wifiPin, LOW);
        wm.startConfigPortal(APSSID, APPassword);
      } else {
        ubidots.reconnect();
        ubidots.subscribeLastValue(PUBLISH_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL);

        digitalWrite(LED_wifiPin, HIGH);
        delay(100);
        digitalWrite(LED_wifiPin, LOW);
        delay(100);
        digitalWrite(LED_wifiPin, HIGH);
        delay(100);
        digitalWrite(LED_wifiPin, LOW);
      }
    }

    if (wm_nonblocking)
      wm.process();  // avoid delays() in loop when non-blocking and other long running code
    checkButton();

    if (abs(static_cast<long>(millis() - timer)) > PUBLISH_FREQUENCY) {
      float voltage = pzem.voltage();
      float current = pzem.current();
      float power = pzem.power();
      float energy = pzem.energy();
      float frequency = pzem.frequency();
      float pf = pzem.pf();

      ubidots.add(PUBLISH_VARIABLE_LABEL1, voltage);  // Insert your variable Labels and the value to be sent
      ubidots.add(PUBLISH_VARIABLE_LABEL2, current);
      ubidots.add(PUBLISH_VARIABLE_LABEL3, power);
      ubidots.add(PUBLISH_VARIABLE_LABEL4, energy);
      ubidots.add(PUBLISH_VARIABLE_LABEL5, frequency);
      ubidots.add(PUBLISH_VARIABLE_LABEL6, pf);
      ubidots.add(PUBLISH_VARIABLE_LABEL7, relay_status);
      ubidots.add(PUBLISH_VARIABLE_LABEL8, trip_flag);

      ubidots.publish(PUBLISH_DEVICE_LABEL);

      unsigned long publishEndTime = millis();  // Measure end time after publishing
      unsigned long publishLatency = publishEndTime - publishStartTime;  // Calculate the latency

      // Send the latency measurement to Ubidots
      ubidots.add(PUBLISH_VARIABLE_LABEL9, publishLatency);
      ubidots.publish(PUBLISH_DEVICE_LABEL);

      Serial.print("Publish Latency: ");
      Serial.println(publishLatency);

      timer = millis();
    }
    ubidots.loop();
  }
}


void safetyTaskFunction(void *pvParameters) {

  pinMode(Relay, OUTPUT);
  //For comparator input, send 3.3V if Ground Fault Detected, 0V if no Ground Fault
  pinMode(From_comparator, INPUT);
  //Reset, this will ensure that the circuit can start conducting again
  //If HIGH, Send 3.3V to MOSFET to discharge the peak detector capacitor to ground
  //Reset the tripped signal to False
  pinMode(Reset_pin, INPUT);
  //Responsible for Discharging the peak detector capacitor
  pinMode(MOSFET_pin, OUTPUT);
  //LED indicators
  pinMode(LED_FaultDetected, OUTPUT);
  pinMode(LED_RelayStatus, OUTPUT);
  //DigitalWrite High Relay
  digitalWrite(Relay, HIGH);
  digitalWrite(LED_RelayStatus, HIGH);
  RelayState = "ON";

  for (;;) {
    //Poll if the trip flag is 0, which is initially 0. This indicates that the Circuit is not tripped
    //If flag becomes 1, bypass this if then else, indicating that Circuit is tripped state
    //Needs to be resetted in order the tripped state to be 0 again
    if (trip_flag == 0) {
      //If the comparator is HIGH, therefore the peak detector voltage is higher than treshold indicating G Fault signal
      //The relay will trip the circuit
      //Trip flag will be set to 1, indicating that the current state of circuit is tripped
      if (digitalRead(From_comparator) == HIGH) {
        //Normally Closed Relay, turn LOW -> Open the circuit
        digitalWrite(Relay, LOW);
        digitalWrite(MOSFET_pin, LOW);
        digitalWrite(LED_RelayStatus, LOW);
        RelayState = "OFF";
        relay_status = 0;
        digitalWrite(LED_FaultDetected, HIGH);
        trip_flag = 1;

      }
      //If the comparator is low, therefore no tripping occurred, the relay should be on LOW making the circuit close
      else {
        //Normally Closed Relay, Turn HIGH -> Close Circuit
        //digitalWrite(Relay, HIGH);
        trip_flag = 0;  //added
        digitalWrite(MOSFET_pin, LOW);
        //digitalWrite(LED_RelayStatus,HIGH);
        digitalWrite(LED_FaultDetected, LOW);
      }
    }

    //Resetting the trip signal and the Peak detector capacitor to 0V
    //Reset button is connected to pin 34, there should be a pull down resistor to ground
    //Pull down to GND resistor makes the pin not to float during open switch condition
    //When button is pushed, indicating a reset signal, the reset signal will make the tripped flag 0, resetting the trip state
    //30.3V will be send to MOSFET (logic level), in order to pull the capacitor voltage to 0.


    if (trip_flag == 1) {
      if (digitalRead(Reset_pin) == 0) {
        trip_flag = 0;
        digitalWrite(Relay, HIGH);
        digitalWrite(LED_RelayStatus, HIGH);
        RelayState = "ON";
        relay_status = 1;
        digitalWrite(MOSFET_pin, HIGH);
      }
      /*
      else if (digitalRead(Reset_pin)==0 && digitalRead(From_comparator)==0){
        trip_flag=0;
        digitalWrite(Relay, HIGH);
        digitalWrite(LED_RelayStatus, HIGH);
        RelayState = "ON";
        digitalWrite(MOSFET_pin, HIGH);
      }
      */
      else {
        trip_flag = 1;
        digitalWrite(Relay, LOW);
        digitalWrite(LED_RelayStatus, LOW);
        RelayState = "OFF";
        relay_status = 0;
        digitalWrite(MOSFET_pin, LOW);
      }
    }


    //Delay Test, Trip time of relay
    /*
    Serial.println("Relay Status:");
    Serial.println(digitalRead(DelayTest));
    */
    delay(1);
  }
}
