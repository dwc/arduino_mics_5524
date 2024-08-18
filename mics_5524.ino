#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoMqttClient.h>
#include <DFRobot_MICS.h>
#include "arduino_secrets.h"

// Default calibration time is three minutes
#define CALIBRATION_TIME 3
#define ADC_PIN A0
#define POWER_PIN 16
#define MQTT_PUBLISH_DELAY 10 * 1000L

WiFiUDP ntp_wifi_client;
NTPClient ntp_client(ntp_wifi_client);
WiFiClient mqtt_wifi_client;
MqttClient mqtt_client(mqtt_wifi_client);
DFRobot_MICS_ADC mics(ADC_PIN, POWER_PIN);
char message[256];

void setup() {
  Serial.begin(115200);

  Serial.println("Starting up...");
  setup_mics();
  setup_wifi();
  setup_ntp();
  setup_mqtt();
}

void setup_mics() {
  Serial.print("Initializing MiCS sensor");
  while (!mics.begin()) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println(". Done!");

  Serial.print("Checking power state of MiCS sensor");
  uint8_t mode = mics.getPowerState();
  if (mode == SLEEP_MODE) {
    mics.wakeUpMode();
    Serial.println(". Successfully woke up sensor!");
  }
  else {
    Serial.println(". Sensor is already awake!");
  }

  Serial.print("Warming up MiCS sensor");
  int delay_seconds = 0;
  while (!mics.warmUpTime(CALIBRATION_TIME)) {
    if (delay_seconds % 10 == 0) {
      Serial.print(".");
    }

    delay(1000);
    ++delay_seconds;
  }

  Serial.println(". Done!");
}

void setup_wifi() {
  Serial.print("Initializing WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println(". Done!");
}

void setup_ntp() {
  Serial.print("Initializing NTP client");
  ntp_client.begin();
  Serial.println(". Done!");
}

void setup_mqtt() {
  Serial.print("Initializing MQTT client");
  mqtt_client.setUsernamePassword(MQTT_USERNAME, MQTT_PASSWORD);
  while (!mqtt_client.connect(MQTT_BROKER, MQTT_PORT)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println(". Done!");
}

void check_connection_status() {
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }

  if (!mqtt_client.connected()) {
    setup_mqtt();
  }
}

void update_data() {
  ntp_client.update();
}

void loop() {
  /*
    Gas type:
    MICS-4514 supports all gas concentration values
    MICS-5524 supports CH4, C2H5OH, H2, NH3, and CO concentration values
    MICS-2714 supports NO2 concentration value
      Methane          (CH4)    (1000 - 25000)PPM
      Ethanol          (C2H5OH) (10   - 500)PPM
      Hydrogen         (H2)     (1    - 1000)PPM
      Ammonia          (NH3)    (1    - 500)PPM
      Carbon Monoxide  (CO)     (1    - 1000)PPM
      Nitrogen Dioxide (NO2)    (0.1  - 10)PPM
  */
  check_connection_status();
  update_data();

  sprintf(message, "{ \"ntp_epoch_time\": %d, \"methane\": \"%f\", \"ethanol\": \"%f\", \"hydrogen\": \"%f\", \"ammonia\": \"%f\", \"carbon_monoxide\": \"%f\" }",
    ntp_client.getEpochTime(), mics.getGasData(CH4), mics.getGasData(C2H5OH), mics.getGasData(H2), mics.getGasData(NH3), mics.getGasData(CO));

  Serial.println(message);

  mqtt_client.poll();
  mqtt_client.beginMessage(MQTT_TOPIC);
  mqtt_client.print(message);
  mqtt_client.endMessage();

  delay(MQTT_PUBLISH_DELAY);
}
