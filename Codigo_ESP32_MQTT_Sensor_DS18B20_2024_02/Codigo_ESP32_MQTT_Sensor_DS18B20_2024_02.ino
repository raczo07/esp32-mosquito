
//#include <Arduino.h>
//Libreria exportar datos en formato JSON
//#include <ArduinoJson.h>

//******************************************************************************************
// Sensor DS18B20

//Descomentar si se instala el sensor
#define SENSOR_INSTALADO

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
//******************************************************************************************

//******************************************************************************************
//MQQT Broker

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "XXXXXXXXXX";
const char* password = "XXXXXXXXXX*";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
//const char* mqtt_server = "YOUR_MQTT_BROKER_IP_ADDRESS";
const char* mqtt_server = "XXXXXXXXXX";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
//******************************************************************************************

//******************************************************************************************
bool estado_suiche = false;
bool estado_led = false;
float temperatura = 0;
const int PIN_LED = 2;
const int PIN_SUICHE = 12;

// Configuración PWM
unsigned int duty = 0;
const int PIN_LED_PWM = 5;
const unsigned int RESOLUCION_PWM = 8;
//******************************************************************************************

//******************************************************************************************
void setup() {
  // put your setup code here, to run once:
  // start serial port
  Serial.begin(115200);

  #ifdef SENSOR_INSTALADO
    Serial.println("Dallas Temperature IC Control Library Demo");
      // Start up the library
    sensors.begin();
  #endif

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SUICHE,INPUT_PULLUP);

  pinMode(PIN_LED_PWM, OUTPUT);
  analogWriteResolution(PIN_LED_PWM, RESOLUCION_PWM);
}
//******************************************************************************************

//******************************************************************************************
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//******************************************************************************************

//******************************************************************************************
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/led") {
    Serial.print("Cambiando estado LED: ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(PIN_LED, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(PIN_LED, LOW);
    }
  }

  if (String(topic) == "esp32/duty") {
    Serial.print("Cambiando valor duty: ");
    duty = messageTemp.toInt();
    //Serial.println(duty+255);
    analogWrite(PIN_LED_PWM, duty);
  }

}
//******************************************************************************************

//******************************************************************************************

/*// Possible values for client.state()
#define MQTT_CONNECTION_TIMEOUT     -4
#define MQTT_CONNECTION_LOST        -3
#define MQTT_CONNECT_FAILED         -2
#define MQTT_DISCONNECTED           -1
#define MQTT_CONNECTED               0
#define MQTT_CONNECT_BAD_PROTOCOL    1
#define MQTT_CONNECT_BAD_CLIENT_ID   2
#define MQTT_CONNECT_UNAVAILABLE     3
#define MQTT_CONNECT_BAD_CREDENTIALS 4
#define MQTT_CONNECT_UNAUTHORIZED    5
*/

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/led");
      client.subscribe("esp32/duty");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//******************************************************************************************

//******************************************************************************************
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    #ifdef SENSOR_INSTALADO
      Serial.print("Requesting temperatures...");
      sensors.requestTemperatures(); // Send the command to get temperatures
      Serial.println("DONE");
      // Temperature in Celsius
      sensors.requestTemperatures(); // Send the command to get temperatures
      temperatura = sensors.getTempCByIndex(0);
    
      if(temperatura != DEVICE_DISCONNECTED_C) 
      {
        Serial.print("Temperature for the device 1 (index 0) is: ");
        Serial.println(temperatura);
      } 
      else
      {
        Serial.println("Error: Could not read temperature data");
      }
    #endif
    
    // Convert the value to a char array
    char tempString[8];
    dtostrf(temperatura, 1, 2, tempString);
    Serial.print("Temperatura: ");
    Serial.println(tempString);
    
    client.publish("esp32/temperatura", tempString);
  }

  if (estado_suiche != digitalRead(PIN_SUICHE)){
    estado_suiche = digitalRead(PIN_SUICHE);
    //char estado_suiche_str;
    String estado_suiche_str = "";
    if(estado_suiche)
      estado_suiche_str = "ON";
    else
      estado_suiche_str = "OFF";
     client.publish("esp32/suiche", estado_suiche_str.c_str());
  }
}
