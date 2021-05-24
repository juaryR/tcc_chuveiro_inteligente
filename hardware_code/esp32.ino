#include <WiFiMulti.h> // biblioteca
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_MAX31855.h"
#include <driver/adc.h>
#include "EmonLib.h"

// This is the device name as defined on AWS IOT
#define DEVICE_NAME "Chuveiro-Inteligente"

// The GPIO pin were the CT sensor is connected to (should be an ADC input)
#define ADC_INPUT_C 34
#define ADC_INPUT_V 35


#define MAXDO   19
#define MAXCS   23
#define MAXCLK  5
#define CONTROLE_PIN  27

// The voltage in our apartment. Usually this is 230V for Europe, 110V for US.
// Ours is higher because the building has its own high voltage cabin.
#define HOME_VOLTAGE 220.0
// Force EmonLib to use 10bit ADC resolution
#define ADC_BITS    12
#define ADC_COUNTS  (1<<ADC_BITS)
const int oneWireBus = 15; // GPIO where the DS18B20 is connected to    
#define vCalibration 745
#define currCalibration   31.33//30   //0.52 

// Add your wifi Settings 
const char* ssid = "";
const char* password = "";
const char* mqttServer = ""; // You mqtt server IP , 
const int mqttPort = 1883;
const char* mqttUser = "chuveiro";
const char* mqttPassword = "159753";
bool WiFi_connected = false;
// Array to store 30 readings (and then transmit in one-go to AWS)
short measurements[30];
short measureIndex = 0;
unsigned long lastMeasurement = 0;
unsigned long timeFinishedSetup = 0;
double power = 0  ;
float volt = 0;
float amps = 0 ;
float kWh = 0;


EnergyMonitor emon1;
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void TaskWiFi( void *parameter );  // Tarefa WiFi


/* ===============    VARIAVEIS     ============== */
TaskHandle_t TaskWiFi_t;

float temperature_in = 0;
float temperature_out = 0;
float current_temperature=0;
float set_temperatura=34;
float last_set_temperatura=-1;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire); 

WiFiClient espClient;
PubSubClient client(espClient);

//
////faz o controle do temporizador (interrupção por tempo)
//hw_timer_t *timer = NULL; 
 
//função que o temporizador irá chamar, para reiniciar o ESP32
void IRAM_ATTR resetModule(){
ets_printf("(watchdog) reiniciar\n"); //imprime no log
esp_restart(); //reinicia o chip
}
 
////função que o configura o temporizador
//void configureWatchdog()
//{
//timer = timerBegin(0, 80, true); //timerID 0, div 80
////timer, callback, interrupção de borda
//timerAttachInterrupt(timer, &resetModule, true);
////timer, tempo (us), repetição
//timerAlarmWrite(timer, 5000000, true);
//timerAlarmEnable(timer); //habilita a interrupção //enable interrupt
//}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

float diff = 1.0;


/* ===============      SETUP       ============== */
void setup() {
 
 
  Serial.begin(115200);
  thermocouple.begin();
  
 // put your setup code here, to run once:
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  analogReadResolution(10);
 // Inicializa tarefa WiFi
 xTaskCreatePinnedToCore(TaskWiFi, "TaskWiFi", 16384, NULL, 1, &TaskWiFi_t, 0);  //Task Function, Task name for humans, Stack size, ... , Priority, Task name, Core number

   timeFinishedSetup = millis();
// Initialize emon library (30 = calibration number)
  emon1.current(ADC_INPUT_C, currCalibration);
  emon1.voltage(ADC_INPUT_V, vCalibration,0);  
 pinMode(CONTROLE_PIN, OUTPUT);
 digitalWrite(CONTROLE_PIN, HIGH);
 
 while (!WiFi_connected) {                             // Aguarda conexão com rede WiFi
    delay(100);
     Serial.println("Iniciando WIFI...");
  }
  
  // Start the DS18B20 sensor
  sensors.begin();

 // Start the WIFI
  client.setServer(mqttServer, mqttPort);

 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
 
  client.setCallback(callback);
 
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 if (client.connect(DEVICE_NAME,mqttUser,mqttPassword)) {
//    if (client.connect("ESP8266Client")) {/
 
      Serial.println("connected");  
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }
 
//  client.publish("esp/test", "Hello from chuveiro");
  client.subscribe("tele/pow1/current/temp/tempset");
 
}
void callback(char* topic, byte* payload, unsigned int length) {
//  Serial.print("Message:");

  payload[length] = '\0';
  String s = String((char*)payload);
  float messageTemp = s.toFloat();

 Serial.print(messageTemp);
//  client.publish("esp/test", "Hello from chuveiro");
  if (String(topic) == "tele/pow1/current/temp/tempset") {
    if(messageTemp != current_temperature & amps == 0 ){
      last_set_temperatura=last_set_temperatura;
      set_temperatura=messageTemp;
             if(messageTemp > current_temperature){
         digitalWrite(CONTROLE_PIN, HIGH);
    }
    else if(messageTemp < current_temperature){
      digitalWrite(CONTROLE_PIN, LOW);
    }
    
  }

  }
}
 
void loop() {
  
// //reseta o temporizador (alimenta o watchdog)
//    timerWrite(timer, 0); 
  // put your main code here, to run repeatedly:
unsigned long currentMillis = millis();


  sensors.requestTemperatures(); 
  
  temperature_in = sensors.getTempCByIndex(0);
  temperature_out = thermocouple.readCelsius();
  
 if(temperature_out>temperature_in+5){
   current_temperature=temperature_out;
 }else{
//  current_temperature=temperature_in;
  current_temperature=temperature_out;
 };
 if(last_set_temperatura > current_temperature){
    if(last_set_temperatura =! -1){ //No value have been set ainda 
           digitalWrite(CONTROLE_PIN, HIGH);
    }
        else if(current_temperature > set_temperatura+1){
       digitalWrite(CONTROLE_PIN, LOW);
     }
 }
   
  // If it's been longer then 1000ms since we took a measurement, take one now!
  if(currentMillis - lastMeasurement > 1000){
     emon1.calcVI(20, 2000);
     volt = int(emon1.Vrms*10)/10.0;
     amps = int(emon1.Irms*100 )/100.0;
     if(amps > 0.09){
         power =  int(emon1.apparentPower*100 )/100.0;
         kWh = kWh + emon1.apparentPower*(millis()-lastMeasurement)/3600000000.0;
     }else{
         amps = 0;
         power=0;  
      
     }
    // amps = emon1.calcIrms(1480); // Calculate Irms only
//     int inter =  (amps*10);
//     amps = inter/10.0;
    lastMeasurement = millis();
  }
  
 
//
   char JSONmessageBuffer[100];
   DynamicJsonDocument doc(1024); 
   doc["ENERGY"]["Today"]= int(kWh*10000)/10000.0;
   doc["ENERGY"]["Power"]= power;
   doc["ENERGY"]["Voltage"]= volt;
   doc["ENERGY"]["Current"]= amps;
   doc["current_temperature"] = current_temperature;
   serializeJson(doc,Serial);
   serializeJson(doc, JSONmessageBuffer,sizeof(JSONmessageBuffer));

  if (client.publish("tele/pow1/current", JSONmessageBuffer) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
    if(!client.connected()){
      client.connect(DEVICE_NAME,mqttUser,mqttPassword);
    }
  }
 
 
  client.loop();
  Serial.println("-------------");
 
  delay(500);

}


void TaskWiFi( void *parameter ) {

  (void) parameter;

  bool flagWiFi_connected = WiFi_connected;

  WiFi.mode(WIFI_STA); // STA (conecta-se em alguem), AP (é conectado por alguem)

  for (;;) {
    WiFi.begin(ssid, password);
    for (int i = 0; i < 10; i++) {

      if (WiFi_connected != flagWiFi_connected) {
        flagWiFi_connected = WiFi_connected;
        Serial.println("<WiFi> Disconnected");
      }

      if (WiFi.status() != WL_CONNECTED) { // WL_CONNECTED é um define de WiFi.h
        WiFi_connected = false;
        vTaskDelay(500);
      } else {

        WiFi_connected = true;
        while (WiFi.status() == WL_CONNECTED) {
          if (WiFi_connected != flagWiFi_connected) {
            flagWiFi_connected = WiFi_connected;
            Serial.println("<WiFi> Connected");
          }
          vTaskDelay(500);
        }
      }
    }
  }
}
