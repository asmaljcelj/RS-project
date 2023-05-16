/*
 ESP8266 MQTT example
 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.
 
 Prerequisites:
 
 PubSubClient -> Install the library through : Arduino -> Tools -> Manage Libraries -> type PubSubClient -> install the package created by Nick O'Leary (newest version)
 
 Git: https://github.com/knolleary/pubsubclient
 API: https://pubsubclient.knolleary.net/
  
ESP8266 first reads acceleration data, then it connects to an MQTT server "broker.mqtt-dashboard.com", and finally publishes acceleration measurements to the topic "RatkoACCData" 
    
 More info:
 "broker.mqtt-dashboard.com" is a publicly available MQTT broker. You can set your own MQTT broker on ESP8266 or your laptop
 MQTT introduction: https://www.hivemq.com/mqtt-essentials/
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include<Wire.h>
#include<Ticker.h>
#include <limits.h>

#define INTERVAL 100
#define PIN_LED 2
#define I2C_ADD_MPU 104
#define I2C_ADD_BMP 118
#define TABLE_SIZE_MPU 7
#define GYRO_OBCUT 131
#define REG_GYRO_CONFIG 0x1B // 27
#define RATE 10
#define I2C_ADD_IO1 32
#define ACC_OUT 59

// Update these with values suitable for your network.

const char* ssid = "esp8266";
const char* password = "adminadmin";
const char* mqtt_server = "broker.mqtt-dashboard.com";
const char* topic = "RsProjekt123"; 


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
#define HISTORY_SIZE 50
char msg[MSG_BUFFER_SIZE];
int value = 0;

Ticker tick;
int32_t table_x[TABLE_SIZE_MPU];
int32_t table_y[TABLE_SIZE_MPU];
int32_t table_z[TABLE_SIZE_MPU];
int32_t SMOOTHING_WINDOW = 2;
int32_t history_x[HISTORY_SIZE];
int32_t history_y[HISTORY_SIZE];
int32_t history_z[HISTORY_SIZE];
int32_t stepCounter = 0;
float delilnik = 131.0f;
float acc_x_calib = 0.0;
float acc_y_calib = 0.0;
float acc_z_calib = 0.0;
float threshold = 1000000.0f;

// funkcije:
void beriPodatke();
void acc_config();

int32_t preveriNajvecjoOs() {
  // 0 = x os
  // 1 = y os
  // 2 = z os
  // initialize
  int32_t max_x = 0;
  int32_t max_y = 0;
  int32_t max_z = 0;
  // begin count
  for (int i = 0; i < HISTORY_SIZE; i++) {
    if (abs(history_x[i] > max_x)) {
      max_x = abs(history_x[i]);
    }
    if (abs(history_y[i] > max_y)) {
      max_y = abs(history_y[i]);
    }
    if (abs(history_z[i] > max_z)) {
      max_z = abs(history_z[i]);
    }
  }
  // get maximum
  if (max_x > max_y && max_x > max_z) {
    return 0;
  } else if (max_y > max_x && max_y > max_z) {
    return 1;
  }
  // default, vrni z
  return 2;
}


void beriPodatke() {
  static uint32_t count = 0;
  digitalWrite(PIN_LED, 0);
  static float acc_x = 0.0f;
  static float acc_y = 0.0f;
  static float acc_z = 0.0f;
  int32_t acc_x;
  int32_t acc_y;
  int32_t acc_z;
  
  //**** MPU-9250
  //**** Naslov registra 
  // "zapiši", od katerega naslova registra dalje želimo brati
  Wire.beginTransmission(I2C_ADD_MPU);
  Wire.write(ACC_OUT);
  Wire.endTransmission();
  
  //** Branje: pospešek
  //** Zdaj mikrokrmilnik bere od naslova ACC_OUT
  //** Bere vseh 6 bajtov (x, y in z os): 
  Wire.requestFrom(I2C_ADD_MPU, 6);
  for (int i = 0; i < 6; i++) {
    if (i < 2) {
      table_x = (int8_t) Wire.read();
      table_x = acc_x << 8;
    } else if (i < 4) {
      table_y = (int8_t) Wire.read();
      table_y = acc_y << 8;    
    } else {
      table_z = (int8_t) Wire.read();
      table_z = acc_z << 8;
    }
  }
  
  
  acc_x += ((table / delilnik)-acc_x_calib)/RATE;
  acc_y += ((table / delilnik)-acc_y_calib)/RATE;
  acc_z += ((table / delilnik)-acc_z_calib)/RATE;


  if (count % RATE == 0)
  {
    // Izpišemo
    Serial.print("ACC_X: X= ");
    Serial.print(acc_x);
    Serial.println("");
    Serial.print("ACC_Y: Y= ");
    Serial.print(acc_y);
    Serial.println("");
    Serial.print("ACC_Z: Z= ");
    Serial.print(acc_z);
    Serial.println("");

    // MQTT
    // sends acceleration data as string stored in msg variable
    client.loop();
    snprintf (msg, MSG_BUFFER_SIZE, "%4.2f", acc_x);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(topic, msg);
    // resetiramo vrednost
    acc_x = 0;
    acc_y = 0;
    acc_z = 0;
  }

  if (count == HISTORY_SIZE) {
    Serial.print("START DETECTION STEP");
    Serial.println("");
    // glajenje (vzemi prejšnji, trenutni in naslednji measurment in vstavi povprečje
    int32_t summed_x = 0;
    int32_t summed_y = 0;
    int32_t summed_z = 0;
    int32_t number_summed_x = 0;
    int32_t number_summed_y = 0;
    int32_t number_summed_z = 0;
    for (int i = 0; i < SMOOTHING_WINDOW; i++) {
      int32_t start_index = i - SMOOTHING_WINDOW;
      for (int smoothing_index = i - SMOOTHING_WINDOW; smoothing_index < i + SMOOTHING_WINDOW; smoothing_index++) {
        if (smoothing_index < 0 || smoothing_index > HISTORY_SIZE) {
          continue;
        }
        summed_x += history_x[i];
        number_summed_x++;
        summed_y += history_y[i];
        number_summed_y++;
        summed_z += history_z[i];
        number_summed_z++;
      }
      int32_t average_x = summed_x / number_summed_x;
      int32_t average_y = summed_y / number_summed_y;
      int32_t average_z = summed_z / number_summed_z;
      history_x[i] = average_x;
      history_y[i] = average_y;
      history_z[i] = average_z;
    } 
  
    //TODO: dinamično nastavljanje meje
    int32_t najvecjaOs = preveriNajvecjoOs();
    int32_t maxHistory;
    if (najvecjaOs == 0) {
      maxHistory = histoty_x;
    }
    if (najvecjaOs == 1) {
      maxHistory = histoty_y;
    }
    if (najvecjaOs == 2) {
      maxHistory = histoty_z;
    }
    // get max and min values
    int max_value = INT_MIN;
    int min_value = INT_MAX;
    for (int i = 0; i < HISTORY_SIZE; i++) {
      if (maxHistory[i] > max_value) {
        max_value = maxHistroy[i];
      }
      if (maxHistory[i] < min_value) {
        min_value = maxHistory[i];
      }    
    }
    // step detection
    for (int i = 1; i < HISTORY_SIZE; i++) {
      int32_t previous = maxHistory[i - 1];
      int32_t current = maxHistory[i];
      if (current < previous && previous > threshold && current < threshold) {
        stepCounter++;
      }
    }
  
    // set new threshold
    threshold = (max_value + min_value) / 2;
    
    //TODO: count calories

    count = 0;
  }
  history_x[count] = acc_x;
  history_y[count] = acc_y;
  history_z[count] = acc_z;
    
  // števec 
  count = count+1;
  digitalWrite(PIN_LED, 1);
}

void acc_calib(){
  
  //digitalWrite(PIN_LED, 0);

  delay(1000);
  
  int rate = 10;
  int samp = 50;
  int32_t table;

  //**** MPU-9250
  // "zapiši", od katerega naslova registra dalje želimo brati
  for (int q=0; q<samp; q++)
  {
    Wire.beginTransmission(I2C_ADD_MPU);
    Wire.write(ACC_X_OUT);
    Wire.endTransmission();
    
    //** Branje: pospeškometera
    Wire.requestFrom(I2C_ADD_MPU, 6);
    for (int i = 0; i < 6; i++) {
    if (i < 2) {
      table_x = (int8_t) Wire.read();
      table_x = acc_x << 8;
    } else if (i < 4) {
      table_y = (int8_t) Wire.read();
      table_y = acc_y << 8;    
    } else {
      table_z = (int8_t) Wire.read();
      table_z = acc_z << 8;
    }
  }

    acc_x_calib += (table_x/ delilnik)/rate;
    acc_y_calib += (table_y/ delilnik)/rate;
    acc_z_calib += (table_z/ delilnik)/rate;
 
    delay(1000/rate);
  }

  acc_x_calib /= (samp/rate);
  acc_y_calib /= (samp/rate);
  acc_z_calib /= (samp/rate);

  Serial.print("** CALIB: ");
  Serial.print("ACC: X= ");
  Serial.print(acc_x_calib);
  Serial.print("ACC: Y= ");
  Serial.print(acc_y_calib);
  Serial.print("ACC: Z= ");
  Serial.print(acc_z_calib);
  
  delay(1000);
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Inicializiramo I2C na podanih pinih
  Wire.begin(12,14);
  // nastavimo frekvenco vodila na 100 kHz
  Wire.setClock(100000);

  acc_calib();
  tick.attach_ms(INTERVAL, beriPodatke);
  
  if (!client.connected()) {
    reconnect();
  }
}

void loop() {

}
