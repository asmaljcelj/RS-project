#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Ticker.h>
#include <limits.h>
#include <BlynkSimpleEsp8266.h>

#define INTERVAL_BERI 100
#define INTERVAL_RESET 86400
#define PIN_LED 2
#define I2C_ADD_MPU 104
#define I2C_ADD_BMP 118
#define TABLE_SIZE_MPU 7
#define GYRO_OBCUT 131
#define REG_GYRO_CONFIG 0x1B // 27
#define RATE 10
#define I2C_ADD_IO1 32
#define ACC_OUT 59
#define BLYNK_PRINT Serial

const char *wifi_ssid = "Jansa-G";
const char *wifi_password = "12jansa34";
const char *blynk_template_id = "TMPL4bPFAc-b0";
const char *blynk_template_name = "rs2023";
const char *blynk_auth_token = "DZcBJHgpw8rjOJGr0kz9MuMi8C415dlp";

#define HISTORY_SIZE 50

Ticker tick_beri;
Ticker tick_reset;
int32_t table_x[TABLE_SIZE_MPU];
int32_t table_y[TABLE_SIZE_MPU];
int32_t table_z[TABLE_SIZE_MPU];
int32_t SMOOTHING_WINDOW = 2;
int32_t history_x[HISTORY_SIZE];
int32_t history_y[HISTORY_SIZE];
int32_t history_z[HISTORY_SIZE];
int32_t stepCounter = 0;
float delilnik = 16384.0f;
float acc_x_calib = 0.0f;
float acc_y_calib = 0.0f;
float acc_z_calib = 0.0f;
// visok threshold na zacetku, da ne zaznamo korakov v mirovanju
float threshold = 1000000.0f;
int32_t countsSinceLastStep = 0;

// funkcije:
void beriPodatke();

void resetDailySteps();

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
  int32_t table_x;
  int32_t table_y;
  int32_t table_z;

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
      if (i % 2 == 0) {
        table_x = table_x << 8;
      }
    } else if (i < 4) {
      table_y = (int8_t) Wire.read();
      if (i % 2 == 0) {
        table_y = table_y << 8;
      }
    } else {
      table_z = (int8_t) Wire.read();
      if (i % 2 == 0) {
        table_z = table_z << 8;
      }
    }
  }

  // izracun pospeska
  acc_x += ((table_x / delilnik) - acc_x_calib) / RATE;
  acc_y += ((table_y / delilnik) - acc_y_calib) / RATE;
  acc_z += ((table_z / delilnik) - acc_z_calib) / RATE;

  if (count % RATE == 0) {
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

    // sends acceleration data converted to m/s^2
    Serial.print("Publishing message on V0: ");
    Serial.println(acc_x * 9.81);
    Blynk.virtualWrite(V0, acc_x * 9.81);
    Serial.print("Publishing message on V1: ");
    Serial.println(acc_x * 9.81);
    Blynk.virtualWrite(V1, acc_y * 9.81);
    Serial.print("Publishing message on V2: ");
    Serial.println(acc_x * 9.81);
    Blynk.virtualWrite(V2, acc_z * 9.81);

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
    int32_t maxHistory[HISTORY_SIZE];
    if (najvecjaOs == 0) {
      memcpy(maxHistory, history_x, HISTORY_SIZE);
    } else if (najvecjaOs == 1) {
      memcpy(maxHistory, history_y, HISTORY_SIZE);
    } else if (najvecjaOs == 2) {
      memcpy(maxHistory, history_z, HISTORY_SIZE);
    }
    // get max and min values
    int max_value = INT_MIN;
    int min_value = INT_MAX;
    for (int i = 0; i < HISTORY_SIZE; i++) {
      if (maxHistory[i] > max_value) {
        max_value = maxHistory[i];
      }
      if (maxHistory[i] < min_value) {
        min_value = maxHistory[i];
      }
    }
    // step detection
    for (int i = 1; i < HISTORY_SIZE; i++) {
      int32_t previous = maxHistory[i - 1];
      int32_t current = maxHistory[i];
      if (current < previous && previous > threshold && current < threshold && countsSinceLastStep > 2) {
        // todo: upostevaj se cas med obema korakom (periodicnost!!!)
        Serial.print("STEP DETECTED");
        Serial.println("");
        stepCounter++;
        countsSinceLastStep = 0;

        Serial.print("Publishing message on V3: ");
        Serial.println(stepCounter);
        Blynk.virtualWrite(V3, stepCounter);

        // Preveri, ali je bil dosežen dnevni cilj korakov (5000 korakov)
        if (stepCounter >= 5000) {
          Serial.print("Publishing message on V5: ");
          Serial.println("Daily step goal reached!");
          Blynk.virtualWrite(V5, "Daily step goal reached!");
        } else {
          Serial.print("Publishing message on V4: ");
          Serial.println("Daily step goal not yet reached!");
          Blynk.virtualWrite(V5, "Daily step goal not yet reached!");
        }
      }
    }

    // set new threshold
    threshold = (max_value + min_value) / 2;

    //TODO: count calories

    // reset counter
    count = 0;
  }
  history_x[count] = acc_x;
  history_y[count] = acc_y;
  history_z[count] = acc_z;

  // števec
  count = count + 1;
  countsSinceLastStep++;
  //digitalWrite(PIN_LED, 1);
}

void resetStepCount() {
  stepCounter = 0;
  Serial.println("Step count reset!");
}


void acc_calib() {

  //digitalWrite(PIN_LED, 0);

  delay(1000);

  int rate = 10;
  int samp = 50;
  int32_t table_x;
  int32_t table_y;
  int32_t table_z;

  //**** MPU-9250
  // "zapiši", od katerega naslova registra dalje želimo brati
  for (int q = 0; q < samp; q++) {
    Wire.beginTransmission(I2C_ADD_MPU);
    Wire.write(ACC_OUT);
    Wire.endTransmission();

    //** Branje: pospeškometera
    Wire.requestFrom(I2C_ADD_MPU, 6);
    for (int i = 0; i < 6; i++) {
      if (i < 2) {
        table_x = (int8_t) Wire.read();
        if (i % 2 == 0) {
          table_x = table_x << 8;
        }
      } else if (i < 4) {
        table_y = (int8_t) Wire.read();
        if (i % 2 == 0) {
          table_y = table_y << 8;
        }
      } else {
        table_z = (int8_t) Wire.read();
        if (i % 2 == 0) {
          table_z = table_z << 8;
        }
      }
    }

    acc_x_calib += (table_x / delilnik) / rate;
    acc_y_calib += (table_y / delilnik) / rate;
    acc_z_calib += (table_z / delilnik) / rate;

    delay(1000 / rate);
  }

  acc_x_calib /= (samp / rate);
  acc_y_calib /= (samp / rate);
  acc_z_calib /= (samp / rate);

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
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

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

void setup() {
  // Initialize the BUILTIN_LED pin as an output
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.begin(115200);
  setup_wifi();

  Blynk.begin(blynk_auth_token, wifi_ssid, wifi_password);

  // Inicializiramo I2C na podanih pinih
  Wire.begin(12, 14);
  // nastavimo frekvenco vodila na 100 kHz
  Wire.setClock(100000);

  acc_calib();
  tick_beri.attach_ms(INTERVAL_BERI, beriPodatke);
  tick_reset.attach(INTERVAL_RESET, resetStepCount);
}

void loop() {
  Blynk.run();
}
