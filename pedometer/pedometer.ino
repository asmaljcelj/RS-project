#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Ticker.h>
#include <limits.h>
#include <BlynkSimpleEsp8266.h>

#define INTERVAL_BERI 100
#define INTERVAL_RESET 86400
#define INTERVAL_CALORIES 2000
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
#define HISTORY_SIZE 50
#define G 9.80665

const char *wifi_ssid = "Jansa-G";
const char *wifi_password = "12jansa34";
const char *blynk_template_id = "TMPL4bPFAc-b0";
const char *blynk_template_name = "rs2023";
const char *blynk_auth_token = "DZcBJHgpw8rjOJGr0kz9MuMi8C415dlp";

Ticker tick_beri;
Ticker tick_reset;
Ticker tick_calories;
int32_t table_x[TABLE_SIZE_MPU];
int32_t table_y[TABLE_SIZE_MPU];
int32_t table_z[TABLE_SIZE_MPU];
int32_t SMOOTHING_WINDOW = 2;
int32_t history_x[HISTORY_SIZE];
int32_t history_y[HISTORY_SIZE];
int32_t history_z[HISTORY_SIZE];
int32_t calorie_step_counter = 0;
float total_calories_burned = 0.0;
int32_t step_counter = 0;
float delilnik = 16384.0f;
float acc_x_calib = 0.0f;
float acc_y_calib = 0.0f;
float acc_z_calib = 0.0f;
// visok threshold na zacetku, da ne zaznamo korakov v mirovanju
float threshold = 1000000.0f;
int32_t counts_since_last_step = 0;
int32_t daily_steps = 5000;
int32_t daily_calories = 2500;
// izmisljeni parametri
float weight = 75.0;
int32_t height = 178;

// funkcije:
void beri_podatke();

void reset_daily();

void acc_config();

float get_stride(int32_t Nsteps, int32_t height) {
  if (Nsteps == 1) {
    return height / 5;
  } else if (Nsteps == 2) {
    return height / 4;
  } else if (Nsteps == 3) {
    return height / 3;
  } else if (Nsteps == 4) {
    return height / 2;
  } else if (Nsteps == 5) {
    return height / 1.2;
  } else if (Nsteps == 6 || Nsteps == 7) {
    return height;
  } else if (Nsteps >= 8) {
    return 1.2 * height;
  }
  // default value
  return 0.0;
}

void call_kalorije_poraba() {
  kalorije_poraba(step_counter);
}

void kalorije_poraba(int32_t nmb_of_steps) {
  //  2 options: 1. stationary, 2. moving
  //  stationary: C = 1 * weight/1800
  //  moving (2 second update): C = speed * weight/400
  //  speed = steps per 2s * stride/2s
  //  stride = table of steps per 2s --> height/5, 4, 3, 2, 1.2, Height, 1.2*Height
  // Height is in cm, weight in kg
  int32_t nmb_of_steps_in_last_2s = nmb_of_steps - calorie_step_counter;
  float current_calories_burned = 0.0;

  if (nmb_of_steps_in_last_2s == 0) {
    // stationary
    current_calories_burned = weight / 1800;

    Serial.print("Publishing message for 'Current calories': ");
    Serial.println(current_calories_burned);
    Blynk.virtualWrite(V6, current_calories_burned);
  } else {
    float stride = get_stride(nmb_of_steps_in_last_2s, height);

    float speed = nmb_of_steps_in_last_2s * stride;
    current_calories_burned = speed * weight / 400;

    Serial.print("Publishing message for 'Current calories': ");
    Serial.println(current_calories_burned);
    Blynk.virtualWrite(V6, current_calories_burned);
  }

  calorie_step_counter += nmb_of_steps;

  // todo total_calories_burned izračun

  Serial.print("Publishing message for 'Calories': ");
  Serial.println(total_calories_burned);
  Blynk.virtualWrite(V4, total_calories_burned);

  // Preveri, ali je bil dosežen dnevni cilj korakov (5000 korakov)
  if (total_calories_burned >= daily_calories) {
    Serial.print("Publishing message for 'Calories message': ");
    Serial.println("Daily calories goal reached!");
    Blynk.virtualWrite(V9, "Daily calories goal reached!");
  } else {
    Serial.print("Publishing message for 'Calories message': ");
    Serial.println("Daily step calories not yet reached!");
    Blynk.virtualWrite(V9, "Daily calories goal not yet reached!");
  }
}

int32_t preveriNajvecjoOs(int32_t s_table_x[TABLE_SIZE_MPU], int32_t s_table_y[TABLE_SIZE_MPU], int32_t s_table_z[TABLE_SIZE_MPU]) {
  // 0 = x os
  // 1 = y os
  // 2 = z os
  // initialize
  int32_t max_x = 0;
  int32_t max_y = 0;
  int32_t max_z = 0;
  // begin count
  for (int i = 0; i < HISTORY_SIZE; i++) {
    if (abs(s_table_x[i] > max_x)) {
      max_x = abs(s_table_x[i]);
    }
    if (abs(s_table_y[i] > max_y)) {
      max_y = abs(s_table_y[i]);
    }
    if (abs(s_table_z[i] > max_z)) {
      max_z = abs(s_table_z[i]);
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

void beri_podatke() {
  static uint32_t count = 0;
  //digitalWrite(PIN_LED, 0);
  static float acc_x = 0.0f;
  static float acc_y = 0.0f;
  static float acc_z = 0.0f;
  int32_t table_x;
  int32_t table_y;
  int32_t table_z;

  //**** MPU-9250
  //**** Naslov registra
  //**** Naslov registra
  // "zapiši", od katerega naslova registra dalje želimo brati
  Wire.beginTransmission(I2C_ADD_MPU);
  Wire.write(ACC_OUT);
  Wire.endTransmission();

  //** Branje: pospešek
  //** Zdaj mikrokrmilnik bere od naslova ACC_OUT
  //** Bere vseh 6 bajtov (x, y in z os):
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
  acc_x = ((table_x * G / delilnik) - acc_x_calib);
  acc_y = ((table_y * G / delilnik) - acc_y_calib);
  acc_z = ((table_z * G / delilnik) - acc_z_calib);

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
  }

  if (count == HISTORY_SIZE) {
    Serial.print("START DETECTION STEP");
    Serial.println("");
    // glajenje (vzemi prejšnji, trenutni in naslednji measurment in vstavi povprečje
    int32_t smoothed_history_x[HISTORY_SIZE];
    int32_t smoothed_history_y[HISTORY_SIZE];
    int32_t smoothed_history_z[HISTORY_SIZE];
    for (int i = 0; i < HISTORY_SIZE; i++) {
      Serial.print("Start smoothing at ");
      Serial.println(i);
      int32_t summed_x = 0;
      int32_t summed_y = 0;
      int32_t summed_z = 0;
      int32_t number_summed_x = 0;
      int32_t number_summed_y = 0;
      int32_t number_summed_z = 0;
      int32_t start_index = i - SMOOTHING_WINDOW;
      Serial.print("START INDEX = ");
      Serial.println(start_index);
      for (int smoothing_index = i - SMOOTHING_WINDOW; smoothing_index < i + SMOOTHING_WINDOW; smoothing_index++) {
        if (smoothing_index < 0 || smoothing_index > HISTORY_SIZE) {
          continue;
        }
        Serial.print("Summing at index ");
        Serial.println(smoothing_index);
        Serial.print("Adding to x ");
        Serial.println(history_x[i]);
        summed_x += history_x[i];
        number_summed_x++;
        Serial.print("Adding to y ");
        Serial.println(history_y[i]);
        summed_y += history_y[i];
        number_summed_y++;
        Serial.print("Adding to z ");
        Serial.println(history_z[i]);
        summed_z += history_z[i];
        number_summed_z++;
      }
      Serial.print("Summing x: ");
      Serial.print(summed_x);
      Serial.print(" and ");
      Serial.println(number_summed_x);
      int32_t average_x = summed_x / number_summed_x;
      Serial.print("Summing y: ");
      Serial.print(summed_y);
      Serial.print(" and ");
      Serial.println(number_summed_y);
      int32_t average_y = summed_y / number_summed_y;
      Serial.print("Summing z: ");
      Serial.print(summed_z);
      Serial.print(" and ");
      Serial.println(number_summed_z);
      int32_t average_z = summed_z / number_summed_z;
      Serial.print("Smoothed history_x[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(average_x);
      smoothed_history_x[i] = average_x;
      Serial.print("Smoothed history_y[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(average_y);
      smoothed_history_y[i] = average_y;
      Serial.print("Smoothed history_z[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(average_z);
      smoothed_history_z[i] = average_z;
    }

    // TODO: dinamično nastavljanje meje
    int32_t najvecjaOs = preveriNajvecjoOs(smoothed_history_x, smoothed_history_y, smoothed_history_z);
    int32_t maxHistory[HISTORY_SIZE];
    if (najvecjaOs == 0) {
      memcpy(maxHistory, smoothed_history_x, HISTORY_SIZE);
    } else if (najvecjaOs == 1) {
      memcpy(maxHistory, smoothed_history_y, HISTORY_SIZE);
    } else if (najvecjaOs == 2) {
      memcpy(maxHistory, smoothed_history_z, HISTORY_SIZE);
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
    Serial.print("max_value = ");
    Serial.println(max_value);
    Serial.print("min_value = ");
    Serial.println(min_value);
    // step detection
    Serial.println("STEP_DETECTION");
    Serial.print("Threshold = ");
    Serial.println(threshold);
    for (int i = 1; i < HISTORY_SIZE; i++) {
      int32_t previous = maxHistory[i - 1];
      int32_t current = maxHistory[i];
      if (current < previous && previous > threshold && current < threshold && counts_since_last_step > 2) {
        // todo: upostevaj se cas med obema korakom (periodicnost!!!)
        Serial.print("STEP DETECTED");
        Serial.println("");
        Serial.print("current = ");
        Serial.println(current);
        Serial.print("previous = ");
        Serial.println(previous);
        step_counter++;
        counts_since_last_step = 0;

        Serial.print("Publishing message for 'Step counter': ");
        Serial.println(step_counter);
        Blynk.virtualWrite(V3, step_counter);

        // Preveri, ali je bil dosežen dnevni cilj korakov (5000 korakov)
        if (step_counter >= daily_steps) {
          Serial.print("Publishing message for 'Steps message': ");
          Serial.println("Daily steps goal reached!");
          Blynk.virtualWrite(V5, "Daily steps goal reached!");
        } else {
          Serial.print("Publishing message for 'Steps message': ");
          Serial.println("Daily steps goal not yet reached!");
          Blynk.virtualWrite(V5, "Daily steps goal not yet reached!");
        }
      }
    }

    // set new threshold
    threshold = (max_value + min_value) / 2;

    // reset counter
    count = 0;
  }

  history_x[count] = acc_x;
  history_y[count] = acc_y;
  history_z[count] = acc_z;

  // števec
  count = count + 1;
  counts_since_last_step++;
  // digitalWrite(PIN_LED, 1);
}

void reset_daily() {
  step_counter = 0;
  total_calories_burned = 0.0;
  Serial.println("Daily steps and calories reset!");
}

void acc_calib() {
  // digitalWrite(PIN_LED, 0);

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

    acc_x_calib += (table_x * G / delilnik);
    acc_y_calib += (table_y * G / delilnik);
    acc_z_calib += (table_z * G / delilnik);

    delay(1000 / rate);
  }

  acc_x_calib /= samp;
  acc_y_calib /= samp;
  acc_z_calib /= samp;

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

BLYNK_WRITE(V7) {
    height = param.asInt();
    Serial.print("Reading message for 'Height': ");
    Serial.println(height);
}

// Weight
BLYNK_WRITE(V8)   {
    weight = param.asFloat();
    Serial.print("Reading message for 'Weight': ");
    Serial.println(weight);
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
  tick_beri.attach_ms(INTERVAL_BERI, beri_podatke);
  tick_reset.attach(INTERVAL_RESET, reset_daily);
  tick_calories.attach_ms(INTERVAL_CALORIES, call_kalorije_poraba);
}

void loop() {
  Blynk.run();
}
