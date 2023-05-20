#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Ticker.h>
#include <float.h>
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
#define RATE 1
#define I2C_ADD_IO1 32
#define ACC_OUT 59
#define BLYNK_PRINT Serial
#define HISTORY_SIZE 50

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
int32_t SMOOTHING_WINDOW = 1;
float history_x[HISTORY_SIZE];
float history_y[HISTORY_SIZE];
float history_z[HISTORY_SIZE];
int32_t calorie_step_counter = 0;
float total_calories_burned = 0.0;
int32_t step_counter = 0;
float delilnik = 16384.0f;
float acc_x_calib = 0.0f;
float acc_y_calib = 0.0f;
float acc_z_calib = 0.0f;
float prev_acc_x = 0.0f;
float prev_acc_y = 0.0f;
float prev_acc_z = 0.0f;
int32_t max_axis = -1;
// visok threshold na zacetku, da ne zaznamo korakov v mirovanju
float threshold = 1000000.0f;
int32_t counts_since_last_step = 0;
int32_t daily_steps = 5000;
int32_t daily_calories = 2500;
// izmisljeni parametri
float weight = 75.0;
int32_t height = 178;
unsigned long cas_prejsnjega_koraka = 0;
float max_value = FLT_MIN;
float min_value = FLT_MAX;

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

  if (nmb_of_steps == calorie_step_counter) {
    // stationary
    current_calories_burned = weight / 1800;

    Serial.print("Publishing message for 'Current calories': ");
    Serial.println(current_calories_burned);
    Blynk.virtualWrite(V6, current_calories_burned);

  } else if (nmb_of_steps > calorie_step_counter) {
    // moving
    int32_t nmb_of_steps_in_last_2s = nmb_of_steps - calorie_step_counter;

    float stride = get_stride(nmb_of_steps_in_last_2s, height);

    float speed = nmb_of_steps_in_last_2s * stride;
    current_calories_burned = speed * weight / 400;

    Serial.print("Publishing message for 'Current calories': ");
    Serial.println(current_calories_burned);
    Blynk.virtualWrite(V6, current_calories_burned);

    calorie_step_counter = nmb_of_steps;
  }

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

void detectStep() {
  unsigned long cas_koraka = millis();
  if (cas_prejsnjega_koraka != 0) {
    unsigned long razlika_korakov = cas_koraka - cas_prejsnjega_koraka;
      if (razlika_korakov >= 200) {
      // todo: upostevaj se cas med obema korakom (periodicnost!!!)
        Serial.print("STEP DETECTED");
        Serial.println("");
        step_counter++;
        counts_since_last_step = 0;
        cas_prejsnjega_koraka = cas_koraka;
  
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
  } else {
    cas_prejsnjega_koraka = cas_koraka;
  }
}

int32_t preveriNajvecjoOs(float s_table_x[HISTORY_SIZE], float s_table_y[HISTORY_SIZE], float s_table_z[HISTORY_SIZE]) {
  // 0 = x os
  // 1 = y os
  // 2 = z os
  // initialize
  float max_x = 0.0f;
  float max_y = 0.0f;
  float max_z = 0.0f;
  // begin count
  for (int i = 0; i < HISTORY_SIZE; i++) {
    if (s_table_x[i] > max_x) {
      max_x = s_table_x[i];
    }
    if (s_table_y[i] > max_y) {
      max_y = s_table_y[i];
    }
    if (s_table_z[i] > max_z) {
      max_z = s_table_z[i];
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
      table_x += (int8_t) Wire.read();
      if (i % 2 == 0) {
        table_x = table_x << 8;
      }
    } else if (i < 4) {
      table_y += (int8_t) Wire.read();
      if (i % 2 == 0) {
        table_y = table_y << 8;
      }
    } else {
      table_z += (int8_t) Wire.read();
      if (i % 2 == 0) {
        table_z = table_z << 8;
      }
    }
  }

  // izracun pospeska
  acc_x += ((table_x / delilnik) - acc_x_calib);
  acc_y += ((table_y / delilnik) - acc_y_calib);
  acc_z += ((table_z / delilnik) - acc_z_calib);

  if (count % RATE == 0) {
    // Izpišemo in shranimo pospesek
    Serial.print("ACC_X: X= ");
    Serial.print(acc_x);
    Serial.println("");
    Serial.print("ACC_Y: Y= ");
    Serial.print(acc_y);
    Serial.println("");
    Serial.print("ACC_Z: Z= ");
    Serial.print(acc_z);
    Serial.println("");

    history_x[count] = acc_x / RATE;
    history_y[count] = acc_y / RATE;
    history_z[count] = acc_z / RATE;

    Serial.print("maxAxis = ");
    Serial.println(max_axis);
    Serial.print("Threshold = ");
    Serial.println(threshold);
    if (max_axis != -1) {
      // prvih 50 meritev ne detektiramo, ker se ne vemo, v kateri osi imamo maximum
      if (max_axis == 0) {
        // x os
        Serial.print("current = ");
        Serial.println(acc_x);
        Serial.print("previous = ");
        Serial.println(prev_acc_x);
        //current < previous && previous > threshold && current < threshold && abs(previous - current) > 0.05
        if (acc_x < prev_acc_x && prev_acc_x > threshold && acc_x < threshold && abs(max_value - min_value) > 0.05) {
          // step detected
          detectStep();
        }
      } else if (max_axis == 1) {
        // y os
        Serial.print("current = ");
        Serial.println(acc_y);
        Serial.print("previous = ");
        Serial.println(prev_acc_y);
        if (acc_y < prev_acc_y && prev_acc_y > threshold && acc_y < threshold && abs(max_value - min_value) > 0.05) {
          // step detected
          
          detectStep();
        }
      } else if (max_axis == 2) {
        Serial.print("current = ");
        Serial.println(acc_z);
        Serial.print("previous = ");
        Serial.println(prev_acc_z);
        if (acc_z < prev_acc_z && prev_acc_z > threshold && acc_z < threshold && abs(max_value - min_value) > 0.05) {
          // step detected
          detectStep();
        }
      }
    }

    // števec
    count = count + 1;
    counts_since_last_step++;

    // nastavi prejsnje vrednosti
    prev_acc_x = acc_x;
    prev_acc_y = acc_y;
    prev_acc_z = acc_z;

    // reset vrednosti
    acc_x = 0;
    acc_y = 0;
    acc_z = 0;
  }

  if (count == HISTORY_SIZE) {
    // izracun meje
    Serial.print("START DETECTION STEP");
    Serial.println("");
    // glajenje (vzemi prejšnji, trenutni in naslednji measurment in vstavi povprečje)
    float smoothed_history_x[HISTORY_SIZE];
    float smoothed_history_y[HISTORY_SIZE];
    float smoothed_history_z[HISTORY_SIZE];
    for (int i = 0; i < HISTORY_SIZE; i++) {
      float summed_x = 0.0f;
      float summed_y = 0.0f;
      float summed_z = 0.0f;
      int32_t number_summed_x = 0;
      int32_t number_summed_y = 0;
      int32_t number_summed_z = 0;
      int32_t start_index = i - SMOOTHING_WINDOW;
      for (int smoothing_index = i - SMOOTHING_WINDOW; smoothing_index <= i + SMOOTHING_WINDOW; smoothing_index++) {
        if (smoothing_index < 0 || smoothing_index >= HISTORY_SIZE) {
          continue;
        }
        summed_x += history_x[smoothing_index];
        number_summed_x++;
        summed_y += history_y[smoothing_index];
        number_summed_y++;
        summed_z += history_z[smoothing_index];
        number_summed_z++;
      }
      float average_x = summed_x / (number_summed_x * 1.0f);
      float average_y = summed_y / (number_summed_y * 1.0f);
      float average_z = summed_z / (number_summed_z * 1.0f);
      smoothed_history_x[i] = average_x;
      smoothed_history_y[i] = average_y;
      smoothed_history_z[i] = average_z;
    }

    // TODO: dinamično nastavljanje meje
    max_axis = preveriNajvecjoOs(smoothed_history_x, smoothed_history_y, smoothed_history_z);
    Serial.print("Najvecja os = ");
    Serial.println(max_axis);
    float maxHistory[HISTORY_SIZE];
    if (max_axis == 0) {
      for (int i = 0; i < HISTORY_SIZE; i++) {
        maxHistory[i] = smoothed_history_x[i];
      }
    } else if (max_axis == 1) {
      for (int i = 0; i < HISTORY_SIZE; i++) {
        maxHistory[i] = smoothed_history_y[i];
      }
    } else if (max_axis == 2) {
      for (int i = 0; i < HISTORY_SIZE; i++) {
        maxHistory[i] = smoothed_history_z[i];
      }
    }
    // get max and min values
    max_value = FLT_MIN;
    min_value = FLT_MAX;
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
    /*
    for (int i = 1; i < HISTORY_SIZE; i++) {
      float previous = maxHistory[i - 1];
      float current = maxHistory[i];
      if (current < previous && previous > threshold && current < threshold && abs(previous - current) > 0.05) {
        unsigned long cas_koraka = millis();
        if (cas_prejsnjega_koraka != 0) {
          unsigned long razlika_korakov = cas_koraka - cas_prejsnjega_koraka;
          if (razlika_korakov >= 200) {
            // todo: upostevaj se cas med obema korakom (periodicnost!!!)
            Serial.print("STEP DETECTED");
            Serial.println("");
            Serial.print("current = ");
            Serial.println(current);
            Serial.print("previous = ");
            Serial.println(previous);
            step_counter++;
            counts_since_last_step = 0;
            cas_prejsnjega_koraka = cas_koraka;

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
        } else {
          cas_prejsnjega_koraka = cas_koraka;
        }
      }
    }
    */

    // set new threshold
    threshold = (max_value + min_value) / 2;

    // reset counter
    count = 0;
  }


  // digitalWrite(PIN_LED, 1);
}

void reset_daily() {
  Serial.println("Daily steps and calories reset");
  step_counter = 0;
  total_calories_burned = 0.0;
}

void init_blynk() {
  Serial.println("Resetting all values");

  // resetiramo vrednosti na nadzorni plošči
  Blynk.virtualWrite(V3, 0);
  Blynk.virtualWrite(V4, 0.0);
  Blynk.virtualWrite(V5, "Daily steps goal not yet reached!");
  Blynk.virtualWrite(V6, 0.0);
  Blynk.virtualWrite(V9, "Daily calories goal not yet reached!");

  // prvič ročno preberemo višino in težo
  Blynk.syncVirtual(V7);
  Blynk.syncVirtual(V8);
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
        table_x += (int8_t) Wire.read();
        if (i % 2 == 0) {
          table_x = table_x << 8;
        }
      } else if (i < 4) {
        table_y += (int8_t) Wire.read();
        if (i % 2 == 0) {
          table_y = table_y << 8;
        }
      } else {
        table_z += (int8_t) Wire.read();
        if (i % 2 == 0) {
          table_z = table_z << 8;
        }
      }
    }

    acc_x_calib += (table_x / delilnik);
    acc_y_calib += (table_y / delilnik);
    acc_z_calib += (table_z / delilnik);

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
  Serial.println();

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

  // na register 107 pošlji vrednost 128
  Wire.beginTransmission(I2C_ADD_MPU);
  Wire.write(107);
  Wire.write(128);
  Wire.endTransmission();
  delay(100);

  // na register 28 poslji 0
  Wire.beginTransmission(I2C_ADD_MPU);
  Wire.write(28);
  Wire.write(0);
  Wire.endTransmission();
  delay(100);

  acc_calib();
  init_blynk();
  tick_beri.attach_ms(INTERVAL_BERI, beri_podatke);
  tick_reset.attach(INTERVAL_RESET, reset_daily);
  tick_calories.attach_ms(INTERVAL_CALORIES, call_kalorije_poraba);
}

void loop() {
  Blynk.run();
}
