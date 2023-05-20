#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Ticker.h>
#include <float.h>
#include <BlynkSimpleEsp8266.h>

#define INTERVAL_BERI 100
#define INTERVAL_RESET 86400
#define INTERVAL_CALORIES 2000
#define I2C_ADD_MPU 104
#define TABLE_SIZE_MPU 7
#define RATE 1
#define ACC_OUT 59
#define HISTORY_SIZE 50

// podatki za vpis v WIFI
const char *wifi_ssid = "Jansa-G";
const char *wifi_password = "12jansa34";
// podatki za vpis v blynk
const char *blynk_template_id = "TMPL4bPFAc-b0";
const char *blynk_template_name = "rs2023";
const char *blynk_auth_token = "DZcBJHgpw8rjOJGr0kz9MuMi8C415dlp";

// tickerji
Ticker tick_beri;
Ticker tick_reset;
Ticker tick_calories;
// tabele za izračun pospeska
int32_t table_x[TABLE_SIZE_MPU];
int32_t table_y[TABLE_SIZE_MPU];
int32_t table_z[TABLE_SIZE_MPU];
// zgodovina meritev pospeska
float history_x[HISTORY_SIZE];
float history_y[HISTORY_SIZE];
float history_z[HISTORY_SIZE];
// velikost okna za glajanje meritev ob izracuna meje
int32_t SMOOTHING_WINDOW = 1;
float total_calories_burned = 0.0;
float delilnik = 16384.0f;
// kalibracijske vrednosti pospeskometra v vsaki dimenziji
float acc_x_calib = 0.0f;
float acc_y_calib = 0.0f;
float acc_z_calib = 0.0f;
// prejsnje vrednosti meritev pospeskov v vsaki dimenziji
float prev_acc_x = 0.0f;
float prev_acc_y = 0.0f;
float prev_acc_z = 0.0f;
// identifikacija osi, kjer je prislo do najvecje meritve (0 = x os, 1 = y os, 2 = z os, -1 = nedefinirano)
int32_t max_axis = -1;
// dnevni cilj stevila korakov
int32_t daily_steps = 5000;
// dnevni cilj porabljenih kalorij
int32_t daily_calories = 2500;
// stevec korakov
int32_t step_counter = 0;
// stevec korakov za namen racunanja kalorij
int32_t calorie_step_counter = 0;
// meja za detekcijo koraka - visoka meja na zacetku, da ne zaznamo korakov v mirovanju
float threshold = 1000000.0f;
// stetje stevila meritev
uint32_t count = 0;

// zacetni parametri teze in visine uporabnika, ob delovanju te podatke pridobimo iz Blynk
float weight = 75.0;
int32_t height = 178;
// skupna prehojena razdalja
float total_distance = 0.0;
// cas v milisekundah od zadnjega zaznanega koraka
unsigned long previous_step_time = 0;
// najvisja in najnizja zabelezena vrednost v zadnjih 50 vzorcih
float max_value = FLT_MIN;
float min_value = FLT_MAX;

// izracun dolzine korak iz uporabnikove visine (povzeto po tabeli iz vira)
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

// ticker funkcija za klic izracuna porabe kalorij glede na stevilo opravljenih korakov
void call_kalorije_poraba() {
  kalorije_poraba(step_counter);
}

void kalorije_poraba(int32_t nmb_of_steps) {
  // 2 opcije: stacionarno ali premikajoce
  // stacionarno: C = 1 * weight / 1800
  // premikajoce (osvezitev na 2 sekundi): C = speed * weight / 400
  // hitrost = (# korakov / 2s) * (dolzina koraka / 2s)
  // dolzina koraka (stride) = tabela korakov na 2s --> visina / 5, 4, 3, 2, 1.2, Visina, 1.2 * Visina
  // visina je v cm, teza v kg
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

    float speed = nmb_of_steps_in_last_2s * ((stride / 2) / 100);
    Serial.print("speed");
    Serial.println(speed);
    Serial.print("stride");
    Serial.println(stride);
    current_calories_burned = speed * (weight / 400.0);

    Serial.print("Publishing message for 'Current calories': ");
    Serial.println(current_calories_burned);
    Blynk.virtualWrite(V6, current_calories_burned);

    calorie_step_counter = nmb_of_steps;
  }

  // todo total_calories_burned izračun
  total_calories_burned += current_calories_burned;

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

  // Distance:
  // Get average length of a step based on height
  float step_length = height * 0.4; 
  float distance_current = nmb_of_steps_in_last_2s * step_length;

  total_distance += distance_current;
  Serial.print("Publishing message for 'Total distance': ");
  Serial.println(total_distance);
  Blynk.virtualWrite(V10, total_distance / 100000);

}

// funckija, ki se klice ob detekciji koraka
void detectStep() {
  // pridobimo cas, ko smo zaznali korak
  unsigned long step_time = millis();
  // ce smo zaznali sploh 1. korak, ga samo pristejemo in nastavimo kot cas zadnjega koraka
  if (previous_step_time != 0) {
    // zaznaj korak le, ce je od prejsnjega minilo vec kot 0.2 sekundi
    unsigned long step_difference = step_time - previous_step_time;
    if (step_difference >= 200) {
      Serial.println("STEP DETECTED");
      // povecaj stevec in nastavi cas zadnjega koraka na cas tega koraka
      step_counter++;
      previous_step_time = step_time;

      // poslji na Blynk popravljeno vrednost stevca korakov
      Serial.print("Publishing message for 'Step counter': ");
      Serial.println(step_counter);
      Blynk.virtualWrite(V3, step_counter);

      // Preveri, ali je bil dosežen dnevni cilj korakov (5000 korakov) in poslji na Blynk
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
    Serial.println("STEP DETECTED");
    // povecaj stevec in nastavi cas zadnjega koraka na cas tega koraka
    step_counter++;
    previous_step_time = step_time;

    // poslji na Blynk popravljeno vrednost stevca korakov
    Serial.print("Publishing message for 'Step counter': ");
    Serial.println(step_counter);
    Blynk.virtualWrite(V3, step_counter);

    // Preveri, ali je bil dosežen dnevni cilj korakov (5000 korakov) in poslji na Blynk
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

// vrni os, v kateri je bila zaznana najvecja vrednost
int32_t checkAxisHighestPeak(float s_table_x[HISTORY_SIZE], float s_table_y[HISTORY_SIZE], float s_table_z[HISTORY_SIZE]) {
  // 0 = x os
  // 1 = y os
  // 2 = z os
  // inicializacija
  float max_x = 0.0f;
  float max_y = 0.0f;
  float max_z = 0.0f;
  // iskanje najvecji vrednosti v vsaki dimenziji
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
  // primerjava in vracanje najvecje vrednosti
  if (max_x > max_y && max_x > max_z) {
    return 0;
  } else if (max_y > max_x && max_y > max_z) {
    return 1;
  }
  return 2;
}

// branje podatkov pospeskometra in detekcija korakov
void beri_podatke() {
  // inicializacija vrednosti tabel za branje in vrednosti za shranjevanje pospeskov
  float acc_x = 0.0f;
  float acc_y = 0.0f;
  float acc_z = 0.0f;
  int32_t table_x = 0;
  int32_t table_y = 0;
  int32_t table_z = 0;

  //branje pospeska iz registrov
  Wire.beginTransmission(I2C_ADD_MPU);
  Wire.write(ACC_OUT);
  Wire.endTransmission();

  // preberemo vseh 6 bajtov (x, y in z os)
  // ce beremo 1. bit iz posamezne osi, naredimo premik za 8 bitov, ker ta vrednost predstavlja zgornji bajt
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

  // izracun pospeska z upostevanjem kalibracije
  acc_x += ((table_x / delilnik) - acc_x_calib);
  acc_y += ((table_y / delilnik) - acc_y_calib);
  acc_z += ((table_z / delilnik) - acc_z_calib);

  // merjenje izvajamo ob dolocenem intervalu
  if (count % RATE == 0) {
    // izracunamo in shranimo izmerjene vrednosti
    history_x[count] = acc_x / RATE;
    history_y[count] = acc_y / RATE;
    history_z[count] = acc_z / RATE;

    // detekcijo korakov ne izvajamo ob 1. intervalu (50 vzorcev), saj se nimamo informacije, v kateri osi imamo najvisji premik 
    if (max_axis != -1) {
      if (max_axis == 0) {
        // x os
        if (acc_x < prev_acc_x && prev_acc_x > threshold && acc_x < threshold && abs(max_value - min_value) > 0.05) {
          detectStep();
        }
      } else if (max_axis == 1) {
        // y os
        if (acc_y < prev_acc_y && prev_acc_y > threshold && acc_y < threshold && abs(max_value - min_value) > 0.05) {
          detectStep();
        }
      } else if (max_axis == 2) {
        // z os
        if (acc_z < prev_acc_z && prev_acc_z > threshold && acc_z < threshold && abs(max_value - min_value) > 0.05) {
          detectStep();
        }
      }
    }

    // povecanje stevca meritev
    count++;

    // nastavi trenutne vrednosti pospeska kot prejsnje vrednosti
    prev_acc_x = acc_x;
    prev_acc_y = acc_y;
    prev_acc_z = acc_z;

    // reset vrednosti trenutnega pospeska
    acc_x = 0;
    acc_y = 0;
    acc_z = 0;
  }

  // na vsakih HISTORY_SIZE (50) vzorcev, dolocimo novo mejo za detekcijo korakov
  if (count == HISTORY_SIZE) {
    // glajenje (vzemi SMOOTHING WINDOW (1) podatkov in trenutni podatek in vstavimo povprečje)
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
      for (int smoothing_index = i - SMOOTHING_WINDOW; smoothing_index <= i + SMOOTHING_WINDOW; smoothing_index++) {
        // ce je trenutni indeks izven meja, ga ignoriramo
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
      // izracun in shranjevanje povprecja
      float average_x = summed_x / (number_summed_x * 1.0f);
      float average_y = summed_y / (number_summed_y * 1.0f);
      float average_z = summed_z / (number_summed_z * 1.0f);
      smoothed_history_x[i] = average_x;
      smoothed_history_y[i] = average_y;
      smoothed_history_z[i] = average_z;
    }

    // najdemo dimenzijo, v kateri smo zaznali najvisjo vrednost
    max_axis = checkAxisHighestPeak(smoothed_history_x, smoothed_history_y, smoothed_history_z);
    float maxHistory[HISTORY_SIZE];
    // shranimo ustrezno tabelo v maxHistory
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
    // posodobitev najvecje in najmanjse vrednosti v intervalu
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

    // izracun nove meje
    threshold = (max_value + min_value) / 2;

    // resetiranje stevca
    count = 0;
  }
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
  Blynk.virtualWrite(V10, 0.0);

  // prvič ročno preberemo višino in težo
  Blynk.syncVirtual(V7);
  Blynk.syncVirtual(V8);
}

// kalibracija vrednosti pospeskometra
void acc_calib() {
  delay(1000);

  int rate = 10;
  int samp = 50;
  int32_t table_x = 0;
  int32_t table_y = 0;
  int32_t table_z = 0;

  for (int q = 0; q < samp; q++) {
    // branje vrednosti pospeskometra
    Wire.beginTransmission(I2C_ADD_MPU);
    Wire.write(ACC_OUT);
    Wire.endTransmission();

    // preberemo vseh 6 bajtov (x, y in z os) pospeskometra
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

  // izracun vrednosti
  acc_x_calib /= samp;
  acc_y_calib /= samp;
  acc_z_calib /= samp;

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

  // na register 107 posljemo vrednost 128
  Wire.beginTransmission(I2C_ADD_MPU);
  Wire.write(107);
  Wire.write(128);
  Wire.endTransmission();
  delay(100);

  // na register 28 posljemo vrednost 0
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
