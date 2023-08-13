#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <CircularBuffer.h>
#include <TinyGPSPlus.h>

//! Implementar Lora

//Defines
#define LOG_MESSAGE false
#define LOG_ERROR true
#define DEBUG_VERBOSE 2
#define SEALEVELPRESSURE_HPA (1018.0)

//Toggle Debug Serial Logs
#define DEBUG DEBUG_VERBOSE

//Fatal Errors Ids
#define BMP_ERROR           1
#define SD_MOUNT_ERROR      2
#define NO_SD_ERROR         3
#define DIR_CREATION_ERROR  4
#define FILE_CREATION_ERROR 5
#define FILE_WRITE_ERROR    6
#define FILE_OPEN_ERROR     7

//Config
#define ROCKET_GOAL             500.0 //m
//definir valor da altitude em que o foguete tem que passar para começar a contar o tempo
#define FLIGHT_START_ALTITUDE   10.0  //m
#define FLIGHT_START_AFTER      11000 //ms
#define RECOVERY_DEPLOY_ALTITUDE  400.0 //m
#define RECOVERY_PIN            4     //same as old servo (D4)
//Files
#define ROOT_DIR           "/"
#define FOLDER_NAME        "flight_record"  //ROOT_DIR + "/" + FOLDER_NAME
#define DATA_FILE_NAME     "data"           //.csv
#define ALTITUDE_FILE_NAME "altitude"       //.csv
#define LOG_FILE_NAME      "log"            //.csv

//BUZZER
#define BUZZER_PIN              34  //Pin
#define BUZZER_BEEP_TIME        100 //ms
#define BUZZER_BEEP_FREQUENCY   400 //Hz
#define BUZZER_ERROR_FREQUENCY  550 //Hz

//Timers
#define DATA_INTERVAL_TIME      500 //ms
#define ALTITUDE_INTERVAL_DELAY 200 //ms

//Variables
bool flightStarted = false;
bool goalReached = false;
bool recovering = false;

unsigned long flightStarted_Time_offset = 0;

unsigned long current_time = 0;
float altitudeOffset = 0;

float apogee = 0;

//BPM
float current_altitude;
float current_temperature;
float current_pressure;
//Acelerometer
float current_roll;
float current_yaw;
float current_pitch;
float current_X_acceleration;
float current_Y_acceleration;
float current_Z_acceleration;
float current_GyroX;
float current_GyroY;
float current_GyroZ;
//GPS
char latitude[12];
char longitude[12];

//Timers
unsigned long dataInterval_time = 0;
unsigned long altitudeInterval_time = 0;

//Objects
Adafruit_BMP280 bmp;
TinyGPSPlus gps;

File root;
String logFilePath;
String dataFilePath;
String altitudeFilePath;
CircularBuffer <float, 6> altitudeBuffer;

//Functions
void beep(int times = 1, int beeps_interval = 0, int time = BUZZER_BEEP_TIME);
void Log(const String& Message, bool isError = false, bool append = true);
void Error(int e);

void initSensors();

void initDir();
void createDir(fs::FS& fs, const char* path);
bool createFile(fs::FS& fs, const char* path, const char* message);
void appendToFile(fs::FS& fs, const char* path, const char* message);
void SD_SaveFlightData();
void SD_SaveAltitude();

void get_Pressure();
void get_Altitude();
void get_Temperature();
void get_Coordenates();
void get_Position();

void set_AltitudeOffset();

void handle_FlightStart();
void handleFlight();
void handle_Apogee();
void handle_Recovery();
bool isFalling();
void activateRecovery();

void handle_SerialLogData();

void setup() {
#if DEBUG
  Serial.begin(115200); //INICIALIZA A SERIAL
#endif
  Log("Setup start");

  Serial2.begin(9600, SERIAL_8N1);

  pinMode(RECOVERY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  beep(3, 100);
  delay(1000);

  initSensors();
  beep();
  delay(500);

  set_AltitudeOffset();
  beep();
  delay(500);

  initDir();
  beep(5, 75);

  Log("Setup end");
}

void loop() {
  handle_FlightStart();

  //Data Interval
  if (millis() - dataInterval_time >= DATA_INTERVAL_TIME) {
    get_Coordenates();
    get_Position();
    get_Temperature();
    get_Pressure();

    if (flightStarted) {
      SD_SaveFlightData();
    }

    handle_SerialLogData();

    dataInterval_time = millis();
  }

  //Altitude Interval
  if (flightStarted && millis() - altitudeInterval_time >= ALTITUDE_INTERVAL_DELAY) {
    get_Altitude();
    SD_SaveAltitude();
    altitudeInterval_time = millis();
  }
}

//Functions
//Sensors
void initSensors() {
//INICIALIZAÇÃO SENSORES
//BMP
  if (!bmp.begin(0x76)) {
    Log(F("Sensor BMP280 não foi identificado! Verifique as conexões."), LOG_ERROR);
    Error(BMP_ERROR);
  }

  //SD
  if (!SD.begin(5)) {
    Log("Card Mount Failed", LOG_ERROR);
    Error(SD_MOUNT_ERROR);
  }
  if (SD.cardType() == CARD_NONE) {
    Log("No SD card attached", LOG_ERROR);
    Error(NO_SD_ERROR);
  }

  //MPU
  //initializeMpuModule();
  /*Wire.begin();
    if (!mpu.setup(0x68)) {
    Serial.println("Mpu not working");
    }
    else {
    mpu.verbose(true);
    mpu.calibrateAccelGyro();
    print_calibration();
    mpu.verbose(false);
    }*/

}

//Check if Rocket started to Rising
void handle_FlightStart() {
  if (!flightStarted) {
    int cont = 0;
    using index_t = decltype(altitudeBuffer)::index_t;

    for (index_t i = 0; i < altitudeBuffer.size() - 3; i++) {
      if (altitudeBuffer[i] > FLIGHT_START_ALTITUDE) {
        cont += 1;
      }
      if (cont >= 3) {
        flightStarted = true;
        flightStarted_Time_offset = millis();
        Log("Flight Started");
      }
    }
  }
  else {
    handleFlight();
  }
}

void handleFlight() {
  current_time = millis() - flightStarted_Time_offset;
  handle_Apogee();
  handle_Recovery();
}

void handle_Apogee() {
  int cont = 0;
  using index_t = decltype(altitudeBuffer)::index_t;

  for (index_t i = 0; i < altitudeBuffer.size() - 3; i++) {
    if (altitudeBuffer[i] > ROCKET_GOAL) {
      cont += 1;
    }
    if (altitudeBuffer[i] > apogee) {
      apogee = altitudeBuffer[i];
    }
    if (cont >= 3 && goalReached == false) {
      Log(String(ROCKET_GOAL) + "m Goal Reached");
      goalReached = true;
    }
  }
}

void handle_Recovery() {
  if (flightStarted) {
    if (!recovering) {
      if (current_time > FLIGHT_START_AFTER) {
        recovering = true;
      }

      if (isFalling()) {
        if (apogee <= RECOVERY_DEPLOY_ALTITUDE) {
          recovering = true;
        }
        else {

          int cont = 0;
          using index_t = decltype(altitudeBuffer)::index_t;
          for (index_t i = 0; i < altitudeBuffer.size() - 3; i++) {
            if (altitudeBuffer[i] < RECOVERY_DEPLOY_ALTITUDE) {
              cont += 1;
            }
            if (cont >= 3) {
              recovering = true;
            }
          }
        }
      }
      else {
        recovering = true;
      }

      if (recovering) {
        Log("Recovery Activated");
      }
    }

    if (recovering) {
      activateRecovery();
    }
  }
}

void activateRecovery() {
}

//Get Altitude and saves on Buffer
void get_Altitude() {
  float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA) - altitudeOffset; // colocar offset

  if (alt < 0) {
    Log("Error reading altitude: " + String(alt));
    alt = 0;
  }

  altitudeBuffer.push(alt);
  current_altitude = alt;
}

//Get Temperature
void get_Temperature() {
  float temp = bmp.readTemperature();
  current_temperature = temp;
}

//Get Pressure
void get_Pressure() {
  current_pressure = bmp.readPressure() / 100;
}

//Get Positon (Accelerometer)
//Aceleration/Rotation
void get_Position() {
   /*if (mpu.update())
      {
      current_roll = get_Roll();
      Serial.print(current_roll);
      Serial.print(", ");

      current_pitch = get_Pitch();
      Serial.print(current_pitch);
      Serial.print(", ");

      current_yaw = get_Yaw();
      Serial.print(current_yaw);
      Serial.print(", ");

      current_X_acceleration = get_AccX();
      Serial.print(current_X_acceleration);
      Serial.print(", ");

      current_Y_acceleration = get_AccY();
      Serial.print(current_Y_acceleration);
      Serial.print(", ");

      current_Z_acceleration = get_AccZ();
      Serial.print(current_Z_acceleration);
      Serial.print(", ");

      current_GyroX = get_GyroX();
      Serial.print(current_GyroX);
      Serial.print(", ");

      current_GyroY = get_GyroY();
      Serial.print(current_GyroY);
      Serial.print(", ");

      current_GyroZ = get_GyroZ();
      Serial.print(current_GyroZ);
      Serial.print(", ");
      }*/
}

//Get Location (GPS)
void get_Coordenates() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      gps.encode(Serial2.read());
      if (gps.location.isValid()) {
        dtostrf(gps.location.lng(), 12, 6, longitude);
        dtostrf(gps.location.lat(), 12, 6, latitude);
      }
      else {
        Log(F("INVALID LOCATION!"));
      }
    }
  }
}

void set_AltitudeOffset() {
  Log("Definindo Altitude Atual:");
  for (int i = 0; i < 10; i++) {
    altitudeOffset += (0.1 * bmp.readAltitude(SEALEVELPRESSURE_HPA));
    delay(50);
  }
  Log("Altitude " + String(altitudeOffset) + " m");
  return;
}

bool isFalling() {
  float delta = 1.0;
  //A variavel serve para minimizar os erros por ruidos permitindo ate uma leitura que contraria a logica da funcao
  float b = altitudeBuffer[altitudeBuffer.size() - 1];
  float a = altitudeBuffer[(altitudeBuffer.size() - 2)];

  if ((b > (a - delta)) && (b < (a + delta))) {
    //Serial.println("  Variação dentro do erro  ");
    return false;
  }

  using index_t = decltype(altitudeBuffer)::index_t;
  for (index_t i = 0; i < altitudeBuffer.size(); i++) {
    if (altitudeBuffer[i] < altitudeBuffer[i + 1]) {
      return false;
    }
  }
  return true;
}

void initDir() {
  // Para criar um novo directorio e arquivo toda vez que ligar a placa
  // mantendo os dados antigos e salvando separado
  Log("Iniciando Diretorio");
  root = SD.open(ROOT_DIR);
  int nDir = 0;

  //Procura a quantidade de diretorios
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      // no more files
      break;
    }
    else {
      nDir++;
    }
    entry.close();
    delay(20);
  }
#if DEBUG == DEBUG_VERBOSE
  Log("Quantidade de Diretorios Existentes: " + String(nDir));
#endif

  //Cria directorio para os dados
  String Folder = String(ROOT_DIR) + String(FOLDER_NAME) + "_" + String(nDir);
  createDir(SD, Folder.c_str());

  //Cria file para dados dos sensores
  String filePath = Folder + DATA_FILE_NAME + ".csv";
  createFile(SD, filePath.c_str(), "Tempo,Altitude,Temperatura,Pressao,latitude,longitude\n");
  dataFilePath = filePath;

  //Cria file para altitude
  filePath = Folder + ALTITUDE_FILE_NAME + ".csv";
  createFile(SD, filePath.c_str(), "Altitude\n");
  altitudeFilePath = filePath;

  //Cria file para log
  filePath = Folder + LOG_FILE_NAME + ".csv";
  createFile(SD, filePath.c_str(), "Tempo,Tipo,Mensagem\n");
  logFilePath = filePath;
}

void createDir(fs::FS& fs, const char* path) {
  Log("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Log(String(path) + " Created");
  }
  else {
    Log(String(path) + " Create Failed", LOG_ERROR);
    Error(DIR_CREATION_ERROR);
  }
}

bool createFile(fs::FS& fs, const char* path, const char* message) {

  Log("Creating file " + String(path));
  File file = fs.open(path, FILE_WRITE);

  if (!file) {
    Log("Failed To Create File " + String(path), LOG_ERROR);
    Error(FILE_CREATION_ERROR);
    return false;
  }

  if (!file.print(message)) {
    Log("Failed To Write On File " + String(path), LOG_ERROR);
    Error(FILE_WRITE_ERROR);
    return false;
  }

  file.close();
  Log("File " + String(path) + " Created");

  return true;
}

void appendToFile(fs::FS& fs, const char* path, const char* message) {
#if DEBUG == DEBUG_VERBOSE
  Log("Appending to file: " + String(path), LOG_MESSAGE, false);
#endif

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Log("Failed to open file ''" + String(path) + "'' for appending", LOG_ERROR);
    Error(FILE_OPEN_ERROR);
    return;
  }
  if (!file.print(message)) {
    Log("Append Failed");
    Error(FILE_WRITE_ERROR);
  }

#if DEBUG == DEBUG_VERBOSE
  Log("Message Appended", LOG_MESSAGE, false);
#endif
  file.close();
}

//Serial Print Data
void handle_SerialLogData() {
#if DEBUG
  Serial.print("Temperatura:");
  Serial.println(current_temperature);
  Serial.print("Altitude: ");
  Serial.println(current_altitude);
  Serial.print("Latitude: ");
  Serial.print(latitude);
  Serial.print(F(", "));
  Serial.print("Longitude: ");
  Serial.print(longitude);
  Serial.println(F(", "));
#endif
}

//Save Data on SD 
void SD_SaveFlightData() {
  String dataMessage =
    String(current_time) + ","
    + String(current_altitude) + ","
    + String(current_temperature) + ","
    + String(current_pressure) + ","
    + String(latitude) + ","
    + String(longitude) + ","
    + "\n";
  appendToFile(SD, dataFilePath.c_str(), dataMessage.c_str());
}

//Save Altitude on SD
void SD_SaveAltitude() {
  String dataMessage =
    String(current_time) + ","
    + String(current_altitude)
    + "\n";
  appendToFile(SD, altitudeFilePath.c_str(), dataMessage.c_str());
}

void beep(int times, int beeps_interval, int time) {
  for (size_t i = 0; i < times; i++)
  {
    tone(BUZZER_PIN, BUZZER_BEEP_FREQUENCY, time);
    delay(beeps_interval + time);
  }
}

void Log(const String& Message, bool isError, bool append) {
  String logMessage =
    String(millis()) + ","
    + (isError ? "Error" : "Info") + ","
    + Message
    + "\n";

  if (append) {
    appendToFile(SD, logFilePath.c_str(), logMessage.c_str());
  }

#if DEBUG
  Serial.println(Message);
#endif

}

void Error(int e) {
  int errorDelay = 800;

  while (true) {
    tone(BUZZER_PIN, BUZZER_ERROR_FREQUENCY, errorDelay * 2);
    delay(errorDelay * 4);

    for (int i = 0; i < e; i++) {
      tone(BUZZER_PIN, BUZZER_ERROR_FREQUENCY, errorDelay);
      delay(errorDelay * 2);
    }
    delay(errorDelay * 4);
  }

}
