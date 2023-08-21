#include <Arduino.h>
#include <LoRa_E32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <CircularBuffer.h>
#include <MPU9250.h> 
#include <TinyGPSPlus.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

//---- System Defines -----
#define LOG_MESSAGE false
#define LOG_ERROR true
#define DEBUG_ON true
#define DEBUG_OFF false
#define DEBUG_LORA false
#define DEBUG_VERBOSE 2
#define SEALEVELPRESSURE_HPA (1018.0)

//Fatal Errors Ids
#define BMP_ERROR           1
#define SD_MOUNT_ERROR      2
#define MPU_ERROR           3
#define NO_SD_ERROR         4
#define DIR_CREATION_ERROR  5
#define FILE_CREATION_ERROR 6
#define FILE_WRITE_ERROR    7
#define FILE_OPEN_ERROR     8

//Log Messages
#define SYSYEM_START_MESSAGE          "Avionic Board Starting"
#define SETUP_END_MESSAGE             "Setup Complete"

#define BMP_OK_MESSAGE                "BMP280 OK!" 
#define LORA_STARTED_MESSAGE          "Lora Started!"

#define INIT_DIR_MESSAGE              "Iniciando Diretorio"
#define CREATING_DIR_MESSAGE          "Criando Diretorio "
#define DIR_CREATED_MESSAGE           "Diretorio Criado" 
#define CREATING_FILE_MESSAGE         "Criando Arquivo: "
#define FILE_CREATED_MESSAGE          "Arquivo Criado: " 
#define SET_ALTTITUDE_OFFSET_MESSAGE  "Seting Altitude Offset"
#define OFFSET_ALTITUDE_MESSAGE       "Altitude Atual:"

#define BMP_ERROR_MESSAGE             "BMP280 init failed!"
#define MPU_ERROR_MESSAGE             "MPU9250 init failed!"
#define SD_MOUNT_ERROR_MESSAGE        "SD Mount init failed!" 
#define NO_SD_ERROR_MESSAGE           "NO SD"
#define ALTITUDE_ERROR_MESSAGE        "Altitude Read Error"
#define GPS_ERROR_MESSAGE             "GPS Error, Invalid Location" 
#define OPEN_FILE_ERROR_MESSAGE       "Failed to open file for appending"    
#define DIR_CREATION_ERROR_MESSAGE    "Erro ao Criar Diretorio" 
#define CREATE_FILE_ERROR_MESSAGE     "Erro ao Criar Arquivo" 
#define WRITE_FILE_ERROR_MESSAGE      "Erro ao Escrever no Arquivo"
#define LORA_SEND_ERROR_MESSAGE       "Erro ao Enviar por Lora"

#define FLIGHT_START_MESSAGE          "Flight Started"
#define GOAL_REACHED_MESSAGE          "Goal Reached"
#define RECOVERY_START_MESSAGE        "Recovery Activated"


//---- End System Defines -----

//---- Flight Configuration -----
#define ROCKET_NAME "Forte A11"

//Toggle Debug Serial Logs DEBUG_ON / DEBUG_VERBOSE / (DEBUG_OFF == DEBUG_LORA)
#define DEBUG DEBUG_ON //DEBUG_VERBOSE 

//Flight Config
#define ROCKET_GOAL                   500.0 //m
#define FLIGHT_START_ALTITUDE         10.0  //m
#define RECOVERY_DEPLOY_MIN_ALTITUDE  400.0 //m
#define RECOVER_AFTER_LAUNCH_MAX_TIME 11000 //ms

#define RECOVERY_PIN               22 // ? LED 2 Pin - Não tem Pino Definido

//Battery
#define BATTERY_MAX_VOLTAGE        7.4 //V
#define BATTERY_MIN_VOLTAGE        6.0 //V
#define BATTERY_SENSOR_PIN         35  //Pin

//Files
const char* ROOT_DIR = "/";
const char* FOLDER_NAME = "/flight_record";  //ROOT_DIR + "/" + FOLDER_NAME
const char* DATA_FILE_NAME = "/sensors";    //.csv
const char* ALTITUDE_FILE_NAME = "/altitude";       //.csv
const char* LOG_FILE_NAME = "/log"; //.csv

//Sensors I2C (MPU, BMP280)
#define I2C_SDA         21
#define I2C_SCL         22

//SD SPI
#define SD_CS           5
#define SD_MOSI         23
#define SD_CLK          18
#define SD_MISO         19

//Lora Serial0 Uart
#define LORA_RX         3 // Serial0 RX
#define LORA_TX         1 // Serial0 TX

//GPS Serial2 UART
#define GPS_RX          16 // Serial2 RX
#define GPS_TX          17 // Serial2 TX
#define GPS_BAUDRATE    4800 // Baudrate

//Lora Config
UART_BPS_RATE LORA_BAUDRATE = UART_BPS_RATE_9600;
#define LORA_AuxPin             15
#define LORA_M0                 2  
#define LORA_M1                 4  
#define LORA_ADDH               0x00
#define LORA_ADDL               0x05    
#define LORA_CHAN               0x17 
#define LORA_FixedTransmission  FT_FIXED_TRANSMISSION         //FIXED TRANSMISSION
#define LORA_WirelessWakeupTime WAKE_UP_250                   //250ms
#define LORA_IoDriveMode        IO_D_MODE_PUSH_PULLS_PULL_UPS // IO_D_MODE_PUSH_PULLS
#define LORA_TransmissionPower  POWER_20                      //20dBm
#define LORA_AirDataRate        AIR_DATA_RATE_010_24          //2.4kbps (default)
#define LORA_UartBaudRate       LORA_BAUDRATE                 //9600
#define LORA_Fec                FEC_1_ON                      //FEC
#define LORA_UaartParity        MODE_00_8N1                   //No Parity

//BUZZER
#define BUZZER_PIN              2  //Pin //?Está no IO34 que é Input Only
#define BUZZER_BEEP_TIME        100 //ms
#define BUZZER_BEEP_FREQUENCY   400 //Hz
#define BUZZER_ERROR_FREQUENCY  550 //Hz

//Timers
#define DATA_INTERVAL_TIME      500 //ms
#define ALTITUDE_INTERVAL_DELAY 200 //ms

//Var
bool flightStarted = false;
bool goalReached = false;
bool startRecovery = false;

float altitudeOffset = 0;
float apogee = 0;

unsigned long flightStarted_Time_offset = 0;

unsigned long current_time = 0;
float current_batteryCharge = 0.0;

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
float current_MagX;
float current_MagY;
float current_MagZ;

//GPS
char latitude[12];
char longitude[12];

//Timers
unsigned long dataInterval_time = 0;
unsigned long altitudeInterval_time = 0;

//Objects
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
MPU9250 mpu;

File root;
String logFilePath;
String dataFilePath;
String altitudeFilePath;
CircularBuffer <float, 6> altitudeBuffer;

LoRa_E32 LoraTransmitter(LORA_RX, LORA_TX, &Serial, LORA_AuxPin, LORA_BAUDRATE);


// fs::FS& current_fileSystem = SD;
fs::FS& current_fileSystem = SPIFFS;
//Functions
void beep(int times = 1, int beeps_interval = 0, int time = BUZZER_BEEP_TIME);
void Log(const String& Message, bool isError = false, bool append = true);
void Error(int e);

void init_Sensors();
void initLora();
void initGPS();
void initAccelerometer();
void initAltimeter();
void initSd();

void init_Dir(fs::FS& fs);
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

float get_GyroX();
float get_GyroY();
float get_GyroZ();
float get_Roll();
float get_Pitch();
float get_Yaw();
float get_AccX();
float get_AccY();
float get_AccZ();
float get_MagX();
float get_MagY();
float get_MagZ();


void get_BatteryCharge();

void set_AltitudeOffset();

void handle_FlightStart();
void handleFlight();
void handle_Apogee();
void handle_Recovery();
bool isFalling();
void activateRecovery();

void handle_SerialPrintData();
void printAccelerometerCalibration();
void LoraPrintParameter(struct Configuration configuration);
void LoraSendMessage(String message);
String normalizePath(String path);

void setup() {
#if DEBUG
  Serial.begin(115200); //INICIALIZA A SERIAL

  while (!SPIFFS.begin()) {
    Serial.println("Erro ao iniciar o SPIFFS");
    delay(1000);
  }
  Serial.println("Started");
  delay(1000);
#endif
  pinMode(RECOVERY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BATTERY_SENSOR_PIN, INPUT);

  beep(5, 75);
  delay(800);

#if !DEBUG

//Lora
  initLora();
  delay(100);
#endif

  init_Dir(current_fileSystem);
  Log(SYSYEM_START_MESSAGE);

  beep();
  delay(800);

  init_Sensors();
  beep();
  delay(500);

  set_AltitudeOffset();
  beep(3, 100);
  delay(1000);

  Log(SETUP_END_MESSAGE);
}

void loop() {
  handle_FlightStart();

  //Data Interval
  if (millis() - dataInterval_time >= DATA_INTERVAL_TIME) {
    get_Coordenates();
    get_Position();
    get_Temperature();
    get_Pressure();
    get_BatteryCharge();
    get_Altitude();

    handle_SerialPrintData();

    if (flightStarted) {
      SD_SaveFlightData();
    }

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
        Log(FLIGHT_START_MESSAGE);
      }
    }
  }
  else {
    handleFlight();
  }
}

//After Flight Started
void handleFlight() {
  current_time = millis() - flightStarted_Time_offset;
  handle_Apogee();
  handle_Recovery();
}

//Check if Apogee
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
      Log(GOAL_REACHED_MESSAGE);
      goalReached = true;
    }
  }
}

//Check if Recovery
void handle_Recovery() {
  if (!startRecovery) {

    if (current_time > RECOVER_AFTER_LAUNCH_MAX_TIME) {
      startRecovery = true;
    }

    if (isFalling()) {
      int cont = 0;
      using index_t = decltype(altitudeBuffer)::index_t;
      for (index_t i = 0; i < altitudeBuffer.size() - 3; i++) {
        if (altitudeBuffer[i] < RECOVERY_DEPLOY_MIN_ALTITUDE) {
          cont += 1;
        }
        if (cont >= 3) {
          startRecovery = true;
        }
      }
    }

    if (startRecovery) {
      Log(RECOVERY_START_MESSAGE);
    }
  }
  else {
    activateRecovery();
  }
}

void activateRecovery() {
}

void init_Sensors() {
//I2C Begin
  Wire.begin();

  // initSd();
  delay(100);
  initAltimeter();
  delay(100);
  initGPS();
  delay(100);
  initAccelerometer();
  delay(100);
}

void initSd() {
  //SD
  if (!SD.begin(5)) {
    Log(SD_MOUNT_ERROR_MESSAGE, LOG_ERROR, false);
    Error(SD_MOUNT_ERROR);
  }
  if (SD.cardType() == CARD_NONE) {
    Log(NO_SD_ERROR_MESSAGE, LOG_ERROR, false);
    Error(NO_SD_ERROR);
  }
}

void initGPS() {
  //GPS
  Serial2.begin(GPS_BAUDRATE);
}

void initAccelerometer() {
  //MPU

  if (!mpu.setup(0x68)) {  // change to your own address

    Log(MPU_ERROR_MESSAGE, LOG_ERROR);
    Error(MPU_ERROR);
  }
  printAccelerometerCalibration();

}

void initAltimeter() {
  //BMP280
  if (!bmp.begin(0x76)) {
    Log(BMP_ERROR_MESSAGE, LOG_ERROR);
    Error(BMP_ERROR);
  }
}

void initLora() {
  LoraTransmitter.begin();
  LoraTransmitter.setMode(MODE_3_SLEEP);

  // After set configuration comment set M0 and M1 to low
  // and reboot if you directly set HIGH M0 and M1 to program
  ResponseStructContainer c;
  c = LoraTransmitter.getConfiguration();
  Configuration configuration = *(Configuration*)c.data;
  LoraPrintParameter(configuration);

  configuration.ADDL = LORA_ADDL;
  configuration.ADDH = LORA_ADDH;
  configuration.CHAN = LORA_CHAN;
  configuration.OPTION.fixedTransmission = LORA_FixedTransmission;
  configuration.OPTION.wirelessWakeupTime = LORA_WirelessWakeupTime;

  configuration.OPTION.fec = LORA_Fec;
  configuration.OPTION.ioDriveMode = LORA_IoDriveMode;
  configuration.OPTION.transmissionPower = LORA_TransmissionPower;

  configuration.SPED.airDataRate = LORA_AirDataRate;
  configuration.SPED.uartBaudRate = LORA_UartBaudRate;
  configuration.SPED.uartParity = LORA_UaartParity;

  LoraTransmitter.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  LoraPrintParameter(configuration);

  LoraTransmitter.resetModule();

  delay(1000);
  LoraTransmitter.setMode(MODE_0_NORMAL);
  delay(1000);

  Log(LORA_STARTED_MESSAGE, false, false);
}

//Get Altitude and saves on Buffer
void get_Altitude() {
  float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA); // colocar offset

  if (alt < 0) {
    Log(ALTITUDE_ERROR_MESSAGE, LOG_ERROR);
    alt = 0;
  }
  float ofsseted_alt = alt - altitudeOffset;

  altitudeBuffer.push(ofsseted_alt);
  current_altitude = ofsseted_alt;
}

//Get Temperature from BMP
void get_Temperature() {
  float temp = bmp.readTemperature();
  current_temperature = temp;
}

//Get Pressure from BMP
void get_Pressure() {
  current_pressure = bmp.readPressure() / 100;
}

//Get Positon (Accelerometer)
void get_Position() {
  if (mpu.update())
  {
    current_roll = get_Roll();
    current_pitch = get_Pitch();
    current_yaw = get_Yaw();
    current_X_acceleration = get_AccX();
    current_Y_acceleration = get_AccY();
    current_Z_acceleration = get_AccZ();
    current_GyroX = get_GyroX();
    current_GyroY = get_GyroY();
    current_GyroZ = get_GyroZ();
    current_MagX = get_MagX();
    current_MagY = get_MagY();
    current_MagZ = get_MagZ();
  }
}

//Accelerometer
float get_GyroX() {
  return mpu.getGyroX();
}
float get_GyroY() {
  return mpu.getGyroY();
}
float get_GyroZ() {
  return mpu.getGyroZ();
}
float get_Roll() {
  return mpu.getRoll();
}
float get_Pitch() {
  return mpu.getPitch();
}
float get_Yaw() {
  return mpu.getYaw();
}
float get_AccX() {
  return mpu.getAccX();
}
float get_AccY() {
  return mpu.getAccY();
}
float get_AccZ() {
  return mpu.getAccZ();
}
float get_MagX() {
  return mpu.getMagX();
}
float get_MagY() {
  return mpu.getMagY();
}
float get_MagZ() {
  return mpu.getMagZ();
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
#if DEBUG== DEBUG_VERBOSE
        Log(GPS_ERROR_MESSAGE, LOG_ERROR);
#endif
      }
    }
  }
}

// BATTERY_MIN_VOLTAGE = 0%
// BATTERY_MAX_VOLTAGE = 100%
void get_BatteryCharge() {
  float reading = analogRead(BATTERY_SENSOR_PIN);
  float voltage = (reading * BATTERY_MAX_VOLTAGE) / 1023.0;
  float percent = (voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100;
  current_batteryCharge = percent;
}

//Set Altitude Offset
void set_AltitudeOffset() {
  Log(SET_ALTTITUDE_OFFSET_MESSAGE);
  for (int i = 0; i < 10; i++) {
    altitudeOffset += (0.1 * bmp.readAltitude(SEALEVELPRESSURE_HPA));
    delay(100);
  }
  Log(OFFSET_ALTITUDE_MESSAGE + String(altitudeOffset));
  return;
}

//Check if falling
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

//Send Message via Lora
void LoraSendMessage(String message) {
  ResponseStatus rs = LoraTransmitter.sendMessage(message);

  if (rs.getResponseDescription() == "Success") {
#if DEBUG == DEBUG_VERBOSE
    Log("Message Sended" + message);
#endif
  }
  else {
    Log(LORA_SEND_ERROR_MESSAGE + rs.getResponseDescription(), LOG_ERROR);
  }
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
    + String(current_roll) + ","
    + String(current_pitch) + ","
    + String(current_yaw) + ","
    + String(current_X_acceleration) + ","
    + String(current_Y_acceleration) + ","
    + String(current_Z_acceleration) + ","
    + String(current_GyroX) + ","
    + String(current_GyroY) + ","
    + String(current_GyroZ) + ","
    + String(apogee) + ","
    + String(current_batteryCharge)
    + "\n";
  appendToFile(current_fileSystem, dataFilePath.c_str(), dataMessage.c_str());
}

//Save Altitude on SD
void SD_SaveAltitude() {
  String dataMessage =
    String(current_time) + ","
    + String(current_altitude)
    + "\n";
  appendToFile(current_fileSystem, altitudeFilePath.c_str(), dataMessage.c_str());
}

//Serial Print Data
void handle_SerialPrintData() {
#if DEBUG
  //BPM
  Serial.print("Temperatura:");
  Serial.println(current_temperature);
  Serial.print("Pressao: ");
  Serial.println(current_pressure);
  Serial.print("Altitude: ");
  Serial.println(current_altitude);

  Serial.print("Latitude: ");
  Serial.println(latitude);
  Serial.print("Longitude: ");
  Serial.println(longitude);

  Serial.print("Roll: ");
  Serial.println(current_roll);
  Serial.print("Pitch: ");
  Serial.println(current_pitch);
  Serial.print("Yaw: ");


#if DEBUG == DEBUG_VERBOSE

  Serial.print("MagX: ");
  Serial.println(current_MagX);
  Serial.print("MagY: ");
  Serial.println(current_MagY);
  Serial.print("MagZ: ");
  Serial.println(current_MagZ);
#endif
  Serial.println(current_yaw);
  Serial.print("X Acceleration: ");
  Serial.println(current_X_acceleration);
  Serial.print("Y Acceleration: ");
  Serial.println(current_Y_acceleration);
  Serial.print("Z Acceleration: ");
  Serial.println(current_Z_acceleration);
  Serial.print("Gyro X: ");
  Serial.println(current_GyroX);
  Serial.print("Gyro Y: ");
  Serial.println(current_GyroY);
  Serial.print("Gyro Z: ");
  Serial.println(current_GyroZ);
#endif
}

//Init Files
void init_Dir(fs::FS& fs) {
  // Para criar um novo directorio e arquivo toda vez que ligar a placa
  // mantendo os dados antigos e salvando separado
  Log(INIT_DIR_MESSAGE, LOG_MESSAGE, false);
  root = fs.open(ROOT_DIR);
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
  Log("Quantidade de Diretorios Existentes: " + String(nDir), LOG_MESSAGE, false);
#endif

  //Cria directorio para os dados
  String Folder = normalizePath(String(ROOT_DIR) + String(FOLDER_NAME) + "_" + String(nDir));
  createDir(current_fileSystem, Folder.c_str());

  //Cria file para dados dos sensores
  String filePath = normalizePath(Folder + DATA_FILE_NAME) + ".csv";
  createFile(current_fileSystem, filePath.c_str(), "Tempo,Altitude,Temperatura,Pressao,latitude,longitude,Roll,Pitch,Yaw,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Apogeu,Bateria\n");

  dataFilePath = filePath;

  //Cria file para altitude
  filePath = normalizePath(Folder + ALTITUDE_FILE_NAME) + ".csv";
  createFile(current_fileSystem, filePath.c_str(), "Altitude\n");
  altitudeFilePath = filePath;

  //Cria file para log
  filePath = normalizePath(Folder + LOG_FILE_NAME) + ".csv";
  createFile(current_fileSystem, filePath.c_str(), "Tempo,Tipo,Mensagem\n");
  logFilePath = filePath;
}

//Create Directory
void createDir(fs::FS& fs, const char* path) {
  Log(String(CREATING_DIR_MESSAGE) + String(path), LOG_MESSAGE, false);
  if (fs.mkdir(path)) {
    Log(DIR_CREATED_MESSAGE, LOG_MESSAGE, false);
  }
  else {
    Log(DIR_CREATION_ERROR_MESSAGE, LOG_ERROR, false);
    Error(DIR_CREATION_ERROR);
  }
}

//Create File
bool createFile(fs::FS& fs, const char* path, const char* message) {

  Log(CREATING_FILE_MESSAGE + String(path), LOG_MESSAGE, false);
  File file = fs.open(path, FILE_WRITE);

  if (!file) {
    Log(CREATE_FILE_ERROR_MESSAGE + String(path), LOG_ERROR, false);
    Error(FILE_CREATION_ERROR);
    return false;
  }

  if (!file.print(message)) {
    Log(WRITE_FILE_ERROR_MESSAGE + String(path), LOG_ERROR, false);
    Error(FILE_WRITE_ERROR);
    return false;
  }

  file.close();
  Log(FILE_CREATED_MESSAGE + String(path), LOG_MESSAGE, false);

  return true;
}

//Append To a File
void appendToFile(fs::FS& fs, const char* path, const char* message) {

#if DEBUG == DEBUG_VERBOSE
  Log("Appending to file: " + String(path), LOG_MESSAGE, false);
#endif

  if (!fs.exists(path)) {
    return;
  }

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Log(OPEN_FILE_ERROR_MESSAGE + String(path), LOG_ERROR);
    Error(FILE_OPEN_ERROR);
    return;
  }
  if (!file.print(message)) {
    Log(WRITE_FILE_ERROR_MESSAGE);
    Error(FILE_WRITE_ERROR);
  }

#if DEBUG == DEBUG_VERBOSE
  Log("Message Appended", LOG_MESSAGE, false);
#endif
  file.close();
}

//Print Accelerometer Calibration
void printAccelerometerCalibration() {
#if DEBUG == DEBUG_VERBOSE
  mpu.verbose(true);

  Serial.println("< MPU Calibration Parameters >");
  Serial.println("Accel Bias [g]: ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("Gyro Bias [deg/s]: ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("Mag Bias [mG]: ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("Mag scale []: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();

  mpu.verbose(false);
#endif
}

//Print Lora Configuration
void LoraPrintParameter(struct Configuration configuration) {
#if DEBUG == DEBUG_VERBOSE
  Serial.println("< Lora Configuration Parameters >");

  Serial.print(F("HEAD : "));  Serial.print(configuration.HEAD, BIN);Serial.print(" ");Serial.print(configuration.HEAD, DEC);Serial.print(" ");Serial.println(configuration.HEAD, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, DEC);
  Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, DEC);
  Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte  : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRate());
  Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRate());

  Serial.print(F("OptionTrans        : "));  Serial.print(configuration.OPTION.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFixedTransmissionDescription());
  Serial.print(F("OptionPullup       : "));  Serial.print(configuration.OPTION.ioDriveMode, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getIODroveModeDescription());
  Serial.print(F("OptionWakeup       : "));  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
  Serial.print(F("OptionFEC          : "));  Serial.print(configuration.OPTION.fec, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFECDescription());
  Serial.print(F("OptionPower        : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());

  Serial.println("----------------------------------------");
#endif
}

//Beep
void beep(int times, int beeps_interval, int time) {
  for (size_t i = 0; i < times; i++)
  {
    tone(BUZZER_PIN, BUZZER_BEEP_FREQUENCY, time);
    delay(beeps_interval + time);
  }
}

//Log and Save
void Log(const String& Message, bool isError, bool append) {
  String logMessage =
    String(millis()) + ","
    + (isError ? "Error" : "Info") + ","
    + Message
    + "\n";


#if DEBUG
  Serial.println(Message);
#else
  LoraSendMessage(logMessage);
#endif


  if (append) {
    appendToFile(current_fileSystem, logFilePath.c_str(), logMessage.c_str());
  }



}

//Fatal Error, Halt
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
String normalizePath(String path) {
  String normalizedPath = "";
  int len = path.length();

  for (int i = 0; i < len; i++) {
    char currentChar = path[i];

    if (currentChar == '/') {
      // Remove barras duplicadas
      while (i < len - 1 && path[i + 1] == '/') {
        i++;
      }
      normalizedPath += '/';
    }
    else if (currentChar == '.') {
   // Resolve referências "." e ".."
      if (i < len - 1 && path[i + 1] == '.') {
        if (i < len - 2 && path[i + 2] == '/') {
          // Encontrou ".." seguido por "/"
          while (normalizedPath.length() > 1 && normalizedPath.charAt(normalizedPath.length() - 1) != '/') {
            normalizedPath.remove(normalizedPath.length() - 1);
          }
          if (normalizedPath.length() > 1) {
            normalizedPath.remove(normalizedPath.length() - 1); // Remove a última "/"
          }
          i += 2;
          continue;
        }
      }
    }
    else {
      normalizedPath += currentChar;
    }
  }

  return normalizedPath;
}