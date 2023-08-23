#include <Arduino.h>
#include <LoRa_E32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <CircularBuffer.h>
#include <MPU9250_asukiaaa.h>
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
#define DEBUG_WIFI    3
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
#define UNKNOW_FILE_ERROR   9

String ERROR_MESSAGES[10] = {
	"NoError"
	"BMP280 init failed!",
	"SD Mount init failed!",
	"MPU9250 init failed!",
	"NO SD",
	"Erro ao Criar Diretorio",
	"Erro ao Criar Arquivo",
	"Erro ao Escrever no Arquivo",
	"Erro ao Abrir Arquivo ",
	"Arquivo Desconhecido Nao Encontrado",
};

//Log Messages
#define SYSYEM_START_MESSAGE          "Avionic Board Starting"
#define SETUP_END_MESSAGE             "Setup Complete"

#define BMP_OK_MESSAGE                "BMP280 OK!" 
#define LORA_STARTED_MESSAGE          "Lora Started!"

#define INIT_DIR_MESSAGE              "Iniciando Diretorio"
#define CREATING_DIR_MESSAGE          "Criando Diretorio "
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
#define UNKNOW_FILE_ERROR_MESSAGE     "Erro ao Abrir Arquivo - Nao Encontrado" 
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
#define DEBUG DEBUG_WIFI //DEBUG_VERBOSE 
#define USE_SD false //SD / SPIFFS


//Flight Config
#define ROCKET_GOAL                   500.0 //m
// #define FLIGHT_START_ALTITUDE      10.0  //m
#define FLIGHT_START_VELOCITY         5.0  // m/s
#define RECOVERY_DEPLOY_VELOCITY     -5.0  // -m/s
#define RECOVERY_DEPLOY_MIN_ALTITUDE  400.0 //m
#define RECOVER_AFTER_LAUNCH_MAX_TIME 15000 //ms

#define RECOVERY_PIN               		27 // ou 26 

//Battery
#define BATTERY_MAX_VOLTAGE        		7.4 //V
#define BATTERY_MIN_VOLTAGE        		6.0 //V
#define BATTERY_SENSOR_PIN         		35  //Pin

//Files
const char* ROOT_DIR = "/";
const char* FOLDER_NAME = "/flight_record"; 				//ROOT_DIR + "/" + FOLDER_NAME
const char* DATA_FILE_NAME = "/sensors";    				//.csv
const char* ALTITUDE_FILE_NAME = "/altitude";       //.csv
const char* LOG_FILE_NAME = "/log"; 								//.csv

#define DATA_FILE_HEADER "Tempo,Altitude,Temperatura,Pressao,latitude,longitude,Roll,Pitch,Yaw,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Apogeu,Bateria\n"
#define ALTITUDE_FILE_HEADER "Tempo,Altitude,Apogeu\n"
#define LOG_FILE_HEADER "Tempo,Tipo,Mensagem\n"

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
#define BUZZER_PIN              26 //Pin Ou 27 //?Está no IO34 que é Input Only
#define BUZZER_CHANNEL          1  //PWM CHANNEL
#define BUZZER_BEEP_TIME        100 //ms
#define BUZZER_BEEP_FREQUENCY   400 //Hz
#define BUZZER_ERROR_FREQUENCY  550 //Hz

//Timers
#define DATA_INTERVAL_TIME      400 //ms
#define ALTITUDE_INTERVAL_DELAY 50 //ms

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
float current_roll = 0.0;
float current_yaw = 0.0;
float current_pitch = 0.0;
float current_X_acceleration;
float current_Y_acceleration;
float current_Z_acceleration;
float current_square_acceleration;

float current_X_gyro;
float current_Y_gyro;
float current_Z_gyro;

float current_MagX;
float current_MagY;
float current_MagZ;
float mDirection;

//GPS
char latitude[12];
char longitude[12];

//Timers
unsigned long dataInterval_time = 0;
unsigned long altitudeInterval_time = 0;

//Objects
LoRa_E32 LoraTransmitter(&Serial, LORA_AuxPin, LORA_BAUDRATE);
// LoRa_E32 LoraTransmitter(LORA_RX, LORA_TX, &Serial, LORA_AuxPin, LORA_BAUDRATE);
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
MPU9250_asukiaaa mpu(0x68);

File root;
String logFilePath = "";
String dataFilePath = "";
String altitudeFilePath = "";


const byte ALTITUDE_BUFFER_SIZE = 6;

CircularBuffer <float, ALTITUDE_BUFFER_SIZE> altitudeBuffer;
CircularBuffer <float, ALTITUDE_BUFFER_SIZE> timeBuffer;


#if USE_SD
fs::FS& current_fileSystem = SD;
#else
fs::FS& current_fileSystem = SPIFFS;
#endif

//Functions
void beep(int times = 1, int beeps_interval = 0, int time = BUZZER_BEEP_TIME);
void Log(const String& Message, bool isError = false, bool appendToLogFile = true);
void Error(int e);

void init_Sensors();
void initLora();
void initGPS();
void initAccelerometer();
void initAltimeter();
void initSd();

String normalizePath(String path);

void init_Dir(fs::FS& fs);
void createDir(fs::FS& fs, const char* path);
bool createFile(fs::FS& fs, const char* path, const char* message);
void appendToFile(fs::FS& fs, const char* path, const char* message, String fileHeader);
void File_SaveFlightData();
void File_SaveAltitude();

void get_Pressure();
void get_Altitude();
void get_Temperature();
void get_Coordenates();
void get_Position();
void get_BatteryCharge();

void set_AltitudeOffset();
float movingAverageFilter();
float calcVelocity();

void handle_FlightStart();
void handleFlight();
void handle_Apogee();
void handle_Recovery();
void activateRecovery();

void handle_SerialPrintData();
void LoraPrintParameter(struct Configuration configuration);
void LoraSendMessage(String message);
void SendJsonData();


#if DEBUG == DEBUG_WIFI
// ! WIFI DEBUGGER
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>

#define WIFI_START_MESSAGE "WiFi Started"
#define JSON_SIZE 2048
const char* ssid = "Catthoobias";
const char* password = "elementanimais";

void onWiFiDisconnect(WiFiEvent_t event);
void WebSocketHandler(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
void generateHTTPResponse(AsyncWebServerRequest* request, StaticJsonDocument<JSON_SIZE> data, int status = 200);
void setHandlers();
void sendDataWs(String message);


AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

AsyncWebSocketClient* client = NULL;

DynamicJsonDocument sensors(4096);

void setHandlers();
void init_Wifi();
void onWiFiDisconnect(WiFiEvent_t event);
#endif

void setup() {
	pinMode(RECOVERY_PIN, OUTPUT);
	pinMode(BATTERY_SENSOR_PIN, INPUT);
	pinMode(BUZZER_PIN, OUTPUT);

#if DEBUG
	Serial.begin(115200); //INICIALIZA A SERIAL
	Serial.println("Started");
	delay(250);
#endif

#if USE_SD
	initSd();
	delay(400);
#else
	while (!SPIFFS.begin(true)) {
		Log("SPIFFS Mount Failed", LOG_ERROR, false);
		delay(250);
	}
#endif

#if DEBUG == DEBUG_WIFI
	init_Wifi();
#endif

#if DEBUG == DEBUG_LORA //Serial Off
	initLora();
	delay(100);
#endif

	beep(5, 75);
	delay(200);

	init_Dir(current_fileSystem);
	Log(SYSYEM_START_MESSAGE);

	beep();
	delay(200);

	init_Sensors();
	beep();
	delay(200);

	set_AltitudeOffset();
	beep(3, 100);
	delay(200);

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

		if (flightStarted || DEBUG) {
			File_SaveFlightData();
		}

		SendJsonData();

		dataInterval_time = millis();
	}

	//Altitude Interval
	if (flightStarted && millis() - altitudeInterval_time >= ALTITUDE_INTERVAL_DELAY) {
		get_Altitude();
		File_SaveAltitude();

		altitudeInterval_time = millis();
	}
}


//Check if Rocket started to Rising
void handle_FlightStart() {
	if (!flightStarted) {
		if (calcVelocity() >= FLIGHT_START_VELOCITY) {
			flightStarted = true;
			flightStarted_Time_offset = millis();
			Log(FLIGHT_START_MESSAGE);
		}
	}
	else {
		handleFlight();
	}
}

float calcVelocity() {
	float velocity = 0.0;

	float altitudeA = 0.0;
	float altitudeB = 0.0;
	float timeA = 0.0;
	float timeB = 0.0;

	for (decltype(altitudeBuffer)::index_t i = 0; i < round(altitudeBuffer.size() / 2); i++) {
		altitudeA += altitudeBuffer[i];
		timeA += timeBuffer[i];
	}
	for (decltype(altitudeBuffer)::index_t i = round(altitudeBuffer.size() / 2); i < altitudeBuffer.size(); i++) {
		altitudeB += altitudeBuffer[i];
		timeB += timeBuffer[i];
	}

	velocity = (altitudeB - altitudeA) / (timeB - timeA);
	return velocity;
}

//After Flight Started
void handleFlight() {
	current_time = millis() - flightStarted_Time_offset;
	handle_Apogee();
	handle_Recovery();
}

//Handle Apogee
void handle_Apogee() {

	float alt = movingAverageFilter();

	if (alt >= apogee) {
		apogee = alt;
	}

	if (apogee >= ROCKET_GOAL && goalReached == false) {
		Log(GOAL_REACHED_MESSAGE);
		goalReached = true;
	}

}

//Handle Recovery start
void handle_Recovery() {
	if (!startRecovery) {

#ifndef RECOVER_AFTER_LAUNCH_MAX_TIME 
		if (RECOVER_AFTER_LAUNCH_MAX_TIME && current_time > RECOVER_AFTER_LAUNCH_MAX_TIME) {
			startRecovery = true;
		}
#endif

		if (calcVelocity() < RECOVERY_DEPLOY_VELOCITY) {
			if (current_altitude < RECOVERY_DEPLOY_MIN_ALTITUDE) {
				startRecovery = true;
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

	delay(2000);
	initAccelerometer();
	delay(500);
	initGPS();
	delay(500);
	initAltimeter();
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
	mpu.setWire(&Wire);
	uint8_t sensorId;

	if (mpu.readId(&sensorId) != 0) {
		Log(MPU_ERROR_MESSAGE, LOG_ERROR);
		Error(MPU_ERROR);
	}
	mpu.beginAccel();
	mpu.beginGyro();
	mpu.beginMag();

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
		return;
	}

	float ofsseted_alt = alt - altitudeOffset;

	altitudeBuffer.push(ofsseted_alt);
	timeBuffer.push(millis());

	//\ Usado Para Apogeu
	current_altitude = movingAverageFilter();
}

//Altitude Filter - Moving Average
float movingAverageFilter() {
	using index_t = decltype(altitudeBuffer)::index_t;
	float sum = 0.0;

	for (index_t i = 0; i < altitudeBuffer.size() - 1; i++) {
		sum += altitudeBuffer[i];
	}
	return sum / altitudeBuffer.size();
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
	uint8_t sensorId;
	int result;

	result = mpu.readId(&sensorId);
	if (result == 0) {
#if DEBUG == DEBUG_VERBOSE
		Log("MPU STARTED ID:" + String(sensorId));
#endif
	}
	else {
		Log("Cannot read sensorId " + String(result), LOG_ERROR);
	}

	result = mpu.accelUpdate();
	if (result == 0) {
		current_X_acceleration = mpu.accelX();
		current_Y_acceleration = mpu.accelY();
		current_Z_acceleration = mpu.accelZ();
		current_square_acceleration = mpu.accelSqrt();
	}
	result = mpu.gyroUpdate();
	if (result == 0) {
		current_X_gyro = mpu.gyroX();
		current_Y_gyro = mpu.gyroY();
		current_Z_gyro = mpu.gyroZ();
	}

	result = mpu.magUpdate();
	if (result != 0) {
#if DEBUG
		Log("Cannot read Magnetometer", LOG_ERROR);
#endif
		mpu.beginMag();
		result = mpu.magUpdate();
	}
	if (result == 0) {
		current_MagX = mpu.magX();
		current_MagY = mpu.magY();
		current_MagZ = mpu.magZ();
		mDirection = mpu.magHorizDirection();
	}

	float pitch = 180 * atan(current_X_acceleration / sqrt(current_Y_acceleration * current_Y_acceleration + current_Z_acceleration * current_Z_acceleration)) / M_PI;
	float roll = 180 * atan(current_Y_acceleration / sqrt(current_X_acceleration * current_X_acceleration + current_Z_acceleration * current_Z_acceleration)) / M_PI;
	float yaw = 180 * atan(current_Z_acceleration / sqrt(current_X_acceleration * current_X_acceleration + current_Z_acceleration * current_Z_acceleration)) / M_PI;
	current_roll = roll;
	current_pitch = pitch;
	current_yaw = yaw;
	// current_pitch = yaw;
	// current_yaw = pitch;

}

//Get Location (GPS)
void get_Coordenates() {

	if (millis() > 5000 && gps.charsProcessed() < 10)
	{
		Log("No GPS detected: check wiring.", LOG_ERROR);
	}

	while (Serial2.available() > 0) {
		if (gps.encode(Serial2.read())) {
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
	int R1 = 15000;
	int R2 = 10000;
	float reading = analogRead(BATTERY_SENSOR_PIN);
	float Max_Reading = R2 / (R1 + R2) * BATTERY_MAX_VOLTAGE;

	float voltage = (reading * BATTERY_MAX_VOLTAGE) / (1023.0 * (3.3 - Max_Reading));
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

	//Fill Buffer
	for (int i = 0; i < altitudeBuffer.size() - 1; i++) {
		get_Altitude();
		delay(50);
	}

	return;
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

void SendJsonData() {
	StaticJsonDocument<JSON_SIZE> data;
	String buffer;

	data["sensors"]["altitude"]["name"] = "Altitude";
	data["sensors"]["altitude"]["value"] = current_altitude;
	data["sensors"]["altitude"]["unity"] = "m";

	data["sensors"]["temperature"]["name"] = "Temperature";
	data["sensors"]["temperature"]["value"] = current_temperature;
	data["sensors"]["temperature"]["unity"] = "°C";

	data["sensors"]["pressure"]["name"] = "Pressure";
	data["sensors"]["pressure"]["value"] = current_pressure;
	data["sensors"]["pressure"]["unity"] = "hPa";

	data["sensors"]["latitude"]["name"] = "Latitude";
	data["sensors"]["latitude"]["value"] = latitude;
	data["sensors"]["latitude"]["unity"] = "°";

	data["sensors"]["longitude"]["name"] = "Longitude";
	data["sensors"]["longitude"]["value"] = longitude;
	data["sensors"]["longitude"]["unity"] = "°";

	data["sensors"]["roll"]["name"] = "Roll";
	data["sensors"]["roll"]["value"] = current_roll;
	data["sensors"]["roll"]["unity"] = "°";

	data["sensors"]["pitch"]["name"] = "Pitch";
	data["sensors"]["pitch"]["value"] = current_pitch;
	data["sensors"]["pitch"]["unity"] = "°";

	data["sensors"]["yaw"]["name"] = "Yaw";
	data["sensors"]["yaw"]["value"] = current_yaw;
	data["sensors"]["yaw"]["unity"] = "°";

	if (current_square_acceleration != 0) {
		data["sensors"]["acceleration"]["name"] = "Acceleration";
		data["sensors"]["acceleration"]["value"] = current_square_acceleration;
		data["sensors"]["acceleration"]["unity"] = "m/s";
	}


	data["sensors"]["vertical_velocity"]["name"] = "Vertical Velocity";
	data["sensors"]["vertical_velocity"]["value"] = calcVelocity();
	data["sensors"]["vertical_velocity"]["unity"] = "m/s";

	data["sensors"]["time"]["name"] = "Flight Time";
	data["sensors"]["time"]["value"] = round(current_time / 1000.0);
	data["sensors"]["time"]["unity"] = "s";


	data["apoggee"] = apogee;

	data["battery"] = "Battery";
	data["battery"] = current_batteryCharge;
	data["battery"] = "%";

	if (!flightStarted) {
		data["status"] = "Ready";
	}

	if (flightStarted) {
		data["status"] = "Rising";

		if (startRecovery) {
			data["status"] = "Recovering";
		}
		else if (calcVelocity() < RECOVERY_DEPLOY_VELOCITY) {
			data["status"] = "Falling";
		}
	}

	serializeJson(data, buffer);

#if DEBUG == DEBUG_WIFI
	sendDataWs(buffer);
#endif

#if DEBUG == DEBUG_LORA
	LoraSendMessage(buffer);
#endif
}
//Save Data on SD 
void File_SaveFlightData() {

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
		+ String(current_square_acceleration) + ","
		+ String(current_X_gyro) + ","
		+ String(current_Y_gyro) + ","
		+ String(current_Z_gyro) + ","
		+ String(apogee) + ","
		+ String(current_batteryCharge)
		+ "\n";
	appendToFile(current_fileSystem, dataFilePath.c_str(), dataMessage.c_str(), DATA_FILE_HEADER);


}

//Save Altitude on SD
void File_SaveAltitude() {
	String dataMessage =
		String(current_time) + ","
		+ String(current_altitude) + ","
		+ String(apogee)
		+ "\n";
	appendToFile(current_fileSystem, altitudeFilePath.c_str(), dataMessage.c_str(), ALTITUDE_FILE_HEADER);
}

//Serial Print Data
void handle_SerialPrintData() {
#if DEBUG
	//BPM
	Serial.print("Temperatura:");
	Serial.print(current_temperature);
	Serial.print(",");
	Serial.print("Pressao: ");
	Serial.print(current_pressure);
	Serial.print(",");
	Serial.print("Altitude: ");
	Serial.print(current_altitude);
	Serial.print(",");

	Serial.print("Roll: ");
	Serial.print(current_roll);
	Serial.print(",");
	Serial.print("Pitch: ");
	Serial.print(current_pitch);
	Serial.print(",");
	Serial.print("Yaw: ");
	Serial.print(current_yaw);
	Serial.print(",");

	Serial.print("Latitude: ");
	Serial.print(latitude);
	Serial.print(",");
	Serial.print("Longitude: ");
	Serial.print(longitude);
	Serial.print(",");

#if DEBUG == DEBUG_VERBOSE
	Serial.print("MagX: ");
	Serial.print(current_MagX);
	Serial.print(",");
	Serial.print("MagY: ");
	Serial.print(current_MagY);
	Serial.print(",");
	Serial.print("MagZ: ");
	Serial.print(current_MagZ);
	Serial.print(",");
	Serial.print("X Acceleration: ");
	Serial.print(current_X_acceleration);
	Serial.print(",");
	Serial.print("Y Acceleration: ");
	Serial.print(current_Y_acceleration);
	Serial.print(",");
	Serial.print("Z Acceleration: ");
	Serial.print(current_Z_acceleration);
	Serial.print(",");
	Serial.print("Gyro X: ");
	Serial.print(current_X_gyro);
	Serial.print(",");
	Serial.print("Gyro Y: ");
	Serial.print(current_Y_gyro);
	Serial.print(",");
	Serial.print("Gyro Z: ");
	Serial.print(current_Z_gyro);
#endif
	Serial.print("\n");


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
	createFile(current_fileSystem, filePath.c_str(), DATA_FILE_HEADER);

	dataFilePath = filePath;

	//Cria file para altitude
	filePath = normalizePath(Folder + ALTITUDE_FILE_NAME) + ".csv";
	createFile(current_fileSystem, filePath.c_str(), ALTITUDE_FILE_HEADER);
	altitudeFilePath = filePath;

	//Cria file para log
	filePath = normalizePath(Folder + LOG_FILE_NAME) + ".csv";
	createFile(current_fileSystem, filePath.c_str(), LOG_FILE_HEADER);
	logFilePath = filePath;
}

//Create Directory
void createDir(fs::FS& fs, const char* path) {
	Log(String(CREATING_DIR_MESSAGE) + String(path), LOG_MESSAGE, false);
	if (!fs.mkdir(path)) {
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
void appendToFile(fs::FS& fs, const char* path, const char* message, String fileHeader) {

	if (!fs.exists(path)) {
		Log(UNKNOW_FILE_ERROR_MESSAGE + String(path), LOG_ERROR);
		Error(UNKNOW_FILE_ERROR);
		return;
	}

	File file = fs.open(path, FILE_APPEND);
	if (!file) {
		Log(OPEN_FILE_ERROR_MESSAGE + String(path), LOG_ERROR, false);
		Error(FILE_OPEN_ERROR);
		return;
	}

	if (!file.print(message)) {
		Log(WRITE_FILE_ERROR_MESSAGE);
		Error(FILE_WRITE_ERROR);
	}

#if DEBUG == DEBUG_VERBOSE
	Log("Message Appended to file:" + String(path), LOG_MESSAGE, false);
#endif
	file.close();
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
		//tone(BUZZER_PIN, BUZZER_BEEP_FREQUENCY, time);
		ledcSetup(BUZZER_CHANNEL, BUZZER_BEEP_FREQUENCY, 8);
		ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

		ledcWriteTone(BUZZER_CHANNEL, BUZZER_BEEP_FREQUENCY);

		delay(time);
		ledcDetachPin(BUZZER_PIN); // Desanexa o pino do PWM
		delay(beeps_interval);
	}
}

//Log and Save
void Log(const String& Message, bool isError, bool appendToLogFile) {
	String logMessage =
		String(millis()) + ","
		+ (isError ? "Error" : "Info") + ","
		+ Message
		+ "\n";

#if DEBUG
	Serial.println(Message);
#endif

#if DEBUG == DEBUG_LORA
	StaticJsonDocument<512> jsonMessage;
	String buffer;
	jsonMessage["log"] = Message;
	serializeJson(jsonMessage, buffer);
	LoraSendMessage(buffer);
#endif

	if (appendToLogFile) {
		appendToFile(current_fileSystem, logFilePath.c_str(), logMessage.c_str(), LOG_FILE_HEADER);
	}
}

//Fatal Error, Halt
void Error(int e) {
	int errorDelay = 800;

	while (true) {
		Log("Fatal Error id " + String(e), LOG_ERROR);
		tone(BUZZER_PIN, BUZZER_ERROR_FREQUENCY, errorDelay * 2);
		delay(errorDelay * 4);

		for (int i = 0; i < e; i++) {
			tone(BUZZER_PIN, BUZZER_ERROR_FREQUENCY, errorDelay);
			delay(errorDelay * 2);
		}
		delay(errorDelay * 4);
		StaticJsonDocument<512> jsonError;
		jsonError["error"]["error_id"] = e;
		jsonError["error"]["error_message"] = ERROR_MESSAGES[e];
		String buffer;
		serializeJson(jsonError, buffer);
		LoraSendMessage(buffer);

#if DEBUG == DEBUG_WIFI
		sendDataWs(buffer);
#endif

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

#if DEBUG == DEBUG_WIFI
void init_Wifi() {
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	while (WiFi.waitForConnectResult() != WL_CONNECTED)
	{
		Log("WiFi Failed!", LOG_ERROR, false);
		delay(1000);
	}

	WiFi.onEvent(onWiFiDisconnect, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

	setHandlers();
	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
	server.begin();

	Log(WIFI_START_MESSAGE, true, false);
	Log("IP Address: " + (WiFi.localIP().toString()), LOG_MESSAGE, false);
}

void onWiFiDisconnect(WiFiEvent_t event)
{
	Log("Conexão WiFi perdida!", LOG_ERROR, false);
	WiFi.begin(ssid, password);

	if (WiFi.waitForConnectResult() != WL_CONNECTED)
	{
		Serial.printf("WiFi Failed!\n");
		return;
	}


}

void generateHTTPResponse(AsyncWebServerRequest* request, StaticJsonDocument<JSON_SIZE> data, int status)
{
	data["status"] = status;
	String jsonString;
	serializeJson(data, jsonString);
	request->send(status, "application/json", jsonString);
}

void setHandlers()
{

	server.on("/index.html", HTTP_ANY, [](AsyncWebServerRequest* request)
		{ request->send(SPIFFS, "/index.html"); });
	server.on("/", HTTP_ANY, [](AsyncWebServerRequest* request)
		{ request->send(SPIFFS, "/index.html"); });

	server.on("/index.js", HTTP_ANY, [](AsyncWebServerRequest* request)
		{ request->send(SPIFFS, "/index.js"); });

	server.on("/main.css", HTTP_ANY, [](AsyncWebServerRequest* request)
		{ request->send(SPIFFS, "/main.css"); });


	server.serveStatic("/", SPIFFS, "/");


	server.onNotFound(
		[](AsyncWebServerRequest* request)
		{
			StaticJsonDocument<JSON_SIZE> data;
			data["message"] = "Not Found";
			generateHTTPResponse(request, data, 404);
		});

	ws.onEvent(WebSocketHandler);
	server.addHandler(&ws);
}

void WebSocketHandler(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
	if (type == WS_EVT_DATA)
	{
#if DEBUG == DEBUG_VERBOSE
		Serial.printf("Received Ws_[#%u] >> %s\n", client->id(), (char*)data);
#endif
		StaticJsonDocument<JSON_SIZE> json_data;
		DeserializationError err = deserializeJson(json_data, (char*)data);
		if (err)
		{
			json_data.clear();
			json_data["status"] = 400;
			json_data["message"] = "Bad Request";
			String jsonString;
			serializeJson(json_data, jsonString);
			client->text(jsonString);
		}

		String jsonString;
		serializeJson(json_data, jsonString);
		client->text(jsonString);
		json_data.clear();
	}
	else if (type == WS_EVT_CONNECT)
	{
		StaticJsonDocument<JSON_SIZE> json_data;
#if DEBUG == DEBUG_VERBOSE
		Serial.printf("Websocket client #%u connected\n", client->id());
#endif

		String jsonString;
		json_data["status"] = "Ready";
		// json_data["message"] = "Hello Client (id) " + String(client->id());
		serializeJson(json_data, jsonString);
		client->text(jsonString);
	}
	else if (type == WS_EVT_DISCONNECT)
	{
#if DEBUG == DEBUG_VERBOSE
		Serial.printf("Websocket client #%u disconnected\n", client->id());
#endif
	}
	else if (type == WS_EVT_ERROR)
	{
#if DEBUG == DEBUG_VERBOSE
		Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
#endif
	}
	else if (type == WS_EVT_PONG)
	{
#if DEBUG == DEBUG_VERBOSE
		Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char*)data : "");
#endif
	}
}

void sendDataWs(String message) {
	ws.cleanupClients();
	for (auto client : ws.getClients())
	{
		if (client->status() != WS_CONNECTED)
		{
#if DEBUG == DEBUG_VERBOSE
			Serial.println("Client #" + client->id() + " has disconnected, Removing.");
#endif
			client->close();
		}
	}

	ws.textAll(message);
}

#endif