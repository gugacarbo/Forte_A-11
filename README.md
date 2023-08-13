# Forte_A-11
---
Código da placa Aviõnica do Foguete Forte A-11

Utilizado editor **VSCode**
Compilador Plugin **PlatformIO**

Código em `src/main.cpp`

---
**platformio.ini**
```js
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
	adafruit/Adafruit BMP280 Library@^2.6.8
	rlogiacco/CircularBuffer@^1.3.3
	tinyu-zhao/TinyGPSPlus-ESP32@^0.0.2
	xreef/EByte LoRa E32 library@^1.5.12
	plerup/EspSoftwareSerial@^8.1.0
	hideakitai/MPU9250@^0.4.8

```
---