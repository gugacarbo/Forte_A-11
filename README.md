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
upload_port = COM16
monitor_speed = 115200
lib_deps = 
	adafruit/Adafruit BMP280 Library@^2.6.8
	rlogiacco/CircularBuffer@^1.3.3
	xreef/EByte LoRa E32 library@^1.5.12
	asukiaaa/MPU9250_asukiaaa@^1.5.13
	bblanchon/ArduinoJson@^6.21.3
	me-no-dev/AsyncTCP@^1.1.1
	https://github.com/me-no-dev/ESPAsyncWebServer.git
	mikalhart/TinyGPSPlus@^1.0.3
```

---
