# Vital_Monitor

One-line description:
An ESP32-C3 based wearable that measures ambient PM2.5 and fingertip heart rate/SpO2, shows them on an OLED, and streams them over Bluetooth to a web app.

Important line to include near the top:
This is an engineering prototype for demonstration and testing. It is not a medical device and its readings should not be used for health decisions.

What it does
Measures PM1.0, PM2.5, and PM10 using a laser particle sensor
Measures heart rate and blood oxygen (SpO2) from a fingertip sensor
Estimates breathing rate — note this is calculated from heart rate and SpO2, not measured directly from the sensor
Shows live readings on a small OLED screen
Sends data over Bluetooth to a browser-based app (no app install needed)
Saves a history of past readings on the device itself
Runs on a rechargeable battery
Hardware used

List these as plain bullets, not a table:

Microcontroller — ESP32-C3 SuperMini
Heart rate / SpO2 sensor — MAX30105
Air quality sensor — Plantower PMS5003
Display — SSD1306 OLED, 0.96 inch
Battery charger — TP4056 module
Boost converter — MT3608 (steps battery voltage up to 5V for the air sensor)
Battery — 3.7V Li-Po, 2000mAh
Wiring / pin connections

Since GitHub markdown tables render fine, you can use a table here if you want — but if you'd rather avoid tables project-wide, list it like this:

MAX30105 (heart rate sensor)
SDA → GPIO6
SCL → GPIO7
VCC → 3.3V
GND → GND

OLED display
SDA → GPIO8
SCL → GPIO9
VCC → 3.3V
GND → GND

PMS5003 (air quality sensor)
TX → GPIO20 (RX on ESP32)
RX → GPIO21 (TX on ESP32)
VCC → 5V (from boost converter)
GND → GND

The OLED and the heart rate sensor use two separate I2C buses, not one shared bus. Don't wire them onto the same pins.

How to build it
Get all the parts listed above
Wire everything according to the pin list
Flash vital_monitor.ino onto the ESP32-C3 using Arduino IDE
In Arduino IDE board settings, set "USB CDC On Boot" to Enabled — otherwise the serial monitor and the air sensor will conflict
Open vitalmonitor.html in Chrome or Edge (Web Bluetooth needs one of these)
Click connect, pick "VitalMonitor" from the device list

Known limitations
SpO2 readings are not reliable yet — needs comparison against a proper pulse oximeter
Breathing rate is a calculated estimate, not a direct measurement
Battery life has not been tested with a timer yet, only calculated
About 1 in 4 measurements come back empty if the finger moves during the 30-second reading window
No real-time clock on board — the device gets the time from your phone/laptop each time it connects, so readings taken before connecting are saved without a timestamp
