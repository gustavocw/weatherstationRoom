/***************************************************************************
  This is a library for the EnviromentNode FTTech
  +--------------------+--------------+--------+-------+
  |     Sensor Name    | Manufacturer |  Model | Click |
  +--------------------+--------------+--------+-------+
  | Rain Gauge         |       -      | WS3000 |   1   |
  +--------------------+--------------+--------+-------+
  | Wind Set           | Campbell     | 034B   |   2   |
  +--------------------+--------------+--------+-------+
  | Particulate Matter | AlphaSense   | OPC-N2 |   2   |
  +--------------------+--------------+--------+-------+
  | Lipo Battery Level |       -      | A8     |   -   |
  +--------------------+--------------+--------+-------+
  | Weather Click      | MikroE       | 1978   |   3   |
  +--------------------+--------------+--------+-------+
  | Solar Radiatio     | Apogee       | SQ-110 |   1   |
  +--------------------+--------------+--------+-------+

  These sensors uses diferent comunication, like I2C, SPI, Serial and
  simpler ones like basic Analog and Digital signals.

  After each reading, the data is sent using XBee Mesh on Click 4

  Please report if you find any issue when using this code so when can
  keep improving it

  CHANGES HISTORY:
  - 3. * Improved interruption for the Rain Gauge
       * Removed tempoTotal from RainGauge as time measurements aren't
          good while using sleeping methods
  - 4. * Redone the code to work with the latests libraries:
            - FTTech SAMD51 Clicks 1.3.5
            - FTTech SAMD51 XBee 1.4.0
       * But you will need to use this specific
            - FTTech SAMD Boards 1.0.4

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 ***************************************************************************/
#include "FTTech_SAMD51Clicks.h"
#include <Adafruit_SleepyDog.h>
#include "itoa.h"
// XBEE 2
#include <FTTech_Xbee.h>
// CAMPBELL WATHER SET SENSOR
#include "WS_034B.h"
// WAETHER CLICK
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define DEG_TO_RAD 0.017453292519943295769236907684886

#define DEBUG false
// ALPHASENSE OPC-N2 PARTICULATE MATTER SENSORK

/* ***************************************************************************
 * XBEE 2
 * ***************************************************************************
 * Used in the first and only 20-pin position with UFL antena's conector
 */
uint8_t kyCmdValue[] = "WaspmoteLinkKey!";
uint8_t keyLength = 16;
int BAUDRATE_XBEE = 115200;
uint32_t address_HB = 0x0013A200;
// XBEE CORRECT
uint32_t address_LB = 0x40B5F379;
// XBEE TEST
// uint32_t address_LB = 0x40F67850;

// int address_HB = 0x00000000;
// int address_LB = 0x0000FFFF;
//   XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
//   ZBTxStatusResponse txStatus = ZBTxStatusResponse();
/* ***************************************************************************
 * CAMPBELL WATHER SET SENSOR
 * ***************************************************************************
 * To connect the WeatherSet with the baord, use +3.3V and GND to power the WD
 * and GND as the reference to the WS
 *
 * WD GND/VCC/Reading
 * WS GND/Reading
 *
 * Marrom    - GND
 * Preto     - A2  - NB(A4)
 * Vermelho  - 3.3V
 *
 * Branco    - GND
 * Verde     - A3  - NB(12)
 */
#define wSpeedPin 12
#define wDirecPin A4
WS_034B objWindSet(wSpeedPin, wDirecPin);
float storeddata[6];
uint8_t dataqnt = 0;
unsigned long timeInitWS = 0;
unsigned long timeOfReadingsWS = 10 * 1000;  // It will read the sensor in  a while loop for this time (ms)
                                             /* ***************************************************************************
                                             * ALPHASENSE OPC-N2 PARTICULATE MATTER SENSOR
                                             * ***************************************************************************
                                             * To connect the OPC-N2, use the default SPI pins.
                                             *
                                             * Vermelho - MOSI  (24)
                                             * Branco   - MISO  (22)
                                             * Marrom   - SCK   (23)
                                             * Verde    - CS    (0) - NB(44)
                                             * Amarelo  - 5V
                                             * Preto    - GND
                                             */
#define CS 44
char infoString[61];
// OPCN2 objOpcn2(CS);
/* ***************************************************************************
 * WAETHER CLICK
 * ***************************************************************************
 * Used on Click3 slot. It uses I2C
 */
#define SEALEVELPRESSURE_HPA (1019)
Adafruit_BME280 bme;
/* ***************************************************************************
 * RAIN GAUGE WS3000
 * ***************************************************************************
 * To connect the WS3000 with the baord, use GND and DigitalPin. Remmember
 * to add an pull up resistor with 10k.
 * WD GND/VCC/Reading
 * WS GND/Reading
 *
 * Laranja   - NB (51) + 10k pullup + 10nF cap signal to gnd
 * Verde     - GND
 *
 *
 */
uint8_t rainGaugePin = 10;
volatile int rainFallCount = 0;
volatile float rainMM = 0;
uint16_t risingTime = 500;  // micro secs
/* ***************************************************************************
 * APOGEE QUANTUM SENSORE SQ-110 Serial Numbers range 0-25369
 * ***************************************************************************
 * This sensor uses 2 outputs, one positive and another negative.
 * WARNING: The negative can't be read. Beaware of the voltage devider
 * to read it.
 *
 * Vermelho - Positive - NB(A0)
 * Black    - Negative - NB(A1)
 * Branco   - GND
 */
#define APPosPin A0
#define APNegPin A1
/* ***************************************************************************/
// #define BatteryPin A8
uint32_t fatorCorrecao = 75;
uint16_t SLEEP_TIME = 3 * 60;  // time in seconds
FTTech_Xbee xbee;              // Create object with the serial parameter
uint8_t payload[100] = { 0 };  // Creates payload, and defines its maximum size

void setup() {
  Serial.begin(9600);
  if (DEBUG) {
    while (!Serial)
      delay(10);
    // wait for Arduino Serial Monitor (native USB boards)
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // FTClicks.begin();
  pinMode(45, OUTPUT);
  pinMode(43, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(52, OUTPUT);
  digitalWrite(45, HIGH);
  digitalWrite(52, HIGH);
  digitalWrite(50, HIGH);
  delay(5000);

  Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  // add OPCN2 if there is a particle sensor
  Serial.println("CS-034B - SQ110 - WeatherClick - WS3000");

  //////////////////////////////////////////////////
  // Init XBee
  Serial.println("+++++++++++++++++++++++++++++++++++++++");
  Serial.println("Creating XBee object and Serial 4 ");
  digitalWrite(A9, HIGH);  // Xbee Reset
  xbee.begin(BAUDRATE_XBEE);

  //////////////////////////////////////////////////
  // FTClicks.XBeeSetEncryptionMode(false, false);
  // FTClicks.XBeeSetEncryptionKey(kyCmdValue, keyLength, false);
  // FTClicks.XBeeWriteEEPROM(false);
  //////////////////////////////////////////////////

  //////////////////////////////////////////////////
  // Init Weather Click
  Serial.println("+++++++++++++++++++++++++++++++++++++++");
  Serial.println("Init  Weather Click");
  delay(2000);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  } else {
    Serial.println("Found BME280 sensor");
  }
  //////////////////////////////////////////////////

  //////////////////////////////////////////////////
  // Init WS3000
  // make the raingauge's pin an input:
  pinMode(rainGaugePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin), onBucketTip, RISING);
  //////////////////////////////////////////////////

  // FTClicks.OFF(1);//has to be on because of the pull up in the rain gauge
  digitalWrite(49, LOW);
  digitalWrite(50, LOW);
  // FTClicks.OFF(4);//has to be on because of the xbee

  Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  if (!DEBUG)
    USBDevice.detach();

  Watchdog.enable(15000);
}

void loop() {
  delay(2000);
  Serial.println("initialized");
  BatteryMeasument();
  Watchdog.reset();
  // CLICK 2
  Serial.println("READING CLICK 2");
  digitalWrite(49, HIGH);
  Serial.println("WeatherSetSensor");
  WeatherSetSensor();
  delay(2000);  // a little bit more time to the ParticulateMatterSensor;
  Watchdog.reset();
  // ParticulateMatterSensor();
  digitalWrite(49, LOW);

  // CLICK 3
  Serial.println("READING CLICK 3");
  digitalWrite(50, HIGH);
  delay(1000);
  bme.begin(0x76);
  Serial.println("Step 2");
  delay(1000);
  Watchdog.reset();
  WeatherClickSensor();
  digitalWrite(50, LOW);
  // CLICK 1
  Serial.println("READING CLICK 1");
  digitalWrite(43, HIGH);
  QuantunSensor();
  WeatherRainGauge();
  Watchdog.reset();
  // FTClicks.OFF(1); //always on to receive rain
  digitalWrite(43, LOW);
  ResetRainGauge();
  Watchdog.reset();

  if (DEBUG) {
    Serial.println("-- >> GOING TO SLEEP << --");
    Serial.flush();
    USBDevice.detach();
  }
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(52, LOW);
  uint32_t sleepMS = FTClicks.sleepForSeconds(SLEEP_TIME);
  digitalWrite(LED_BUILTIN, HIGH);
  if (DEBUG) {
    USBDevice.attach();
    while (!Serial) {
    };
  }

  digitalWrite(52, HIGH);

  Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
}



void onBucketTip() {
  noInterrupts();
  unsigned long lastPulseTime = micros();
  while ((micros() - lastPulseTime) < 2 * risingTime) {
  };
  rainFallCount++;
  rainMM += 0.2794;  // calculate precipitation (mm)
  interrupts();
}

void ResetRainGauge(void) {
  rainFallCount = 0;
  rainMM = 0;
}

void WeatherRainGauge(void) {
  char sensor[] = "WS3000";
  xbee.messageClearPayload();
  Serial.print("RainFallCount: ");
  Serial.print(rainFallCount / 2);
  Serial.print(", RainMM: ");
  Serial.print((rainMM / 2));
  xbee.message(sensor, 0, (rainFallCount / 2), (rainMM / 2));
  xbee.getOutPayload(payload);
  xbee.sendPayload(address_HB, address_LB);
}

void QuantunSensor(void) {
  analogReadResolution(12);
  uint32_t APpositve = analogRead(APPosPin);
  uint32_t APnegative = analogRead(APNegPin);
  float APPosValue = APpositve * 3.3 / (pow(2, 12) - 1);
  float APNegValue = APnegative * 3.3 / (pow(2, 12) - 1);
  float Diff = (APPosValue - APNegValue) * 5 / 3.6;
  if (Diff < 0) {
    Diff = 0;
  }
  analogReadResolution(10);

  if (DEBUG) {
    Serial.print("Positive: ");
    Serial.print(APpositve);
    Serial.print("\tVolts: ");
    Serial.println(APpositve * 3.3 / (pow(2, 12) - 1));
    Serial.print("Negative: ");
    Serial.print(APnegative);
    Serial.print("\tVolts: ");
    Serial.println(APnegative * 3.3 / (pow(2, 12) - 1));
    Serial.print("Diff: ");
    Serial.println(Diff);
  }

  char sensor[] = "SQ100";
  xbee.messageClearPayload();
  xbee.message(sensor, Diff);
  xbee.getOutPayload(payload);
  xbee.sendPayload(address_HB, address_LB);
}


#define LIPO_BATTERY A8

void BatteryMeasument() {
  float bat = analogRead(LIPO_BATTERY) + fatorCorrecao;
  float voltage = FTClicks.readBattery();  // bat*3.3/1023 * (330000 + 1000000)/(330000);
  if (DEBUG) {
    Serial.print("Bateria Analog: ");
    Serial.println(bat);
    Serial.print("Bateria: ");
    Serial.print(voltage);
    Serial.println("[V]");
  }
  char sensor[] = "BAT";
  xbee.messageClearPayload();
  xbee.message(sensor, bat, voltage);
  xbee.getOutPayload(payload);
  xbee.sendPayload(address_HB, address_LB);
}

void WeatherSetSensor() {
  Watchdog.disable();
  Serial.println("before readWindSet");
  objWindSet.readWindSet();

  if (dataqnt == 0) {
    dataqnt++;
    storeddata[0] = storeddata[1] = objWindSet.freq;
  } else {
    storeddata[0] = 0;
    for (uint8_t i = 0; i < dataqnt - 1; i++) {
      storeddata[i + 1] = storeddata[i];
    }
    storeddata[1] = objWindSet.freq;
    for (uint8_t i = 1; i < dataqnt; i++) {
      storeddata[0] += storeddata[i];
    }
    if (dataqnt < 6) {
      dataqnt++;
    }
    storeddata[0] = storeddata[0] / dataqnt;
  }
  float wdRad = objWindSet.windDirecDegrees * DEG_TO_RAD;
  float heatX = objWindSet.freqKmph * cos(wdRad);
  float heatY = objWindSet.freqKmph * sin(wdRad);


  if (DEBUG) {
    Serial.print("WSpeed [Hz]: ");
    Serial.print(objWindSet.freq);
    Serial.print("WSpeed [Km/h]: ");
    Serial.print(objWindSet.freqKmph);
    Serial.print("\tWD Degree: ");
    Serial.println(objWindSet.windDirecDegrees);
    Serial.print("\tHeatX: ");
    Serial.println(heatX);
    Serial.print("\tHeatY: ");
    Serial.println(heatY);
  }
  char sensor[] = "034B";
  Watchdog.enable(10000);
  xbee.messageClearPayload();
  xbee.message(sensor, storeddata[0], objWindSet.windDirecDegrees, heatX, heatY);
  xbee.getOutPayload(payload);
  xbee.sendPayload(address_HB, address_LB);
}

void WeatherClickSensor() {
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();
  if (DEBUG) {
    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println("*C");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println("hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(altitude);
    Serial.println("m");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println("%");
  }

  char sensor[] = "CLK_WTH";
  xbee.messageClearPayload();
  xbee.message(sensor, temp, pressure, altitude, humidity);
  xbee.getOutPayload(payload);
  xbee.sendPayload(address_HB, address_LB);
}
