#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>

// === กำหนดขาพินสำหรับบอร์ด ESP32 รุ่นปกติ ===
#define BOOT_BTN   0   // ปุ่ม BOOT บนบอร์ด
#define I2C_SDA    14  // เซ็นเซอร์เกียร์ CJMCU-90393
#define I2C_SCL    13  // เซ็นเซอร์เกียร์ CJMCU-90393
#define GPS_RX_PIN 16  // ต่อเข้าขา TX ของ ATGM336H-5N
#define GPS_TX_PIN 17  // ต่อเข้าขา RX ของ ATGM336H-5N
#define RPM_PIN    27  // ต่อเข้าขาสัญญาณวัดรอบ (ผ่านวงจรแบ่งแรงดัน 3.0V)

// === ตั้งค่าตัวหารสัญญาณวัดรอบเครื่อง 4M40 ===
const float PULSES_PER_REV = 2.0; // เซ็นเซอร์ปั๊มโซล่า 4M40 = 2 พัลส์ต่อ 1 รอบเครื่อง

// === ตั้งค่า UUID สำหรับ Bluetooth (BLE) ===
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

Adafruit_MLX90393 mlx = Adafruit_MLX90393();
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
Preferences pref;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// === ตัวแปรระบบเกียร์ ===
struct GearCoord { float x; float y; };
GearCoord gearCal[7]; // 0=N, 1=1, 2=2, 3=3, 4=4, 5=5, 6=R
const char* gearNames[] = {"N", "1", "2", "3", "4", "5", "R"};
const float THRESHOLD = 150.0;

float currentX = 0, currentY = 0, currentZ = 0;
int currentGear = 0;
int calStep = -1; // -1 = โหมดปกติ, 0-6 = กำลังคาริเบรต

// === ตัวแปร GPS และวัดรอบ ===
int currentSpeed = 0;
int satCount = 0;
int currentRPM = 0;
float smoothRPM = 0;

volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseMicros = 0;
unsigned long lastRpmCalcTime = 0;
unsigned long lastSerialTime = 0;
unsigned long lastBleTime = 0;

// === ตัวแปรปุ่มกด ===
bool lastBtnState = HIGH;
unsigned long btnPressTime = 0;
bool longPressHandled = false;

void IRAM_ATTR onRpmPulse() {
  unsigned long now = micros();
  if (now - lastPulseMicros > 1500) { // กรองสัญญาณรบกวนความถี่สูง
    pulseCount++;
    lastPulseMicros = now;
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println(">>> [BLE] มือถือเชื่อมต่อสำเร็จ! <<<");
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println(">>> [BLE] มือถือตัดการเชื่อมต่อ กำลังรอเชื่อมต่อใหม่... <<<");
  }
};

void saveSingleGear(int g, float x, float y) {
  gearCal[g].x = x;
  gearCal[g].y = y;
  pref.begin("mt_web", false);
  pref.putFloat(("x" + String(g)).c_str(), x);
  pref.putFloat(("y" + String(g)).c_str(), y);
  pref.end();
}

void loadSettings() {
  pref.begin("mt_web", true);
  for (int i = 0; i < 7; i++) {
    gearCal[i].x = pref.getFloat(("x" + String(i)).c_str(), 0.0);
    gearCal[i].y = pref.getFloat(("y" + String(i)).c_str(), 0.0);
  }
  pref.end();
}

int detectGear(float x, float y) {
  int matched = 0;
  float minDist = 9999.0;
  for (int i = 0; i < 7; i++) {
    float d = sqrt(pow(x - gearCal[i].x, 2) + pow(y - gearCal[i].y, 2));
    if (d < minDist) {
      minDist = d;
      matched = i;
    }
  }
  return (minDist > THRESHOLD) ? 0 : matched;
}

void handleOnboardButton() {
  bool reading = digitalRead(BOOT_BTN);
  if (reading == LOW && lastBtnState == HIGH) {
    btnPressTime = millis();
    longPressHandled = false;
  }
  if (reading == LOW && !longPressHandled) {
    if (millis() - btnPressTime >= 2000) {
      longPressHandled = true;
      calStep = (calStep == -1) ? 0 : -1;
      Serial.println(calStep == 0 ? "\n>>> [CALIBRATE] เข้าสู่โหมดตั้งค่าเกียร์ N <<<" : "\n>>> [CALIBRATE] ยกเลิกการตั้งค่า <<<");
    }
  }
  if (reading == HIGH && lastBtnState == LOW) {
    unsigned long dur = millis() - btnPressTime;
    if (!longPressHandled && dur > 50 && dur < 2000) {
      if (calStep >= 0 && calStep < 7) {
        saveSingleGear(calStep, currentX, currentY);
        Serial.printf(">>> [SAVED] บันทึกเกียร์ %s เรียบร้อย <<<\n", gearNames[calStep]);
        calStep++;
        if (calStep >= 7) {
          calStep = -1;
          Serial.println(">>> [DONE] บันทึกครบทั้ง 7 ตำแหน่งเรียบร้อย! <<<\n");
        }
      }
    }
  }
  lastBtnState = reading;
}

// แปลงวันที่และเวลา GPS (UTC) เป็นเวลาประเทศไทย (UTC+7)
void getThaiDateTime(char* dateBuf, char* timeBuf) {
  if (!gps.time.isValid() || !gps.date.isValid() || gps.date.year() < 2024) {
    strcpy(dateBuf, "--/--/----");
    strcpy(timeBuf, "--:--:--");
    return;
  }
  int yr = gps.date.year();
  int mo = gps.date.month();
  int dy = gps.date.day();
  int hr = gps.time.hour() + 7;
  int mn = gps.time.minute();
  int sc = gps.time.second();

  if (hr >= 24) {
    hr -= 24;
    dy++;
    int daysInMonth[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    if ((yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0)) daysInMonth[2] = 29;
    if (dy > daysInMonth[mo]) {
      dy = 1;
      mo++;
      if (mo > 12) { mo = 1; yr++; }
    }
  }
  sprintf(dateBuf, "%02d/%02d/%04d", dy, mo, yr);
  sprintf(timeBuf, "%02d:%02d:%02d", hr, mn, sc);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nกำลังเริ่มต้นระบบ Strada 4M40 Cluster (BLE)...");

  pinMode(BOOT_BTN, INPUT_PULLUP);
  pinMode(RPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), onRpmPulse, FALLING);

  Wire.begin(I2C_SDA, I2C_SCL);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (!mlx.begin_I2C(0x0C, &Wire) && !mlx.begin_I2C(0x18, &Wire)) {
    Serial.println("[ERROR] ไม่พบเซ็นเซอร์ CJMCU-90393! กรุณาตรวจสอบสาย D13/D14");
    while (1) delay(500);
  }
  mlx.setGain(MLX90393_GAIN_1X);
  mlx.setResolution(MLX90393_X, MLX90393_RES_16);
  mlx.setResolution(MLX90393_Y, MLX90393_RES_16);
  mlx.setResolution(MLX90393_Z, MLX90393_RES_16);

  loadSettings();

  BLEDevice::init("MT-Gear-BLE");
  BLEDevice::setMTU(185); // ขยายขนาดแพ็กเก็ต BLE ให้ส่ง JSON ได้ครบในครั้งเดียว
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[OK] เปิด Bluetooth ชื่อ: MT-Gear-BLE พร้อมเชื่อมต่อ!");
}

void loop() {
  // 1. อ่านข้อมูลจากโมดูล GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  if (gps.speed.isValid()) currentSpeed = (int)gps.speed.kmph();
  if (gps.satellites.isValid()) satCount = gps.satellites.value();

  // 2. คำนวณรอบเครื่องยนต์ทุก 150ms พร้อมตัวกรองให้นุ่มนวล
  if (millis() - lastRpmCalcTime >= 150) {
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float elapsedSec = (millis() - lastRpmCalcTime) / 1000.0;
    lastRpmCalcTime = millis();

    int rawRPM = (pulses > 0) ? (int)((pulses * 60.0) / (elapsedSec * PULSES_PER_REV)) : 0;
    smoothRPM = (smoothRPM * 0.6) + (rawRPM * 0.4);
    currentRPM = (int)smoothRPM;
  }

  // 3. อ่านตำแหน่งเกียร์และปุ่ม BOOT
  mlx.readData(&currentX, &currentY, &currentZ);
  currentGear = detectGear(currentX, currentY);
  handleOnboardButton();

  // 4. ส่งข้อมูลขึ้นเว็บแดชบอร์ดผ่าน BLE ทุกๆ 100ms
  if (deviceConnected && (millis() - lastBleTime >= 100)) {
    lastBleTime = millis();
    char dStr[16], tStr[16];
    getThaiDateTime(dStr, tStr);

    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"g\":\"%s\",\"i\":%d,\"s\":%d,\"r\":%d,\"sat\":%d,\"d\":\"%s\",\"t\":\"%s\",\"c\":%d}",
             gearNames[currentGear], currentGear, currentSpeed, currentRPM, satCount, dStr, tStr, calStep);
    pCharacteristic->setValue((uint8_t*)payload, strlen(payload));
    pCharacteristic->notify();
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(300);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // 5. แสดงสถานะใน Serial Monitor ทุกๆ 300ms
  if (millis() - lastSerialTime >= 300) {
    lastSerialTime = millis();
    if (calStep == -1) {
      Serial.printf("[LIVE] เกียร์: %-2s | รอบ: %4d RPM | ความเร็ว: %3d km/h (SAT: %2d)\n",
                    gearNames[currentGear], currentRPM, currentSpeed, satCount);
    } else {
      Serial.printf("[CAL -> เกียร์ %s] X: %6.1f | Y: %6.1f (กด BOOT เพื่อบันทึก)\n",
                    gearNames[calStep], currentX, currentY);
    }
  }

  delay(10);
}