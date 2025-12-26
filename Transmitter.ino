#include <WiFi.h>
#include <esp_now.h>
#include <driver/ledc.h>

// -----------------------------
// Motor Pins
// -----------------------------
#define ENA_FRONT 25
#define IN1_FRONT 26   // Front Left IN1
#define IN2_FRONT 27   // Front Left IN2
#define ENB_FRONT 14
#define IN3_FRONT 12   // Front Right IN3
#define IN4_FRONT 13   // Front Right IN4

#define ENA_BACK 33
#define IN1_BACK 32    // Rear Left IN1
#define IN2_BACK 23    // Rear Left IN2
#define ENB_BACK 19
#define IN3_BACK 4     // Rear Right IN3
#define IN4_BACK 2     // Rear Right IN4

// -----------------------------
// PWM
// -----------------------------
const int PWMFreq = 30000;
const int PWMResolution = 8;
const int PWMChannelA = 0;
const int PWMChannelB = 1;
const int PWMChannelC = 2;
const int PWMChannelD = 3;

// -----------------------------
// ESP-NOW Data Struct
// -----------------------------
typedef struct struct_message {
  byte xAxisValue;
  byte yAxisValue;
  byte zAxisValue;
} struct_message;

struct_message receiverData;

unsigned long lastRecvTime = 0;

// -----------------------------
// Motor Control
// -----------------------------
void stopAll() {
  digitalWrite(IN1_FRONT, LOW);
  digitalWrite(IN2_FRONT, LOW);
  digitalWrite(IN3_FRONT, LOW);
  digitalWrite(IN4_FRONT, LOW);
  digitalWrite(IN1_BACK, LOW);
  digitalWrite(IN2_BACK, LOW);
  digitalWrite(IN3_BACK, LOW);
  digitalWrite(IN4_BACK, LOW);
}

// Forward
void moveForward() {
  digitalWrite(IN1_FRONT, HIGH); digitalWrite(IN2_FRONT, LOW);  // Front Left FWD
  digitalWrite(IN3_FRONT, HIGH); digitalWrite(IN4_FRONT, LOW);  // Front Right FWD
  digitalWrite(IN1_BACK, HIGH);  digitalWrite(IN2_BACK, LOW);   // Rear Left FWD
  digitalWrite(IN3_BACK, HIGH);  digitalWrite(IN4_BACK, LOW);   // Rear Right FWD
}

// Backward
void moveBackward() {
  digitalWrite(IN1_FRONT, LOW); digitalWrite(IN2_FRONT, HIGH);
  digitalWrite(IN3_FRONT, LOW); digitalWrite(IN4_FRONT, HIGH);
  digitalWrite(IN1_BACK, LOW);  digitalWrite(IN2_BACK, HIGH);
  digitalWrite(IN3_BACK, LOW);  digitalWrite(IN4_BACK, HIGH);
}

// Strafe Left (sideways)
void moveLeft() {
  digitalWrite(IN1_FRONT, HIGH); digitalWrite(IN2_FRONT, LOW);   // FL FWD
  digitalWrite(IN3_FRONT, LOW);  digitalWrite(IN4_FRONT, HIGH);  // FR REV
  digitalWrite(IN1_BACK, LOW);   digitalWrite(IN2_BACK, HIGH);   // RL REV
  digitalWrite(IN3_BACK, HIGH);  digitalWrite(IN4_BACK, LOW);    // RR FWD
}

// Strafe Right (sideways)
void moveRight() {
  digitalWrite(IN1_FRONT, LOW);  digitalWrite(IN2_FRONT, HIGH);  // FL REV
  digitalWrite(IN3_FRONT, HIGH); digitalWrite(IN4_FRONT, LOW);   // FR FWD
  digitalWrite(IN1_BACK, HIGH);  digitalWrite(IN2_BACK, LOW);    // RL FWD
  digitalWrite(IN3_BACK, LOW);   digitalWrite(IN4_BACK, HIGH);   // RR REV
}

// Rotate Right (Clockwise)
void rotateRight() {
  digitalWrite(IN1_FRONT, HIGH); digitalWrite(IN2_FRONT, LOW);
  digitalWrite(IN3_FRONT, LOW);  digitalWrite(IN4_FRONT, HIGH);
  digitalWrite(IN1_BACK, HIGH);  digitalWrite(IN2_BACK, LOW);
  digitalWrite(IN3_BACK, LOW);   digitalWrite(IN4_BACK, HIGH);
}

// Rotate Left (Counterclockwise)
void rotateLeft() {
  digitalWrite(IN1_FRONT, LOW);  digitalWrite(IN2_FRONT, HIGH);
  digitalWrite(IN3_FRONT, HIGH); digitalWrite(IN4_FRONT, LOW);
  digitalWrite(IN1_BACK, LOW);   digitalWrite(IN2_BACK, HIGH);
  digitalWrite(IN3_BACK, HIGH);  digitalWrite(IN4_BACK, LOW);
}

// -----------------------------
// Data Receive Callback
// -----------------------------
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(receiverData)) return;

  memcpy(&receiverData, incomingData, sizeof(receiverData));
  int x = receiverData.xAxisValue;
  int y = receiverData.yAxisValue;
  int z = receiverData.zAxisValue;

  Serial.printf("X:%d  Y:%d  Z:%d\n", x, y, z);

  // Movement logic
  if (y < 75) moveForward();
  else if (y > 175) moveBackward();
  else if (x < 75) moveLeft();
  else if (x > 175) moveRight();
  else if (z > 175) rotateRight();
  else if (z < 75) rotateLeft();
  else stopAll();

  lastRecvTime = millis();
}

// -----------------------------
// Setup
// -----------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Mecanum Bot Receiver Ready");

  pinMode(IN1_FRONT, OUTPUT);
  pinMode(IN2_FRONT, OUTPUT);
  pinMode(IN3_FRONT, OUTPUT);
  pinMode(IN4_FRONT, OUTPUT);
  pinMode(IN1_BACK, OUTPUT);
  pinMode(IN2_BACK, OUTPUT);
  pinMode(IN3_BACK, OUTPUT);
  pinMode(IN4_BACK, OUTPUT);

  // PWM setup
  ledcSetup(PWMChannelA, PWMFreq, PWMResolution);
  ledcAttachPin(ENA_FRONT, PWMChannelA);
  ledcSetup(PWMChannelB, PWMFreq, PWMResolution);
  ledcAttachPin(ENB_FRONT, PWMChannelB);
  ledcSetup(PWMChannelC, PWMFreq, PWMResolution);
  ledcAttachPin(ENA_BACK, PWMChannelC);
  ledcSetup(PWMChannelD, PWMFreq, PWMResolution);
  ledcAttachPin(ENB_BACK, PWMChannelD);

  // Full power by default
  ledcWrite(PWMChannelA, 255);
  ledcWrite(PWMChannelB, 255);
  ledcWrite(PWMChannelC, 255);
  ledcWrite(PWMChannelD, 255);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

// -----------------------------
// Loop
// -----------------------------
void loop() {
  if (millis() - lastRecvTime > 3000) stopAll();
}
