#include <AccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ═══════════════════════════════════════════════════════
// SLAVE ID - CHANGE THIS FOR EACH SLAVE (1 through 10)
// ═══════════════════════════════════════════════════════
const int SLAVE_ID = 10; // ⚠️ UPDATE THIS FOR EACH SLAVE!

// ═══════════════════════════════════════════════════════
// MASTER MAC ADDRESS (same for all slaves)
// ═══════════════════════════════════════════════════════
uint8_t masterAddress[] = {0x30, 0x76, 0xF5, 0xF7, 0x7C, 0xB0};

// ═══════════════════════════════════════════════════════
// TRAVEL CONSTANTS (90,000 steps = full travel)
// ═══════════════════════════════════════════════════════
const long FULL    = 90000;
const long THREE_Q = 67500;
const long HALF    = 45000;
const long THIRD   = 30000;
const long QTR     = 22500;

// ═══════════════════════════════════════════════════════
// TIMING
// ═══════════════════════════════════════════════════════
const unsigned long HOME_TIMEOUT_MS      = 180000;
const unsigned long DEBOUNCE_MS          = 3;
const unsigned long HEARTBEAT_INTERVAL_MS = 2000; // Send heartbeat every 2s

// ═══════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════
const int ENABLE_PIN = 12;

AccelStepper m1(AccelStepper::DRIVER, 25, 26);
AccelStepper m2(AccelStepper::DRIVER, 32, 33);
AccelStepper m3(AccelStepper::DRIVER, 22, 23);
AccelStepper m4(AccelStepper::DRIVER, 27,  4);
AccelStepper m5(AccelStepper::DRIVER, 16, 17);

AccelStepper* motors[] = {&m1, &m2, &m3, &m4, &m5};
const int homePins[]   = {21, 19, 18, 14, 13};

// ═══════════════════════════════════════════════════════
// COMMUNICATION STRUCTURE
// No struct change needed — slaveId field is repurposed:
// Master → Slave: slaveId=0 means broadcast, slaveId=X means targeted at slave X
// Slave → Master: slaveId=SLAVE_ID always (so master knows who sent)
// ═══════════════════════════════════════════════════════
typedef struct {
  char command[32];
  int  value;
  int  slaveId;
} Message;

Message incomingMsg;
Message outgoingMsg;

// ═══════════════════════════════════════════════════════
// STATE FLAGS
// ═══════════════════════════════════════════════════════
volatile int activePattern  = 0; // 0=Idle, 1-4=Pattern, 99=Homing
bool connectedToMaster      = false;
int  foundChannel           = 1;
bool isHomed                = false; // Tracks whether this slave has homed

unsigned long lastFuzzyUpdate   = 0;
unsigned long lastHeartbeatSent = 0;

// ═══════════════════════════════════════════════════════
// FUZZY VELOCITY CONTROL
// ═══════════════════════════════════════════════════════
float calculateFuzzySpeed(float distanceToGo, float maxSpeed) {
  float d = abs(distanceToGo);
  if (d < 2000)  return maxSpeed * 0.3f;
  if (d > 15000) return maxSpeed;
  return (maxSpeed * 0.3f) + (maxSpeed * 0.7f * ((d - 2000.0f) / 13000.0f));
}

void applyFuzzyControl() {
  unsigned long now = millis();
  if (now - lastFuzzyUpdate >= 20) {
    lastFuzzyUpdate = now;
    for (int i = 0; i < 5; i++)
      motors[i]->setMaxSpeed(calculateFuzzySpeed(motors[i]->distanceToGo(), 4500));
  }
}

// ═══════════════════════════════════════════════════════
// HEARTBEAT — non-blocking, sends every 2s
// Called from loop(), runAllUntilDone(), homeAllMotors()
// so master always knows we're alive even during movement
// value carries isHomed state so master can track it
// ═══════════════════════════════════════════════════════
void sendHeartbeat() {
  unsigned long now = millis();
  if (now - lastHeartbeatSent >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatSent = now;
    sendToMaster("SLAVE_HEARTBEAT", isHomed ? 1 : 0);
  }
}

// ═══════════════════════════════════════════════════════
// MOTOR HELPERS
// ═══════════════════════════════════════════════════════
void runAllUntilDone() {
  while (m1.distanceToGo() != 0 || m2.distanceToGo() != 0 ||
         m3.distanceToGo() != 0 || m4.distanceToGo() != 0 ||
         m5.distanceToGo() != 0) {
    if (activePattern == 0) return; // Emergency stop
    applyFuzzyControl();
    m1.run(); m2.run(); m3.run(); m4.run(); m5.run();
    sendHeartbeat(); // Keep master updated during movement
  }
}

void moveAll(long pos) {
  m1.moveTo(pos); m2.moveTo(pos); m3.moveTo(pos);
  m4.moveTo(-pos); // Motor 4 reversed
  m5.moveTo(pos);
  runAllUntilDone();
}

void moveOne(AccelStepper &motor, long pos) {
  if (&motor == &m4) motor.moveTo(-pos);
  else               motor.moveTo(pos);
  while (motor.distanceToGo() != 0) {
    if (activePattern == 0) return;
    applyFuzzyControl();
    m1.run(); m2.run(); m3.run(); m4.run(); m5.run();
    sendHeartbeat();
  }
}

// ═══════════════════════════════════════════════════════
// HOMING
// ═══════════════════════════════════════════════════════
void homeAllMotors() {
  Serial.println("🏠 Homing...");
  isHomed = false;

  bool  h[5]         = {false, false, false, false, false};
  unsigned long tStart[5];
  unsigned long lowSince[5];
  bool  pinWasLow[5] = {false, false, false, false, false};

  unsigned long now = millis();
  for (int i = 0; i < 5; i++) {
    tStart[i]  = now;
    lowSince[i] = 0;
  }

  m1.moveTo(-999999); m2.moveTo(-999999); m3.moveTo(-999999);
  m4.moveTo( 999999);
  m5.moveTo(-999999);

  while (!(h[0] && h[1] && h[2] && h[3] && h[4])) {
    unsigned long t = millis();
    sendHeartbeat(); // Stay visible to master during homing

    for (int i = 0; i < 5; i++) {
      if (h[i]) continue;

      bool pinLow = (digitalRead(homePins[i]) == LOW);

      if (pinLow) {
        if (!pinWasLow[i]) {
          pinWasLow[i] = true;
          lowSince[i]  = t;
        } else if (t - lowSince[i] >= DEBOUNCE_MS) {
          motors[i]->stop();
          motors[i]->setCurrentPosition(0);
          h[i] = true;
          Serial.printf("  ✓ M%d homed (nail contact)\n", i + 1);
        }
      } else {
        if (pinWasLow[i]) Serial.printf("  ~ M%d noise rejected\n", i + 1);
        pinWasLow[i] = false;
        lowSince[i]  = 0;
      }

      if (!h[i] && (t - tStart[i] >= HOME_TIMEOUT_MS)) {
        motors[i]->stop();
        motors[i]->setCurrentPosition(0);
        h[i] = true;
        Serial.printf("  ⚠️ M%d TIMEOUT — force-zeroed\n", i + 1);
      }
    }

    m1.run(); m2.run(); m3.run(); m4.run(); m5.run();
  }

  isHomed = true;
  Serial.println("✅ Homing complete!");
  sendToMaster("HOMING_DONE", 1);
}

// ═══════════════════════════════════════════════════════
// PATTERNS
// ═══════════════════════════════════════════════════════
void patternOne() {
  // Staircase Drop — simultaneous move to different heights
  m1.moveTo(QTR);
  m2.moveTo(HALF);
  m3.moveTo(THREE_Q);
  m4.moveTo(-FULL);
  m5.moveTo(FULL);
  runAllUntilDone(); delay(800);
  moveAll(FULL);     delay(600);
  moveAll(5000);     delay(1000);
}

void patternTwo() {
  // Wave Cascade M1 → M5
  moveOne(m1, FULL); delay(100);
  moveOne(m2, FULL); delay(100);
  moveOne(m3, FULL); delay(100);
  moveOne(m4, FULL); delay(100);
  moveOne(m5, FULL); delay(500);
  moveAll(5000);
}

void patternThree() {
  // Alternating Pulse
  for (int r = 0; r < 3; r++) {
    m1.moveTo(FULL);  m3.moveTo(FULL);  m5.moveTo(FULL);
    m2.moveTo(QTR);   m4.moveTo(-QTR);
    runAllUntilDone(); delay(400);
    m1.moveTo(QTR);   m3.moveTo(QTR);   m5.moveTo(QTR);
    m2.moveTo(FULL);  m4.moveTo(-FULL);
    runAllUntilDone(); delay(400);
  }
  moveAll(5000); delay(1000);
}

void patternFour() {
  // Breathing Motion
  m1.moveTo(THIRD);  m2.moveTo(THREE_Q);
  m3.moveTo(FULL);   m4.moveTo(-THREE_Q); m5.moveTo(THIRD);
  runAllUntilDone(); delay(800);
  moveAll(5000);     delay(500);
  m1.moveTo(FULL);   m2.moveTo(THREE_Q);
  m3.moveTo(THIRD);  m4.moveTo(-THREE_Q); m5.moveTo(FULL);
  runAllUntilDone(); delay(800);
  moveAll(5000);     delay(1000);
}

// ═══════════════════════════════════════════════════════
// COMMUNICATION
// ═══════════════════════════════════════════════════════
void sendToMaster(const char* cmd, int val) {
  strcpy(outgoingMsg.command, cmd);
  outgoingMsg.value   = val;
  outgoingMsg.slaveId = SLAVE_ID;
  esp_now_send(masterAddress, (uint8_t*)&outgoingMsg, sizeof(outgoingMsg));
}

void onDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  memcpy(&incomingMsg, data, sizeof(incomingMsg));

  // First message = master found, lock in
  if (!connectedToMaster) { connectedToMaster = true; return; }

  // Ignore master heartbeats
  if (strcmp(incomingMsg.command, "HEARTBEAT") == 0) return;

  // ✅ KEY CHANGE: Filter targeted messages
  // slaveId == 0 → broadcast (accept)
  // slaveId == SLAVE_ID → targeted at us (accept)
  // anything else → not for us (ignore)
  if (incomingMsg.slaveId != 0 && incomingMsg.slaveId != SLAVE_ID) return;

  if      (strcmp(incomingMsg.command, "PATTERN_1") == 0) activePattern = 1;
  else if (strcmp(incomingMsg.command, "PATTERN_2") == 0) activePattern = 2;
  else if (strcmp(incomingMsg.command, "PATTERN_3") == 0) activePattern = 3;
  else if (strcmp(incomingMsg.command, "PATTERN_4") == 0) activePattern = 4;
  else if (strcmp(incomingMsg.command, "HOME")      == 0) activePattern = 99;
  else if (strcmp(incomingMsg.command, "STOP")      == 0) {
    activePattern = 0;
    m1.stop(); m2.stop(); m3.stop(); m4.stop(); m5.stop();
  }
}

// ═══════════════════════════════════════════════════════
// CHANNEL SCANNING
// ═══════════════════════════════════════════════════════
void scanForMaster() {
  int ch = 1;
  while (!connectedToMaster) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    unsigned long start = millis();
    while (millis() - start < 400) {
      if (connectedToMaster) { foundChannel = ch; return; }
      yield();
    }
    if (++ch > 13) ch = 1;
  }
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.printf("\n╔══════════════════════════════════════╗\n");
  Serial.printf("║  KINETIC RAIN — SLAVE %-2d              ║\n", SLAVE_ID);
  Serial.printf("╚══════════════════════════════════════╝\n\n");

  WiFi.mode(WIFI_STA);
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // Disabled during boot

  for (int i = 0; i < 5; i++) {
    pinMode(homePins[i], INPUT_PULLUP);
    motors[i]->setMaxSpeed(4500);
    motors[i]->setAcceleration(3200);
    motors[i]->setMinPulseWidth(20);
  }

  delay(200);
  digitalWrite(ENABLE_PIN, LOW); // Enable drivers after settle

  WiFi.disconnect();
  esp_now_init();
  esp_now_register_recv_cb(onDataReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peerInfo);

  Serial.println("🔍 Scanning for Master...");
  scanForMaster();

  esp_now_del_peer(masterAddress);
  peerInfo.channel = foundChannel;
  esp_now_add_peer(&peerInfo);

  Serial.printf("✅ Master found on channel %d\n\n", foundChannel);

  // ✅ KEY CHANGE: Stay IDLE — do NOT auto-home
  activePattern = 0;
  Serial.println("⏳ IDLE — waiting for HOME command from dashboard\n");
}

// ═══════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════
void loop() {
  sendHeartbeat(); // Keep master updated while idle

  if (activePattern > 0) {
    if      (activePattern == 1)  patternOne();
    else if (activePattern == 2)  patternTwo();
    else if (activePattern == 3)  patternThree();
    else if (activePattern == 4)  patternFour();
    else if (activePattern == 99) homeAllMotors();

    if (activePattern != 0 && activePattern != 99)
      sendToMaster("PATTERN_DONE", 1);

    activePattern = 0;
  }

  yield();
}