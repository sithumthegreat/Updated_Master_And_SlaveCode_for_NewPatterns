#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_now.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════════
// WIFI CONFIGURATION
// ═══════════════════════════════════════════════════════
const char* ssid     = "Sadewna";
const char* password = "sadewna1";

// ═══════════════════════════════════════════════════════
// SLAVE CONFIGURATION
// ═══════════════════════════════════════════════════════
const int NUM_SLAVES    = 10;
const int ACTIVE_SLAVES = 10;

// If no heartbeat received within this window, slave is offline
const unsigned long SLAVE_TIMEOUT_MS = 6000;

uint8_t slaveAddresses[NUM_SLAVES][6] = {
  {0x68, 0x09, 0x47, 0x44, 0x51, 0x1C},  // Slave 1
  {0x68, 0x09, 0x47, 0x60, 0x5D, 0x84},  // Slave 2
  {0x70, 0x4B, 0xCA, 0x4D, 0x64, 0x04},  // Slave 3
  {0x8C, 0x94, 0xDF, 0xAA, 0x44, 0xFC},  // Slave 4
  {0x68, 0x09, 0x47, 0x51, 0xB8, 0x3C},  // Slave 5
  {0x68, 0x09, 0x47, 0x5F, 0x1C, 0x40},  // Slave 6
  {0x30, 0x76, 0xF5, 0xF7, 0xD3, 0x64},  // Slave 7
  {0xF4, 0x2D, 0xC9, 0x71, 0x52, 0xA8},  // Slave 8
  {0xB4, 0xBF, 0xE9, 0x1A, 0xDA, 0x70},  // Slave 9
  {0x68, 0x09, 0x47, 0x5F, 0x1F, 0x4C},  // Slave 10
};

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ═══════════════════════════════════════════════════════
// COMMUNICATION STRUCTURE
// Protocol note:
//   Master → Slave  slaveId=0  broadcast to all
//   Master → Slave  slaveId=X  targeted at slave X only
//   Slave  → Master slaveId=SLAVE_ID always
// ═══════════════════════════════════════════════════════
typedef struct {
  char command[32];
  int  value;
  int  slaveId;
} Message;

Message outgoingMsg;
Message incomingMsg;

// ═══════════════════════════════════════════════════════
// SYSTEM STATE
// ═══════════════════════════════════════════════════════
String systemStatus    = "INITIALIZING";
bool   autoCycleRunning = false;
int    currentPattern   = 0;

unsigned long lastBeacon       = 0;
unsigned long lastTimeoutCheck = 0;

// ✅ Per-slave granular state
bool          slaveConnected[NUM_SLAVES]     = {};
bool          slaveHomed[NUM_SLAVES]         = {};
bool          slaveRunning[NUM_SLAVES]       = {};
bool          slaveDone[NUM_SLAVES]          = {};
unsigned long lastSlaveHeartbeat[NUM_SLAVES] = {};

// ═══════════════════════════════════════════════════════
// BROADCAST TO DASHBOARD
// Now includes connected, homed, running per slave
// ═══════════════════════════════════════════════════════
void broadcastToReact() {
  StaticJsonDocument<2048> doc;
  doc["status"]          = systemStatus;
  doc["autoCycleRunning"]= autoCycleRunning;
  doc["currentPattern"]  = currentPattern;
  doc["numSlaves"]       = NUM_SLAVES;
  doc["activeSlaves"]    = ACTIVE_SLAVES;

  JsonArray arr = doc["slaves"].to<JsonArray>();
  for (int i = 0; i < NUM_SLAVES; i++) {
    JsonObject s = arr.add<JsonObject>();
    s["id"]        = i + 1;
    s["connected"] = slaveConnected[i];
    s["homed"]     = slaveHomed[i];
    s["running"]   = slaveRunning[i];
  }

  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}



//this sends a broadcast packet to slave
void broadcastToAllSlaves(const char* cmd, int value) {
  strcpy(outgoingMsg.command, cmd);
  outgoingMsg.value   = value;
  outgoingMsg.slaveId = 0; // broadcast
  Serial.printf("\n📡 Broadcast: %s\n", cmd);
  for (int i = 0; i < ACTIVE_SLAVES; i++) {
    esp_err_t r = esp_now_send(slaveAddresses[i], (uint8_t*)&outgoingMsg, sizeof(outgoingMsg));
    Serial.printf("  → Slave %d: %s\n", i+1, r == ESP_OK ? "✓" : "✗ FAILED");
  }
}


// SEND TARGETED COMMAND TO ONE SPECIFIC SLAVE
// slaveId in message = target, so slave filters by it

void sendToSlave(int targetId, const char* cmd, int value) {
  if (targetId < 1 || targetId > NUM_SLAVES) return;
  strcpy(outgoingMsg.command, cmd);
  outgoingMsg.value   = value;
  outgoingMsg.slaveId = targetId; // targeted
  esp_err_t r = esp_now_send(slaveAddresses[targetId - 1],
                              (uint8_t*)&outgoingMsg, sizeof(outgoingMsg));
  Serial.printf("📡 → Slave %d (targeted) %s: %s\n",
                targetId, cmd, r == ESP_OK ? "✓" : "✗ FAILED");
}

// ═══════════════════════════════════════════════════════
// CHECK IF ALL CONNECTED SLAVES ARE DONE
// ═══════════════════════════════════════════════════════
bool allActiveSlavesDone() {
  int count = 0;
  for (int i = 0; i < ACTIVE_SLAVES; i++) {
    if (slaveConnected[i]) {
      count++;
      if (!slaveDone[i]) return false;
    }
  }
  return count > 0; // At least one must be connected
}


// ESP-NOW RECEIVE CALLBACK-this works in setup() function

void onDataReceive(const esp_now_recv_info_t *recv_info,
                   const uint8_t *data, int len) {
  memcpy(&incomingMsg, data, sizeof(incomingMsg));
  int id = incomingMsg.slaveId;
  if (id < 1 || id > NUM_SLAVES) return;
  int idx = id - 1;

  // ── Heartbeat — update connection state ──────────────
  if (strcmp(incomingMsg.command, "SLAVE_HEARTBEAT") == 0) {
    if (!slaveConnected[idx])
      Serial.printf("✓ Slave %d came online\n", id);
    slaveConnected[idx]     = true;
    lastSlaveHeartbeat[idx] = millis();
    broadcastToReact();
    return;
  }

  Serial.printf("📩 Slave %d: %s\n", id, incomingMsg.command);

  // ── Homing done ───────────────────────────────────────
  if (strcmp(incomingMsg.command, "HOMING_DONE") == 0) {
    slaveHomed[idx]   = true;
    slaveRunning[idx] = false;
    Serial.printf("✓ Slave %d homed and ready\n", id);

    // Check if all connected slaves are now homed
    bool allHomed = true;
    for (int i = 0; i < ACTIVE_SLAVES; i++)
      if (slaveConnected[i] && !slaveHomed[i]) { allHomed = false; break; }
    if (allHomed) systemStatus = "ALL_SLAVES_READY";
  }
  // ── Pattern done ──────────────────────────────────────
  else if (strcmp(incomingMsg.command, "PATTERN_DONE") == 0) {
    slaveDone[idx]    = true;
    slaveRunning[idx] = false;

    if (allActiveSlavesDone()) {
      Serial.println("✅ ALL SLAVES DONE");
      if (autoCycleRunning) {
        for (int i = 0; i < NUM_SLAVES; i++) slaveDone[i] = false;
        currentPattern = (currentPattern % 4) + 1;
        Serial.printf("🔄 Auto-Cycle → Pattern %d\n", currentPattern);
        delay(1000);
        char cmd[32];
        sprintf(cmd, "PATTERN_%d", currentPattern);
        for (int i = 0; i < ACTIVE_SLAVES; i++)
          if (slaveConnected[i]) slaveRunning[i] = true;
        broadcastToAllSlaves(cmd, 1);
        systemStatus = String("PATTERN_") + currentPattern + "_RUNNING";
      } else {
        systemStatus = "ALL_SLAVES_READY";
      }
    }
  }
  // ── Stopped ───────────────────────────────────────────
  else if (strcmp(incomingMsg.command, "STOPPED") == 0) {
    slaveRunning[idx] = false;
    systemStatus = "STOPPED";
  }

  broadcastToReact();
}

// ═══════════════════════════════════════════════════════
// WEBSOCKET EVENT HANDLER
// ═══════════════════════════════════════════════════════
void onWebSocketEvent(AsyncWebSocket* server,
                      AsyncWebSocketClient* client,
                      AwsEventType type,
                      void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("🌐 Dashboard: %s\n", client->remoteIP().toString().c_str());
    broadcastToReact();
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 &&
        info->len == len && info->opcode == WS_TEXT) {

      data[len] = 0;
      String msg = (char*)data;
      Serial.printf("🌐 Dashboard: %s\n", msg.c_str());

      // ── Global pattern commands ───────────────────────
      if (msg.startsWith("PATTERN_") && !msg.startsWith("PATTERN_DONE")) {
        autoCycleRunning = false;
        currentPattern   = msg.substring(8).toInt();
        for (int i = 0; i < NUM_SLAVES; i++) {
          slaveDone[i] = false;
          if (slaveConnected[i]) slaveRunning[i] = true;
        }
        broadcastToAllSlaves(msg.c_str(), 1);
        systemStatus = msg + "_RUNNING";
      }
      // ── Auto-cycle ────────────────────────────────────
      else if (msg == "START_AUTO_CYCLE") {
        autoCycleRunning = true;
        currentPattern   = 1;
        for (int i = 0; i < NUM_SLAVES; i++) {
          slaveDone[i] = false;
          if (slaveConnected[i]) slaveRunning[i] = true;
        }
        broadcastToAllSlaves("PATTERN_1", 1);
        systemStatus = "PATTERN_1_RUNNING";
      }
      // ── Individual HOME: HOME_3 homes slave 3 ─────────
      else if (msg.startsWith("HOME_")) {
        int targetId = msg.substring(5).toInt();
        if (targetId >= 1 && targetId <= NUM_SLAVES
            && slaveConnected[targetId - 1]) {
          slaveHomed[targetId - 1]   = false;
          slaveRunning[targetId - 1] = true;
          sendToSlave(targetId, "HOME", 1);
          systemStatus = "HOMING";
          Serial.printf("🏠 Homing Slave %d\n", targetId);
        }
      }
      // ── Stop all ──────────────────────────────────────
      else if (msg == "STOP_ALL") {
        autoCycleRunning = false;
        for (int i = 0; i < NUM_SLAVES; i++) slaveRunning[i] = false;
        broadcastToAllSlaves("STOP", 1);
        systemStatus = "STOPPED";
      }
      else if (msg == "REQUEST_STATUS") {
        // Just reply below
      }

      broadcastToReact();
    }
  }
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║  KINETIC RAIN — MASTER CONTROLLER    ║");
  Serial.printf( "║  Active Slaves: %-2d / %-2d               ║\n",
                 ACTIVE_SLAVES, NUM_SLAVES);
  Serial.println("╚══════════════════════════════════════╝\n");

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500); Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n✅ WiFi — IP: %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n❌ WiFi failed");

  Serial.printf("Master MAC: %s\n\n", WiFi.macAddress().c_str());

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onDataReceive);
    for (int i = 0; i < ACTIVE_SLAVES; i++) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, slaveAddresses[i], 6);
      peer.channel = WiFi.channel();
      peer.ifidx   = WIFI_IF_STA;
      if (esp_now_add_peer(&peer) == ESP_OK)
        Serial.printf("✓ Slave %d registered\n", i + 1);
      else
        Serial.printf("✗ Slave %d failed\n", i + 1);
    }
  } else {
    Serial.println("❌ ESP-NOW init failed");
  }

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.begin();

  Serial.println("\n✅ MASTER READY\n");
}

// ═══════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════
void loop() {
  ws.cleanupClients();

  // Heartbeat to slaves — helps maintain channel lock
  if (millis() - lastBeacon > 3000) {
    broadcastToAllSlaves("HEARTBEAT", WiFi.channel());
    lastBeacon = millis();
  }

  // ✅ Check for slave connection timeouts every second
  if (millis() - lastTimeoutCheck > 1000) {
    lastTimeoutCheck = millis();
    bool changed = false;
    for (int i = 0; i < NUM_SLAVES; i++) {
      if (slaveConnected[i] &&
          (millis() - lastSlaveHeartbeat[i] > SLAVE_TIMEOUT_MS)) {
        slaveConnected[i] = false;
        slaveHomed[i]     = false;
        slaveRunning[i]   = false;
        Serial.printf("⚠️ Slave %d timed out — offline\n", i + 1);
        changed = true;
      }
    }
    if (changed) broadcastToReact();
  }
}