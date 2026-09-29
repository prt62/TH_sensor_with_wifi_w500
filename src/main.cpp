// ==========================================================================
// Industrial Sensor Node V16.2 - ESP32 + W5500 + Wi-Fi + AM2301B (I2C) + BT Classic
// Features: Hybrid LAN (DHCP/Static), Dual-WAN Failover, 32-bit Seq, FSM Safe
// V16.2 CHANGES (Production Bug Fixes):
//   1. FIX: Swapped 1-wire DHT logic for native Wire (I2C) to read AM2301B 
//      on pins 21 (SDA) and 22 (SCL).
//   2. FIX: Corrected external watchdog DONE pin to IO32 matching schematic.
//   3. FIX: Defaulted network to STATIC IP (use_dhcp = false) to prevent 
//      hangs if NVS is cleared/factory reset.
//   4. FIX: Added feedWatchdog() around WiFi.begin() and WiFi.reconnect() 
//      to prevent starvation during fallback switching.
//   5. FIX: Replaced manual MAC bit-flip with ESP_MAC_ETH for guaranteed 
//      global collision prevention on production fleets.
// ==========================================================================
// COMMAND LIST (DO NOT MODIFY INSTRUCTIONS):
/*
 AUTH,123456#
 GET#
 WIFI#
 LAN#
 LOG#
 WIFI,Ajeevi_Net,Pass1234#
 SERVER,192.168.1.100,8080#
 DATA,60#
 PASS,newpassword#
 DHCP#
 STATIC#
 NET,192.168.1.100,255.255.255.0,192.168.1.1,8.8.8.8#
 ALERT,T,1,35.0#      
 ALERT,H,0,80.0#      
 REST#
 FACTORY#
*/

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Ethernet.h>
#include "BluetoothSerial.h"
#include <Wire.h>           // [FIX V16.2] Replaced DHT with Native I2C
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>

// --------------------------------------------------------------------------
// 1. PIN DEFINITIONS
// --------------------------------------------------------------------------
#define AM2301B_ADDR 0x38   // [FIX V16.2] AM2301B I2C Address

#define W5500_MOSI 13
#define W5500_MISO 12       // NOTE: GPIO12 is an ESP32 boot-strapping pin (MTDI / flash
                            // voltage select). Confirm your board keeps it low/floating
                            // at power-up for a 3.3V flash, or boot can fail intermittently.
#define W5500_SCLK 14
#define W5500_CS   15
#define W5500_RST  25

#define LED_D3 4
#define LED_D4 2
#define WDT_DONE_PIN 32     // [FIX V16.2] Connected to TPL5010 DONE (TP1)

// --------------------------------------------------------------------------
// 2. GLOBAL VARIABLES
// --------------------------------------------------------------------------
BluetoothSerial SerialBT;
Preferences preferences;

WiFiClient wifiClient;
EthernetClient ethClient;
Client* tcpClient = &wifiClient;

bool nvsReady = false; 

char wifi_ssid[33] = {0};
char wifi_pass[64] = {0};
char server_ip[16] = {0};
int server_port = 0;
int data_freq = 60;

int comm_mode = 2; // Preferred Mode (NVS)
int active_interface = 2; // Actually Running Interface

// V16 Hybrid LAN Variables
bool use_dhcp = false;     // [FIX V16.2] Defaulted to false (Static LAN)
char lan_ip[16] = "192.168.1.100";
char lan_sn[16] = "255.255.255.0";
char lan_gw[16] = "192.168.1.1";
char lan_dns[16] = "8.8.8.8";

uint8_t eth_mac[6];
char g_macStr[13] = "000000000000";

float currentTemp = 0.0;
float currentHum = 0.0;
bool sensorValid = false;
uint8_t sensorFailCount = 0;
uint8_t sensorGoodCount = 0;
bool isContinuousLogActive = false;
unsigned long lastContinuousLogTime = 0;

char bt_pass[32] = {0};
bool btAuthenticated = false;
unsigned long lastAuthTime = 0; 
uint8_t authFailedAttempts = 0;
unsigned long authLockoutTime = 0; 

char tcpRxBuffer[257];
int tcpRxIndex = 0;
bool tcpRxOverflow = false;
unsigned long lastTcpRxActivity = 0; 

char btRxBuffer[128];
int btRxIndex = 0;
bool btRxOverflow = false;

enum ConnState {
    NET_DISCONNECTED,
    NET_CONNECTING,
    NET_CONNECTED
};
ConnState netState = NET_DISCONNECTED;
unsigned long connectStartedAt = 0;
const unsigned long CONNECT_TIMEOUT_MS = 2000; 

struct TcpState {
    unsigned long lastDataSendTime = 0;
    uint32_t packetSerialNum = 0; 
    bool isTimeSynced = false;
    unsigned long lastTimeReqSent = 0;
    bool wasConnected = false;
    unsigned long lastConnTryTime = 0;
};
TcpState tcpState;

struct TcpAckState {
    char lastSentPacket[256] = {0};
    bool isWaitingForAck = false;
    unsigned long packetSentAt = 0;
};
TcpAckState ackState;
int retryCount = 0;
unsigned long ACK_TIMEOUT_MS = 30000;

struct AlertConfig {
    float threshold = 0.0f;
    bool enabled = false;
};
AlertConfig tempAlert;
AlertConfig humAlert;

// --------------------------------------------------------------------------
// 3. HELPERS & FAILOVER LOGIC
// --------------------------------------------------------------------------
void btLog(const char* msg) {
    Serial.println(msg);
    if (SerialBT.hasClient()) {
        SerialBT.print(msg);
        SerialBT.print("\r\n");
    }
}

void safeSaveString(const char* key, const char* val) {
    if (!nvsReady) return;
    String old = preferences.getString(key, "");
    if (!old.equals(val)) preferences.putString(key, val);
}

void safeSaveInt(const char* key, int val) {
    if (!nvsReady) return;
    if (preferences.getInt(key, -1) != val) preferences.putInt(key, val);
}

bool isValidIP(const char* ip) {
    int a, b, c, d;
    char extra;
    if (sscanf(ip, "%d.%d.%d.%d%c", &a, &b, &c, &d, &extra) != 4) return false;
    return a >= 0 && a <= 255 && b >= 0 && b <= 255 && c >= 0 && c <= 255 && d >= 0 && d <= 255;
}

void feedWatchdog() {
    static unsigned long lastWdtFeed = 0;
    if (millis() - lastWdtFeed > 1000) {
        lastWdtFeed = millis();
        digitalWrite(WDT_DONE_PIN, HIGH);
        delayMicroseconds(50); 
        digitalWrite(WDT_DONE_PIN, LOW);
    }
}

void handleLEDs() {
    static unsigned long lastBlink = 0;
    static bool toggleState = false;
    
    int blinkRate = 1000; 
    if (!sensorValid) blinkRate = 100; 
    else if (!tcpClient->connected() || !tcpState.isTimeSynced) blinkRate = 250; 
    
    if (millis() - lastBlink > blinkRate) {
        lastBlink = millis();
        toggleState = !toggleState;
        if (active_interface == 1) { 
            digitalWrite(LED_D4, HIGH);
            digitalWrite(LED_D3, toggleState);
        } else {
            digitalWrite(LED_D3, HIGH);
            digitalWrite(LED_D4, toggleState);
        }
    }
}

void checkDailyRestart() {
    static const unsigned long RESTART_INTERVAL = 86400000UL; 
    if (millis() > RESTART_INTERVAL) {
        btLog("[SYS] 24-Hour Uptime Reached. Auto-Restarting...");
        delay(1000); 
        esp_restart();
    }
}

void resetNetworkRetryState() {
    netState = NET_DISCONNECTED;
    connectStartedAt = 0;
    tcpState.lastConnTryTime = 0;
    tcpState.wasConnected = false;
    ackState.isWaitingForAck = false;
    retryCount = 0;
    tcpRxIndex = 0;
    tcpRxOverflow = false;
    lastTcpRxActivity = millis();
}

void initEthernet() {
    feedWatchdog();
    if (use_dhcp) {
        Ethernet.begin(eth_mac, 2000, 1000); // DHCP Call
    } else {
        IPAddress ip, sn, gw, dns;
        ip.fromString(lan_ip);
        sn.fromString(lan_sn);
        gw.fromString(lan_gw);
        dns.fromString(lan_dns);
        Ethernet.begin(eth_mac, ip, dns, gw, sn); // Static Call
    }
    feedWatchdog();
}

void activateInterface(int mode) {
    active_interface = mode;
    wifiClient.stop();
    ethClient.stop();
    resetNetworkRetryState();

    if (active_interface == 2) {
        tcpClient = &wifiClient;
        if (wifi_ssid[0] != '\0' && WiFi.getMode() != WIFI_STA) {
            WiFi.mode(WIFI_STA);
            feedWatchdog(); // [FIX V16.2] Prevent starvation
            WiFi.begin(wifi_ssid, wifi_pass);
            feedWatchdog();
        }
        btLog("[SYS] Interface Active: Wi-Fi");
    } else {
        tcpClient = &ethClient;
        if (comm_mode == 1) {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
        }
        btLog("[SYS] Interface Active: LAN");
        if (Ethernet.linkStatus() == LinkON) initEthernet(); 
    }
}

void switchCommunicationMode(int mode) {
    comm_mode = mode;
    safeSaveInt("c_mode", comm_mode);
    btLog(mode == 2 ? "[CMD] Preferred Mode: Wi-Fi" : "[CMD] Preferred Mode: LAN");
    activateInterface(mode);
}

void handleAutoFailover() {
    static unsigned long lastWifiConnectedTime = millis();
    static unsigned long lastFailoverCheck = 0;
    
    if (millis() - lastFailoverCheck < 2000) return; 
    lastFailoverCheck = millis();

    bool lanLink = (Ethernet.linkStatus() == LinkON);
    bool wifiConfigured = (wifi_ssid[0] != '\0');
    bool wifiConnected = (WiFi.status() == WL_CONNECTED);

    if (wifiConnected) lastWifiConnectedTime = millis();

    if (comm_mode == 1) { 
        if (active_interface == 1 && !lanLink && wifiConfigured) {
            btLog("[FAILOVER] LAN Cable Cut! Switching to Wi-Fi Backup...");
            activateInterface(2);
        }
        else if (active_interface == 2 && lanLink) {
            btLog("[FAILOVER] LAN Cable Restored! Reverting to Primary (LAN)...");
            activateInterface(1);
        }
    }
    else if (comm_mode == 2) { 
        if (active_interface == 2) {
            if (millis() - lastWifiConnectedTime > 60000UL && lanLink) {
                btLog("[FAILOVER] Wi-Fi unreachable for 60s! Switching to LAN Backup...");
                activateInterface(1);
            }
        }
        else if (active_interface == 1) { 
            if (wifiConnected) { 
                btLog("[FAILOVER] Wi-Fi Restored! Reverting to Primary (Wi-Fi)...");
                activateInterface(2);
                lastWifiConnectedTime = millis();
            }
            else if (!lanLink) { 
                btLog("[FAILOVER] LAN Backup Lost! Reverting to Wi-Fi...");
                activateInterface(2);
                lastWifiConnectedTime = millis(); 
            }
        }
    }
}

// --------------------------------------------------------------------------
// 4. NON-BLOCKING BLUETOOTH COMMAND PARSER
// --------------------------------------------------------------------------
void processBtCommand(char* cmd) {
    if (strncmp(cmd, "AUTH,", 5) == 0) {
        if (authFailedAttempts >= 5) {
            if (millis() > authLockoutTime) authFailedAttempts = 0;
            else {
                btLog("[AUTH_ERR] Locked out. Please wait.");
                return;
            }
        }
        if (strcmp(cmd + 5, bt_pass) == 0) {
            btAuthenticated = true;
            authFailedAttempts = 0;
            lastAuthTime = millis();
            btLog("[AUTH] SUCCESS. Device Unlocked for 5 mins.");
        } else {
            authFailedAttempts++;
            btLog("[AUTH] FAILED.");
            if (authFailedAttempts >= 5) {
                authLockoutTime = millis() + 60000UL; 
                btLog("[AUTH_ERR] Too many failures. Locked for 60s.");
            }
        }
        return;
    }
    else if (strcmp(cmd, "GET") == 0) {
        char statusMsg[512];
        const char* actStr = (active_interface == 2) ? "Wi-Fi" : "LAN";
        const char* prefStr = (comm_mode == 2) ? "Wi-Fi" : "LAN";
        const char* ipMode = use_dhcp ? "DHCP" : "STATIC";

        if (btAuthenticated) {
            lastAuthTime = millis(); 
            snprintf(statusMsg, sizeof(statusMsg),
                "=== System Status (Full) ===\r\n"
                "MAC: %s\r\nACTIVE: %s (Pref: %s)\r\n"
                "LAN MODE: %s\r\n"
                "SSID: %s\r\nPASS: ********\r\n" 
                "SERVER: %s:%d\r\nSYNC: %s\r\nFREQ: %d sec\r\n"
                "SENSOR: %s (T:%.1f H:%.1f)\r\n"
                "ALERT: T-en:%d T-thr:%.1f | H-en:%d H-thr:%.1f\r\n",
                g_macStr, actStr, prefStr, ipMode,
                wifi_ssid, server_ip, server_port,
                tcpState.isTimeSynced ? "YES" : "NO", data_freq,
                sensorValid ? "OK" : "FAULT", currentTemp, currentHum,
                tempAlert.enabled ? 1 : 0, tempAlert.threshold,
                humAlert.enabled ? 1 : 0, humAlert.threshold
            );
        } else {
            snprintf(statusMsg, sizeof(statusMsg),
                "=== System Status (Public) ===\r\n"
                "AUTH: LOCKED\r\nMAC: %s\r\nACTIVE: %s (Pref: %s)\r\n"
                "SYNC: %s\r\nSENSOR: %s (T:%.1f H:%.1f)\r\n",
                g_macStr, actStr, prefStr,
                tcpState.isTimeSynced ? "YES" : "NO",
                sensorValid ? "OK" : "FAULT", currentTemp, currentHum
            );
        }
        btLog(statusMsg);
        return;
    }

    if (!btAuthenticated) {
        btLog("[ERR] Send AUTH,password# first.");
        return;
    }
    lastAuthTime = millis(); 

    if (strcmp(cmd, "WIFI") == 0) switchCommunicationMode(2);
    else if (strcmp(cmd, "LAN") == 0) switchCommunicationMode(1);
    else if (strcmp(cmd, "LOG") == 0) {
        isContinuousLogActive = !isContinuousLogActive;
        btLog(isContinuousLogActive ? "[CMD] Log STARTED" : "[CMD] Log STOPPED");
    }
    else if (strcmp(cmd, "DHCP") == 0) {
        use_dhcp = true;
        safeSaveInt("l_dhcp", 1);
        btLog("[CMD] LAN MODE SET TO DHCP");
        if (active_interface == 1) activateInterface(comm_mode);
    }
    else if (strcmp(cmd, "STATIC") == 0) {
        use_dhcp = false;
        safeSaveInt("l_dhcp", 0);
        btLog("[CMD] LAN MODE SET TO STATIC");
        if (active_interface == 1) activateInterface(comm_mode);
    }
    else if (strcmp(cmd, "FACTORY") == 0) {
        btLog("[CMD] FACTORY RESET! Restarting...");
        preferences.clear(); delay(500); esp_restart();
    }
    else if (strcmp(cmd, "REST") == 0) {
        btLog("[CMD] RESTARTING...");
        delay(500); esp_restart();
    }
    else if (strncmp(cmd, "PASS,", 5) == 0) {
        int passLen = strlen(cmd) - 5;
        if (passLen < 1 || passLen > 31) {
            btLog("[ERR] Password must be 1-31 chars");
        } else {
            strncpy(bt_pass, cmd + 5, sizeof(bt_pass) - 1);
            bt_pass[sizeof(bt_pass) - 1] = '\0';
            safeSaveString("bt_pass", bt_pass);
            btAuthenticated = false; 
            btLog("[CMD] AUTH PASSWORD UPDATED. Session locked.");
        }
    }
    else if (strncmp(cmd, "NET,", 4) == 0) {
        char* c1 = strchr(cmd, ',');
        char* c2 = c1 ? strchr(c1 + 1, ',') : NULL;
        char* c3 = c2 ? strchr(c2 + 1, ',') : NULL;
        char* c4 = c3 ? strchr(c3 + 1, ',') : NULL;
        if (c1 && c2 && c3 && c4) {
            *c2 = '\0'; *c3 = '\0'; *c4 = '\0';
            if (isValidIP(c1+1) && isValidIP(c2+1) && isValidIP(c3+1) && isValidIP(c4+1)) {
                strncpy(lan_ip, c1+1, 15); lan_ip[15] = '\0';
                strncpy(lan_sn, c2+1, 15); lan_sn[15] = '\0';
                strncpy(lan_gw, c3+1, 15); lan_gw[15] = '\0';
                strncpy(lan_dns, c4+1, 15); lan_dns[15] = '\0';
                safeSaveString("l_ip", lan_ip);
                safeSaveString("l_sn", lan_sn);
                safeSaveString("l_gw", lan_gw);
                safeSaveString("l_dns", lan_dns);
                btLog("[CMD] STATIC NET CONFIG UPDATED");
                if (active_interface == 1 && !use_dhcp) activateInterface(comm_mode);
            } else btLog("[ERR] Invalid IP Configuration");
        }
    }
    else if (strncmp(cmd, "WIFI,", 5) == 0) {
        char* c1 = strchr(cmd, ',');
        char* c2 = c1 ? strchr(c1 + 1, ',') : NULL;
        if (c1 && c2) {
            *c2 = '\0';
            strncpy(wifi_ssid, c1 + 1, sizeof(wifi_ssid) - 1);
            wifi_ssid[sizeof(wifi_ssid) - 1] = '\0';
            strncpy(wifi_pass, c2 + 1, sizeof(wifi_pass) - 1);
            wifi_pass[sizeof(wifi_pass) - 1] = '\0';
            safeSaveString("w_ssid", wifi_ssid);
            safeSaveString("w_pass", wifi_pass);
            btLog("[CMD] WIFI UPDATED");
            if (active_interface == 2) {
                wifiClient.stop();
                resetNetworkRetryState();
                WiFi.disconnect(true);
                feedWatchdog();
                WiFi.begin(wifi_ssid, wifi_pass);
                feedWatchdog();
            }
        }
    }
    else if (strncmp(cmd, "SERVER,", 7) == 0) {
        char* c1 = strchr(cmd, ',');
        char* c2 = c1 ? strchr(c1 + 1, ',') : NULL;
        if (c1 && c2) {
            *c2 = '\0';
            char tempIP[16] = {0};
            strncpy(tempIP, c1 + 1, sizeof(tempIP) - 1);
            tempIP[sizeof(tempIP) - 1] = '\0';
            char* endPtr;
            long tempPortLong = strtol(c2 + 1, &endPtr, 10);

            if (isValidIP(tempIP) && *endPtr == '\0' && tempPortLong >= 1 && tempPortLong <= 65535) {
                strncpy(server_ip, tempIP, sizeof(server_ip) - 1);
                server_ip[sizeof(server_ip) - 1] = '\0';
                server_port = (int)tempPortLong;
                safeSaveString("s_ip", server_ip);
                safeSaveInt("s_port", server_port);
                tcpClient->stop();
                resetNetworkRetryState();
                btLog("[CMD] SERVER UPDATED");
            } else btLog("[ERR] Invalid IP/Port");
        }
    }
    else if (strncmp(cmd, "DATA,", 5) == 0) {
        char* c1 = strchr(cmd, ',');
        if (c1) {
            char* endPtr;
            long val = strtol(c1 + 1, &endPtr, 10);
            if (*endPtr == '\0' && val >= 15 && val <= 3600) {
                data_freq = (int)val;
                safeSaveInt("d_freq", data_freq);
                ACK_TIMEOUT_MS = (data_freq > 30) ? 30000 : (data_freq * 1000) / 2;
                btLog("[CMD] DATA FREQ UPDATED");
            } else btLog("[ERR] Freq must be 15-3600");
        }
    }
    else if (strncmp(cmd, "ALERT,", 6) == 0) {
        char* c1 = strchr(cmd, ',');
        char* c2 = c1 ? strchr(c1 + 1, ',') : NULL;
        char* c3 = c2 ? strchr(c2 + 1, ',') : NULL;
        if (c1 && c2 && c3) {
            *c2 = '\0'; *c3 = '\0';
            const char* type = c1 + 1;      // "T" or "H"
            int en = atoi(c2 + 1);
            char* endPtr;
            float thr = strtof(c3 + 1, &endPtr);
            bool validPayload = (*endPtr == '\0') && (en == 0 || en == 1);

            if (!validPayload) {
                btLog("[ERR] Invalid ALERT payload");
            } else if (strcmp(type, "T") == 0) {
                tempAlert.enabled = (en == 1);
                tempAlert.threshold = thr;
                if (nvsReady) {
                    preferences.putUChar("tmp_en", (uint8_t)en);
                    preferences.putFloat("tmp_thr", thr);
                }
                btLog("[CMD] TEMP ALERT UPDATED");
            } else if (strcmp(type, "H") == 0) {
                humAlert.enabled = (en == 1);
                humAlert.threshold = thr;
                if (nvsReady) {
                    preferences.putUChar("hum_en", (uint8_t)en);
                    preferences.putFloat("hum_thr", thr);
                }
                btLog("[CMD] HUM ALERT UPDATED");
            } else {
                btLog("[ERR] ALERT type must be T or H");
            }
        } else btLog("[ERR] Invalid ALERT format");
    }
    else btLog("[ERR] Unknown command.");
}

void handleBluetoothIncoming() {
    bool isBtConnected = SerialBT.hasClient();
    if (btAuthenticated && (millis() - lastAuthTime > 300000UL)) {
        btAuthenticated = false;
        btLog("[SYS] Auth session timeout. Locked.");
    }
    
    static bool wasBtConnected = false;
    if (wasBtConnected && !isBtConnected) btAuthenticated = false;
    wasBtConnected = isBtConnected;

    int bytesProcessed = 0;
    while (SerialBT.available() && bytesProcessed++ < 128) {
        char c = SerialBT.read();
        if (c == '\n' || c == '\r') continue; 
        
        if (btRxOverflow) {
            if (c == '#') { btRxIndex = 0; btRxOverflow = false; }
            continue;
        } 
        
        if (btRxIndex < (int)sizeof(btRxBuffer) - 1) {
            btRxBuffer[btRxIndex++] = c;
            if (c == '#') { 
                btRxBuffer[btRxIndex - 1] = '\0'; 
                processBtCommand(btRxBuffer);
                btRxIndex = 0;
            }
        } else {
            btRxOverflow = true;
            btLog("[BT_ERR] Cmd too long. Discarded.");
        }
    }
}

// --------------------------------------------------------------------------
// 5. TCP PROTOCOL HANDLERS
// --------------------------------------------------------------------------
void dropDeadConnection(const char* reason) {
    btLog(reason);
    tcpClient->stop();
    resetNetworkRetryState();
}

bool sendDataPacket() {
    struct tm timeinfo;
    char timeStr[10] = "000000";
    char dateStr[10] = "00000000";

    if (tcpState.isTimeSynced && getLocalTime(&timeinfo, 10)) {
        strftime(timeStr, sizeof(timeStr), "%H%M%S", &timeinfo);
        strftime(dateStr, sizeof(dateStr), "%d%m%Y", &timeinfo);
    }

    int alertCode = 101;
    if (!sensorValid) alertCode = 999;
    else if (tempAlert.enabled && currentTemp >= tempAlert.threshold) alertCode = 201;
    else if (humAlert.enabled && currentHum >= humAlert.threshold) alertCode = 202;

    int dataSource = (active_interface == 2) ? 1 : 2; 
    int sigStrength = (active_interface == 2) ? abs(WiFi.RSSI()) : 0;
    float sendTemp = sensorValid ? currentTemp : -999.0;
    float sendHum = sensorValid ? currentHum : -999.0;

    int n = snprintf(ackState.lastSentPacket, sizeof(ackState.lastSentPacket),
             "*@,1001,%s,1,%d,%u,%s,%s,1226,%d,1,%d,%.2f,%.2f,##\r\n",
             g_macStr, dataSource, (unsigned int)tcpState.packetSerialNum,
             timeStr, dateStr, sigStrength, alertCode, sendTemp, sendHum);

    if (n < 0 || n >= (int)sizeof(ackState.lastSentPacket)) {
        dropDeadConnection("[NET_ERR_TX_FORMAT] Pkt truncated!");
        return false;
    }

    char logBuf[64];
    snprintf(logBuf, sizeof(logBuf), "[TCP_TX] Sending Pkt %u...", tcpState.packetSerialNum);
    btLog(logBuf);

    size_t len = strlen(ackState.lastSentPacket);
    if (tcpClient->print(ackState.lastSentPacket) != len) {
        dropDeadConnection("[NET_ERR_TX_FAILED] Dropping connection.");
        return false;
    }

    btLog("[TCP_TX] Pkt TX OK. Waiting ACK...");
    ackState.packetSentAt = millis();
    ackState.isWaitingForAck = true;
    retryCount = 0;
    return true;
}

void handleTcp() {
    if (server_ip[0] == '\0' || server_port == 0) return;

    bool interfaceReady = false;
    if (active_interface == 2 && WiFi.status() == WL_CONNECTED) interfaceReady = true;
    if (active_interface == 1 && Ethernet.linkStatus() == LinkON && Ethernet.localIP() != IPAddress(0, 0, 0, 0)) interfaceReady = true;

    if (!interfaceReady) {
        if (netState != NET_DISCONNECTED) {
            if(active_interface == 2) btLog("[NET_ERR_WIFI_LOST] Interface down.");
            else btLog("[NET_ERR_LAN_LINK_DOWN] Interface down.");
            tcpClient->stop();
            resetNetworkRetryState();
        }
        return;
    }

    if (!tcpClient->connected()) {
        if (tcpState.wasConnected) {
            dropDeadConnection("[TCP] Connection Lost. States Cleared.");
            return;
        }

        if (netState == NET_CONNECTING) {
            if (millis() - connectStartedAt > CONNECT_TIMEOUT_MS) {
                netState = NET_DISCONNECTED;
            } else return; 
        }

        if (netState == NET_DISCONNECTED && millis() - tcpState.lastConnTryTime > 5000) {
            tcpState.lastConnTryTime = millis();
            netState = NET_CONNECTING;
            connectStartedAt = millis();
            btLog("[TCP] Connecting to Server...");

            feedWatchdog();
            bool ok = false;
            if (active_interface == 2) {
                ok = wifiClient.connect(server_ip, server_port, 1500); 
            } else {
                ok = ethClient.connect(server_ip, server_port); 
            }
            feedWatchdog();

            if (ok) {
                tcpClient->setTimeout(200);
                tcpState.wasConnected = true;
                tcpState.lastTimeReqSent = 0;
                netState = NET_CONNECTED;
                btLog("[TCP] Connected!");
            } else {
                netState = NET_DISCONNECTED;
                btLog("[NET_ERR_SERVER_TIMEOUT] Connect attempt failed.");
            }
        }
        return;
    } else netState = NET_CONNECTED;

    if (!tcpState.isTimeSynced) {
        if (tcpState.lastTimeReqSent == 0 || millis() - tcpState.lastTimeReqSent >= 15000) {
            tcpState.lastTimeReqSent = millis();
            const char* timeReq = "*@,3,205,##\r\n";
            btLog("[TCP_TX] Requesting Time Sync");
            if (tcpClient->print(timeReq) != strlen(timeReq)) {
                dropDeadConnection("[NET_ERR_TX_FAILED] Time Sync TX Failed!");
            }
        }
        return;
    }

    if (ackState.isWaitingForAck) {
        if (millis() - ackState.packetSentAt >= ACK_TIMEOUT_MS) {
            if (retryCount < 3) {
                retryCount++;
                btLog("[TCP_RETRY] Retrying Pkt.");
                if (tcpClient->print(ackState.lastSentPacket) != strlen(ackState.lastSentPacket)) {
                    dropDeadConnection("[NET_ERR_TX_FAILED] Retry TX Failed!");
                    return;
                }
                ackState.packetSentAt = millis();
            } else {
                btLog("[NET_ERR_ACK_TIMEOUT] Max retries. Dropped Pkt.");
                ackState.isWaitingForAck = false;
                if (++tcpState.packetSerialNum > 999999) tcpState.packetSerialNum = 0;
                tcpState.lastDataSendTime = millis();
            }
        }
        return;
    }

    if (data_freq > 0 && (millis() - tcpState.lastDataSendTime >= (data_freq * 1000UL))) {
        if (sendDataPacket()) {
            tcpState.lastDataSendTime = millis();
        }
    }
}

void parseTcpMessage() {
    if (tcpRxIndex < 5) return;

    if (!tcpState.isTimeSynced && strncmp(tcpRxBuffer, "*@,3,205,", 9) == 0) {
        int hr, mn, sc, day, mon, yr;
        if (sscanf(tcpRxBuffer, "*@,3,205,%d,%d,%d,%d,%d,%d,##", &hr, &mn, &sc, &day, &mon, &yr) == 6) {
            if (hr >= 0 && hr <= 23 && mn >= 0 && mn <= 59 && sc >= 0 && sc <= 59 &&
                day >= 1 && day <= 31 && mon >= 1 && mon <= 12 && yr >= 2000 && yr <= 2099) {

                struct tm tm_s = {};
                tm_s.tm_hour = hr; tm_s.tm_min = mn; tm_s.tm_sec = sc;
                tm_s.tm_mday = day; tm_s.tm_mon = mon - 1;
                tm_s.tm_year = yr - 1900;
                tm_s.tm_isdst = -1;

                struct tm check = tm_s;
                time_t t = mktime(&tm_s);
                if (t == (time_t)-1) {
                    btLog("[TCP_ERR] mktime failed"); return;
                }
                if (tm_s.tm_mday != check.tm_mday || tm_s.tm_mon != check.tm_mon || tm_s.tm_year != check.tm_year) {
                    btLog("[TCP_ERR] Invalid calendar date rejected"); return;
                }

                struct timeval tv = {.tv_sec = t, .tv_usec = 0};
                if (settimeofday(&tv, NULL) == 0) {
                    tcpState.isTimeSynced = true;
                    tcpState.lastDataSendTime = millis() - (data_freq * 1000UL);
                    btLog("[TCP_RX] Time Synced (IST)!");
                } else btLog("[TCP_ERR] settimeofday failed");
            } else btLog("[TCP_ERR] Time payload out of bounds");
        }
    }
    else if (tcpState.isTimeSynced && strncmp(tcpRxBuffer, "@$,2,", 5) == 0) {
        uint32_t ackNum = 0; int status = -1;
        char extra[3] = {0}; 
        if (sscanf(tcpRxBuffer, "@$,2,%u,%d,%2s", &ackNum, &status, extra) == 3 && strcmp(extra, "##") == 0) {
            if (ackNum == tcpState.packetSerialNum && (status == 101 || status == 201 || status == 202 || status == 999 || status == 1)) {
                ackState.isWaitingForAck = false;
                if (++tcpState.packetSerialNum > 999999) tcpState.packetSerialNum = 0;
                if (tcpState.packetSerialNum % 50 == 0) safeSaveInt("p_seq", tcpState.packetSerialNum);
                btLog("[TCP_RX] ACK OK.");
            } else btLog("[TCP_RX] ACK ignored (seq/status mismatch).");
        } else btLog("[TCP_ERR] ACK payload malformed");
    }
}

void handleTcpIncoming() {
    static char lastChar = 0;
    if (tcpClient->available()) lastTcpRxActivity = millis();

    int bytesProcessed = 0;
    while (tcpClient->available() && bytesProcessed++ < 256) {
        char c = tcpClient->read();
        if (c == '\n' || c == '\r') continue;

        if (tcpRxOverflow) {
            if (lastChar == '#' && c == '#') {
                tcpRxIndex = 0; tcpRxOverflow = false; lastChar = 0; continue;
            }
            lastChar = c; continue;
        }

        if (tcpRxIndex < 255) {
            tcpRxBuffer[tcpRxIndex++] = c;
            tcpRxBuffer[tcpRxIndex] = '\0';
        } else {
            tcpRxOverflow = true; lastChar = c; continue;
        }

        if (tcpRxIndex >= 2 && tcpRxBuffer[tcpRxIndex - 2] == '#' && tcpRxBuffer[tcpRxIndex - 1] == '#') {
            parseTcpMessage();
            tcpRxIndex = 0; lastChar = 0; continue;
        }
        lastChar = c;
    }

    if ((tcpRxIndex > 0 || tcpRxOverflow) && millis() - lastTcpRxActivity > 10000) {
        btLog("[NET_ERR_RX_TIMEOUT] Resync: clearing stale buffer.");
        tcpRxIndex = 0; tcpRxOverflow = false; lastChar = 0;
    }
}

// --------------------------------------------------------------------------
// 6. SETUP()
// --------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    setenv("TZ", "IST-5:30", 1);
    tzset();

    pinMode(LED_D3, OUTPUT);
    pinMode(LED_D4, OUTPUT);
    pinMode(WDT_DONE_PIN, OUTPUT);
    digitalWrite(WDT_DONE_PIN, LOW);

    Serial.println("\n--- Industrial Sensor Node V16.2 ---");
    
    if (preferences.begin("sensor_app", false)) nvsReady = true;
    else Serial.println("[ERR] NVS Init Failed! Proceeding with defaults.");

    String t_btpass = preferences.getString("bt_pass", "123456");
    strncpy(bt_pass, t_btpass.c_str(), sizeof(bt_pass) - 1);
    bt_pass[sizeof(bt_pass) - 1] = '\0';

    String t_ssid = preferences.getString("w_ssid", "");
    String t_pass = preferences.getString("w_pass", "");
    String t_ip = preferences.getString("s_ip", "");

    strncpy(wifi_ssid, t_ssid.c_str(), sizeof(wifi_ssid) - 1);
    wifi_ssid[sizeof(wifi_ssid) - 1] = '\0';
    strncpy(wifi_pass, t_pass.c_str(), sizeof(wifi_pass) - 1);
    wifi_pass[sizeof(wifi_pass) - 1] = '\0';
    
    use_dhcp = (preferences.getInt("l_dhcp", 0) == 1); // [FIX V16.2] Safely defaults to STATIC
    String t_lan_ip = preferences.getString("l_ip", "192.168.1.100");
    String t_lan_sn = preferences.getString("l_sn", "255.255.255.0");
    String t_lan_gw = preferences.getString("l_gw", "192.168.1.1");
    String t_lan_dns = preferences.getString("l_dns", "8.8.8.8");

    strncpy(lan_ip, t_lan_ip.c_str(), 15); lan_ip[15] = '\0';
    strncpy(lan_sn, t_lan_sn.c_str(), 15); lan_sn[15] = '\0';
    strncpy(lan_gw, t_lan_gw.c_str(), 15); lan_gw[15] = '\0';
    strncpy(lan_dns, t_lan_dns.c_str(), 15); lan_dns[15] = '\0';

    if (isValidIP(t_ip.c_str())) {
        strncpy(server_ip, t_ip.c_str(), sizeof(server_ip) - 1);
        server_ip[sizeof(server_ip) - 1] = '\0';
    } else if (t_ip.length() > 0) Serial.println("[NVS] Invalid server IP found. Ignoring.");

    server_port = preferences.getInt("s_port", 0);
    if (server_port < 1 || server_port > 65535) {
        Serial.println("[NVS] Invalid server port. Defaulting to 0.");
        server_port = 0;
    }

    int savedFreq = preferences.getInt("d_freq", 60);
    if (savedFreq < 15 || savedFreq > 3600) { data_freq = 60; safeSaveInt("d_freq", 60); }
    else data_freq = savedFreq;

    int savedMode = preferences.getInt("c_mode", 2);
    if (savedMode != 1 && savedMode != 2) { comm_mode = 2; safeSaveInt("c_mode", 2); }
    else comm_mode = savedMode;

    int savedSeq = preferences.getInt("p_seq", 0);
    if (savedSeq < 0 || savedSeq > 999999) savedSeq = 0;
    savedSeq += 50;
    if (savedSeq > 999999) savedSeq = 0;
    tcpState.packetSerialNum = savedSeq;
    safeSaveInt("p_seq", savedSeq);

    tempAlert.enabled = (preferences.getUChar("tmp_en", 0) == 1);
    tempAlert.threshold = preferences.getFloat("tmp_thr", 0.0f);
    humAlert.enabled = (preferences.getUChar("hum_en", 0) == 1);
    humAlert.threshold = preferences.getFloat("hum_thr", 0.0f);

    ACK_TIMEOUT_MS = (data_freq > 30) ? 30000 : (data_freq * 1000) / 2;

    uint64_t chipid = ESP.getEfuseMac();
    uint8_t *mac = (uint8_t *)&chipid;
    snprintf(g_macStr, sizeof(g_macStr), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // [FIX V16.2] Requesting the genuine ESP32 factory Ethernet MAC for industrial deployments
    esp_read_mac(eth_mac, ESP_MAC_ETH);

    Wire.begin(21, 22); // [FIX V16.2] Replaces dht.begin(). Initializes I2C on SDA(21), SCL(22).
    
    String btName = "Ajeevi_TH_" + String(g_macStr).substring(8);
    SerialBT.begin(btName);

    pinMode(W5500_RST, OUTPUT);
    digitalWrite(W5500_RST, LOW); delay(50);
    digitalWrite(W5500_RST, HIGH); delay(200);

    SPI.begin(W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS);
    Ethernet.init(W5500_CS);
    ethClient.setConnectionTimeout(1000); 

    activateInterface(comm_mode); 
    lastTcpRxActivity = millis();
}

// --------------------------------------------------------------------------
// 7. MAIN LOOP()
// --------------------------------------------------------------------------
void loop() {
    feedWatchdog(); 
    checkDailyRestart();
    handleLEDs();
    handleBluetoothIncoming();
    handleAutoFailover(); 

    if (isContinuousLogActive && SerialBT.hasClient() && (millis() - lastContinuousLogTime > 1000)) {
        lastContinuousLogTime = millis();
        char lb[64];
        snprintf(lb, sizeof(lb), "[LOG] T:%.1f H:%.1f TCP:%s", currentTemp, currentHum, tcpClient->connected() ? "OK" : "DOWN");
        btLog(lb);
    }

    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead > 2000) {
        lastSensorRead = millis();
        
        // [FIX V16.2] Trigger Native I2C Measurement 
        Wire.beginTransmission(AM2301B_ADDR);
        Wire.write(0xAC);
        Wire.write(0x33);
        Wire.write(0x00);
        Wire.endTransmission();
        
        feedWatchdog();
        delay(80); // Measurement time required by AM2301B
        feedWatchdog();
        
        float t = NAN;
        float h = NAN;
        
        Wire.requestFrom(AM2301B_ADDR, (uint8_t)6);
        if (Wire.available() == 6) {
            uint8_t data[6];
            for (int i = 0; i < 6; i++) data[i] = Wire.read();
            
            if ((data[0] & 0x80) == 0) { // Check if not busy
                uint32_t hum_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
                uint32_t temp_raw = (((uint32_t)(data[3] & 0x0F)) << 16) | ((uint32_t)data[4] << 8) | data[5];
                
                h = ((float)hum_raw * 100.0) / 1048576.0;
                t = ((float)temp_raw * 200.0) / 1048576.0 - 50.0;
            }
        }

        if (!isnan(t) && !isnan(h) && t >= -40.0 && t <= 80.0 && h >= 0.0 && h <= 100.0) {
            sensorFailCount = 0;
            if (sensorGoodCount < 3) sensorGoodCount++;
            if (sensorGoodCount >= 3) {
                currentTemp = t; currentHum = h;
                sensorValid = true;
            }
        } else {
            sensorGoodCount = 0;
            if (sensorFailCount < 3) sensorFailCount++;
            if (sensorFailCount >= 3) sensorValid = false;
        }
    }

    if (comm_mode == 2 || active_interface == 2) {
        static unsigned long lastWifiRetry = 0;
        if (WiFi.status() != WL_CONNECTED && wifi_ssid[0] != '\0') {
            if (millis() - lastWifiRetry > 15000) {
                lastWifiRetry = millis();
                feedWatchdog();     // [FIX V16.2] Protect reconnect blocks
                WiFi.reconnect();
                feedWatchdog();
            }
        }
    }
    
    // Dynamic Ethernet polling
    if (comm_mode == 1 || active_interface == 1)
     {
        if (use_dhcp) Ethernet.maintain(); // Lease renewal
        static unsigned long lastLanRetry = 0;
        if (Ethernet.linkStatus() == LinkON && Ethernet.localIP() == IPAddress(0, 0, 0, 0))
         {
            if (millis() - lastLanRetry > 10000) {
                lastLanRetry = millis();
                initEthernet(); 
            }
        }
    }

    handleTcp();
    handleTcpIncoming();
}