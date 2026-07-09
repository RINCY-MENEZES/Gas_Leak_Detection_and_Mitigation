#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─── USER CONFIGURATION ──────────────────────────────────────────────────────
#define ALERT_PHONE_NUMBER      "+1234567890"  // Replace with your phone number in international format

#define MQ6_WARNING_THRESHOLD   1000
#define MQ6_DANGER_THRESHOLD    1400
#define TEMP_DANGER_THRESHOLD   60.0f
#define MQ6_WARMUP_MS           60000UL
#define SMS_COOLDOWN_MS         30000UL
#define CALL_COOLDOWN_MS        60000UL
#define CALL_RING_DURATION_MS   20000UL
#define CALL_MAX_RETRIES        3
#define ADC_SAMPLES             10
#define SERIAL_BAUD             115200
#define GSM_BAUD                9600

// SIM800L 2G network constants
#define NET_REG_TIMEOUT_MS      45000UL // Max time to wait for network registration (ms) per attempt
#define NET_REG_MAX_ATTEMPTS    3 // Number of full registration attempts before giving up
#define NET_REFRESH_INTERVAL_MS 30000UL // How often to refresh signal and network status in normal loop (ms)
#define COPS_SCAN_TIMEOUT_MS    90000UL // AT+COPS=? scan timeout — can be up to 3 minutes on some networks

// OLED
#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_RESET_PIN          -1
#define OLED_I2C_ADDR           0x3C

// ─── PIN DEFINITIONS ─────────────────────────────────────────────────────────
#define PIN_MQ6_AOUT        0
#define PIN_DS18B20         2
#define PIN_GSM_TX          21
#define PIN_GSM_RX          20
#define PIN_RELAY_VALVE     9
#define PIN_RELAY_FAN       8
#define PIN_BUZZER          3
#define PIN_OLED_SDA        6
#define PIN_OLED_SCL        5

// ─── RELAY HELPERS ───────────────────────────────────────────────────────────
#define RELAY_ON    LOW
#define RELAY_OFF   HIGH

// ─── ENUMS ───────────────────────────────────────────────────────────────────
typedef enum {
    STATE_WARMUP,
    STATE_NORMAL,
    STATE_WARNING,
    STATE_EMERGENCY,
    STATE_FAULT
} SystemState;

typedef enum {
    CALL_IDLE,
    CALL_DIALING,
    CALL_RINGING,
    CALL_CONNECTED,
    CALL_ENDED,
    CALL_FAILED
} CallStatus;

// Network registration state (mirrors +CREG stat field)
typedef enum {
    NET_NOT_REGISTERED  = 0,  // not searching
    NET_REGISTERED_HOME = 1,  // registered, home network
    NET_SEARCHING       = 2,  // searching
    NET_REG_DENIED      = 3,  // registration denied
    NET_UNKNOWN         = 4,
    NET_REGISTERED_ROAM = 5   // registered, roaming
} NetRegState;

// ─── OBJECT INSTANCES ────────────────────────────────────────────────────────
OneWire             oneWire(PIN_DS18B20);
DallasTemperature   tempSensor(&oneWire);
HardwareSerial      gsmSerial(2);
Adafruit_SSD1306    oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);

// ─── STATE GLOBALS ───────────────────────────────────────────────────────────
SystemState   currentState          = STATE_WARMUP;
SystemState   prevState             = STATE_WARMUP;
CallStatus    callStatus            = CALL_IDLE;

bool          gsmReady              = false;   // SIM800L responded to AT
bool          gsmNetReg             = false;   // registered on 2G network
bool          emergencyActive       = false;
bool          callPlacedThisEvent   = false;
uint8_t       callRetryCount        = 0;

// ─── NETWORK INFO ─────────────────────────────────────────────────────────────
int           gsmSignal             = 0;    // CSQ raw (0–31, 99=unknown)
NetRegState   netRegState           = NET_NOT_REGISTERED;
char          operatorName[24]      = "---";  // populated by AT+COPS?
uint8_t       networkRegAttempts    = 0;

// ─── TIMING ──────────────────────────────────────────────────────────────────
unsigned long warmupStartMs         = 0;
unsigned long lastSmsMs             = 0;
unsigned long lastCallMs            = 0;
unsigned long callStartMs           = 0;
unsigned long lastReadMs            = 0;
unsigned long lastOledMs            = 0;
unsigned long lastBuzzToggleMs      = 0;
unsigned long uptimeSeconds         = 0;
unsigned long lastUptimeMs          = 0;
unsigned long lastNetRefreshMs      = 0;

bool          buzzState             = false;

// ─── SENSOR VALUES ───────────────────────────────────────────────────────────
int    mq6Raw       = 0;
float  mq6Voltage   = 0.0f;
float  temperature  = 0.0f;

// ─── FORWARD DECLARATIONS ────────────────────────────────────────────────────
void         initPins();
void         initOLED();
void         initGSM();
bool         sim800lWakeAndSync();
bool         sim800lSetBand2G();
bool         sim800lConnectNetwork();
bool         sim800lWaitRegistration(uint32_t timeoutMs);
void         sim800lScanOperators();
bool         sim800lAutoSelect();
void         sim800lRefreshNetwork();
void         sim800lReadOperatorName();
void         printNetworkInfo();

void         updateSystemState();
void         handleWarmup();
void         handleNormal();
void         handleWarning();
void         handleEmergency();
void         activateRelays();
void         deactivateRelays();
void         buzzerOn();
void         buzzerOff();
void         buzzerPattern(uint8_t beeps, uint16_t onMs, uint16_t offMs);
bool         sendSMS(const char* number, const char* message);
void         placeCall(const char* number);
void         hangUp();
void         pollCallStatus();
bool         sendATCommand(const char* cmd, const char* expected, uint32_t timeoutMs);
String       sendATCommandResponse(const char* cmd, uint32_t timeoutMs);
void         flushGSM();
int          readMQ6Averaged();
float        readTemperature();
void         updateOLED();
void         drawWarmupScreen();
void         drawNetworkScanScreen(const char* statusLine);
void         drawNormalScreen();
void         drawWarningScreen();
void         drawEmergencyScreen();
void         drawFaultScreen();
void         drawSignalBars(int x, int y, int csq);
void         printStatus();
String       stateToString(SystemState s);
String       callStatusToString(CallStatus cs);

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println(F("\n=============================================="));
    Serial.println(F("  Gas Leak Detection System v2.2"));
    Serial.println(F("  SIM800L 2G Network Edition"));
    Serial.println(F("=============================================="));

    initPins();
    Serial.println(F("[INIT] GPIO configured"));

    initOLED();
    Serial.println(F("[INIT] OLED ready"));

    // DS18B20
    tempSensor.begin();
    if (tempSensor.getDeviceCount() == 0) {
        Serial.println(F("[WARN] DS18B20 not found!"));
        currentState = STATE_FAULT;
    } else {
        tempSensor.setResolution(12);
        tempSensor.setWaitForConversion(false);
        tempSensor.requestTemperatures();
        Serial.println(F("[INIT] DS18B20 ready"));
    }

    // ADC
    analogSetAttenuation(ADC_11db);
    analogReadResolution(12);
    Serial.println(F("[INIT] ADC: 12-bit, 11dB"));

    // SIM800L UART
    gsmSerial.begin(GSM_BAUD, SERIAL_8N1, PIN_GSM_RX, PIN_GSM_TX);
    delay(2000);  // allow SIM800L to settle after power-on

    // Full GSM init including 2G network search
    initGSM();

    // MQ-6 warm-up
    warmupStartMs = millis();
    if (currentState != STATE_FAULT) currentState = STATE_WARMUP;
    Serial.print(F("[INIT] MQ-6 warm-up: "));
    Serial.print(MQ6_WARMUP_MS / 1000);
    Serial.println(F("s\n"));

    buzzerPattern(2, 50, 100);
    Serial.println(F("[INIT] System ready"));
}
//  MAIN LOOP
void loop() {
    unsigned long now = millis();

    // Uptime ticker
    if (now - lastUptimeMs >= 1000UL) {
        lastUptimeMs = now;
        uptimeSeconds++;
    }

    // Refresh GSM network status every NET_REFRESH_INTERVAL_MS
    if (now - lastNetRefreshMs >= NET_REFRESH_INTERVAL_MS) {
        lastNetRefreshMs = now;
        sim800lRefreshNetwork();
    }

    // Sensor reads every 500 ms
    if (now - lastReadMs >= 500UL) {
        lastReadMs  = now;
        mq6Raw      = readMQ6Averaged();
        mq6Voltage  = (mq6Raw / 4095.0f) * 3.3f;
        temperature = readTemperature();
        tempSensor.requestTemperatures();
        updateSystemState();
        printStatus();
    }

    // OLED every 300 ms
    if (now - lastOledMs >= 300UL) {
        lastOledMs = now;
        updateOLED();
    }

    // State machine
    switch (currentState) {
        case STATE_WARMUP:    handleWarmup();    break;
        case STATE_NORMAL:    handleNormal();    break;
        case STATE_WARNING:   handleWarning();   break;
        case STATE_EMERGENCY: handleEmergency(); break;
        case STATE_FAULT:
            if (now - lastBuzzToggleMs >= 1500UL) {
                lastBuzzToggleMs = now;
                buzzState = !buzzState;
                buzzState ? buzzerOn() : buzzerOff();
            }
            break;
    }

    // Poll call URC
    if (callStatus == CALL_DIALING ||
        callStatus == CALL_RINGING ||
        callStatus == CALL_CONNECTED) {
        pollCallStatus();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SIM800L — INIT  (top-level sequencer)
// ═══════════════════════════════════════════════════════════════════════════════

void initGSM() {
    Serial.println(F("[GSM] ── Init sequence start ──"));

    // 1. Wake module and sync baud
    if (!sim800lWakeAndSync()) {
        Serial.println(F("[GSM] Module unresponsive — GSM disabled."));
        gsmReady  = false;
        gsmNetReg = false;
        return;
    }
    gsmReady = true;

    // 2. Core AT configuration
    sendATCommand("ATE0",        "OK", 1000);   // echo off
    sendATCommand("AT+CMEE=2",   "OK", 1000);   // verbose errors
    sendATCommand("AT+CMGF=1",   "OK", 2000);   // SMS text mode
    sendATCommand("AT+CSCS=\"GSM\"", "OK", 1000); // GSM charset
    sendATCommand("AT+CLIP=1",   "OK", 1000);   // caller ID
    sendATCommand("ATS0=0",      "OK", 1000);   // no auto-answer
    sendATCommand("AT+CLVL=0",   "OK", 1000);   // speaker off
    sendATCommand("AT+CMUT=1",   "OK", 1000);   // mic mute
    // Delete all stored SMS to free SIM storage
    sendATCommand("AT+CMGDA=\"DEL ALL\"", "OK", 5000);

    // 3. Lock to 2G bands and connect to network
    sim800lSetBand2G();

    bool connected = sim800lConnectNetwork();
    if (!connected) {
        Serial.println(F("[GSM] WARNING: No 2G network found — SMS/call unavailable."));
        Serial.println(F("[GSM] System continues monitoring — GSM retried every 30 s."));
        gsmNetReg = false;
    }

    // 4. Print network info to serial for debugging
    printNetworkInfo();

    Serial.println(F("[GSM] ── Init sequence complete ──\n"));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SIM800L — WAKE & BAUD SYNC
// ═══════════════════════════════════════════════════════════════════════════════

// Sends "AT" up to 8 times. SIM800L autobauds on first few chars after
// power-on. Returns true once "OK" is received.
bool sim800lWakeAndSync() {
    Serial.print(F("[GSM] Waking SIM800L"));
    flushGSM();
    for (int i = 0; i < 8; i++) {
        gsmSerial.println(F("AT"));
        delay(400);
        String r = "";
        unsigned long t = millis();
        while (millis() - t < 400) {
            while (gsmSerial.available()) r += (char)gsmSerial.read();
        }
        if (r.indexOf("OK") >= 0) {
            Serial.println(F(" OK"));
            // Fix baud rate in NVM so it persists across resets
            String ipr = "AT+IPR=";
            ipr += GSM_BAUD;
            sendATCommand(ipr.c_str(), "OK", 1000);
            sendATCommand("AT&W", "OK", 1000);
            return true;
        }
        Serial.print(F("."));
    }
    Serial.println(F(" FAILED"));
    return false;
}
// ═════════════════════════════════════════════════════════════════════════════
//  SIM800L — LOCK TO 2G BANDS
// ═════════════════════════════════════════════════════════════════════════════
bool sim800lSetBand2G() {
    Serial.println(F("[GSM] Setting 2G band: GSM900 + DCS1800"));

    // AT+CBAND="GSM850_900_1800_1900" — all 2G, let module pick best
    if (sendATCommand("AT+CBAND=\"GSM850_900_1800_1900\"", "OK", 3000)) {
        Serial.println(F("[GSM] Band locked: all 2G (850/900/1800/1900 MHz)"));
        return true;
    }

    // Some SIM800L firmware versions use AT+CBANDCFG
    if (sendATCommand("AT+CBANDCFG=\"GSM\",0,1,2,3", "OK", 3000)) {
        Serial.println(F("[GSM] Band locked via CBANDCFG"));
        return true;
    }

    // Fallback: no explicit band lock — module uses factory default (all bands)
    Serial.println(F("[GSM] Band command not supported — using factory defaults"));
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SIM800L — 2G NETWORK SEARCH & CONNECT  (main new function)
// ═══════════════════════════════════════════════════════════════════════════════
bool sim800lConnectNetwork() {
    Serial.println(F("[GSM] ── 2G network search ──"));
    drawNetworkScanScreen("Searching 2G...");

    for (uint8_t attempt = 1; attempt <= NET_REG_MAX_ATTEMPTS; attempt++) {
        Serial.print(F("[GSM] Attempt ")); Serial.print(attempt);
        Serial.print(F("/")); Serial.println(NET_REG_MAX_ATTEMPTS);

        // ── Step A: try automatic registration ────────────────────────────────
        Serial.println(F("[GSM]  A) Auto-select: AT+COPS=0"));
        drawNetworkScanScreen("Auto select...");
        sendATCommand("AT+COPS=0", "OK", 5000);  // mode=0: automatic

        if (sim800lWaitRegistration(NET_REG_TIMEOUT_MS)) {
            sim800lReadOperatorName();
            Serial.print(F("[GSM] Registered (auto) on: "));
            Serial.println(operatorName);
            drawNetworkScanScreen("Connected!");
            delay(600);
            gsmNetReg = true;
            return true;
        }

        Serial.println(F("[GSM]  Auto-select failed — scanning visible operators."));

        // ── Step B: scan all visible 2G operators in vicinity ─────────────────
        Serial.println(F("[GSM]  B) Scanning operators (AT+COPS=?) — may take 60-90 s"));
        drawNetworkScanScreen("Scanning ops...");

        String scanResult = sendATCommandResponse("AT+COPS=?", COPS_SCAN_TIMEOUT_MS);

        if (scanResult.indexOf("+COPS:") < 0) {
            Serial.println(F("[GSM]  Scan returned no results — SIM issue or no signal"));
            drawNetworkScanScreen("No operators");
            delay(1000);
            continue;  // retry outer loop
        }

        Serial.println(F("[GSM]  Scan result:"));
        Serial.println(scanResult);

        // ── Step C: parse numeric codes of GSM (2G) operators from scan ───────
        String codes[6];      // up to 6 operator codes to try
        uint8_t codeCount = 0;
        int searchPos = 0;

        while (codeCount < 6) {
            int entryStart = scanResult.indexOf('(', searchPos);
            if (entryStart < 0) break;
            int entryEnd   = scanResult.indexOf(')', entryStart);
            if (entryEnd < 0) break;

            String entry = scanResult.substring(entryStart + 1, entryEnd);
            searchPos = entryEnd + 1;

            // Check stat (first char): 1 = available, 2 = current
            char statChar = entry.charAt(0);
            if (statChar != '1' && statChar != '2') continue;
            int commaCount = 0;
            int numStart   = -1;
            for (int i = 0; i < (int)entry.length(); i++) {
                if (entry.charAt(i) == ',') {
                    commaCount++;
                    if (commaCount == 3) { numStart = i + 1; break; }
                }
            }
            if (numStart < 0) continue;

            // Extract the numeric code (quoted string)
            int qStart = entry.indexOf('"', numStart);
            int qEnd   = entry.indexOf('"', qStart + 1);
            if (qStart < 0 || qEnd < 0) continue;
            String numCode = entry.substring(qStart + 1, qEnd);

            // Check act — next field after the numeric code
            int actComma = entry.indexOf(',', qEnd + 1);
            if (actComma >= 0) {
                String actStr = entry.substring(actComma + 1);
                actStr.trim();
                int act = actStr.toInt();
                // act 0 = GSM (2G). Skip 3G (2) and LTE (7) entries.
                if (act != 0) {
                    Serial.print(F("[GSM]   Skipping non-2G operator (act="));
                    Serial.print(act); Serial.println(F(")"));
                    continue;
                }
            }

            if (numCode.length() >= 5) {
                codes[codeCount++] = numCode;
                Serial.print(F("[GSM]   Found 2G operator code: ")); Serial.println(numCode);
            }
        }

        if (codeCount == 0) {
            Serial.println(F("[GSM]  No 2G operators found in scan."));
            drawNetworkScanScreen("No 2G found");
            delay(1000);
            continue;
        }

        // ── Step D: try manual registration on each found 2G operator ─────────
        for (uint8_t ci = 0; ci < codeCount; ci++) {
            String opsMsg = "Try: " + codes[ci];
            drawNetworkScanScreen(opsMsg.c_str());

            Serial.print(F("[GSM]  Trying manual: AT+COPS=1,2,\""));
            Serial.print(codes[ci]); Serial.println(F("\""));

            // AT+COPS=1,2,"<numericCode>" — manual select, numeric format
            String copsCmd = "AT+COPS=1,2,\"" + codes[ci] + "\"";
            // This command itself may block for up to 30 s during registration
            bool cmdOk = sendATCommand(copsCmd.c_str(), "OK", 30000UL);

            if (cmdOk && sim800lWaitRegistration(15000UL)) {
                sim800lReadOperatorName();
                Serial.print(F("[GSM]  Registered (manual) on: "));
                Serial.println(operatorName);
                drawNetworkScanScreen("Connected!");
                delay(600);
                gsmNetReg = true;
                return true;
            }

            Serial.println(F("[GSM]   Manual attempt failed — trying next operator."));
            // De-register before trying next
            sendATCommand("AT+COPS=2", "OK", 5000);  // mode=2: deregister
            delay(500);
        }

        // ── Step E: all manual attempts failed — fall back to auto for retry ──
        Serial.println(F("[GSM]  All manual attempts failed — back to auto mode."));
        sendATCommand("AT+COPS=0", "OK", 3000);
        drawNetworkScanScreen("Retrying...");
        delay(2000);
    }

    // All attempts exhausted
    Serial.println(F("[GSM] Could not register on any 2G network."));
    drawNetworkScanScreen("No network!");
    delay(1000);
    gsmNetReg = false;
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SIM800L — WAIT FOR REGISTRATION
// ═══════════════════════════════════════════════════════════════════════════════
bool sim800lWaitRegistration(uint32_t timeoutMs) {
    unsigned long start = millis();
    Serial.print(F("[GSM]  Waiting for registration"));

    while (millis() - start < timeoutMs) {
        String resp = sendATCommandResponse("AT+CREG?", 2000);

        // +CREG: <n>,<stat>  or  +CREG: <stat>
        int cregIdx = resp.indexOf("+CREG:");
        if (cregIdx >= 0) {
            // Find the stat value — it's the last integer before OK
            // Could be "+CREG: 0,1" or "+CREG: 1"
            String cregLine = resp.substring(cregIdx + 7);
            cregLine.trim();

            // If there's a comma, stat is after it; otherwise it's the first token
            int commaPos = cregLine.indexOf(',');
            int stat = 0;
            if (commaPos >= 0) {
                stat = cregLine.substring(commaPos + 1).toInt();
            } else {
                stat = cregLine.toInt();
            }

            netRegState = (NetRegState)stat;

            if (stat == NET_REGISTERED_HOME || stat == NET_REGISTERED_ROAM) {
                Serial.println(stat == 5 ? F(" ROAMING") : F(" HOME"));
                return true;
            }
            if (stat == NET_REG_DENIED) {
                Serial.println(F(" DENIED"));
                return false;
            }
        }

        Serial.print(F("."));
        delay(2000);
    }

    Serial.println(F(" TIMEOUT"));
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SIM800L — SCAN OPERATORS (serial log only, no return value)
// ═══════════════════════════════════════════════════════════════════════════════

// Called optionally for diagnostics. Logs all visible operators to Serial.
void sim800lScanOperators() {
    Serial.println(F("[GSM] Full operator scan (diagnostic):"));
    String result = sendATCommandResponse("AT+COPS=?", COPS_SCAN_TIMEOUT_MS);
    if (result.length() > 0) {
        Serial.println(result);
    } else {
        Serial.println(F("[GSM] Scan returned no data."));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SIM800L — READ CURRENT OPERATOR NAME
// ═══════════════════════════════════════════════════════════════════════════════
void sim800lReadOperatorName() {
    String resp = sendATCommandResponse("AT+COPS?", 2000);
    int copsIdx = resp.indexOf("+COPS:");
    if (copsIdx < 0) {
        strncpy(operatorName, "Unknown", sizeof(operatorName));
        return;
    }
    int q1 = resp.indexOf('"', copsIdx);
    int q2 = resp.indexOf('"', q1 + 1);
    if (q1 >= 0 && q2 > q1) {
        String name = resp.substring(q1 + 1, q2);
        name.toCharArray(operatorName, sizeof(operatorName));
    } else {
        strncpy(operatorName, "Unknown", sizeof(operatorName));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SIM800L — PERIODIC REFRESH (called from loop every 30 s)
// ═══════════════════════════════════════════════════════════════════════════════

void sim800lRefreshNetwork() {
    if (!gsmReady) return;

    // Check registration
    String creg = sendATCommandResponse("AT+CREG?", 2000);
    bool wasReg = gsmNetReg;

    if (creg.indexOf("+CREG: 0,1") >= 0 || creg.indexOf("+CREG: 0,5") >= 0 ||
        creg.indexOf("+CREG: 1")   >= 0 || creg.indexOf("+CREG: 5")   >= 0) {
        gsmNetReg = true;
        // Update stat field
        netRegState = (creg.indexOf(",5") >= 0 || creg.indexOf(": 5") >= 0)
                      ? NET_REGISTERED_ROAM : NET_REGISTERED_HOME;
    } else {
        gsmNetReg = false;
        netRegState = NET_NOT_REGISTERED;
    }

    // Log if status changed
    if (gsmNetReg != wasReg) {
        if (gsmNetReg) {
            sim800lReadOperatorName();
            Serial.print(F("[GSM] Network RESTORED — "));
            Serial.println(operatorName);
        } else {
            Serial.println(F("[GSM] Network LOST — attempting re-registration."));
            // Try to re-connect in background (non-blocking auto mode)
            sendATCommand("AT+COPS=0", "OK", 5000);
        }
    }

    // Refresh signal quality
    String csqResp = sendATCommandResponse("AT+CSQ", 2000);
    int csqIdx = csqResp.indexOf("+CSQ:");
    if (csqIdx >= 0) {
        // "+CSQ: XX,YY" — extract XX
        String csqPart = csqResp.substring(csqIdx + 5);
        csqPart.trim();
        if (csqPart.startsWith(" ")) csqPart = csqPart.substring(1);
        gsmSignal = csqPart.toInt();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SIM800L — PRINT NETWORK INFO TO SERIAL
// ═══════════════════════════════════════════════════════════════════════════════

void printNetworkInfo() {
    Serial.println(F("[GSM] ── Network Info ──"));

    // Operator
    sim800lReadOperatorName();
    Serial.print(F("[GSM]  Operator  : ")); Serial.println(operatorName);

    // Registration
    Serial.print(F("[GSM]  Reg state : "));
    switch (netRegState) {
        case NET_REGISTERED_HOME: Serial.println(F("Registered (Home)")); break;
        case NET_REGISTERED_ROAM: Serial.println(F("Registered (Roaming)")); break;
        case NET_SEARCHING:       Serial.println(F("Searching")); break;
        case NET_REG_DENIED:      Serial.println(F("Denied")); break;
        case NET_NOT_REGISTERED:  Serial.println(F("Not registered")); break;
        default:                  Serial.println(F("Unknown")); break;
    }

    // Signal quality
    String csqR = sendATCommandResponse("AT+CSQ", 2000);
    int ci = csqR.indexOf("+CSQ:");
    if (ci >= 0) {
        gsmSignal = csqR.substring(ci + 5).toInt();
    }
    Serial.print(F("[GSM]  CSQ       : ")); Serial.print(gsmSignal);
    if (gsmSignal == 99) {
        Serial.println(F(" (unknown/no signal)"));
    } else {
        int dBm = -113 + (gsmSignal * 2);
        Serial.print(F(" → ")); Serial.print(dBm); Serial.println(F(" dBm"));
    }

    // Current band (if firmware supports AT+CBAND?)
    String bandR = sendATCommandResponse("AT+CBAND?", 2000);
    if (bandR.indexOf("+CBAND:") >= 0) {
        int bi = bandR.indexOf("+CBAND:");
        String bandLine = bandR.substring(bi + 7);
        bandLine.trim();
        Serial.print(F("[GSM]  Band      : ")); Serial.println(bandLine);
    }

    Serial.println(F("[GSM] ─────────────────"));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════════

void updateSystemState() {
    if (currentState == STATE_WARMUP) {
        if (millis() - warmupStartMs >= MQ6_WARMUP_MS) {
            currentState = STATE_NORMAL;
            Serial.println(F("[STATE] WARMUP → NORMAL"));
            buzzerPattern(3, 80, 80);
        }
        return;
    }
    if (currentState == STATE_FAULT) return;

    prevState = currentState;

    if (temperature > TEMP_DANGER_THRESHOLD && temperature < 125.0f) {
        currentState = STATE_EMERGENCY;
    } else if (mq6Raw >= MQ6_DANGER_THRESHOLD) {
        currentState = STATE_EMERGENCY;
    } else if (mq6Raw >= MQ6_WARNING_THRESHOLD) {
        currentState = STATE_WARNING;
    } else {
        currentState = STATE_NORMAL;
    }

    if (currentState != prevState) {
        Serial.print(F("[STATE] "));
        Serial.print(stateToString(prevState));
        Serial.print(F(" → "));
        Serial.println(stateToString(currentState));

        if (prevState == STATE_EMERGENCY && currentState != STATE_EMERGENCY) {
            callPlacedThisEvent = false;
            callRetryCount      = 0;
            if (callStatus != CALL_IDLE &&
                callStatus != CALL_ENDED &&
                callStatus != CALL_FAILED) {
                hangUp();
            }
            callStatus = CALL_IDLE;
        }
    }
}

// ─── WARMUP ──────────────────────────────────────────────────────────────────
void handleWarmup() {
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat >= 5000UL) {
        lastHeartbeat = millis();
        buzzerPattern(1, 40, 0);
    }
}

// ─── NORMAL ──────────────────────────────────────────────────────────────────
void handleNormal() {
    if (emergencyActive) {
        emergencyActive = false;
        deactivateRelays();
        buzzerOff();
        Serial.println(F("[NORMAL] All actuators off. System clear."));
        buzzerPattern(2, 100, 150);
    }
}

// ─── WARNING ─────────────────────────────────────────────────────────────────
void handleWarning() {
    unsigned long now = millis();

    if (now - lastBuzzToggleMs >= (buzzState ? 200UL : 800UL)) {
        lastBuzzToggleMs = now;
        buzzState = !buzzState;
        buzzState ? buzzerOn() : buzzerOff();
    }

    if (gsmNetReg && (now - lastSmsMs >= SMS_COOLDOWN_MS)) {
        char msg[160];
        snprintf(msg, sizeof(msg),
            "GAS ALERT [WARNING]\nMQ-6: %d (%.2fV)\nTemp: %.1fC\nElevated gas level.",
            mq6Raw, mq6Voltage, temperature);
        if (sendSMS(ALERT_PHONE_NUMBER, msg)) {
            lastSmsMs = millis();
            Serial.println(F("[SMS] Warning SMS sent."));
        }
    }
}

// ─── EMERGENCY ───────────────────────────────────────────────────────────────
void handleEmergency() {
    unsigned long now = millis();

    if (!emergencyActive) {
        emergencyActive = true;
        activateRelays();
        Serial.println(F("[EMERGENCY] Valve CLOSED | Fan ON | Buzzer ON"));

        if (gsmNetReg) {
            char msg[160];
            snprintf(msg, sizeof(msg),
                "!! GAS EMERGENCY !!\nMQ-6: %d (%.2fV)\nTemp: %.1fC\nGas CUT. Fan ON. EVACUATE!",
                mq6Raw, mq6Voltage, temperature);
            if (sendSMS(ALERT_PHONE_NUMBER, msg)) {
                lastSmsMs = now;
                Serial.println(F("[SMS] Emergency SMS sent."));
            }
        } else {
            Serial.println(F("[EMRG] GSM not registered — SMS skipped."));
        }
    }

    if (now - lastBuzzToggleMs >= 100UL) {
        lastBuzzToggleMs = now;
        buzzState = !buzzState;
        buzzState ? buzzerOn() : buzzerOff();
    }

    // Voice call — only if registered
    bool callIdle = (callStatus == CALL_IDLE   ||
                     callStatus == CALL_ENDED  ||
                     callStatus == CALL_FAILED);

    if (gsmNetReg && callIdle && !callPlacedThisEvent &&
        callRetryCount < CALL_MAX_RETRIES) {
        if (now - lastCallMs >= CALL_COOLDOWN_MS) {
            Serial.print(F("[CALL] Emergency call attempt "));
            Serial.print(callRetryCount + 1); Serial.print(F("/")); Serial.println(CALL_MAX_RETRIES);
            placeCall(ALERT_PHONE_NUMBER);
            callRetryCount++;
            if (callRetryCount >= CALL_MAX_RETRIES) {
                callPlacedThisEvent = true;
            }
        }
    }

    if ((callStatus == CALL_DIALING || callStatus == CALL_RINGING) &&
        (now - callStartMs >= CALL_RING_DURATION_MS)) {
        Serial.println(F("[CALL] Ring timeout — hanging up."));
        hangUp();
        callStatus  = CALL_FAILED;
        lastCallMs  = now;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ACTUATORS
// ═══════════════════════════════════════════════════════════════════════════════

void activateRelays() {
    digitalWrite(PIN_RELAY_VALVE, RELAY_ON);
    digitalWrite(PIN_RELAY_FAN,   RELAY_ON);
    buzzerOn();
}

void deactivateRelays() {
    digitalWrite(PIN_RELAY_VALVE, RELAY_OFF);
    digitalWrite(PIN_RELAY_FAN,   RELAY_OFF);
    buzzerOff();
}

void buzzerOn()  { digitalWrite(PIN_BUZZER, HIGH); }
void buzzerOff() { digitalWrite(PIN_BUZZER, LOW);  }

void buzzerPattern(uint8_t beeps, uint16_t onMs, uint16_t offMs) {
    for (uint8_t i = 0; i < beeps; i++) {
        buzzerOn();  delay(onMs);
        buzzerOff(); if (i < beeps - 1) delay(offMs);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GSM — SMS
// ═══════════════════════════════════════════════════════════════════════════════

bool sendSMS(const char* number, const char* message) {
    if (!gsmReady || !gsmNetReg) {
        Serial.println(F("[SMS] Not ready/registered — skipped."));
        return false;
    }

    gsmSerial.print(F("AT+CMGS=\"")); gsmSerial.print(number); gsmSerial.println(F("\""));
    delay(1000);

    unsigned long t = millis();
    bool gotPrompt = false;
    while (millis() - t < 5000UL) {
        if (gsmSerial.available() && gsmSerial.read() == '>') { gotPrompt = true; break; }
    }
    if (!gotPrompt) { Serial.println(F("[SMS] No '>' prompt.")); return false; }

    gsmSerial.print(message);
    delay(100);
    gsmSerial.write(26);  // Ctrl+Z

    unsigned long start = millis();
    String resp = "";
    while (millis() - start < 15000UL) {
        while (gsmSerial.available()) resp += (char)gsmSerial.read();
        if (resp.indexOf("+CMGS:") >= 0) { Serial.println(F("[SMS] Sent OK.")); return true; }
        if (resp.indexOf("ERROR")  >= 0) { Serial.println(F("[SMS] ERROR."));   return false; }
        delay(10);
    }
    Serial.println(F("[SMS] Timeout."));
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GSM — VOICE CALL
// ═══════════════════════════════════════════════════════════════════════════════

void placeCall(const char* number) {
    if (!gsmReady || !gsmNetReg) {
        Serial.println(F("[CALL] Not ready/registered."));
        callStatus = CALL_FAILED;
        return;
    }
    Serial.print(F("[CALL] Dialing: ")); Serial.println(number);
    gsmSerial.print(F("ATD")); gsmSerial.print(number); gsmSerial.println(F(";"));
    callStartMs = millis();
    lastCallMs  = millis();
    callStatus  = CALL_DIALING;
}

void hangUp() {
    Serial.println(F("[CALL] ATH — hanging up."));
    gsmSerial.println(F("ATH"));
    delay(500);
    callStatus = CALL_ENDED;
}

void pollCallStatus() {
    static String callBuf = "";
    while (gsmSerial.available()) {
        char c = gsmSerial.read();
        callBuf += c;
        if (c == '\n') {
            callBuf.trim();
            if (callBuf.length() > 0) {
                Serial.print(F("[GSM] ")); Serial.println(callBuf);
                if (callBuf.indexOf("NO CARRIER") >= 0 ||
                    callBuf.indexOf("NO ANSWER")  >= 0 ||
                    callBuf.indexOf("BUSY")        >= 0) {
                    callStatus = CALL_FAILED;
                    lastCallMs = millis();
                }
                if (callBuf.indexOf("OK") >= 0 && callStatus == CALL_DIALING) {
                    callStatus = CALL_RINGING;
                    Serial.println(F("[CALL] Ringing..."));
                }
                if (callBuf.indexOf("+CLCC:") >= 0) {
                    callStatus = CALL_CONNECTED;
                    Serial.println(F("[CALL] Connected."));
                }
            }
            callBuf = "";
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  AT COMMAND HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

bool sendATCommand(const char* cmd, const char* expected, uint32_t timeoutMs) {
    flushGSM();
    gsmSerial.println(cmd);
    unsigned long start = millis();
    String resp = "";
    while (millis() - start < timeoutMs) {
        while (gsmSerial.available()) resp += (char)gsmSerial.read();
        if (resp.indexOf(expected) >= 0) return true;
        if (resp.indexOf("ERROR")  >= 0) return false;
        delay(10);
    }
    return false;
}

String sendATCommandResponse(const char* cmd, uint32_t timeoutMs) {
    flushGSM();
    gsmSerial.println(cmd);
    unsigned long start = millis();
    String resp = "";
    while (millis() - start < timeoutMs) {
        while (gsmSerial.available()) resp += (char)gsmSerial.read();
        if (resp.indexOf("OK")    >= 0) break;
        if (resp.indexOf("ERROR") >= 0) break;
        delay(10);
    }
    return resp;
}

void flushGSM() {
    while (gsmSerial.available()) gsmSerial.read();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SENSORS
// ═══════════════════════════════════════════════════════════════════════════════

int readMQ6Averaged() {
    long sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) { sum += analogRead(PIN_MQ6_AOUT); delay(2); }
    return (int)(sum / ADC_SAMPLES);
}

float readTemperature() {
    float t = tempSensor.getTempCByIndex(0);
    if (t == DEVICE_DISCONNECTED_C || t < -55.0f || t > 125.0f) return temperature;
    return t;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  OLED
// ═══════════════════════════════════════════════════════════════════════════════

void initOLED() {
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        Serial.println(F("[WARN] OLED init failed!"));
        return;
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(8, 16);  oled.println(F("Gas Leak Detector"));
    oled.setCursor(8, 28);  oled.println(F("SIM800L 2G v2.2"));
    oled.setCursor(8, 44);  oled.println(F("Initialising..."));
    oled.display();
}

void updateOLED() {
    switch (currentState) {
        case STATE_WARMUP:    drawWarmupScreen();    break;
        case STATE_NORMAL:    drawNormalScreen();    break;
        case STATE_WARNING:   drawWarningScreen();   break;
        case STATE_EMERGENCY: drawEmergencyScreen(); break;
        case STATE_FAULT:     drawFaultScreen();     break;
    }
}
void drawNetworkScanScreen(const char* statusLine) {
    oled.clearDisplay();
    oled.setTextSize(1);

    // Header
    oled.fillRect(0, 0, 128, 11, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(14, 2);
    oled.println(F("SIM800L 2G SETUP"));
    oled.setTextColor(SSD1306_WHITE);

    // Animated dots — cycle every 500 ms
    static uint8_t dotCount = 0;
    static unsigned long lastDot = 0;
    if (millis() - lastDot >= 500) { lastDot = millis(); dotCount = (dotCount + 1) % 4; }
    char dots[5] = "    ";
    for (uint8_t i = 0; i < dotCount; i++) dots[i] = '.';

    oled.setCursor(0, 14);  oled.println(F("Searching 2G network"));
    oled.setCursor(0, 26);  oled.println(statusLine);
    oled.setCursor(110, 26); oled.println(dots);

    // Operator (may be "---" during scan)
    oled.setCursor(0, 38);
    oled.print(F("Op: ")); oled.println(operatorName);

    // Signal bars (may be 0 during scan)
    oled.setCursor(0, 50);
    oled.print(F("Signal: "));
    drawSignalBars(56, 50, gsmSignal);

    // Attempt counter
    oled.setCursor(80, 50);
    oled.print(F("Att:"));
    oled.print(networkRegAttempts);
    oled.print(F("/"));
    oled.print(NET_REG_MAX_ATTEMPTS);

    oled.display();
}

// ── Warmup screen ─────────────────────────────────────────────────────────────
void drawWarmupScreen() {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(20, 0);  oled.println(F("GAS LEAK DETECTOR"));
    oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);
    oled.setCursor(0, 14);  oled.println(F("Warming up sensor..."));

    unsigned long elapsed = millis() - warmupStartMs;
    if (elapsed > MQ6_WARMUP_MS) elapsed = MQ6_WARMUP_MS;
    int barW = (int)((elapsed * 118UL) / MQ6_WARMUP_MS);
    oled.drawRect(4, 28, 120, 10, SSD1306_WHITE);
    oled.fillRect(5, 29, barW, 8, SSD1306_WHITE);

    oled.setCursor(0, 42);
    oled.print(F("Ready in: "));
    oled.print((MQ6_WARMUP_MS - elapsed) / 1000);
    oled.println(F("s"));

    oled.setCursor(0, 54);
    oled.print(F("MQ6: ")); oled.print(mq6Raw);
    oled.display();
}

// ── Normal screen ─────────────────────────────────────────────────────────────
void drawNormalScreen() {
    oled.clearDisplay();
    oled.setTextSize(1);

    oled.fillRect(0, 0, 128, 11, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(30, 2);  oled.println(F("* SYSTEM NORMAL *"));
    oled.setTextColor(SSD1306_WHITE);

    oled.setCursor(0, 14);
    oled.print(F("Gas:"));
    int gasBarW = map(mq6Raw, 0, 4095, 0, 88);
    oled.fillRect(30, 14, gasBarW, 7, SSD1306_WHITE);
    oled.drawRect(30, 14, 88, 7, SSD1306_WHITE);
    if (mq6Raw >= MQ6_WARNING_THRESHOLD) { oled.setCursor(120, 14); oled.print(F("!")); }

    oled.setCursor(0, 24);
    oled.print(F("ADC:")); oled.print(mq6Raw);
    oled.print(F(" V:"));  oled.print(mq6Voltage, 1);

    oled.setCursor(0, 34);
    oled.print(F("Temp: ")); oled.print(temperature, 1); oled.println(F(" C"));

    oled.setCursor(0, 44);
    oled.print(F("Valve:"));
    oled.print(digitalRead(PIN_RELAY_VALVE) == RELAY_ON ? F("CLSD") : F("OPEN"));
    oled.print(F(" Fan:"));
    oled.println(digitalRead(PIN_RELAY_FAN) == RELAY_ON ? F("ON") : F("OFF"));

    // Bottom row: uptime + operator (truncated) + signal bars
    oled.setCursor(0, 54);
    oled.print(uptimeSeconds / 60); oled.print(F("m"));
    oled.print(uptimeSeconds % 60); oled.print(F("s "));
    // Show operator name, max 8 chars
    char opShort[9];
    strncpy(opShort, operatorName, 8);
    opShort[8] = '\0';
    oled.print(gsmNetReg ? opShort : "NO NET");
    drawSignalBars(104, 54, gsmNetReg ? gsmSignal : 0);
    oled.display();
}

// ── Warning screen ────────────────────────────────────────────────────────────
void drawWarningScreen() {
    oled.clearDisplay();
    oled.setTextSize(1);

    static bool blink = false;
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink >= 400) { lastBlink = millis(); blink = !blink; }

    if (blink) {
        oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
        oled.setTextColor(SSD1306_BLACK);
    }
    oled.setCursor(18, 2);   oled.println(F("!! GAS WARNING !!"));
    oled.setTextColor(SSD1306_WHITE);

    oled.setCursor(0, 14);  oled.print(F("MQ-6 ADC : ")); oled.println(mq6Raw);
    oled.setCursor(0, 24);  oled.print(F("Voltage  : ")); oled.print(mq6Voltage, 2); oled.println(F("V"));
    oled.setCursor(0, 34);  oled.print(F("Temp     : ")); oled.print(temperature, 1); oled.println(F(" C"));
    oled.setCursor(0, 44);
    oled.println(gsmNetReg ? F("SMS alert active") : F("NO NET: SMS skip"));
    oled.setCursor(0, 54);  oled.print(F("Threshold: ")); oled.println(MQ6_WARNING_THRESHOLD);
    oled.display();
}

// ── Emergency screen ──────────────────────────────────────────────────────────
void drawEmergencyScreen() {
    oled.clearDisplay();
    oled.setTextSize(1);

    static bool inv = false;
    static unsigned long lastInv = 0;
    if (millis() - lastInv >= 250) { lastInv = millis(); inv = !inv; }

    if (inv) {
        oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
        oled.setTextColor(SSD1306_BLACK);
    }
    oled.setCursor(10, 2);   oled.println(F("!!! EMERGENCY !!!"));
    oled.setTextColor(SSD1306_WHITE);

    oled.setCursor(0, 14);
    oled.print(F("MQ6:")); oled.print(mq6Raw);
    oled.print(F("  T:")); oled.print(temperature, 0); oled.println(F("C"));

    oled.setCursor(0, 24);  oled.println(F("Valve:CLOSED Fan:ON"));
    oled.setCursor(0, 34);  oled.print(F("Call: ")); oled.println(callStatusToString(callStatus));
    oled.setCursor(0, 44);
    oled.print(F("Att:")); oled.print(callRetryCount);
    oled.print(F("/"));    oled.print(CALL_MAX_RETRIES);
    oled.print(F(" Net:")); oled.println(gsmNetReg ? F("OK") : F("NO"));
    oled.setCursor(0, 54);  oled.println(F("EVACUATE AREA NOW"));
    oled.display();
}

// ── Fault screen ──────────────────────────────────────────────────────────────
void drawFaultScreen() {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(22, 2);  oled.println(F("SENSOR FAULT"));
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 16);  oled.println(F("DS18B20 not found."));
    oled.setCursor(0, 28);  oled.println(F("Check 4.7k pullup"));
    oled.setCursor(0, 40);  oled.println(F("and wiring (GPIO4)"));
    oled.setCursor(0, 54);  oled.print(F("MQ6: ")); oled.print(mq6Raw);
    oled.display();
}

// ── Signal bars (4 bars) ──────────────────────────────────────────────────────
void drawSignalBars(int x, int y, int csq) {
    int bars = 0;
    if (csq >= 2)  bars = 1;
    if (csq >= 8)  bars = 2;
    if (csq >= 16) bars = 3;
    if (csq >= 24) bars = 4;
    for (int i = 0; i < 4; i++) {
        int bh = 2 + i * 2;
        int bx = x + i * 5;
        int by = y + (8 - bh);
        if (i < bars) oled.fillRect(bx, by, 4, bh, SSD1306_WHITE);
        else          oled.drawRect(bx, by, 4, bh, SSD1306_WHITE);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PIN INIT
// ═══════════════════════════════════════════════════════════════════════════════

void initPins() {
    pinMode(PIN_RELAY_VALVE, OUTPUT);
    pinMode(PIN_RELAY_FAN,   OUTPUT);
    pinMode(PIN_BUZZER,      OUTPUT);
    pinMode(PIN_MQ6_AOUT,    INPUT);

    digitalWrite(PIN_RELAY_VALVE, RELAY_OFF);
    digitalWrite(PIN_RELAY_FAN,   RELAY_OFF);
    digitalWrite(PIN_BUZZER,      LOW);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DEBUG
// ═══════════════════════════════════════════════════════════════════════════════

void printStatus() {
    Serial.print(F("[DATA] "));
    Serial.print(stateToString(currentState));
    Serial.print(F(" | ADC=")); Serial.print(mq6Raw);
    Serial.print(F(" V="));     Serial.print(mq6Voltage, 2);
    Serial.print(F(" T="));     Serial.print(temperature, 1);
    Serial.print(F("C | Valve="));
    Serial.print(digitalRead(PIN_RELAY_VALVE) == RELAY_ON ? F("CLOSED") : F("OPEN"));
    Serial.print(F(" Fan="));
    Serial.print(digitalRead(PIN_RELAY_FAN) == RELAY_ON ? F("ON") : F("OFF"));
    Serial.print(F(" | Call="));
    Serial.print(callStatusToString(callStatus));
    Serial.print(F(" | Net="));
    Serial.print(gsmNetReg ? operatorName : "UNREG");
    Serial.print(F(" CSQ="));
    Serial.println(gsmSignal);
}

String stateToString(SystemState s) {
    switch (s) {
        case STATE_WARMUP:    return "WARMUP";
        case STATE_NORMAL:    return "NORMAL";
        case STATE_WARNING:   return "WARNING";
        case STATE_EMERGENCY: return "EMERGENCY";
        case STATE_FAULT:     return "FAULT";
        default:              return "UNKNOWN";
    }
}

String callStatusToString(CallStatus cs) {
    switch (cs) {
        case CALL_IDLE:      return "Idle";
        case CALL_DIALING:   return "Dialing";
        case CALL_RINGING:   return "Ringing";
        case CALL_CONNECTED: return "Connected";
        case CALL_ENDED:     return "Ended";
        case CALL_FAILED:    return "Failed";
        default:             return "Unknown";
    }
}
