// KITT scanner bar - 8 zone Larson scanner, multi-mode
// ESP32-S3 + TIP122 low-side switches + Chanzon 10V COB LEDs (12V rail + ballast resistor)
//
// Each zone: GPIO -> 1k base resistor -> TIP122 base
//            TIP122 collector -> COB cathode
//            COB anode -> ballast resistor -> +12V
//            TIP122 emitter -> GND
//            10k pulldown from base to GND (prevents flash on boot)
//
// Buttons (all momentary pushbuttons, one leg to GPIO, other leg to GND,
// using internal pullups - no external resistor needed):
//   BTN_FASTER_PIN -> speeds the sweep up, one step per press, up to MAX_SPEED_LEVEL
//   BTN_SLOWER_PIN -> slows it back down, one step per press, down to the base speed
//   BTN_POWER_PIN  -> toggles the whole bar on/off. Turning it back ON replays the
//                     2-second all-on flash, same as a fresh power-up.
//   BTN_MODE_PIN   -> cycles: original 8-zone sweep -> 6-zone sweep -> center-out
//                     mirrored -> back to original. Speed applies to all three modes.
//   BTN_BRIGHT_UP_PIN / BTN_BRIGHT_DOWN_PIN -> step overall brightness up/down in
//                     10% increments, 0-100%. Also settable from the web page slider.
//
// Web control: connects to your WiFi and serves a small control page at its IP address
// with buttons for power, mode, speed, and a brightness slider - fill in
// WIFI_SSID / WIFI_PASSWORD below. The IP address is printed to Serial (115200 baud)
// a few seconds after boot.

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

Preferences prefs;
WebServer server(80);

const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// If WIFI_SSID isn't found/connectable within a few seconds (e.g. no network at a car
// show), the ESP32-S3 creates its own hotspot instead using these credentials - connect
// your phone's WiFi directly to this network, then browse to http://192.168.4.1
const char* AP_SSID     = "KITT-Scanner";
const char* AP_PASSWORD = "scanner123"; // WPA2 requires 8+ characters

// ---- Configuration ----
const uint8_t NUM_ZONES = 8;
const uint8_t ZONE_PINS[NUM_ZONES] = {4, 5, 6, 7, 15, 16, 17, 18};

const uint8_t BTN_FASTER_PIN = 8;
const uint8_t BTN_SLOWER_PIN = 9;
const uint8_t BTN_POWER_PIN  = 10;
const uint8_t BTN_MODE_PIN   = 11;
const uint8_t BTN_BRIGHT_UP_PIN   = 12;
const uint8_t BTN_BRIGHT_DOWN_PIN = 13;
const uint8_t BRIGHTNESS_STEP_PERCENT = 10; // each button press moves brightness by this much
const unsigned long BUTTON_DEBOUNCE_MS = 200; // ignore repeat triggers within this window per button

const uint32_t PWM_FREQ_HZ   = 1000;   // safe for TIP122 Darlington switching speed
const uint8_t  PWM_RESOLUTION = 8;     // 0-255 brightness steps

// Scanner behaviour - tune these to taste
const float BASE_SWEEP_PERIOD_MS = 3000.0f; // time for one full sweep at speed level 0 (slowest)
const float SPEED_STEP_FACTOR     = 0.85f;  // each speed level is this fraction of the previous period (faster)
const uint8_t MAX_SPEED_LEVEL     = 5;      // 5 presses of "faster" reaches max speed
const float TRAIL_DECAY       = 0.30f;   // 0-1, how fast the trailing fade drops off per zone step
const float SECOND_LIGHT_FACTOR = 1.0f;  // extra dim applied only to the zone immediately behind
                                          // the head, independent of TRAIL_DECAY - 1.0 = no extra dim
const uint8_t MAX_BRIGHTNESS  = 255;     // cap if you want to dim the whole bar globally
const uint16_t UPDATE_MS      = 18;      // frame update rate (~62.5 Hz), independent of sweep speed

// Mode 1 (six-light version) uses this contiguous range of physical zones -
// the two outermost zones stay dark. Adjust if you'd rather it be zones 0-5 instead.
const uint8_t SIX_ZONE_START = 1;
const uint8_t SIX_ZONE_COUNT = 6;

const uint16_t STARTUP_ON_MS = 2000; // all zones lit for this long on power-up / power-button-on

enum ScannerMode { MODE_ORIGINAL = 0, MODE_SIX_ZONE = 1, MODE_CENTER_OUT = 2, MODE_COUNT = 3 };

// ---- State ----
uint8_t currentMode = MODE_ORIGINAL;

float scannerPos = 0.0f;   // used by MODE_ORIGINAL and MODE_SIX_ZONE (local range 0..activeCount-1)
int8_t direction = 1;

float centerOffset = 0.0f; // used by MODE_CENTER_OUT: 0 = both heads at center, max = both heads at the ends
int8_t offsetDir = 1;

unsigned long lastUpdate = 0;
float zoneBrightness[NUM_ZONES] = {0}; // persists across frames, independent of sweep direction

float speedPeriodsMs[MAX_SPEED_LEVEL + 1]; // period in ms for each speed level, computed in setup()
uint8_t speedLevel = 0;                    // 0 = base/slowest, MAX_SPEED_LEVEL = fastest

float currentStepPerMs = 0.0f; // zones per ms at the current speed level
float frameDecay = 1.0f;       // per-frame trail decay at the current speed level

int8_t secondZoneA = -1; // zone(s) one step behind the head this frame, -1 = none
int8_t secondZoneB = -1; // (center-out mode has two heads, so up to two "second" zones)

bool lastFasterState = HIGH;
bool lastSlowerState = HIGH;
unsigned long lastFasterMs = 0;
unsigned long lastSlowerMs = 0;

bool systemOn = true;
bool lastPowerState = HIGH;
unsigned long lastPowerMs = 0;

bool lastModeState = HIGH;
unsigned long lastModeMs = 0;

uint8_t brightnessPercent = 100; // 0 = fully off, 100 = full brightness - independent of power on/off
bool lastBrightUpState = HIGH;
bool lastBrightDownState = HIGH;
unsigned long lastBrightUpMs = 0;
unsigned long lastBrightDownMs = 0;

bool inStartupFlash = false;
unsigned long startupFlashUntil = 0;

// Recompute the derived timing values whenever the speed level changes
void updateSweepTiming() {
  float periodMs = speedPeriodsMs[speedLevel];
  currentStepPerMs = (NUM_ZONES - 1) / (periodMs / 2.0f);
  float stepPerFrame = currentStepPerMs * UPDATE_MS;
  frameDecay = powf(TRAIL_DECAY, stepPerFrame);
}

void clearZones() {
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    zoneBrightness[i] = 0;
    ledcWrite(ZONE_PINS[i], 0);
  }
}

void allZonesFull() {
  uint8_t duty = (uint8_t) (MAX_BRIGHTNESS * (brightnessPercent / 100.0f));
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    ledcWrite(ZONE_PINS[i], duty);
  }
}

// Start the non-blocking "all zones on for STARTUP_ON_MS" flash used both by the
// power button turning back on and (optionally) elsewhere at runtime.
void beginStartupFlash() {
  inStartupFlash = true;
  startupFlashUntil = millis() + STARTUP_ON_MS;
  allZonesFull();
}

void increaseSpeed() {
  if (speedLevel < MAX_SPEED_LEVEL) {
    speedLevel++;
    updateSweepTiming();
    prefs.putUChar("speedLvl", speedLevel);
  }
}

void decreaseSpeed() {
  if (speedLevel > 0) {
    speedLevel--;
    updateSweepTiming();
    prefs.putUChar("speedLvl", speedLevel);
  }
}

void togglePower() {
  systemOn = !systemOn;
  if (!systemOn) {
    inStartupFlash = false;
    clearZones();
  } else {
    beginStartupFlash();
  }
}

void cycleMode() {
  currentMode = (currentMode + 1) % MODE_COUNT;
  // Reset motion state so the new mode starts clean, no stale trail from the old one
  scannerPos = 0.0f;
  direction = 1;
  centerOffset = 0.0f;
  offsetDir = 1;
  clearZones();
}

void setBrightness(int pct) {
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  brightnessPercent = (uint8_t) pct;
  prefs.putUChar("brightPct", brightnessPercent);
}

void increaseBrightness() { setBrightness(brightnessPercent + BRIGHTNESS_STEP_PERCENT); }
void decreaseBrightness() { setBrightness((int)brightnessPercent - BRIGHTNESS_STEP_PERCENT); }

void checkButtons() {
  unsigned long now = millis();
  bool faster = digitalRead(BTN_FASTER_PIN);
  bool slower = digitalRead(BTN_SLOWER_PIN);

  // Falling edge = button just pressed (INPUT_PULLUP reads LOW when pressed)
  if (faster == LOW && lastFasterState == HIGH && (now - lastFasterMs) > BUTTON_DEBOUNCE_MS) {
    increaseSpeed();
    lastFasterMs = now;
  }
  if (slower == LOW && lastSlowerState == HIGH && (now - lastSlowerMs) > BUTTON_DEBOUNCE_MS) {
    decreaseSpeed();
    lastSlowerMs = now;
  }
  lastFasterState = faster;
  lastSlowerState = slower;

  bool power = digitalRead(BTN_POWER_PIN);
  if (power == LOW && lastPowerState == HIGH && (now - lastPowerMs) > BUTTON_DEBOUNCE_MS) {
    togglePower();
    lastPowerMs = now;
  }
  lastPowerState = power;

  bool mode = digitalRead(BTN_MODE_PIN);
  if (mode == LOW && lastModeState == HIGH && (now - lastModeMs) > BUTTON_DEBOUNCE_MS) {
    cycleMode();
    lastModeMs = now;
  }
  lastModeState = mode;

  bool brightUp = digitalRead(BTN_BRIGHT_UP_PIN);
  if (brightUp == LOW && lastBrightUpState == HIGH && (now - lastBrightUpMs) > BUTTON_DEBOUNCE_MS) {
    increaseBrightness();
    lastBrightUpMs = now;
  }
  lastBrightUpState = brightUp;

  bool brightDown = digitalRead(BTN_BRIGHT_DOWN_PIN);
  if (brightDown == LOW && lastBrightDownState == HIGH && (now - lastBrightDownMs) > BUTTON_DEBOUNCE_MS) {
    decreaseBrightness();
    lastBrightDownMs = now;
  }
  lastBrightDownState = brightDown;
}

// Blend a moving head into the persistent per-zone brightness array, interpolating
// between the two nearest physical zones for smooth sub-step motion.
void applyHeadBrightness(float physicalPos) {
  uint8_t lower = (uint8_t) physicalPos;
  uint8_t upper = (lower + 1 < NUM_ZONES) ? lower + 1 : lower;
  float frac = physicalPos - lower;
  zoneBrightness[lower] = max(zoneBrightness[lower], MAX_BRIGHTNESS * (1.0f - frac));
  zoneBrightness[upper] = max(zoneBrightness[upper], MAX_BRIGHTNESS * frac);
}

// MODE_ORIGINAL and MODE_SIX_ZONE share this logic - a single head ping-ponging
// across a contiguous range of physical zones [startZone, startZone+zoneCount-1].
void updateSingleSweep(uint8_t startZone, uint8_t zoneCount) {
  scannerPos += direction * currentStepPerMs * UPDATE_MS;
  float maxPos = zoneCount - 1;
  if (scannerPos >= maxPos) {
    scannerPos = maxPos;
    direction = -1;
  } else if (scannerPos <= 0) {
    scannerPos = 0;
    direction = 1;
  }

  for (uint8_t i = 0; i < NUM_ZONES; i++) zoneBrightness[i] *= frameDecay;

  applyHeadBrightness(startZone + scannerPos);

  // The zone immediately behind the head (trailing side) - one full zone step back
  int16_t headZoneIdx = startZone + (int16_t) roundf(scannerPos);
  int16_t secondIdx = headZoneIdx - direction;
  secondZoneA = (secondIdx >= startZone && secondIdx < startZone + zoneCount) ? secondIdx : -1;
  secondZoneB = -1;
}

// MODE_CENTER_OUT - two heads move outward from the center pair in mirror image,
// then reverse back inward, repeating. Assumes an even NUM_ZONES.
void updateCenterOut() {
  float maxOffset = (NUM_ZONES / 2) - 1; // = 3 for 8 zones (reaches the outer edges)

  centerOffset += offsetDir * currentStepPerMs * UPDATE_MS;
  if (centerOffset >= maxOffset) {
    centerOffset = maxOffset;
    offsetDir = -1;
  } else if (centerOffset <= 0) {
    centerOffset = 0;
    offsetDir = 1;
  }

  for (uint8_t i = 0; i < NUM_ZONES; i++) zoneBrightness[i] *= frameDecay;

  float posLow = ((NUM_ZONES / 2) - 1) - centerOffset; // moves from center down toward zone 0
  float posHigh = (NUM_ZONES / 2) + centerOffset;      // moves from center up toward the last zone
  applyHeadBrightness(posLow);
  applyHeadBrightness(posHigh);

  // Each head's trailing neighbor is one zone step back along its own direction of travel.
  // posLow moves opposite to offsetDir; posHigh moves the same way as offsetDir.
  int16_t lowIdx = (int16_t) roundf(posLow);
  int16_t highIdx = (int16_t) roundf(posHigh);
  int16_t secondLow = lowIdx + offsetDir;
  int16_t secondHigh = highIdx - offsetDir;
  secondZoneA = (secondLow >= 0 && secondLow < NUM_ZONES) ? secondLow : -1;
  secondZoneB = (secondHigh >= 0 && secondHigh < NUM_ZONES) ? secondHigh : -1;
}

const char* modeName(uint8_t m) {
  switch (m) {
    case MODE_ORIGINAL: return "Original (8 zone)";
    case MODE_SIX_ZONE: return "Six zone";
    case MODE_CENTER_OUT: return "Center-out mirrored";
    default: return "?";
  }
}

const char PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>KITT Scanner Control</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:20px}
h1{color:#e11}
button{font-size:1.1em;padding:14px 22px;margin:8px;border:none;border-radius:8px;
  background:#333;color:#eee;min-width:110px}
button:active{background:#555}
#power{background:#900}
#status{margin-top:20px;font-size:1.1em;line-height:1.7em}
</style></head><body>
<h1>KITT Scanner</h1>
<div>
  <button id="power" onclick="post('/power')">Power</button>
  <button onclick="post('/mode')">Mode</button>
</div>
<div>
  <button onclick="post('/slower')">- Slower</button>
  <button onclick="post('/faster')">+ Faster</button>
</div>
<div>
  <label for="bright">Brightness</label><br>
  <input type="range" id="bright" min="0" max="100" value="100"
    oninput="document.getElementById('brightVal').innerText = this.value + '%'"
    onchange="setBrightness(this.value)" style="width:80%">
  <div id="brightVal">100%</div>
</div>
<div id="status">loading...</div>
<script>
async function post(path){
  await fetch(path, {method:'POST'});
  refresh();
}
async function setBrightness(val){
  await fetch('/brightness?value=' + val, {method:'POST'});
  refresh();
}
async function refresh(){
  const r = await fetch('/status');
  const s = await r.json();
  document.getElementById('status').innerHTML =
    'Power: ' + (s.on ? 'ON' : 'OFF') + '<br>' +
    'Mode: ' + s.mode + '<br>' +
    'Speed level: ' + s.speed + ' / ' + s.maxSpeed + '<br>' +
    'Brightness: ' + s.brightness + '%';
  document.getElementById('bright').value = s.brightness;
  document.getElementById('brightVal').innerText = s.brightness + '%';
}
refresh();
setInterval(refresh, 2000);
</script></body></html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", PAGE_HTML);
}

void handleStatus() {
  String json = "{";
  json += "\"on\":" + String(systemOn ? "true" : "false") + ",";
  json += "\"mode\":\"" + String(modeName(currentMode)) + "\",";
  json += "\"speed\":" + String(speedLevel) + ",";
  json += "\"maxSpeed\":" + String(MAX_SPEED_LEVEL) + ",";
  json += "\"brightness\":" + String(brightnessPercent);
  json += "}";
  server.send(200, "application/json", json);
}

void handlePower() { togglePower(); server.send(200, "text/plain", "ok"); }
void handleMode()  { cycleMode();   server.send(200, "text/plain", "ok"); }
void handleFaster(){ increaseSpeed(); server.send(200, "text/plain", "ok"); }
void handleSlower(){ decreaseSpeed(); server.send(200, "text/plain", "ok"); }
void handleBrightness() {
  if (server.hasArg("value")) {
    setBrightness(server.arg("value").toInt());
  }
  server.send(200, "text/plain", "ok");
}

void setupWebServer() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected to ");
    Serial.print(WIFI_SSID);
    Serial.print(". Open http://");
    Serial.println(WiFi.localIP());
  } else {
    // No known network found (e.g. at a car show) - broadcast our own hotspot instead
    Serial.println();
    Serial.println("No WiFi found - starting standalone hotspot instead.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("Connect your phone to WiFi network \"");
    Serial.print(AP_SSID);
    Serial.print("\" (password: ");
    Serial.print(AP_PASSWORD);
    Serial.println(")");
    Serial.print("Then open http://");
    Serial.println(WiFi.softAPIP());
  }

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/power", HTTP_POST, handlePower);
  server.on("/mode", HTTP_POST, handleMode);
  server.on("/faster", HTTP_POST, handleFaster);
  server.on("/slower", HTTP_POST, handleSlower);
  server.on("/brightness", HTTP_POST, handleBrightness);
  server.begin();
}

void setup() {
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    ledcAttach(ZONE_PINS[i], PWM_FREQ_HZ, PWM_RESOLUTION); // Arduino-ESP32 core 3.x API
  }

  pinMode(BTN_FASTER_PIN, INPUT_PULLUP);
  pinMode(BTN_SLOWER_PIN, INPUT_PULLUP);
  pinMode(BTN_POWER_PIN, INPUT_PULLUP);
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
  pinMode(BTN_BRIGHT_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_BRIGHT_DOWN_PIN, INPUT_PULLUP);

  // Build the speed level table: level 0 is the base period, each level after
  // that is SPEED_STEP_FACTOR times the previous (progressively faster).
  speedPeriodsMs[0] = BASE_SWEEP_PERIOD_MS;
  for (uint8_t i = 1; i <= MAX_SPEED_LEVEL; i++) {
    speedPeriodsMs[i] = speedPeriodsMs[i - 1] * SPEED_STEP_FACTOR;
  }

  // Restore the last speed level from flash (defaults to 0 the very first boot)
  prefs.begin("kittscan", false);
  speedLevel = prefs.getUChar("speedLvl", 0);
  if (speedLevel > MAX_SPEED_LEVEL) speedLevel = 0; // guard against corrupt/out-of-range data
  updateSweepTiming();

  brightnessPercent = prefs.getUChar("brightPct", 100);
  if (brightnessPercent > 100) brightnessPercent = 100; // guard against corrupt/out-of-range data

  // Startup sequence on power-up: all zones lit for STARTUP_ON_MS, then hand off to the scanner
  allZonesFull();
  delay(STARTUP_ON_MS);
  clearZones();

  setupWebServer();
}

void loop() {
  server.handleClient(); // always serviced, even while powered off, so the web page still works

  // Check buttons every pass, not gated by UPDATE_MS, so presses feel responsive
  checkButtons();

  if (!systemOn) return; // powered off - zones already forced to 0 in checkButtons()

  if (inStartupFlash) {
    if ((long)(millis() - startupFlashUntil) < 0) {
      return; // still in the all-on flash triggered by the power button
    }
    inStartupFlash = false;
    clearZones();
  }

  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_MS) return;
  lastUpdate = now;

  switch (currentMode) {
    case MODE_ORIGINAL:
      updateSingleSweep(0, NUM_ZONES);
      break;
    case MODE_SIX_ZONE:
      updateSingleSweep(SIX_ZONE_START, SIX_ZONE_COUNT);
      break;
    case MODE_CENTER_OUT:
      updateCenterOut();
      break;
  }

  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    float b = zoneBrightness[i];
    if (i == secondZoneA || i == secondZoneB) {
      b *= SECOND_LIGHT_FACTOR; // extra dim for display only - stored value/decay untouched
    }
    b *= (brightnessPercent / 100.0f); // overall brightness scale, display only
    uint8_t duty = (uint8_t) constrain(b, 0, MAX_BRIGHTNESS);
    ledcWrite(ZONE_PINS[i], duty);
  }
}
