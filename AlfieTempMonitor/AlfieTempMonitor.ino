#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int tempPin  = A0;  // TMP36 Vout
const int fanPin = 11;
const float vRef   = 5.0; // Uno ADC reference (default)

LiquidCrystal_I2C lcd(0x27, 16, 2);

const uint8_t LCD_COLS = 16;

// Fan behaviour
const uint8_t FAN_OFF = 0;
const uint8_t FAN_ON  = 255;   // full speed (0-255)
const unsigned long FAN_KICK_MS = 300; // spin-up help for low duty starts (optional)
bool fanWasOff = true;

// Bi-colour 2-pin LED across D1/D2
const int biPinA = 1;          // D1
const int biPinB = 2;          // D2
float thresholdC = 22.0;       // Red if over threshold, else green

// Momentary push buttons
const int btnUpPin   = 8;      // Increase threshold by 1°C
const int btnDownPin = 9;      // Decrease threshold by 1°C

const unsigned long DEBOUNCE_MS          = 50;    // debounce window
const unsigned long THRESHOLD_DISPLAY_MS = 2000;  // how long to show new threshold
const float THRESHOLD_MIN_C              = 0.0;   // minimum allowed threshold
const float THRESHOLD_MAX_C              = 50.0;  // maximum allowed threshold

// Hold-to-repeat timing
const unsigned long HOLD_DELAY_MS   = 600;   // initial pause before repeat starts
const unsigned long REPEAT_FAST_MS  = 100;   // fast repeat interval (after HOLD_FAST_MS)
const unsigned long REPEAT_SLOW_MS  = 300;   // slow repeat interval (before HOLD_FAST_MS)
const unsigned long HOLD_FAST_MS    = 2000;  // switch to fast repeat after holding this long

// Debounce + hold state for each button
int  btnUpLastReading   = HIGH;
int  btnUpStableState   = HIGH;
unsigned long btnUpLastChangeMs   = 0;
unsigned long btnUpPressedMs      = 0;  // when the button was first confirmed pressed
unsigned long btnUpLastRepeatMs   = 0;  // when we last fired a repeat

int  btnDownLastReading = HIGH;
int  btnDownStableState = HIGH;
unsigned long btnDownLastChangeMs = 0;
unsigned long btnDownPressedMs    = 0;
unsigned long btnDownLastRepeatMs = 0;

// Threshold display state
bool showingThreshold          = false;
unsigned long thresholdShowMs  = 0;

// -------- Fixed graph scale (NO autoscale) --------
const float GRAPH_MIN_C = 10.0;
const float GRAPH_MAX_C = 25.0;

// -------- Bar timing --------
// Each bar represents 30 minutes
const unsigned long BAR_PERIOD_MS = 30UL * 60UL * 1000UL;

// We have 16 columns available for bars.
// We'll show 15 completed 30-min averages + 1 current in-progress average.
const uint8_t BAR_COLS = LCD_COLS; // 16

// Rolling average settings (for “current temp” smoothing)
const uint8_t NUM_SAMPLES = 6;
const unsigned long SAMPLE_INTERVAL_MS  = 1000;  // take a reading every 1s
const unsigned long DISPLAY_INTERVAL_MS = 5000;  // update display every 5s

float samplesC[NUM_SAMPLES];
uint8_t sampleIndex = 0;
uint8_t samplesFilled = 0;
float sampleSumC = 0.0;

unsigned long lastSampleMs  = 0;
unsigned long lastDisplayMs = 0;

// Track last values actually shown on LCD (in tenths, matching 1dp display)
int lastShownC10 = 99999;

// -------- History of COMPLETED 30-min buckets --------
float bucketHistoryC[BAR_COLS];
uint8_t bucketHistoryFilled = 0;

// Track last rendered graph levels so we only redraw when it changes
uint8_t lastGraphLevel[LCD_COLS];

// Current 30-min bucket accumulator
unsigned long bucketStartMs = 0;
float bucketSumC = 0.0;
unsigned long bucketCount = 0;

// Custom chars 0..7 are bar heights 0..7 (0 empty, 7 full)
void initBarChars() {
  for (uint8_t level = 0; level < 8; level++) {
    byte glyph[8];
    for (uint8_t row = 0; row < 8; row++) {
      glyph[row] = (row >= (8 - level)) ? B11111 : B00000;
    }
    lcd.createChar(level, glyph);
  }
}

void biOff() {
  digitalWrite(biPinA, LOW);
  digitalWrite(biPinB, LOW);
}
void biGreen() {
  digitalWrite(biPinA, HIGH);
  digitalWrite(biPinB, LOW);
}
void biRed() {
  digitalWrite(biPinA, LOW);
  digitalWrite(biPinB, HIGH);
}

float readTempC() {
  int raw = analogRead(tempPin);
  float voltage = raw * (vRef / 1023.0);
  return (voltage - 0.5) * 100.0; // TMP36 conversion
}

// Rolling average insert (O(1))
void addSample(float tempC) {
  if (samplesFilled < NUM_SAMPLES) {
    samplesC[sampleIndex] = tempC;
    sampleSumC += tempC;
    samplesFilled++;
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;
  } else {
    sampleSumC -= samplesC[sampleIndex];
    samplesC[sampleIndex] = tempC;
    sampleSumC += tempC;
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;
  }
}

float getAverageC() {
  if (samplesFilled == 0) return NAN;
  return sampleSumC / samplesFilled;
}

int toTenths(float x) {
  return (int)(x * 10.0 + (x >= 0 ? 0.5 : -0.5));
}

// Push a COMPLETED 30-min bucket average into history (shift left, append right)
void pushBucketHistory(float avgC) {
  if (bucketHistoryFilled < BAR_COLS) {
    bucketHistoryC[bucketHistoryFilled++] = avgC;
  } else {
    for (uint8_t i = 0; i < BAR_COLS - 1; i++) bucketHistoryC[i] = bucketHistoryC[i + 1];
    bucketHistoryC[BAR_COLS - 1] = avgC;
  }
}

// Map temp to a level 0..7 using FIXED scale (10C..25C)
uint8_t tempToLevel(float tempC) {
  float span = GRAPH_MAX_C - GRAPH_MIN_C;
  if (span < 0.001) return 0;

  float norm = (tempC - GRAPH_MIN_C) / span;  // 0..1
  if (norm < 0) norm = 0;
  if (norm > 1) norm = 1;

  uint8_t level = (uint8_t)(norm * 7.0 + 0.5);
  if (level > 7) level = 7;
  return level;
}

// Current in-progress 30-minute bucket average (for the rightmost bar)
float currentBucketAvg() {
  if (bucketCount == 0) return NAN;
  return bucketSumC / (float)bucketCount;
}

// Draw the graph on row 1 (second row).
// Cols 0..14 = last 15 completed 30-min averages
// Col  15    = current in-progress 30-min average
void drawGraphRow() {
  const uint8_t completedBarsToShow = LCD_COLS - 1; // 15

  // Show the most recent 15 completed buckets
  int startIndex = (int)bucketHistoryFilled - (int)completedBarsToShow;
  if (startIndex < 0) startIndex = 0;

  // Completed bars
  for (uint8_t i = 0; i < completedBarsToShow; i++) {
    uint8_t lcdCol = i; // 0..14
    int histIndex = startIndex + i;

    uint8_t level = 0;
    if (histIndex >= 0 && histIndex < bucketHistoryFilled) {
      level = tempToLevel(bucketHistoryC[histIndex]);
    }
    lcd.setCursor(lcdCol, 1);
    lcd.write(byte(level));
  }

  // Current in-progress bar (rightmost)
  float cur = currentBucketAvg();
  uint8_t curLevel = 0;
  if (!isnan(cur)) curLevel = tempToLevel(cur);

  lcd.setCursor(LCD_COLS - 1, 1); // col 15
  lcd.write(byte(curLevel));
}

bool graphChanged() {
  const uint8_t completedBarsToShow = LCD_COLS - 1; // 15

  int startIndex = (int)bucketHistoryFilled - (int)completedBarsToShow;
  if (startIndex < 0) startIndex = 0;

  // Completed bars
  for (uint8_t i = 0; i < completedBarsToShow; i++) {
    uint8_t lcdCol = i; // 0..14
    int histIndex = startIndex + i;

    uint8_t level = 0;
    if (histIndex >= 0 && histIndex < bucketHistoryFilled) {
      level = tempToLevel(bucketHistoryC[histIndex]);
    }
    if (level != lastGraphLevel[lcdCol]) return true;
  }

  // Current bar
  float cur = currentBucketAvg();
  uint8_t curLevel = 0;
  if (!isnan(cur)) curLevel = tempToLevel(cur);

  if (curLevel != lastGraphLevel[LCD_COLS - 1]) return true;

  return false;
}

void snapshotGraphLevels() {
  const uint8_t completedBarsToShow = LCD_COLS - 1; // 15

  int startIndex = (int)bucketHistoryFilled - (int)completedBarsToShow;
  if (startIndex < 0) startIndex = 0;

  // Completed bars
  for (uint8_t i = 0; i < completedBarsToShow; i++) {
    uint8_t lcdCol = i; // 0..14
    int histIndex = startIndex + i;

    uint8_t level = 0;
    if (histIndex >= 0 && histIndex < bucketHistoryFilled) {
      level = tempToLevel(bucketHistoryC[histIndex]);
    }
    lastGraphLevel[lcdCol] = level;
  }

  // Current bar
  float cur = currentBucketAvg();
  uint8_t curLevel = 0;
  if (!isnan(cur)) curLevel = tempToLevel(cur);
  lastGraphLevel[LCD_COLS - 1] = curLevel;
}

// Returns true once when the button transitions to a stable LOW (initial press).
// Records pressedMs so hold-to-repeat can use it.
bool buttonPressed(int pin,
                   int &lastReading,
                   int &stableState,
                   unsigned long &lastChangeMs,
                   unsigned long &pressedMs,
                   unsigned long &lastRepeatMs,
                   unsigned long now) {
  int reading = digitalRead(pin);
  if (reading != lastReading) {
    lastChangeMs = now;
    lastReading  = reading;
  }
  if ((now - lastChangeMs) >= DEBOUNCE_MS && reading != stableState) {
    stableState = reading;
    if (stableState == LOW) {
      pressedMs    = now;
      lastRepeatMs = now;
      return true;   // newly pressed
    }
  }
  return false;
}

// Returns true on each hold-to-repeat tick while the button stays pressed.
// Uses an accelerating interval: slow after HOLD_DELAY_MS, fast after HOLD_FAST_MS.
bool buttonHeldTick(int stableState,
                    unsigned long pressedMs,
                    unsigned long &lastRepeatMs,
                    unsigned long now) {
  if (stableState != LOW) return false;            // button not held
  unsigned long held = now - pressedMs;
  if (held < HOLD_DELAY_MS) return false;          // still in initial delay

  unsigned long interval = (held >= HOLD_FAST_MS) ? REPEAT_FAST_MS : REPEAT_SLOW_MS;
  if (now - lastRepeatMs >= interval) {
    lastRepeatMs = now;
    return true;
  }
  return false;
}

// Show the current threshold value on row 0 of the LCD.
void displayThreshold() {
  String line = "Threshold:" + String(thresholdC, 1) + (char)223 + "C";
  uint8_t len = line.length();
  if (len > LCD_COLS) len = LCD_COLS;
  uint8_t col = (LCD_COLS - len) / 2;

  lcd.setCursor(0, 0);
  for (uint8_t i = 0; i < LCD_COLS; i++) lcd.print(' ');
  lcd.setCursor(col, 0);
  lcd.print(line.substring(0, len));
}

void setup() {
  pinMode(biPinA, OUTPUT);
  pinMode(biPinB, OUTPUT);
  biOff();

  pinMode(btnUpPin,   INPUT_PULLUP);
  pinMode(btnDownPin, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  initBarChars();

  // Init graph cache
  for (uint8_t i = 0; i < LCD_COLS; i++) lastGraphLevel[i] = 255;

  lcd.setCursor(0, 0);
  lcd.print("Alfie Temp");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(1000);

  // Prime rolling average
  for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
    addSample(readTempC());
    delay(50);
  }

  pinMode(fanPin, OUTPUT);
  analogWrite(fanPin, FAN_OFF);

  // Seed bucket history so graph isn’t blank
  float startC = getAverageC();
  for (uint8_t i = 0; i < BAR_COLS; i++) pushBucketHistory(startC);

  // Start the 30-min bucket timer now
  bucketStartMs = millis();
  bucketSumC = 0.0;
  bucketCount = 0;

  lastSampleMs  = millis();
  lastDisplayMs = millis();
}

void loop() {
  unsigned long now = millis();

  // 0) Read push buttons and adjust threshold (with hold-to-repeat acceleration)
  bool upPressed   = buttonPressed(btnUpPin,   btnUpLastReading,   btnUpStableState,   btnUpLastChangeMs,   btnUpPressedMs,   btnUpLastRepeatMs,   now);
  bool upHeld      = buttonHeldTick(btnUpStableState,   btnUpPressedMs,   btnUpLastRepeatMs,   now);
  bool downPressed = buttonPressed(btnDownPin, btnDownLastReading, btnDownStableState, btnDownLastChangeMs, btnDownPressedMs, btnDownLastRepeatMs, now);
  bool downHeld    = buttonHeldTick(btnDownStableState, btnDownPressedMs, btnDownLastRepeatMs, now);

  if (upPressed || upHeld) {
    if (thresholdC < THRESHOLD_MAX_C) {
      thresholdC += 1.0;
    }
    showingThreshold = true;
    thresholdShowMs  = now;
    displayThreshold();
  }
  if (downPressed || downHeld) {
    if (thresholdC > THRESHOLD_MIN_C) {
      thresholdC -= 1.0;
    }
    showingThreshold = true;
    thresholdShowMs  = now;
    displayThreshold();
  }

  // When the threshold-display window expires, revert to normal view
  if (showingThreshold && (now - thresholdShowMs >= THRESHOLD_DISPLAY_MS)) {
    showingThreshold = false;
    lastShownC10 = 99999;  // force a full redraw on the next display tick
  }

  // 1) Sample every second
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;

    float rawC = readTempC();
    addSample(rawC);

    // Accumulate 30-min bucket using the smoothed value
    float avgNow = getAverageC();
    if (!isnan(avgNow)) {
      bucketSumC += avgNow;
      bucketCount++;
    }

    // Finalize bucket(s) if 30 mins elapsed (while handles long delays)
    while (now - bucketStartMs >= BAR_PERIOD_MS) {
      float finalized = NAN;
      if (bucketCount > 0) finalized = bucketSumC / (float)bucketCount;
      if (isnan(finalized)) finalized = getAverageC();

      pushBucketHistory(finalized);

      // Advance bucket boundary by exactly one period
      bucketStartMs += BAR_PERIOD_MS;
      bucketSumC = 0.0;
      bucketCount = 0;
    }
  }

  // 2) Update display every 5 seconds (only redraw if changed)
  if (now - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = now;

    float avgC = getAverageC();
    int c10 = toTenths(avgC);

    // LED + FAN based on averaged temperature
    if (avgC > thresholdC) {
      biRed();

      // Optional kick-start when turning on
      if (fanWasOff) {
        analogWrite(fanPin, 255);
        delay(FAN_KICK_MS);
        fanWasOff = false;
      }

      analogWrite(fanPin, FAN_ON);   // run fan (PWM)
    } else {
      biGreen();
      analogWrite(fanPin, FAN_OFF);  // stop fan
      fanWasOff = true;
    }

    bool tempChanged = (c10 != lastShownC10);
    bool gChanged = graphChanged();

    if (tempChanged || gChanged) {
      // Row 0: centered temperature (skip if currently showing threshold)
      if (!showingThreshold) {
        String line0 = "-= " + String(avgC, 1) + (char)223 + "C =-";
        uint8_t len = line0.length();
        if (len > LCD_COLS) len = LCD_COLS;
        uint8_t col = (LCD_COLS - len) / 2;

        lcd.setCursor(0, 0);
        for (uint8_t i = 0; i < LCD_COLS; i++) lcd.print(' ');
        lcd.setCursor(col, 0);
        lcd.print(line0.substring(0, len));
      }

      // Row 1: graph (no ticks)
      drawGraphRow();
      snapshotGraphLevels();

      lastShownC10 = c10;
    }
  }
}
