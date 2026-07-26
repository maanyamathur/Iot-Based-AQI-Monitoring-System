/*
 * Arduino Nano - 10 Chained P10 Panels (Serpentine Wiring)
 * Layout: 5->4->3->2->1 (top row) then 6->7->8->9->10 (bottom row)
 * Role: PURE DISPLAY DRIVER ONLY.
 * All formatting/switching logic lives on the ESP32 now.
 * Nano just receives "<upperLine|lowerLine>" and draws it. Nothing else.
 */

#include <SPI.h>
#include <DMD.h>
#include <TimerOne.h>
#include "Arial_Black_16.h"
#include <avr/wdt.h>

// ─── P10 PANEL CONFIGURATION ─────────────────────────────────────────────────
#define DISPLAYS_WIDE 5   // 5 panels wide (160 pixels)
#define DISPLAYS_HIGH 2   // 2 panels stacked (32 pixels)
DMD dmd(DISPLAYS_WIDE, DISPLAYS_HIGH);

// ─── SERIAL RECEIVE BUFFER (small — only two short text lines now) ──────────
const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

// ─── CURRENT SCREEN TEXT (persists until next valid packet arrives) ─────────
char cUpper[32] = "CONNECTING";
char cLower[32] = "PLEASE WAIT";

void ScanDMD() {
  dmd.scanDisplayBySPI();
}

void setup() {
  // MUST be first two lines: clears watchdog reset flag and disables WDT immediately.
  // Without this, a watchdog-triggered reset can re-trigger before setup() finishes,
  // causing the Nano to loop-reset forever and never actually draw anything.
  MCUSR = 0;
  wdt_disable();

  Serial.begin(9600); // Same UART line from ESP32 (Serial1 / GPIO4-5 on ESP32 side)

  Timer1.initialize(5000);
  Timer1.attachInterrupt(ScanDMD);

  dmd.clearScreen(true);
  dmd.selectFont(Arial_Black_16);

  // Show initial placeholder immediately so panel isn't blank before first packet
  drawCurrentScreen();

  // Watchdog: if loop() ever hangs for 8s (e.g. stuck serial read), Nano auto-resets.
  // Safe here since Timer1's ISR (ScanDMD) keeps the panel refreshed independently of loop().
  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset(); // Feed watchdog every pass — proves loop() is alive, not frozen

  recvWithStartEndMarkers();

  if (newData) {
    processPacket();
    newData = false;
  }
}

// ─── Non-blocking Packet Reader (same bulletproof recovery as before) ───────
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1; // Clamp instead of wrapping to avoid overwrite corruption
        }
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
      ndx = 0;
    }
  }
}

// ─── Split "upperLine|lowerLine" and update display text ───────────────────
void processPacket() {
  char *sep = strchr(receivedChars, '|');
  if (sep == NULL) {
    return; // Malformed packet — keep showing the last good screen, don't corrupt it
  }
  *sep = '\0';

  strncpy(cUpper, receivedChars, sizeof(cUpper) - 1);
  cUpper[sizeof(cUpper) - 1] = '\0';

  strncpy(cLower, sep + 1, sizeof(cLower) - 1);
  cLower[sizeof(cLower) - 1] = '\0';

  drawCurrentScreen();
}

// ─── Draw cUpper/cLower to the panel (instant pop-up, no animation) ────────
void drawCurrentScreen() {
  dmd.clearScreen(true);
  dmd.drawString(4, 16, cUpper, strlen(cUpper), GRAPHICS_NORMAL); // bottom row
  dmd.drawString(4, 0,  cLower, strlen(cLower), GRAPHICS_NORMAL); // top row
}
