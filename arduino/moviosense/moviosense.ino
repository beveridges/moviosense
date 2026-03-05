/*
  XIAO nRF52840 Sense + (1) W25Qxx SPI flash + (2) microSD (SPI) merged test
  + prints detected flash capacity (bytes / MB / Mbit)

  Goal:
  - Run in TWO modes WITHOUT changing code:
    A) USB only: power + logs over USB Serial (Serial)
    B) Battery + FTDI: power from battery, logs over UART (Serial1) to FTDI

  How it works:
  - At boot, we start Serial (USB) and Serial1 (UART).
  - We wait briefly (~1.5s) to see if USB Serial is actually opened.
    If yes -> use Serial.
    If no  -> use Serial1.
  - LED blink indicator:
      1 blink = USB Serial chosen
      2 blinks = FTDI Serial1 chosen

  IMPORTANT:
  - W25Qxx and microSD share the same SPI bus (SCK/MOSI/MISO).
  - They MUST have different CS pins.
  - Only one device's CS should be LOW at a time.

  Example wiring:
    SPI bus:
      SCK  -> SCK (both flash + SD)
      MOSI -> MOSI (both)
      MISO -> MISO (both)

    Chip selects:
      FLASH_CS -> D1  (example)
      SD_CS    -> D2  (example)

  FTDI wiring (for battery+FTDI mode):
    XIAO D6 (TX / Serial1 TX) -> FTDI RXD
    XIAO D7 (RX / Serial1 RX) -> FTDI TXD (optional)
    XIAO GND                  -> FTDI GND
    FTDI must be 3.3V logic. Do NOT connect FTDI VCC if powering XIAO from battery.

  Power:
    - XIAO is 3.3V logic.
    - Ensure flash + SD module(s) are compatible with 3.3V.
*/

#include <SPI.h>
#include <SD.h>

// ----------------- YOUR PIN ALLOCATION (unchanged) -----------------
static const uint8_t FLASH_CS = D1;   // <-- flash CS
static const uint8_t SD_CS    = D2;   // <-- SD CS (must be different!)

// ----------------- W25Qxx commands -----------------
static const uint8_t CMD_RDID   = 0x9F; // Read JEDEC ID
static const uint8_t CMD_WREN   = 0x06; // Write enable
static const uint8_t CMD_RDSR1  = 0x05; // Read status reg-1
static const uint8_t CMD_SE     = 0x20; // Sector erase 4KB
static const uint8_t CMD_PP     = 0x02; // Page program
static const uint8_t CMD_READ   = 0x03; // Read data

static const uint32_t TEST_ADDR = 0x000000; // start of flash

// ----------------- Debug port auto-select -----------------
Stream* DBG = nullptr;

// If your LED blinks inverted, flip this to 1.
#define LED_ACTIVE_LOW 1

static inline void ledOn() {
#if LED_ACTIVE_LOW
  digitalWrite(LED_BUILTIN, LOW);
#else
  digitalWrite(LED_BUILTIN, HIGH);
#endif
}
static inline void ledOff() {
#if LED_ACTIVE_LOW
  digitalWrite(LED_BUILTIN, HIGH);
#else
  digitalWrite(LED_BUILTIN, LOW);
#endif
}

void blinkModeIndicator(bool usbMode) {
  int blinks = usbMode ? 1 : 2;
  for (int i = 0; i < blinks; i++) {
    ledOn();
    delay(200);
    ledOff();
    delay(200);
  }
  delay(500);
}

// Pick USB Serial if it is opened quickly; otherwise use Serial1 (FTDI UART).
void initDebugPort() {
  Serial1.begin(115200);   // UART to FTDI (XIAO: D6 TX, D7 RX)
  Serial.begin(115200);    // USB CDC

  uint32_t t0 = millis();
  bool usbMode = false;

  while ((millis() - t0) < 1500) {
    // On many Arduino cores, `if (Serial)` becomes true when the host opens it.
    if (Serial) {
      DBG = &Serial;
      usbMode = true;
      break;
    }
    delay(10);
  }

  if (!DBG) {
    DBG = &Serial1;
  }

  blinkModeIndicator(usbMode);

  DBG->println("\n[DBG] Debug port selected:");
  DBG->println(usbMode ? "USB Serial (Serial)" : "FTDI UART (Serial1)");
}

static inline void deselectAll() {
  digitalWrite(FLASH_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
}

uint8_t readStatus1() {
  digitalWrite(FLASH_CS, LOW);
  SPI.transfer(CMD_RDSR1);
  uint8_t s = SPI.transfer(0x00);
  digitalWrite(FLASH_CS, HIGH);
  return s;
}

void waitWhileBusy(uint32_t timeout_ms = 5000) {
  uint32_t start = millis();
  while (readStatus1() & 0x01) { // WIP bit
    if (millis() - start > timeout_ms) {
      DBG->println("ERROR: Flash busy timeout");
      return;
    }
    delay(1);
  }
}

void writeEnable() {
  digitalWrite(FLASH_CS, LOW);
  SPI.transfer(CMD_WREN);
  digitalWrite(FLASH_CS, HIGH);
}

void readJedec(uint8_t &mfg, uint8_t &memType, uint8_t &capacity) {
  digitalWrite(FLASH_CS, LOW);
  SPI.transfer(CMD_RDID);
  mfg      = SPI.transfer(0x00);
  memType  = SPI.transfer(0x00);
  capacity = SPI.transfer(0x00);
  digitalWrite(FLASH_CS, HIGH);
}

// -------- Capacity decode helpers --------

// Winbond-friendly lookup for common W25Q capacity codes
bool winbondCapacityBytes(uint8_t capCode, uint32_t &bytesOut) {
  switch (capCode) {
    case 0x14: bytesOut = 1UL   * 1024UL * 1024UL;  return true; //  8 Mbit  = 1 MB
    case 0x15: bytesOut = 2UL   * 1024UL * 1024UL;  return true; // 16 Mbit  = 2 MB
    case 0x16: bytesOut = 4UL   * 1024UL * 1024UL;  return true; // 32 Mbit  = 4 MB
    case 0x17: bytesOut = 8UL   * 1024UL * 1024UL;  return true; // 64 Mbit  = 8 MB
    case 0x18: bytesOut = 16UL  * 1024UL * 1024UL;  return true; // 128 Mbit = 16 MB
    case 0x19: bytesOut = 32UL  * 1024UL * 1024UL;  return true; // 256 Mbit = 32 MB
    case 0x20: bytesOut = 64UL  * 1024UL * 1024UL;  return true; // 512 Mbit = 64 MB
    case 0x21: bytesOut = 128UL * 1024UL * 1024UL;  return true; // 1 Gbit   = 128 MB
    default: return false;
  }
}

// Generic JEDEC “density” style rule (often works, but not guaranteed for all parts)
uint32_t jedecDensityBytesGuess(uint8_t capCode) {
  // Many flashes use: bytes = 1 << capCode
  // Example: 0x17 => 1<<23 = 8,388,608 bytes = 8 MB
  if (capCode < 16 || capCode > 30) return 0;
  return (uint32_t)(1UL << capCode);
}

void printFlashCapacity(uint8_t mfg, uint8_t memType, uint8_t capCode) {
  (void)mfg; (void)memType;

  DBG->println("\n--- Flash capacity decode ---");
  DBG->print("Capacity code: 0x");
  DBG->println(capCode, HEX);

  uint32_t bytesTable = 0;
  bool tableOK = winbondCapacityBytes(capCode, bytesTable);

  if (tableOK) {
    float mb = bytesTable / (1024.0f * 1024.0f);
    float mbit = (bytesTable * 8.0f) / (1024.0f * 1024.0f);
    DBG->print("Capacity (table): ");
    DBG->print(bytesTable);
    DBG->print(" bytes  (~");
    DBG->print(mb, 2);
    DBG->print(" MB, ");
    DBG->print(mbit, 0);
    DBG->println(" Mbit)");
  } else {
    DBG->println("Capacity (table): unknown code (not in lookup).");
  }

  uint32_t bytesGuess = jedecDensityBytesGuess(capCode);
  if (bytesGuess) {
    float mb = bytesGuess / (1024.0f * 1024.0f);
    float mbit = (bytesGuess * 8.0f) / (1024.0f * 1024.0f);
    DBG->print("Capacity (JEDEC guess): ");
    DBG->print(bytesGuess);
    DBG->print(" bytes  (~");
    DBG->print(mb, 2);
    DBG->print(" MB, ");
    DBG->print(mbit, 0);
    DBG->println(" Mbit)");
  } else {
    DBG->println("Capacity (JEDEC guess): could not decode.");
  }
}

// -------- end helpers --------

void sectorErase4K(uint32_t addr) {
  writeEnable();
  digitalWrite(FLASH_CS, LOW);
  SPI.transfer(CMD_SE);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  digitalWrite(FLASH_CS, HIGH);
  waitWhileBusy(15000);
}

void pageProgram(uint32_t addr, const uint8_t *data, size_t len) {
  // len must be <= 256 and must not cross a page boundary
  writeEnable();
  digitalWrite(FLASH_CS, LOW);
  SPI.transfer(CMD_PP);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (size_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(FLASH_CS, HIGH);
  waitWhileBusy(5000);
}

void flashRead(uint32_t addr, uint8_t *out, size_t len) {
  digitalWrite(FLASH_CS, LOW);
  SPI.transfer(CMD_READ);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  for (size_t i = 0; i < len; i++) {
    out[i] = SPI.transfer(0x00);
  }
  digitalWrite(FLASH_CS, HIGH);
}

bool testFlash() {
  DBG->println("\n=== W25Qxx FLASH TEST ===");
  deselectAll();

  uint8_t mfg, memType, cap;
  readJedec(mfg, memType, cap);

  DBG->print("JEDEC ID: 0x"); DBG->print(mfg, HEX);
  DBG->print(" 0x");         DBG->print(memType, HEX);
  DBG->print(" 0x");         DBG->println(cap, HEX);

  // Print capacity info right after reading JEDEC
  printFlashCapacity(mfg, memType, cap);

  if (mfg == 0xFF || mfg == 0x00) {
    DBG->println("ERROR: JEDEC looks invalid (check wiring / CS / power).");
    return false;
  }

  DBG->println("Erasing 4KB sector at 0x000000...");
  sectorErase4K(TEST_ADDR);
  DBG->println("Erase done.");

  uint8_t tx[32];
  for (int i = 0; i < (int)sizeof(tx); i++) tx[i] = (uint8_t)(i + 1);

  DBG->println("Programming first 32 bytes...");
  pageProgram(TEST_ADDR, tx, sizeof(tx));
  DBG->println("Program done.");

  uint8_t rx[32] = {0};
  flashRead(TEST_ADDR, rx, sizeof(rx));

  DBG->println("Read-back:");
  bool ok = true;
  for (int i = 0; i < (int)sizeof(rx); i++) {
    if (rx[i] < 16) DBG->print('0');
    DBG->print(rx[i], HEX);
    DBG->print(i == (int)sizeof(rx)-1 ? "\n" : " ");
    if (rx[i] != tx[i]) ok = false;
  }

  DBG->println(ok ? "FLASH VERIFY: OK" : "FLASH VERIFY: FAIL");
  return ok;
}

bool testSD() {
  DBG->println("\n=== microSD TEST ===");
  deselectAll();

  if (!SD.begin(SD_CS)) {
    DBG->println("SD init failed! (check wiring / CS / format FAT32)");
    return false;
  }
  DBG->println("SD init OK.");

  File file = SD.open("test.txt", FILE_WRITE);
  if (!file) {
    DBG->println("File open failed.");
    return false;
  }

  file.println("Hello from XIAO nRF52840! SD write works.");
  file.close();
  DBG->println("Write done: test.txt");

  // Optional read-back
  File r = SD.open("test.txt", FILE_READ);
  if (r) {
    DBG->println("Contents of test.txt:");
    while (r.available()) DBG->write(r.read());
    r.close();
    DBG->println("\nRead done.");
  }

  return true;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  ledOff();

  pinMode(FLASH_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  deselectAll();

  initDebugPort();

  DBG->println("\nMerged SPI Test: W25Qxx + microSD");

  SPI.begin();

  // Conservative SPI settings that work for many flash + SD modules.
  // You can increase later if stable.
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  bool flashOK = testFlash();

  // SD libraries sometimes reconfigure SPI internally; make sure nobody else is selected.
  deselectAll();
  bool sdOK = testSD();

  DBG->println("\n=== SUMMARY ===");
  DBG->print("Flash: "); DBG->println(flashOK ? "OK" : "FAIL");
  DBG->print("SD   : "); DBG->println(sdOK ? "OK" : "FAIL");

  DBG->println("\nNOTE: Your 4GB card should work fine as long as it's FAT32.");
}

void loop() {
  // Heartbeat blink (independent of mode indicator)
  static uint32_t last = 0;
  if (millis() - last >= 500) {
    last = millis();
    static bool state = false;
    state = !state;
    if (state) ledOn(); else ledOff();
  }
}