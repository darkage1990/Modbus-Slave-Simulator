/*
  ESP32 Modbus Sensor Simulator
  - TFT_eSPI (ILI9341 240x320)
  - Rotary encoder + 2 buttons (Select, Back)
  - RS-485 (UART1) Modbus RTU Slave
  - Parameters mirrored in Holding Registers with correct resolution step
  - Serial config menu (baud, parity, data bits, stop bits)

  Libraries (Arduino Library Manager or matching your provided zip):
    - TFT_eSPI by Bodmer
    - Encoder by Paul Stoffregen
    - Button2 by Lennart Hennigs
    - ModbusRTU by Alexander Emelianov (aka emelianov)  (aka "modbus-esp8266")

  Make sure your TFT_eSPI User_Setup matches your ILI9341 wiring.
*/

#include <TFT_eSPI.h>
#include <Encoder.h>
#include <Button2.h>
#include <ModbusRTU.h>

// ---------------- Pin map (adjust if needed) ----------------
static const int PIN_RS485_RX = 16;  // UART1 RX
static const int PIN_RS485_TX = 17;  // UART1 TX
static const int PIN_RS485_DERE = 4; // MAX485 DE/RE tied together

static const int PIN_ENC_CLK = 33;
static const int PIN_ENC_DT = 32;
static const int PIN_BTN_SEL = 25;
static const int PIN_BTN_BACK = 26;

// ---------------- Display ----------------
TFT_eSPI tft = TFT_eSPI(); // size set by TFT_eSPI config

// ---------------- Inputs ----------------
Encoder enc(PIN_ENC_DT, PIN_ENC_CLK);
Button2 btnSelect(PIN_BTN_SEL);
Button2 btnBack(PIN_BTN_BACK);

// ---------------- Modbus RTU ----------------
HardwareSerial RS485(1);
ModbusRTU mb;

// ---------------- App state ----------------
enum class Screen : uint8_t
{
  HOME = 0,
  PARAM_LIST,
  PARAM_EDIT,
  SERIAL_MENU,
  SERIAL_EDIT
};

Screen screen = Screen::HOME;
int listIndex = 0; // generic list cursor
int editIndex = 0; // which item is being edited
long encPrev = 0;

// ---------------- Parameters & registers ----------------
// Holding register mapping:
// 1: pH       (0.01 step)
// 2: TDS ppm  (1 step)
// 3: TSS NTU  (1 step)  -- "Turbidity"
// 4: COD mg/L (1 step)
// 5: BOD mg/L (1 step)
// 6: DO mg/L  (0.01 step)
// 7: NH3-N mg/L (0.01 step)

struct Param
{
  const char *name;
  const char *unit;
  float minVal;
  float maxVal;
  float step;   // UI increment, also scaling for Hreg
  uint16_t reg; // Hreg address (1..)
  float value;  // actual (float)
};

Param params[] = {
    {"pH", "pH", 0.00f, 14.00f, 0.01f, 1, 7.00f},
    {"TDS", "ppm", 0.0f, 1008.0f, 1.0f, 2, 500.0f},
    {"TSS", "NTU", 0.0f, 1000.0f, 1.0f, 3, 100.0f}, // Turbidity
    {"COD", "mg/L", 0.0f, 1300.0f, 1.0f, 4, 200.0f},
    {"BOD", "mg/L", 0.0f, 350.0f, 1.0f, 5, 50.0f},
    {"DO", "mg/L", 0.00f, 20.00f, 0.01f, 6, 8.00f},
    {"NH3-N", "mg/L", 0.00f, 1000.0f, 0.01f, 7, 5.00f}};
static const int PARAM_COUNT = sizeof(params) / sizeof(params[0]);

// ---------------- Serial configuration model ----------------
struct SerialCfg
{
  uint32_t baud;
  uint8_t dataBits; // 7 or 8
  char parity;      // 'N','E','O'
  uint8_t stopBits; // 1 or 2
};

SerialCfg scfg = {9600, 8, 'N', 1};
const uint32_t BAUDS[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
const int BAUD_COUNT = sizeof(BAUDS) / sizeof(BAUDS[0]);

// Which field is being edited in serial menu
enum class SerialField : uint8_t
{
  BAUD = 0,
  PARITY,
  DATABITS,
  STOPBITS
};
SerialField serialField = SerialField::BAUD;

// ---------------- Utils ----------------
template <typename T>
T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

String parityToString(char p)
{
  switch (p)
  {
  case 'E':
    return "Even";
  case 'O':
    return "Odd";
  default:
    return "None";
  }
}

uint32_t parityToMode(char p, uint8_t databits, uint8_t stopbits)
{
  // Compose Arduino's SERIAL_* mode (e.g., SERIAL_8N1, SERIAL_8E1, etc.)
  // For ESP32 core, supported macros: SERIAL_8N1, SERIAL_8E1, SERIAL_8O1, SERIAL_7N1, etc.
  if (databits == 7)
  {
    if (p == 'E')
      return (stopbits == 2) ? SERIAL_7E2 : SERIAL_7E1;
    if (p == 'O')
      return (stopbits == 2) ? SERIAL_7O2 : SERIAL_7O1;
    return (stopbits == 2) ? SERIAL_7N2 : SERIAL_7N1;
  }
  else
  { // 8
    if (p == 'E')
      return (stopbits == 2) ? SERIAL_8E2 : SERIAL_8E1;
    if (p == 'O')
      return (stopbits == 2) ? SERIAL_8O2 : SERIAL_8O1;
    return (stopbits == 2) ? SERIAL_8N2 : SERIAL_8N1;
  }
}

void rs485Reinit()
{
  RS485.end();
  delay(20);
  RS485.begin(scfg.baud, parityToMode(scfg.parity, scfg.dataBits, scfg.stopBits),
              PIN_RS485_RX, PIN_RS485_TX);
  // With ModbusRTU (emelianov), begin can take driver (DE/RE) pin:
  mb.begin(&RS485, PIN_RS485_DERE); // auto driver control
  mb.slave(1);                      // Slave ID
}

// Scale float to 16-bit register using the defined step
uint16_t toReg(const Param &p)
{
  // round to nearest step then cast
  float scaled = p.value / p.step;
  if (scaled < 0)
    scaled = 0; // guard
  return (uint16_t)lroundf(scaled);
}
float fromReg(const Param &p, uint16_t regval)
{
  return (float)regval * p.step;
}

// ---------------- Drawing ----------------
void drawHeader(const char *title)
{
  tft.fillRect(0, 0, tft.width(), 26, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(title, 8, 5, 2);
}

void drawHome()
{
  tft.fillScreen(TFT_BLACK);
  drawHeader("WQMS Modbus Sensor Simulator");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  int y = 32;
  for (int i = 0; i < PARAM_COUNT; i++)
  {
    char line[64];
    // choose decimals based on step
    int dp = (params[i].step < 0.1f) ? 2 : 0;
    snprintf(line, sizeof(line), "%-6s : %.*f %s",
             params[i].name, dp, params[i].value, params[i].unit);
    tft.drawString(line, 10, y, 2);
    y += 22;
  }
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("[Select]=Menus   [Back]=Refresh", 10, tft.height() - 20, 2);
}

void drawParamList()
{
  tft.fillScreen(TFT_BLACK);
  drawHeader("Parameters");
  int y = 32;
  for (int i = 0; i < PARAM_COUNT; i++)
  {
    uint16_t bg = (i == listIndex) ? TFT_DARKGREY : TFT_BLACK;
    uint16_t fg = (i == listIndex) ? TFT_YELLOW : TFT_WHITE;
    tft.fillRect(0, y - 2, tft.width(), 20, bg);
    tft.setTextColor(fg, bg);
    int dp = (params[i].step < 0.1f) ? 2 : 0;
    char line[64];
    snprintf(line, sizeof(line), "%-6s : %.*f %s",
             params[i].name, dp, params[i].value, params[i].unit);
    tft.drawString(line, 10, y, 2);
    y += 22;
  }
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Rotate to choose, Select=Edit, Back=Home", 10, tft.height() - 20, 2);
}

void drawParamEdit()
{
  tft.fillScreen(TFT_BLACK);
  drawHeader("Edit Parameter");
  Param &p = params[editIndex];
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString(p.name, 10, 40, 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  int dp = (p.step < 0.1f) ? 2 : 0;

  char val[64];
  snprintf(val, sizeof(val), "%.*f %s", dp, p.value, p.unit);
  tft.drawString(val, 10, 90, 4);

  char rng[64];
  snprintf(rng, sizeof(rng), "Min %.*f  Max %.*f  Step %.*f",
           dp, p.minVal, dp, p.maxVal, dp, p.step);
  tft.drawString(rng, 10, 140, 2);

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Rotate=Adjust  Sel=Save  Back=Cancel", 10, tft.height() - 20, 2);
}

void drawSerialMenu()
{
  tft.fillScreen(TFT_BLACK);
  drawHeader("Serial Settings (RS-485)");
  const char *labels[] = {"Baud", "Parity", "Data bits", "Stop bits"};
  for (int i = 0; i < 4; i++)
  {
    bool sel = ((int)serialField == i);
    uint16_t bg = sel ? TFT_DARKGREY : TFT_BLACK;
    uint16_t fg = sel ? TFT_YELLOW : TFT_WHITE;
    tft.fillRect(0, 32 + i * 24 - 2, tft.width(), 22, bg);
    tft.setTextColor(fg, bg);
    String value;
    if (i == 0)
      value = String(scfg.baud);
    else if (i == 1)
      value = parityToString(scfg.parity);
    else if (i == 2)
      value = String(scfg.dataBits);
    else
      value = String(scfg.stopBits);
    String line = String(labels[i]) + " : " + value;
    tft.drawString(line, 10, 32 + i * 24, 2);
  }
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Rotate=Move  Select=Edit  Back=Home", 10, tft.height() - 20, 2);
}

void drawSerialEdit()
{
  tft.fillScreen(TFT_BLACK);
  drawHeader("Edit Serial Field");
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  String fname = (serialField == SerialField::BAUD) ? "Baud" : (serialField == SerialField::PARITY) ? "Parity"
                                                           : (serialField == SerialField::DATABITS) ? "Data bits"
                                                                                                    : "Stop bits";
  tft.drawString(fname, 10, 40, 4);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  String v = (serialField == SerialField::BAUD) ? String(scfg.baud) : (serialField == SerialField::PARITY) ? parityToString(scfg.parity)
                                                                  : (serialField == SerialField::DATABITS) ? String(scfg.dataBits)
                                                                                                           : String(scfg.stopBits);
  tft.drawString(v, 10, 90, 4);

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Rotate=Change  Sel=Apply  Back=Cancel", 10, tft.height() - 20, 2);
}

// ---------------- Input handlers ----------------
void onSelect(Button2 &)
{
  switch (screen)
  {
  case Screen::HOME:
    screen = Screen::PARAM_LIST;
    listIndex = 0;
    encPrev = enc.read();
    drawParamList();
    break;
  case Screen::PARAM_LIST:
    editIndex = listIndex;
    screen = Screen::PARAM_EDIT;
    encPrev = enc.read();
    drawParamEdit();
    break;
  case Screen::PARAM_EDIT:
  {
    // Save: write to holding register
    mb.Hreg(params[editIndex].reg, toReg(params[editIndex]));
    screen = Screen::PARAM_LIST;
    drawParamList();
    break;
  }
  case Screen::SERIAL_MENU:
    screen = Screen::SERIAL_EDIT;
    encPrev = enc.read();
    drawSerialEdit();
    break;
  case Screen::SERIAL_EDIT:
    // Apply serial change & reinit UART/Modbus
    rs485Reinit();
    screen = Screen::SERIAL_MENU;
    drawSerialMenu();
    break;
  }
}

void onBack(Button2 &)
{
  switch (screen)
  {
  case Screen::HOME:
    drawHome(); // refresh
    break;
  case Screen::PARAM_LIST:
    screen = Screen::HOME;
    drawHome();
    break;
  case Screen::PARAM_EDIT:
    // cancel edit (no write)
    screen = Screen::PARAM_LIST;
    drawParamList();
    break;
  case Screen::SERIAL_MENU:
    screen = Screen::HOME;
    drawHome();
    break;
  case Screen::SERIAL_EDIT:
    screen = Screen::SERIAL_MENU;
    drawSerialMenu();
    break;
  }
}

// Quick helper: long-press Select from HOME opens serial menu
void onSelectLong(Button2 &)
{
  if (screen == Screen::HOME)
  {
    screen = Screen::SERIAL_MENU;
    serialField = SerialField::BAUD;
    encPrev = enc.read();
    drawSerialMenu();
  }
}

// ---------------- Setup & Loop ----------------
void setup()
{
  // Serial debug
  Serial.begin(115200);
  delay(100);

  // Buttons
  pinMode(PIN_BTN_SEL, INPUT_PULLUP);
  pinMode(PIN_BTN_BACK, INPUT_PULLUP);
  btnSelect.setLongClickTime(600);
  btnSelect.setPressedHandler(onSelect);
  btnSelect.setLongClickDetectedHandler(onSelectLong);
  btnBack.setPressedHandler(onBack);

  // TFT
  tft.init();
  tft.setRotation(1); // landscape
  tft.fillScreen(TFT_BLACK);
  drawHome();

  // RS-485 UART & Modbus
  pinMode(PIN_RS485_DERE, OUTPUT);
  digitalWrite(PIN_RS485_DERE, LOW); // receive by default
  rs485Reinit();                     // starts RS485 and mb

  // Create holding registers and preload values
  for (int i = 0; i < PARAM_COUNT; i++)
  {
    mb.addHreg(params[i].reg, toReg(params[i]));
  }

  encPrev = enc.read();
}

void loop()
{
  // Modbus task (must be called often)
  mb.task();

  // Let buttons process
  btnSelect.loop();
  btnBack.loop();

  // If a Modbus master wrote new values, reflect into UI
  for (int i = 0; i < PARAM_COUNT; i++)
  {
    uint16_t rv = mb.Hreg(params[i].reg);
    float newVal = fromReg(params[i], rv);
    if (fabsf(newVal - params[i].value) > (params[i].step * 0.5f))
    {
      params[i].value = clamp(newVal, params[i].minVal, params[i].maxVal);
      if (screen == Screen::HOME)
        drawHome();
      else if (screen == Screen::PARAM_LIST)
        drawParamList();
      else if (screen == Screen::PARAM_EDIT && editIndex == i)
        drawParamEdit();
    }
  }

  // Handle rotary encoder
  long now = enc.read();
  if (now != encPrev)
  {
    long diff = (now - encPrev);
    encPrev = now;

    switch (screen)
    {
    case Screen::HOME:
      // no cursor; use long-press Select to open Serial Menu
      break;

    case Screen::PARAM_LIST:
    {
      int ni = listIndex + (diff > 0 ? 1 : -1);
      ni = clamp(ni, 0, PARAM_COUNT - 1);
      if (ni != listIndex)
      {
        listIndex = ni;
        drawParamList();
      }
      break;
    }

    case Screen::PARAM_EDIT:
    {
      Param &p = params[editIndex];
      float delta = (diff > 0 ? p.step : -p.step);
      float nv = clamp(p.value + delta, p.minVal, p.maxVal);
      if (fabsf(nv - p.value) >= (p.step * 0.5f))
      {
        p.value = nv;
        drawParamEdit();
      }
      break;
    }

    case Screen::SERIAL_MENU:
    {
      int fi = (int)serialField + (diff > 0 ? 1 : -1);
      fi = clamp(fi, 0, 3);
      if (fi != (int)serialField)
      {
        serialField = (SerialField)fi;
        drawSerialMenu();
      }
      break;
    }

    case Screen::SERIAL_EDIT:
    {
      if (serialField == SerialField::BAUD)
      {
        // Find current index in BAUDS, then step it
        int idx = 0;
        for (int i = 0; i < BAUD_COUNT; i++)
          if (BAUDS[i] == scfg.baud)
          {
            idx = i;
            break;
          }
        idx = clamp(idx + (diff > 0 ? 1 : -1), 0, BAUD_COUNT - 1);
        scfg.baud = BAUDS[idx];
      }
      else if (serialField == SerialField::PARITY)
      {
        // N -> E -> O -> N ...
        char order[3] = {'N', 'E', 'O'};
        int ci = 0;
        while (order[ci] != scfg.parity && ci < 3)
          ci++;
        ci = (ci + (diff > 0 ? 1 : -1) + 3) % 3;
        scfg.parity = order[ci];
      }
      else if (serialField == SerialField::DATABITS)
      {
        scfg.dataBits = (diff > 0) ? 8 : 7;
      }
      else
      { // STOPBITS
        scfg.stopBits = (diff > 0) ? 2 : 1;
      }
      drawSerialEdit();
      break;
    }
    }
  }

  // Periodically keep Hregs synced with our internal values (when user edits)
  static uint32_t tSync = 0;
  if (millis() - tSync > 300)
  {
    tSync = millis();
    for (int i = 0; i < PARAM_COUNT; i++)
    {
      uint16_t cur = mb.Hreg(params[i].reg);
      uint16_t need = toReg(params[i]);
      if (cur != need)
        mb.Hreg(params[i].reg, need);
    }
  }
}
