/***********************************************************************************************************************
 * An RC5 to XpressNet converter for example to connect the Iris remote control of
 * Uhlenbrock (and any other RC5 remote control).
 **********************************************************************************************************************/

/***********************************************************************************************************************
   I N C L U D E S
 **********************************************************************************************************************/
#include "Arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RC5.h>
#include <StateMachine.h>
#include <Wire.h>
#include <XpressNet.h>
#include <eeprom.h>

/***********************************************************************************************************************
   D E F I N E S
 **********************************************************************************************************************/
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define XPRESSNET_ADDRESS 30 // XpNet address of device
#define XPRESSNET_PIN 22     // Read/write pin 485 chip
#define IR_PIN 2             // IR sensor pin

struct LocInfo
{
    uint16_t Address;
    uint8_t Speed;
    uint8_t Steps;
    uint8_t Direction;
    uint16_t Functions;
};

/***********************************************************************************************************************
   D A T A   D E C L A R A T I O N S (exported, local)
 **********************************************************************************************************************/

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static RC5 rc5(IR_PIN);
static XpressNetClass XPNet;
static StateMachine Stm = StateMachine();
static LocInfo locInfo;
static LocInfo locInfoPrevious;
static uint8_t Rc5Toggle               = 0;
static uint8_t Rc5TogglePrevious       = 0xFF;
static uint8_t Rc5Address              = 0;
static uint8_t Rc5Command              = 0;
static bool Rc5NewData                 = false;
static bool PowerOnStart               = true;
static uint32_t previousMillis         = 0;
static byte XpressNetPowerStat         = 0xFF;
static byte XpressNetPowerStatPrevious = 0xFF;
static uint16_t LocAddressSelect       = 0;
static uint8_t LocActualSpeed          = 0;
static uint8_t LocActualDirection      = 0;
static uint16_t LocActualFunctions     = 0;
static uint8_t LocLastSelected         = 0;
static uint16_t TurnOutAddress         = 0;
static uint32_t TurnOutAddressTimeout  = 0;
static uint32_t TurnOutSymbolTimeOut   = 0;
static uint8_t FunctionOffset          = 0;
static uint32_t FunctionOffsetTime     = 0;
static bool LocInfoChanged             = false;
static bool locInfoRefresh             = false;

int EepromLocAddressA            = 0;
int EepromLocAddressB            = 0x02;
int EepromLocAddressC            = 0x04;
int EepromLocAddressD            = 0x06;
int EepromLocAddressLastSelected = 0x08;
int EepromTurnoutAddressA        = 0x10;
int EepromTurnoutAddressB        = 0x12;
int EepromTurnoutAddressC        = 0x14;
int EepromTurnoutAddressD        = 0x16;

/**
 * Forward direction loc symbol, see for conversion of an image https://diyusthad.com/image2cpp .
 */
const unsigned char locBitmapFw[] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00,
    0x70, 0x1c, 0x1c, 0x00, 0x30, 0x18, 0x1e, 0x00, 0x30, 0x18, 0x1e, 0x00, 0x38, 0x18, 0x3e, 0x00, 0x7f, 0xff, 0xff,
    0x00, 0x7f, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 0x80, 0x70, 0x7f, 0xff, 0x00, 0x60, 0x3f,
    0xff, 0x00, 0x62, 0x3f, 0xff, 0x00, 0x62, 0x38, 0xf1, 0x80, 0x60, 0x30, 0x61, 0x80, 0x30, 0x78, 0x61, 0x80, 0x38,
    0xd8, 0xf3, 0x00, 0x1f, 0xcf, 0x9f, 0x00, 0x02, 0x02, 0x04, 0x00 };

/**
 * Backward direction loc symbol.
 */
const unsigned char locBitmapBw[] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x80, 0x00, 0x0f, 0xff, 0x80,
    0x0e, 0x0e, 0x03, 0x80, 0x1e, 0x06, 0x03, 0x00, 0x1e, 0x06, 0x03, 0x00, 0x1f, 0x06, 0x07, 0x00, 0x3f, 0xff, 0xff,
    0x80, 0x3f, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 0x80, 0x3f, 0xff, 0x83, 0x80, 0x3f, 0xff,
    0x01, 0x80, 0x3f, 0xff, 0x11, 0x80, 0x63, 0xc7, 0x11, 0x80, 0x61, 0x83, 0x01, 0x80, 0x61, 0x87, 0x83, 0x00, 0x33,
    0xc6, 0xc7, 0x00, 0x3e, 0x7c, 0xfe, 0x00, 0x08, 0x10, 0x10, 0x00 };

/**
 * Light bulb symbol.
 */
const unsigned char lightBulb[] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x0c, 0xcc, 0x00, 0x04, 0x08, 0x00,
    0x40, 0xc0, 0x80, 0x63, 0xf1, 0x80, 0x04, 0x08, 0x00, 0x0c, 0x0c, 0x00, 0xc8, 0x04, 0xc0, 0xe8, 0x04, 0xc0, 0x08,
    0x04, 0x00, 0x0c, 0x0c, 0x00, 0x24, 0x09, 0x00, 0x66, 0x19, 0x80, 0x02, 0x10, 0x00, 0x03, 0x30, 0x00, 0x00, 0x00,
    0x00, 0x01, 0xe0, 0x00, 0x01, 0xe0, 0x00, 0x01, 0x20, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00 };

/**
 * Background for function.
 */
const unsigned char function[] PROGMEM = { 0x1e, 0x00, 0x3f, 0x00, 0x7f, 0x80, 0xff, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xff,
    0xc0, 0x7f, 0x80, 0x3f, 0x00, 0x1e, 0x00 };

/**
 * Turnout symbol.
 */
const unsigned char turnout[] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x07, 0xf0, 0x00,
    0x00, 0x0f, 0xf0, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x7f, 0xf0, 0x00, 0x01, 0xff, 0xe0,
    0x00, 0x03, 0xff, 0x80, 0x00, 0x03, 0xff, 0x80, 0x00, 0x07, 0xff, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x1f, 0xfc,
    0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0xff, 0xf0, 0x00, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff,
    0xff, 0xf0, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xf0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00 };

/**
 * Diverging turnout symbol.
 */
const unsigned char turnoutDiverging[] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x07,
    0xf0, 0x00, 0x00, 0x0c, 0x10, 0x00, 0x00, 0x38, 0x10, 0x00, 0x00, 0x38, 0x10, 0x00, 0x00, 0x70, 0x70, 0x00, 0x01,
    0xc0, 0xe0, 0x00, 0x03, 0x83, 0x80, 0x00, 0x03, 0x83, 0x80, 0x00, 0x06, 0x07, 0x00, 0x00, 0x1c, 0x0c, 0x00, 0x00,
    0x1c, 0x0c, 0x00, 0x00, 0x38, 0x38, 0x00, 0x00, 0xe0, 0x70, 0x00, 0xff, 0xc1, 0xff, 0xf0, 0xff, 0xc1, 0xff, 0xf0,
    0x80, 0x03, 0xff, 0xf0, 0x80, 0x07, 0xff, 0xf0, 0x80, 0x07, 0xff, 0xf0, 0x80, 0x1f, 0xff, 0xf0, 0xff, 0xff, 0xff,
    0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/**
 * Forward turnout symbol.
 */
const unsigned char turnoutForward[] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x07, 0xf0,
    0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x7f, 0xf0, 0x00, 0x01, 0xff,
    0xe0, 0x00, 0x03, 0xff, 0x80, 0x00, 0x03, 0xff, 0x80, 0x00, 0x07, 0xff, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x1f,
    0xfc, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0xff, 0xf0, 0x00, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xf0, 0x80,
    0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x10, 0x80, 0x00, 0x00, 0x10, 0xff, 0xff, 0xff, 0xf0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x000 };

/**
 * States of the system.
 */
State* StmStateInit         = Stm.addState(&StateInit);
State* StmStateEmergency    = Stm.addState(&StateEmergency);
State* StmStateShortCircuit = Stm.addState(&StateShortCircuit);
State* StmStateServiceMode  = Stm.addState(&StateServiceMode);
State* StmStatePowerOff     = Stm.addState(&StatePowerOff);
State* StmStatePowerOn      = Stm.addState(&StatePowerOn);
State* StmStateGetLocInfo   = Stm.addState(&StateGetLocInfo);
State* StmStateSelectLoc    = Stm.addState(&StateSelectLoc);
State* StmStateTurnOut      = Stm.addState(&StateTurnOut);

/* Conversion table for normal speed to 28 steps DCC speed. */
const uint8_t SpeedStep28TableToDcc[29]
    = { 16, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31 };

/* Conversion table for 28 steps DCC speed to normal speed. */
const uint8_t SpeedStep28TableFromDcc[32] = { 0, 0, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 0, 0, 2, 4, 6, 8,
    10, 12, 14, 16, 18, 20, 22, 24, 26, 28 };

/***********************************************************************************************************************
   L O C A L   F U N C T I O N S
 **********************************************************************************************************************/

/***********************************************************************************************************************
 */
void ShowInitSreen(void)
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 26);
    display.println(F(" RC5 XPNET"));
    display.print(F("   1.0.0"));
    display.display();
}

/***********************************************************************************************************************
 */
void UpdateStatusRow(const __FlashStringHelper* StatusRowPtr, bool FullClear)
{
    if (FullClear == true)
    {
        display.fillRect(0, 0, 127, 10, SSD1306_BLACK);
    }
    else
    {
        display.fillRect(0, 0, 50, 10, SSD1306_BLACK);
    }

    display.setTextSize(0);
    display.setCursor(3, 0);
    display.println(StatusRowPtr);
    display.display();
}

/***********************************************************************************************************************
 */
void ShowFPlus4Function(void)
{
    display.fillRect(69, 0, 20, 10, SSD1306_BLACK);
    display.setTextSize(0);
    display.setCursor(70, 0);
    display.println("F+4");
    display.display();
}

/***********************************************************************************************************************
 */
void ShowFPlus8Function(void)
{
    display.fillRect(69, 0, 20, 10, SSD1306_BLACK);
    display.setTextSize(0);
    display.setCursor(70, 0);
    display.println("F+8");
    display.display();
}

/***********************************************************************************************************************
 */
void ShowFPlusRemove(void)
{
    display.fillRect(69, 0, 20, 10, SSD1306_BLACK);
    display.display();
}

/***********************************************************************************************************************
 * redraw screen with loc information.
 */
void ShowLocInfo()
{
    uint8_t Index;

    // Show address, first clear loc info screen.
    display.fillRect(0, 10, 127, 54, SSD1306_BLACK);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(3, 11);
    display.print(locInfo.Address);

    // Show speed
    display.setCursor(3, 31);
    if (locInfo.Steps == 2)
    {
        display.print(SpeedStep28TableFromDcc[locInfo.Speed]);
    }
    else
    {
        display.print(locInfo.Speed);
    }

    // Show direction
    display.fillRect(69, 10, 29, 23, SSD1306_BLACK);
    if (locInfo.Direction == 1)
    {
        display.drawBitmap(70, 10, locBitmapFw, 26, 20, SSD1306_WHITE);
    }
    else
    {
        display.drawBitmap(70, 10, locBitmapBw, 26, 20, SSD1306_WHITE);
    }

    // Show which loc is selected
    display.fillRect(100, 0, 28, 8, SSD1306_BLACK);
    display.setTextSize(0);
    display.setCursor(100, 0);

    switch (LocLastSelected)
    {
    case 0: display.print("A"); break;
    case 1: display.print("B"); break;
    case 2: display.print("C"); break;
    case 3: display.print("D"); break;
    }

    // Show decoder speed steps
    display.setCursor(110, 0);
    switch (locInfo.Steps)
    {
    case 0: display.print("14"); break;
    case 2: display.print("28"); break;
    case 4: display.print("128"); break;
    default: break;
    }

    // Show functions, first light.
    if ((locInfo.Functions & 0x01) == 0x01)
    {
        display.drawBitmap(70, 32, lightBulb, 18, 22, SSD1306_WHITE);
    }

    // Show function F1..F10 (F10 as 0)
    display.setTextColor(SSD1306_BLACK);
    for (Index = 0; Index < 10; Index++)
    {
        // Index + 1 because bit 0 is light.
        if (locInfo.Functions & (1 << (Index + 1)))
        {
            display.drawBitmap(4 + (Index * 12), 54, function, 10, 10, SSD1306_WHITE);
            display.setCursor(6 + (Index * 12), 56);
            if (Index < 9)
            {
                display.print(Index + 1);
            }
            else
            {
                display.print("0");
            }
        }
    }

    display.setTextColor(SSD1306_WHITE);
    display.display();
    LocInfoChanged = false;
}

/***********************************************************************************************************************
 */
void SendTurnOutCommand(uint8_t EepromAddress, uint8_t Direction)
{
    uint8_t TurnOutData = 0x08;
    TurnOutAddress      = ((uint16_t)(EEPROM.read(EepromAddress)) << 8);
    TurnOutAddress |= (uint16_t)(EEPROM.read(EepromAddress + 1));

    if (TurnOutAddress <= 9999)
    {
        TurnOutData |= Direction;
        XPNet.setTrntPos((TurnOutAddress - 1) >> 8, (uint8_t)(TurnOutAddress - 1), TurnOutData);

        // Show in status row address and text
        display.fillRect(25, 0, 38, 8, SSD1306_BLACK);
        display.setTextSize(0);
        display.setCursor(25, 0);
        display.print(TurnOutAddress);
        if (Direction == 0)
        {
            display.print("/");
        }
        else
        {
            display.print("-");
        }
        TurnOutAddressTimeout = millis();
        display.display();
    }
}

/***********************************************************************************************************************
 */
void StateInit()
{
    if (Stm.executeOnce)
    {
        // Read loc address and set to default when required...
        LocLastSelected = (uint8_t)EEPROM.read(EepromLocAddressLastSelected);

        switch (LocLastSelected)
        {
        case 0:
            locInfo.Address = ((uint16_t)(EEPROM.read(EepromLocAddressA)) << 8);
            locInfo.Address |= (uint16_t)EEPROM.read(EepromLocAddressA + 1);
            break;
        case 1:
            locInfo.Address = ((uint16_t)(EEPROM.read(EepromLocAddressB)) << 8);
            locInfo.Address |= (uint16_t)EEPROM.read(EepromLocAddressB + 1);
            break;
        case 2:
            locInfo.Address = ((uint16_t)(EEPROM.read(EepromLocAddressC)) << 8);
            locInfo.Address |= (uint16_t)EEPROM.read(EepromLocAddressC + 1);
            break;
        case 3:
            locInfo.Address = ((uint16_t)(EEPROM.read(EepromLocAddressD)) << 8);
            locInfo.Address |= (uint16_t)EEPROM.read(EepromLocAddressD + 1);
            break;
        default:
            // Probably no selection done yet, set 3 and A as default.
            locInfo.Address = 3;
            LocLastSelected = 0;
            EEPROM.write(EepromLocAddressA, locInfo.Address >> 8);
            EEPROM.write(EepromLocAddressA + 1, locInfo.Address & 0xFF);
            EEPROM.write(EepromLocAddressLastSelected, LocLastSelected);
            break;
        }

        if (locInfo.Address > 9999)
        {
            locInfo.Address = 3;
        }
        XPNet.getLocoInfo(locInfo.Address >> 8, locInfo.Address & 0xFF);
    }
    else
    {
    }
}

/***********************************************************************************************************************
 */
void StateEmergency()
{
    if (Stm.executeOnce)
    {
        XPNet.setPower(csEmergencyStop);
        UpdateStatusRow(F("EMERGENCY"), true);
        PowerOnStart = false;
    }
    else
    {
    }
}

/***********************************************************************************************************************
 */
void StateShortCircuit()
{
    if (Stm.executeOnce)
    {
        UpdateStatusRow(F("SHORT"), true);
        PowerOnStart = false;
    }
    else
    {
    }
}

/***********************************************************************************************************************
 */
void StateServiceMode()
{
    if (Stm.executeOnce)
    {
        ShowInitSreen();
        UpdateStatusRow(F("SERVICE"), true);
        PowerOnStart = false;
    }
    else
    {
    }
}

/***********************************************************************************************************************
 */
void StatePowerOff()
{
    if (Stm.executeOnce)
    {
        UpdateStatusRow(F("OFF"), true);

        if (PowerOnStart == false)
        {
            locInfo.Speed = 0;
            XPNet.setLocoDrive(locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps, 0);
            ShowLocInfo();
        }
        else
        {
            // After initial power on and power off state keep showing start screen.
            PowerOnStart = false;
        }
    }
}

/***********************************************************************************************************************
 */
void StatePowerOn()
{
    bool TransmitSpeedDirectionData = false;
    uint16_t LocAddress             = 0;
    if (Stm.executeOnce)
    {
        locInfoRefresh = false;
        UpdateStatusRow(F("ON"), true);
        ShowLocInfo();
        PowerOnStart   = false;
        FunctionOffset = 0;
        ShowFPlusRemove();
    }
    else
    {
        if (LocInfoChanged == true)
        {
            ShowLocInfo();
        }

        if (Rc5NewData == true)
        {
            switch (Rc5Command)
            {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9: Rc5NewData = false; break;
            case 16:
                // Right button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x0)
                {
                    LocActualDirection = 0x80;
                }
                break;
            case 17:
                // Left button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x80)
                {
                    LocActualDirection = 0x0;
                }
                break;
            case 19:
                // off (light) button
                LocActualFunctions &= ~(1 << 0);
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, LocActualFunctions & 0x01, 0);
                Rc5NewData = false;
                ShowFPlusRemove();
                FunctionOffset = 0;
                break;
            case 20:
                // F0 (Light) button on
                LocActualFunctions |= 1;
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, LocActualFunctions & 0x01, 0);
                Rc5NewData = false;
                ShowFPlusRemove();
                FunctionOffset = 0;
                break;
            case 21:
                // F1 button toggle function
                LocActualFunctions ^= (1 << (1 + FunctionOffset));
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF,
                    (LocActualFunctions >> (1 + FunctionOffset)) & 0x01, (1 + FunctionOffset));
                Rc5NewData         = false;
                FunctionOffsetTime = millis();
                break;
            case 22:
                // F2 button toggle function
                LocActualFunctions ^= (1 << (2 + FunctionOffset));
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF,
                    (LocActualFunctions >> (2 + FunctionOffset)) & 0x01, (2 + FunctionOffset));
                Rc5NewData         = false;
                FunctionOffsetTime = millis();
                break;
            case 23:
                // F3 button toggle function
                LocActualFunctions ^= (1 << (3 + FunctionOffset));
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF,
                    (LocActualFunctions >> (3 + FunctionOffset)) & 0x01, (3 + FunctionOffset));
                Rc5NewData         = false;
                FunctionOffsetTime = millis();
                break;
            case 24:
                // F4 button toggle function
                LocActualFunctions ^= (1 << (4 + FunctionOffset));
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF,
                    (LocActualFunctions >> (4 + FunctionOffset)) & 0x01, (4 + FunctionOffset));
                Rc5NewData         = false;
                FunctionOffsetTime = millis();
                break;
            case 29:
                // F+4 button
                Rc5NewData = false;
                if ((FunctionOffset == 0) || (FunctionOffset == 8))
                {
                    FunctionOffset     = 4;
                    FunctionOffsetTime = millis();
                    ShowFPlus4Function();
                }
                else
                {
                    ShowFPlusRemove();
                    FunctionOffset = 0;
                }
                break;
            case 30:
                // F+8 button
                Rc5NewData = false;
                if ((FunctionOffset == 0) || (FunctionOffset == 4))
                {
                    FunctionOffset     = 8;
                    FunctionOffsetTime = millis();
                    ShowFPlus8Function();
                }
                else
                {
                    ShowFPlusRemove();
                    FunctionOffset = 0;
                }
                break;
            case 32:
                // + button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x80)
                {
                    LocActualSpeed++;
                }
                else
                {
                    if (LocActualSpeed >= 1)
                    {
                        LocActualSpeed--;
                    }
                    else
                    {
                        // Speed 0, change direction.
                        LocActualDirection = 0x80;
                    }
                }
                break;
            case 33:
                // - button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x80)
                {
                    if (LocActualSpeed >= 1)
                    {
                        LocActualSpeed--;
                    }
                    else
                    {
                        // Speed 0, change direction.
                        LocActualDirection = 0x0;
                    }
                }
                else
                {
                    LocActualSpeed++;
                }
                break;
            case 40:
                // Turnout A red.
                SendTurnOutCommand(EepromTurnoutAddressA, 0);
                Rc5NewData = false;
                break;
            case 41:
                // Turnout A green.
                SendTurnOutCommand(EepromTurnoutAddressA, 1);
                Rc5NewData = false;
                break;
            case 42:
                // Turnout B red.
                SendTurnOutCommand(EepromTurnoutAddressB, 0);
                Rc5NewData = false;
                break;
            case 43:
                // Turnout B green.
                SendTurnOutCommand(EepromTurnoutAddressB, 1);
                Rc5NewData = false;
                break;
            case 44:
                // Turnout C red.
                SendTurnOutCommand(EepromTurnoutAddressC, 0);
                Rc5NewData = false;
                break;
            case 45:
                // Turnout C green.
                SendTurnOutCommand(EepromTurnoutAddressC, 1);
                Rc5NewData = false;
                break;
            case 46:
                // Turnout D red.
                SendTurnOutCommand(EepromTurnoutAddressD, 0);
                Rc5NewData = false;
                break;
            case 47:
                // Turnout D green.
                SendTurnOutCommand(EepromTurnoutAddressD, 1);
                Rc5NewData = false;
                break;
            case 51:
                // A button to select loc
                LocAddress = ((uint16_t)(EEPROM.read(EepromLocAddressA)) << 8);
                LocAddress |= (uint16_t)EEPROM.read(EepromLocAddressA + 1);

                if (LocLastSelected != 0)
                {
                    if (LocAddress < 9999)
                    {
                        locInfoRefresh  = true;
                        locInfo.Address = LocAddress;
                        LocLastSelected = 0;
                        EEPROM.write(EepromLocAddressLastSelected, LocLastSelected);
                        Stm.transitionTo(StmStateGetLocInfo);
                    }
                }
                break;
            case 52:
                // B Button to select loc
                LocAddress = ((uint16_t)(EEPROM.read(EepromLocAddressB)) << 8);
                LocAddress |= (uint16_t)EEPROM.read(EepromLocAddressB + 1);

                if (LocLastSelected != 1)
                {
                    if (LocAddress < 9999)
                    {
                        locInfoRefresh  = true;
                        locInfo.Address = LocAddress;
                        LocLastSelected = 1;
                        EEPROM.write(EepromLocAddressLastSelected, LocLastSelected);
                        Stm.transitionTo(StmStateGetLocInfo);
                    }
                }
                break;
            case 53:
                // C Button to select loc
                LocAddress = ((uint16_t)(EEPROM.read(EepromLocAddressC)) << 8);
                LocAddress |= (uint16_t)EEPROM.read(EepromLocAddressC + 1);

                if (LocLastSelected != 2)
                {
                    if (LocAddress < 9999)
                    {
                        locInfoRefresh  = true;
                        locInfo.Address = LocAddress;
                        LocLastSelected = 2;
                        EEPROM.write(EepromLocAddressLastSelected, LocLastSelected);
                        Stm.transitionTo(StmStateGetLocInfo);
                    }
                }
                break;
            case 54:
                // D Button to select loc
                LocAddress = ((uint16_t)(EEPROM.read(EepromLocAddressD)) << 8);
                LocAddress |= (uint16_t)EEPROM.read(EepromLocAddressD + 1);

                if (LocLastSelected != 3)
                {
                    if (LocAddress < 9999)
                    {
                        locInfoRefresh  = true;
                        locInfo.Address = LocAddress;
                        LocLastSelected = 3;
                        EEPROM.write(EepromLocAddressLastSelected, LocLastSelected);
                        Stm.transitionTo(StmStateGetLocInfo);
                    }
                }
                break;
            default: break;
            }

            if (TransmitSpeedDirectionData == true)
            {
                Rc5NewData = false;
                switch (locInfo.Steps)
                {
                case 1:
                    // 14 speed docoder.
                    if (LocActualSpeed > 14)
                    {
                        LocActualSpeed = 14;
                    }
                    XPNet.setLocoDrive(locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps,
                        SpeedStep28TableToDcc[LocActualSpeed] | LocActualDirection);

                    break;
                case 2:
                    // 28 speed decoder.
                    if (LocActualSpeed > 28)
                    {
                        LocActualSpeed = 28;
                    }
                    XPNet.setLocoDrive(locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps,
                        SpeedStep28TableToDcc[LocActualSpeed] | LocActualDirection);
                    break;
                case 4:
                    // 128 speed decoder.
                    if (LocActualSpeed > 127)
                    {
                        LocActualSpeed = 127;
                    }

                    XPNet.setLocoDrive(locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps,
                        LocActualSpeed | LocActualDirection);
                    break;
                }

                XPNet.getLocoInfo(locInfo.Address >> 8, locInfo.Address & 0xFF);

                ShowFPlusRemove();
                FunctionOffset = 0;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
                Serial.print("Speed : ");
                Serial.print(LocActualSpeed);
                Serial.print("Direction : ");
                Serial.print(LocActualDirection);
                Serial.print("Steps : ");
                Serial.print(locInfo.Steps);
                Serial.println("");
#endif
            }
        }
    }
}

/***********************************************************************************************************************
 */
void StateTurnOut()
{
    uint8_t TurnOutData      = 0;
    uint8_t TurnOutDirection = 0;
    bool SendTurnoutData     = false;

    if (Stm.executeOnce)
    {
        display.fillRect(0, 0, 127, 10, SSD1306_BLACK);
        UpdateStatusRow(F("TURNOUT"), true);
        PowerOnStart = false;

        display.fillRect(0, 10, 127, 54, SSD1306_BLACK);

        display.setTextSize(2);
        display.setCursor(3, 20);
        display.print(TurnOutAddress);
        display.display();
    }
    else
    {
        if (Rc5NewData == true)
        {
            switch (Rc5Command)
            {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                TurnOutAddress *= 10;
                TurnOutAddress += (uint16_t)(Rc5Command);

                if (TurnOutAddress > 9999)
                {
                    TurnOutAddress = 0;
                }

                // Show address.
                display.fillRect(2, 13, 50, 22, SSD1306_BLACK);
                display.setTextSize(2);
                display.setCursor(3, 20);
                display.print(TurnOutAddress);
                display.display();
                Rc5NewData = false;
                break;
            case 20:
                // Use F0 button to reset address.
                TurnOutAddress = 0;

                // Show address.
                display.fillRect(2, 13, 50, 22, SSD1306_BLACK);
                display.setTextSize(2);
                display.setCursor(3, 20);
                display.print(TurnOutAddress);
                display.display();
                Rc5NewData = false;
                break;
            case 21:
                // F1 to set turn out diversing without storing
                if (TurnOutAddress > 0)
                {
                    Rc5NewData       = false;
                    TurnOutDirection = 0;
                    SendTurnoutData  = true;
                }
                break;
            case 24:
                // F4 to set turn out forward without storing
                if (TurnOutAddress > 0)
                {
                    Rc5NewData       = false;
                    TurnOutDirection = 1;
                    SendTurnoutData  = true;
                }
                break;
            case 40:
                Rc5NewData       = false;
                TurnOutDirection = 0;
                SendTurnoutData  = true;
                EEPROM.write(EepromTurnoutAddressA, TurnOutAddress >> 8);
                EEPROM.write(EepromTurnoutAddressA + 1, TurnOutAddress & 0xFF);

                display.fillRect(110, 0, 10, 8, SSD1306_BLACK);
                display.setCursor(110, 0);
                display.setTextSize(1);
                display.print("A");
                break;
            case 41:
                Rc5NewData       = false;
                TurnOutDirection = 1;
                SendTurnoutData  = true;
                EEPROM.write(EepromTurnoutAddressA, TurnOutAddress >> 8);
                EEPROM.write(EepromTurnoutAddressA + 1, TurnOutAddress & 0xFF);

                display.fillRect(110, 0, 10, 8, SSD1306_BLACK);
                display.setCursor(110, 0);
                display.setTextSize(1);
                display.print("A");
                break;
            case 42:
                Rc5NewData       = false;
                TurnOutDirection = 0;
                SendTurnoutData  = true;
                EEPROM.write(EepromTurnoutAddressB, TurnOutAddress >> 8);
                EEPROM.write(EepromTurnoutAddressB + 1, TurnOutAddress & 0xFF);

                display.fillRect(110, 0, 10, 8, SSD1306_BLACK);
                display.setCursor(110, 0);
                display.setTextSize(1);
                display.print("B");
                break;
            case 43:
                Rc5NewData       = false;
                TurnOutDirection = 1;
                SendTurnoutData  = true;
                EEPROM.write(EepromTurnoutAddressB, TurnOutAddress >> 8);
                EEPROM.write(EepromTurnoutAddressB + 1, TurnOutAddress & 0xFF);

                display.fillRect(110, 0, 10, 8, SSD1306_BLACK);
                display.setCursor(110, 0);
                display.setTextSize(1);
                display.print("B");
                break;
            case 44:
                Rc5NewData       = false;
                TurnOutDirection = 0;
                SendTurnoutData  = true;
                EEPROM.write(EepromTurnoutAddressC, TurnOutAddress >> 8);
                EEPROM.write(EepromTurnoutAddressC + 1, TurnOutAddress & 0xFF);

                display.fillRect(110, 0, 10, 8, SSD1306_BLACK);
                display.setCursor(110, 0);
                display.setTextSize(1);
                display.print("C");
                break;
            case 45:
                Rc5NewData       = false;
                TurnOutDirection = 1;
                SendTurnoutData  = true;
                EEPROM.write(EepromTurnoutAddressC, TurnOutAddress >> 8);
                EEPROM.write(EepromTurnoutAddressC + 1, TurnOutAddress & 0xFF);

                display.fillRect(110, 0, 10, 8, SSD1306_BLACK);
                display.setCursor(110, 0);
                display.setTextSize(1);
                display.print("C");
                break;
            case 46:
                Rc5NewData       = false;
                TurnOutDirection = 0;
                SendTurnoutData  = true;
                EEPROM.write(EepromTurnoutAddressD, TurnOutAddress >> 8);
                EEPROM.write(EepromTurnoutAddressD + 1, TurnOutAddress & 0xFF);

                display.fillRect(110, 0, 10, 8, SSD1306_BLACK);
                display.setCursor(110, 0);
                display.setTextSize(1);
                display.print("D");
                break;
            case 47:
                Rc5NewData       = false;
                TurnOutDirection = 1;
                SendTurnoutData  = true;
                EEPROM.write(EepromTurnoutAddressD, TurnOutAddress >> 8);
                EEPROM.write(EepromTurnoutAddressD + 1, TurnOutAddress & 0xFF);

                display.fillRect(110, 0, 10, 8, SSD1306_BLACK);
                display.setCursor(110, 0);
                display.setTextSize(1);
                display.print("D");
                break;
            default: break;
            }
        }

        if (SendTurnoutData == true)
        {
            SendTurnoutData = false;

            TurnOutData = 0x08;
            if (TurnOutDirection == 1)
            {
                TurnOutData |= 1;
            }
            XPNet.setTrntPos((TurnOutAddress - 1) >> 8, (uint8_t)(TurnOutAddress - 1), TurnOutData);

            // Draw turnout symbol
            display.fillRect(70, 12, 28, 28, SSD1306_BLACK);
            if (TurnOutDirection == 1)
            {
                display.drawBitmap(70, 12, turnoutDiverging, 28, 28, SSD1306_WHITE);
            }
            else
            {
                display.drawBitmap(70, 12, turnoutForward, 28, 28, SSD1306_WHITE);
            }

            display.display();

            TurnOutSymbolTimeOut = millis();
        }
    }
}

/***********************************************************************************************************************
 */
void StateGetLocInfo()
{
    if (Stm.executeOnce)
    {
        UpdateStatusRow(F("GET LOC"), true);
        locInfoRefresh = true;
        PowerOnStart   = false;
    }
    else
    {
        if (LocInfoChanged == false)
        {
            XPNet.getLocoInfo(locInfo.Address >> 8, locInfo.Address & 0xFF);
        }
        else
        {
            if (locInfo.Steps == 2)
            {
                LocActualSpeed = SpeedStep28TableFromDcc[locInfo.Speed];
            }
            else
            {
                LocActualSpeed = locInfo.Speed;
            }

            locInfoRefresh = false;
            Stm.transitionTo(StmStatePowerOn);
        }
    }
}

/***********************************************************************************************************************
 */
void StateSelectLoc()
{
    if (Stm.executeOnce)
    {
        UpdateStatusRow(F("SELECT LOC"), true);

        LocAddressSelect = 0;
        display.fillRect(110, 0, 28, 8, SSD1306_BLACK);
        display.fillRect(0, 8, 128, 56, SSD1306_BLACK);
        display.setTextSize(2);
        display.setCursor(1, 14);
        display.print("Loc  :");
        display.setCursor(75, 14);
        display.print(LocAddressSelect);
        display.display();
    }
    else
    {
        if (Rc5NewData == true)
        {
            switch (Rc5Command)
            {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                // Increase selected address.
                LocAddressSelect *= 10;
                LocAddressSelect += (uint16_t)(Rc5Command);

                // Limit
                if (LocAddressSelect > 9999)
                {
                    LocAddressSelect = 0;
                }

                // Display
                display.fillRect(74, 10, 51, 64, SSD1306_BLACK);
                display.setCursor(75, 14);
                display.setTextSize(2);
                display.print(LocAddressSelect);
                display.display();

                Rc5NewData = false;
                break;
            case 13:
                // Loc button to exit.
                Rc5NewData = false;
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                }
                Stm.transitionTo(StmStateGetLocInfo);
                break;
            case 20:
                // Use F0 button to reset address.
                LocAddressSelect = 0;
                display.fillRect(74, 10, 51, 64, SSD1306_BLACK);
                display.setCursor(75, 14);
                display.setTextSize(2);
                display.print(LocAddressSelect);
                display.display();
                Rc5NewData = false;
                break;
            case 51:
                // A button
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                    LocLastSelected = 0;
                    EEPROM.write(EepromLocAddressA, locInfo.Address >> 8);
                    EEPROM.write(EepromLocAddressA + 1, locInfo.Address & 0xFF);
                    EEPROM.write(EepromLocAddressLastSelected, LocLastSelected);

                    display.fillRect(100, 0, 9, 8, SSD1306_BLACK);
                    display.setCursor(100, 0);
                    display.setTextSize(0);
                    display.print("A");
                    display.display();
                }
                Rc5NewData = false;
                break;
            case 52:
                // B button
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                    LocLastSelected = 1;
                    EEPROM.write(EepromLocAddressB, locInfo.Address >> 8);
                    EEPROM.write(EepromLocAddressB + 1, locInfo.Address & 0xFF);
                    EEPROM.write(EepromLocAddressLastSelected, LocLastSelected);

                    display.fillRect(100, 0, 9, 8, SSD1306_BLACK);
                    display.setCursor(100, 0);
                    display.setTextSize(0);
                    display.print("B");
                    display.display();
                }
                Rc5NewData = false;
                break;
            case 53:
                // C button
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                    LocLastSelected = 2;
                    EEPROM.write(EepromLocAddressC, locInfo.Address >> 8);
                    EEPROM.write(EepromLocAddressC + 1, locInfo.Address & 0xFF);
                    EEPROM.write(EepromLocAddressLastSelected, LocLastSelected);

                    display.fillRect(100, 0, 9, 8, SSD1306_BLACK);
                    display.setCursor(100, 0);
                    display.setTextSize(0);
                    display.print("C");
                    display.display();
                }
                Rc5NewData = false;
                break;
            case 54:
                // D button
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                    LocLastSelected = 3;
                    EEPROM.write(EepromLocAddressD, locInfo.Address >> 8);
                    EEPROM.write(EepromLocAddressD + 1, locInfo.Address & 0xFF);
                    EEPROM.write(EepromLocAddressLastSelected, LocLastSelected);

                    display.fillRect(100, 0, 9, 8, SSD1306_BLACK);
                    display.setCursor(100, 0);
                    display.setTextSize(0);
                    display.print("D");
                    display.display();
                }
                Rc5NewData = false;
                break;

            default: Rc5NewData = false; break;
            }
        }
    }
}

/***********************************************************************************************************************
 */
bool transitionEmergency()
{
    bool Result = false;
    if (XpressNetPowerStat == csEmergencyStop)
    {
        Result = true;
    }

    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionShortCircuit()
{
    bool Result = false;
    if (XpressNetPowerStat == csShortCircuit)
    {
        if (XpressNetPowerStatPrevious != XpressNetPowerStat)
        {
            XpressNetPowerStatPrevious = XpressNetPowerStat;
            Result                     = true;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionServiceMode()
{
    bool Result = false;
    if (XpressNetPowerStat == csServiceMode)
    {
        if (XpressNetPowerStatPrevious != XpressNetPowerStat)
        {
            XpressNetPowerStatPrevious = XpressNetPowerStat;
            Result                     = true;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionPowerOff()
{
    bool Result = false;
    if (XpressNetPowerStat == csTrackVoltageOff)
    {
        if (XpressNetPowerStatPrevious != XpressNetPowerStat)
        {
            XpressNetPowerStatPrevious = XpressNetPowerStat;
            Result                     = true;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionPowerOn()
{
    bool Result = false;
    if (XpressNetPowerStat == csNormal)
    {
        if (XpressNetPowerStatPrevious != XpressNetPowerStat)
        {
            XpressNetPowerStatPrevious = XpressNetPowerStat;
            Result                     = true;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionRc5StopButton()
{
    bool Result = false;
    if (Rc5NewData == true)
    {
        if (Rc5Command == 12)
        {
            // Based on central station state transmit power on or off.
            // Based on the status change applicable state will be entered.
            // When calling setPower() in states generated sometimes runarounds..
            switch (XpressNetPowerStat)
            {
            case csNormal: XPNet.setPower(csTrackVoltageOff); break;
            case csShortCircuit:
            case csEmergencyStop:
            case csTrackVoltageOff: XPNet.setPower(csNormal); break;
            case csServiceMode: XPNet.setPower(csTrackVoltageOff); break;
            }
            Rc5NewData = false;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionRc5SelectLocButton()
{
    bool Result = false;
    if (Rc5NewData == true)
    {
        if (Rc5Command == 13)
        {
            Result     = true;
            Rc5NewData = false;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionRc5TurnOutButton()
{
    bool Result = false;
    if (Rc5NewData == true)
    {
        if (Rc5Command == 14)
        {
            Result     = true;
            Rc5NewData = false;
        }
    }
    return (Result);
}

/***********************************************************************************************************************
 */
bool transitionFunctionOffSetReset()
{
    if ((millis() - FunctionOffsetTime) > 10000)
    {
        FunctionOffsetTime = millis();
        if (FunctionOffset != 0)
        {
            ShowFPlusRemove();
            FunctionOffset = 0;
        }
    }

    return (false);
}

/***********************************************************************************************************************
 */
bool transitionTurnOutDirectionShowDisableStatusRow()
{
    if ((millis() - TurnOutAddressTimeout) > 1500)
    {
        TurnOutAddressTimeout = millis();
        display.fillRect(25, 0, 30, 8, SSD1306_BLACK);
        display.display();
    }

    return (false);
}

/***********************************************************************************************************************
 */
bool transitionTurnOutDirectionShowDisable()
{
    if ((millis() - TurnOutSymbolTimeOut) > 1000)
    {
        TurnOutSymbolTimeOut = millis();
        display.fillRect(70, 12, 28, 28, SSD1306_BLACK);
        display.drawBitmap(70, 12, turnout, 28, 28, SSD1306_WHITE);
        display.display();
    }

    return (false);
}

/***********************************************************************************************************************
   E X P O R T E D   F U N C T I O N S
 **********************************************************************************************************************/

/**
 ***********************************************************************************************************************
   @brief   The setup function is called once at startup of the sketch
   @return  None
 **********************************************************************************************************************/
void setup()
{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    Serial.begin(57600);
    Serial.println("Starting");
#endif

    // Init display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        Serial.println(F("SSD1306 allocation failed"));
#endif
        for (;;)
            ; // Don't proceed, loop forever
    }

    ShowInitSreen();
    UpdateStatusRow(F("CONNECTING"), true);

    // Init Xpressnet.
    XPNet.start(XPRESSNET_ADDRESS, XPRESSNET_PIN);

    previousMillis = millis();

    locInfoPrevious.Address = 0xFFFF;

    // All transitions for the state machine.
    StmStateInit->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateInit->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateInit->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateInit->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateInit->addTransition(&transitionShortCircuit, StmStateShortCircuit);

    StmStatePowerOff->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStatePowerOff->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStatePowerOff->addTransition(&transitionEmergency, StmStateEmergency);
    StmStatePowerOff->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStatePowerOff->addTransition(&transitionRc5StopButton, StmStateGetLocInfo);
    StmStatePowerOff->addTransition(&transitionRc5SelectLocButton, StmStateSelectLoc);

    StmStatePowerOn->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStatePowerOn->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStatePowerOn->addTransition(&transitionEmergency, StmStateEmergency);
    StmStatePowerOn->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStatePowerOn->addTransition(&transitionRc5StopButton, StmStatePowerOff);
    StmStatePowerOn->addTransition(&transitionRc5SelectLocButton, StmStateSelectLoc);
    StmStatePowerOn->addTransition(&transitionRc5TurnOutButton, StmStateTurnOut);
    StmStatePowerOn->addTransition(&transitionFunctionOffSetReset, StmStatePowerOn);
    StmStatePowerOn->addTransition(&transitionTurnOutDirectionShowDisableStatusRow, StmStatePowerOn);

    StmStateEmergency->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateEmergency->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateEmergency->addTransition(&transitionServiceMode, StmStateEmergency);
    StmStateEmergency->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateEmergency->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateShortCircuit->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateShortCircuit->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateShortCircuit->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateShortCircuit->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateShortCircuit->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateServiceMode->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateServiceMode->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateServiceMode->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateServiceMode->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateServiceMode->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateGetLocInfo->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateGetLocInfo->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateGetLocInfo->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateGetLocInfo->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateGetLocInfo->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateSelectLoc->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateSelectLoc->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateSelectLoc->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateSelectLoc->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateSelectLoc->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateTurnOut->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateTurnOut->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateTurnOut->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateTurnOut->addTransition(&transitionRc5StopButton, StmStatePowerOff);
    StmStateTurnOut->addTransition(&transitionRc5TurnOutButton, StmStateGetLocInfo);
    StmStateTurnOut->addTransition(&transitionTurnOutDirectionShowDisable, StmStateTurnOut);
}

/**
 ***********************************************************************************************************************
   @brief   The loop function is called in an endless loop
   @return  None
 **********************************************************************************************************************/
void loop()
{

    if (rc5.read((uint8_t*)(&Rc5Toggle), (uint8_t*)(&Rc5Address), (uint8_t*)(&Rc5Command)))
    {
        if (Rc5Toggle != Rc5TogglePrevious)
        {
            Rc5TogglePrevious = Rc5Toggle;
            Rc5NewData        = true;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
            Serial.print("a:");
            Serial.print(Rc5Address);
            Serial.print(" c:");
            Serial.print(Rc5Command);
            Serial.print(" t:");
            Serial.println(Rc5Toggle);
#endif
            Stm.run();
            previousMillis = millis();
        }
    }

    XPNet.receive();

    if (millis() - previousMillis >= 20)
    {
        Stm.run();
        previousMillis = millis();
    }
}

/***********************************************************************************************************************
 * XpressNet callback function for system status.
 */
void notifyXNetPower(uint8_t State)
{
    XpressNetPowerStat = State;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    Serial.print("XnetPower : ");
    Serial.println(State);
#endif
}

/***********************************************************************************************************************
 * XpressNet callback function for loc data.
 */
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction,
    uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean Req)
{
    bool LocDataValid = true;

    uint16_t Address = ((uint16_t)(Adr_High) << 8) | (uint16_t)(Adr_Low);

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    Serial.print("A: ");
    Serial.print(Address);
    Serial.print(" S: ");
    Serial.print(Speed);
    Serial.print(" D:  ");
    Serial.print(Steps);
    Serial.print(" Dir : ");
    Serial.print(Direction);
    Serial.print(" F0 : ");
    Serial.print(F0);
    Serial.print(" F1 : ");
    Serial.println(F1);
#endif

    if (locInfo.Address == Address)
    {
        locInfo.Address   = Address;
        locInfo.Speed     = Speed;
        locInfo.Direction = Direction;
        // Function F0..F4
        locInfo.Functions = (F0 >> 4) & 0x01;
        locInfo.Functions |= (F0 << 1) & 0x1E;
        // Function F5..F12, higher functions are not processed.
        locInfo.Functions |= ((uint16_t)(F1) << 5);

        switch (Steps)
        {
        case 0:
        case 2:
        case 4: locInfo.Steps = Steps; break;
        default: LocDataValid = false; break;
        }
    }

    if (LocDataValid == true)
    {
        if ((memcmp(&locInfo, &locInfoPrevious, sizeof(locInfo)) != 0) || (locInfoRefresh == true))
        {
            LocInfoChanged = true;
            memcpy(&locInfoPrevious, &locInfo, sizeof(locInfo));

            if (locInfoRefresh == true)
            {
                switch (locInfo.Steps)
                {
                case 0:
                case 4: LocActualSpeed = Speed; break;
                case 2:
                    // Convert DCC speed to normal speed
                    LocActualSpeed = SpeedStep28TableFromDcc[Speed];
                    break;
                }

                if (locInfo.Direction == 1)
                {
                    LocActualDirection = 128;
                }
                else
                {
                    LocActualDirection = 0;
                }

                LocActualFunctions = locInfo.Functions;
            }
        }
    }

    Busy = Busy;
    Req  = Req;
    F2   = F2;
    F3   = F3;
}
