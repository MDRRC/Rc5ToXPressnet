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
#include <avr/wdt.h>
#include <eeprom.h>

/***********************************************************************************************************************
   D E F I N E S
 **********************************************************************************************************************/
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin # (or -1 if sharing Arduino reset pin)
#define XPRESSNET_PIN 22  // Read/write pin 485 chip
#define IR_PIN 2          // IR sensor pin
#define EEPROM_VERSION 40 // EEPROM version, just a number...

/**
 * Typedef struct for actual locomotive info.
 */
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

/**
 * Application variables.
 */
static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static RC5 rc5(IR_PIN);
static XpressNetClass XPNet;
static LocInfo locInfo;
static LocInfo locInfoPrevious;
static StateMachine Stm                  = StateMachine();
static uint8_t Rc5Toggle                 = 0;
static uint8_t Rc5TogglePrevious         = 0xFF;
static uint8_t Rc5Address                = 0;
static uint8_t Rc5Command                = 0;
static bool Rc5NewData                   = false;
static bool PowerOnStart                 = true;
static uint32_t previousMillis           = 0;
static byte XpressNetPowerStat           = 0xFF;
static byte XpressNetPowerStatPrevious   = 0xFF;
static uint16_t LocAddressSelect         = 0;
static uint8_t LocActualSpeed            = 0;
static uint8_t LocActualDirection        = 0;
static uint16_t LocActualFunctions       = 0;
static uint8_t LocLastSelected           = 0;
static uint32_t LocUpdateTimeOut         = 0;
static uint16_t TurnOutAddress           = 0;
static uint32_t TurnOutAddressTimeout    = 0;
static uint32_t TurnOutSymbolTimeOut     = 0;
static uint32_t LocInfoUpdateTimeOut     = 0;
static uint8_t FunctionOffset            = 0;
static uint32_t FunctionOffsetTime       = 0;
static bool LocInfoChanged               = false;
static bool locInfoRefresh               = false;
static uint16_t LocTurnOutAddressDefault = 3;
static uint16_t LocTurnoutAddressMax     = 9999;
static uint16_t LocTurnoutAddressMin     = 0;
static uint16_t ConfigEnterCode          = 0;
static uint8_t XpNetAddress              = 30;
static bool ConfigChanged                = false;
static bool ButtonOnOffEmergency         = false;
static bool ButtonBehaviourPlusMinus     = false;
static bool AcOption                     = false;
/**
 * Definitions for EEPROM addresses.
 */
int EepromAddressVersion         = 0x00;
int EepromAddressPower           = 0x02;
int EepromAddressUpDown          = 0x04;
int EepromXpNetAddress           = 0x06;
int EepromAddressLocA            = 0x08;
int EepromAddressLocB            = 0x0A;
int EepromAddressLocC            = 0x0B;
int EepromAddressLocD            = 0x0D;
int EepromAddressLocLastSelected = 0x10;
int EepromAddressTurnoutA        = 0x12;
int EepromAddressTurnoutB        = 0x14;
int EepromAddressTurnoutC        = 0x16;
int EepromAddressTurnoutD        = 0x18;
int EepromAddressAcOption        = 0x1A;

/**
 * Button definitions.
 */
static const uint8_t Rc5Button_1                  = 1;
static const uint8_t Rc5Button_2                  = 2;
static const uint8_t Rc5Button_3                  = 3;
static const uint8_t Rc5Button_4                  = 4;
static const uint8_t Rc5Button_5                  = 5;
static const uint8_t Rc5Button_6                  = 6;
static const uint8_t Rc5Button_7                  = 7;
static const uint8_t Rc5Button_8                  = 8;
static const uint8_t Rc5Button_9                  = 9;
static const uint8_t Rc5Button_0                  = 0;
static const uint8_t Rc5Button_OnOff              = 12;
static const uint8_t Rc5Button_Loc                = 13;
static const uint8_t Rc5Button_TurnOutSingle      = 14;
static const uint8_t Rc5Button_Right              = 16;
static const uint8_t Rc5Button_Left               = 17;
static const uint8_t Rc5Button_LightOff           = 19;
static const uint8_t Rc5Button_F0                 = 20;
static const uint8_t Rc5Button_F1                 = 21;
static const uint8_t Rc5Button_F2                 = 22;
static const uint8_t Rc5Button_F3                 = 23;
static const uint8_t Rc5Button_F4                 = 24;
static const uint8_t Rc5Button_FPlus4             = 29;
static const uint8_t Rc5Button_FPlus8             = 30;
static const uint8_t Rc5Button_Plus               = 32;
static const uint8_t Rc5Button_Minus              = 33;
static const uint8_t Rc5Button_TurnOutDiversing_1 = 40;
static const uint8_t Rc5Button_TurnoutForward_1   = 41;
static const uint8_t Rc5Button_TurnOutDiversing_2 = 42;
static const uint8_t Rc5Button_TurnoutForward_2   = 43;
static const uint8_t Rc5Button_TurnOutDiversing_3 = 44;
static const uint8_t Rc5Button_TurnoutForward_3   = 45;
static const uint8_t Rc5Button_TurnOutDiversing_4 = 46;
static const uint8_t Rc5Button_TurnoutForward_4   = 47;
static const uint8_t Rc5Button_A                  = 51;
static const uint8_t Rc5Button_B                  = 52;
static const uint8_t Rc5Button_C                  = 53;
static const uint8_t Rc5Button_D                  = 54;

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
State* StmStateConfig       = Stm.addState(&StateConfig);

/* Conversion table for normal speed to 28 steps DCC speed. */
const uint8_t SpeedStep28TableToDcc[29]
    = { 16, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31 };

/* Conversion table for 28 steps DCC speed to normal speed. */
const uint8_t SpeedStep28TableFromDcc[32] = { 0, 0, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 0, 0, 2, 4, 6, 8,
    10, 12, 14, 16, 18, 20, 22, 24, 26, 28 };

/* Conversion table for normal speed to "15" steps DCC/MM speed. Speed step 1 is emergency stop and skipped by this
 * table. */
const uint8_t SpeedStep14TableToMMDcc[15] = { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

/* Conversion table for "15" steps DCC/MM speed to normal speed. */
const uint8_t SpeedStep14TableFromMMDcc[16] = { 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };

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
    display.print(F("   1.0.3"));
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
    switch (locInfo.Steps)
    {
    case 0: display.print(SpeedStep14TableFromMMDcc[locInfo.Speed]); break;
    case 2: display.print(SpeedStep28TableFromDcc[locInfo.Speed]); break;
    default: display.print(locInfo.Speed); break;
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
    default: display.print(" "); break;
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
void ShowSelectecLocInStatusRow(const char* strPr)
{
    display.fillRect(100, 0, 9, 8, SSD1306_BLACK);
    display.setCursor(100, 0);
    display.setTextSize(0);
    display.print(strPr);
    display.display();
}

/***********************************************************************************************************************
 */
void SendTurnOutCommand(uint8_t EepromAddress, uint8_t Direction)
{
    uint8_t TurnOutData = 0x08;
    TurnOutAddress      = ((uint16_t)(EEPROM.read(EepromAddress)) << 8);
    TurnOutAddress |= (uint16_t)(EEPROM.read(EepromAddress + 1));

    if (TurnOutAddress <= LocTurnoutAddressMax)
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
void LocStore(int EepromAddress, uint16_t Address, uint8_t LastSelected)
{
    EEPROM.write(EepromAddress, Address >> 8);
    EEPROM.write(EepromAddress + 1, Address & 0xFF);

    EEPROM.write(EepromAddressLocLastSelected, LastSelected);
}

/***********************************************************************************************************************
 */
void TurnOutStoreAndUpdateIndication(int EepromAddress, uint16_t Address, const char* StrPtr)
{
    EEPROM.write(EepromAddress, Address >> 8);
    EEPROM.write(EepromAddress + 1, Address & 0xFF);

    display.fillRect(110, 0, 10, 8, SSD1306_BLACK);
    display.setCursor(110, 0);
    display.setTextSize(1);
    display.print(StrPtr);
}

/***********************************************************************************************************************
 */
uint16_t LocToggleFunction(uint16_t Address, uint16_t Function, uint8_t Offset)
{
    LocActualFunctions ^= (1 << (Function + Offset));
    XPNet.setLocoFunc(
        Address >> 8, Address & 0xFF, (LocActualFunctions >> (Function + Offset)) & 0x01, (Function + Offset));

    LocUpdateTimeOut   = millis();
    FunctionOffsetTime = millis();

    return (LocActualFunctions);
}

/***********************************************************************************************************************
 */
void LocSelectButtonAToD(int EepromAddress, uint8_t lastselected)
{
    uint16_t LocAddress;

    // Get loc address
    LocAddress = ((uint16_t)(EEPROM.read(EepromAddress)) << 8);
    LocAddress |= (uint16_t)EEPROM.read(EepromAddress + 1);

    if (lastselected != LocLastSelected)
    {
        if (LocAddress < LocTurnoutAddressMax)
        {
            locInfoRefresh  = true;
            locInfo.Address = LocAddress;
            LocLastSelected = lastselected;
            EEPROM.write(EepromAddressLocLastSelected, LocLastSelected);
            Stm.transitionTo(StmStateGetLocInfo);
        }
    }
}

/***********************************************************************************************************************
 */
void StateInit()
{
    uint16_t Index = 0;

    if (Stm.executeOnce)
    {
        // First check if EEPROM version is ok
        if ((uint8_t)EEPROM.read(EepromAddressVersion) != EEPROM_VERSION)
        {
            // Nope, erase all data.
            for (Index = 0; Index < EEPROM.length(); Index++)
            {
                EEPROM.write(Index, 0xFF);
            }

            EEPROM.write(EepromAddressVersion, EEPROM_VERSION);
        }

        // Get actual XpressNet device address.
        XpNetAddress = (uint8_t)EEPROM.read(EepromXpNetAddress);
        if ((XpNetAddress == 0) || (XpNetAddress > 31))
        {
            XpNetAddress = 30;
            EEPROM.write(EepromXpNetAddress, XpNetAddress);
        }

        // Init Xpressnet.
        XPNet.start(XpNetAddress, XPRESSNET_PIN);

        // OnOff button behavior.
        switch ((uint8_t)EEPROM.read(EepromAddressPower))
        {
        case 0:
            // Power off
            ButtonOnOffEmergency = false;
            break;
        case 1:
            // Emergency stop
            ButtonOnOffEmergency = true;
            break;
        default:
            // Power off
            EEPROM.write(EepromAddressPower, 0);
            ButtonOnOffEmergency = false;
            break;
        }

        // Up/down buttons behavior
        switch ((uint8_t)EEPROM.read(EepromAddressUpDown))
        {
        case 0:
            // Only one speed step step when pressed continuously.
            ButtonBehaviourPlusMinus = false;
            break;
        case 1:
            // Increase speed continuously when pressing up/down
            ButtonBehaviourPlusMinus = true;
            break;
        default:
            // Only one speed step step when pressed continuously.
            ButtonBehaviourPlusMinus = false;
            EEPROM.write(EepromAddressUpDown, 0);
            break;
        }

        // Get Ac option. Ac means Alternative control. When false the +/- can be used
        // to change direction at speed 0 and continue driving. When true only speed
        // increase / decrease is possible, change direction with left/right button.
        switch (EEPROM.read(EepromAddressAcOption))
        {
        case 0: AcOption = false; break;
        case 1: AcOption = true; break;
        default:
            AcOption = true;
            EEPROM.write(EepromAddressAcOption, 1);
            break;
        }

        // Read loc address and set to default when required...
        LocLastSelected = (uint8_t)EEPROM.read(EepromAddressLocLastSelected);

        switch (LocLastSelected)
        {
        case 0:
            locInfo.Address = ((uint16_t)(EEPROM.read(EepromAddressLocA)) << 8);
            locInfo.Address |= (uint16_t)EEPROM.read(EepromAddressLocA + 1);
            break;
        case 1:
            locInfo.Address = ((uint16_t)(EEPROM.read(EepromAddressLocB)) << 8);
            locInfo.Address |= (uint16_t)EEPROM.read(EepromAddressLocB + 1);
            break;
        case 2:
            locInfo.Address = ((uint16_t)(EEPROM.read(EepromAddressLocC)) << 8);
            locInfo.Address |= (uint16_t)EEPROM.read(EepromAddressLocC + 1);
            break;
        case 3:
            locInfo.Address = ((uint16_t)(EEPROM.read(EepromAddressLocD)) << 8);
            locInfo.Address |= (uint16_t)EEPROM.read(EepromAddressLocD + 1);
            break;
        default:
            // Probably no selection done yet or default data or EEPROM reset, set 3 and A as default.
            locInfo.Address = LocTurnOutAddressDefault;
            LocLastSelected = 0;
            EEPROM.write(EepromAddressLocA, locInfo.Address >> 8);
            EEPROM.write(EepromAddressLocA + 1, locInfo.Address & 0xFF);
            EEPROM.write(EepromAddressLocLastSelected, LocLastSelected);
            break;
        }

        if (locInfo.Address > LocTurnoutAddressMax)
        {
            locInfo.Address = LocTurnOutAddressDefault;
        }
        XPNet.getLocoInfo(locInfo.Address >> 8, locInfo.Address & 0xFF);
    }
    else
    {
    }
}

/***********************************************************************************************************************
 * In emergency allow change of functions and direction but no speed change!
 */
void StateEmergency()
{
    bool TransmitSpeedDirectionData = false;

    if (Stm.executeOnce)
    {
        XPNet.setPower(csEmergencyStop);
        UpdateStatusRow(F("EMERGENCY"), true);

        if (PowerOnStart == false)
        {
            ConfigEnterCode = 0;

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
            case Rc5Button_Right:
                // Right button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x0)
                {
                    LocActualDirection = 0x80;
                }
                break;
            case Rc5Button_Left:
                // Left button
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x80)
                {
                    LocActualDirection = 0x0;
                }
                break;
            case Rc5Button_LightOff:
                // Off (light) button
                LocActualFunctions &= ~(1 << 0);
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, LocActualFunctions & 0x01, 0);
                Rc5NewData       = false;
                LocUpdateTimeOut = millis();
                break;
            case Rc5Button_F0:
                // F0 (Light) button
                LocActualFunctions = LocToggleFunction(locInfo.Address, 0, 0);
                Rc5NewData         = false;
                break;
            case Rc5Button_F1:
            case Rc5Button_F2:
            case Rc5Button_F3:
            case Rc5Button_F4:
                // F1 .. F4 button toggle function
                LocActualFunctions = LocToggleFunction(locInfo.Address, Rc5Command - 20, FunctionOffset);
                Rc5NewData         = false;
                break;
            case Rc5Button_FPlus4:
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
            case Rc5Button_FPlus8:
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
            case Rc5Button_TurnOutDiversing_1:
                // Turnout A red.
                SendTurnOutCommand(EepromAddressTurnoutA, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnoutForward_1:
                // Turnout A green.
                SendTurnOutCommand(EepromAddressTurnoutA, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnOutDiversing_2:
                // Turnout B red.
                SendTurnOutCommand(EepromAddressTurnoutB, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnoutForward_2:
                // Turnout B green.
                SendTurnOutCommand(EepromAddressTurnoutB, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnOutDiversing_3:
                // Turnout C red.
                SendTurnOutCommand(EepromAddressTurnoutC, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnoutForward_3:
                // Turnout C green.
                SendTurnOutCommand(EepromAddressTurnoutC, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnOutDiversing_4:
                // Turnout D red.
                SendTurnOutCommand(EepromAddressTurnoutD, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnoutForward_4:
                // Turnout D green.
                SendTurnOutCommand(EepromAddressTurnoutD, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_A:
                LocSelectButtonAToD(EepromAddressLocA, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_B:
                // B Button to select loc
                LocSelectButtonAToD(EepromAddressLocB, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_C:
                // C Button to select loc
                LocSelectButtonAToD(EepromAddressLocC, 2);
                Rc5NewData = false;
                break;
            case Rc5Button_D:
                // D Button to select loc
                LocSelectButtonAToD(EepromAddressLocD, 3);
                Rc5NewData = false;
                break;
            default: break;
            }
        }

        if (TransmitSpeedDirectionData == true)
        {
            Rc5NewData       = false;
            LocActualSpeed   = 0;
            LocUpdateTimeOut = millis();

            XPNet.setLocoDrive(
                locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps, LocActualSpeed | LocActualDirection);

            XPNet.getLocoInfo(locInfo.Address >> 8, locInfo.Address & 0xFF);

            ShowFPlusRemove();
            FunctionOffset = 0;
        }
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
            ConfigEnterCode = 0;

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

    if (Rc5NewData == true)
    {
        switch (Rc5Command)
        {
        case Rc5Button_0:
        case Rc5Button_1:
        case Rc5Button_2:
        case Rc5Button_3:
        case Rc5Button_4:
        case Rc5Button_5:
        case Rc5Button_6:
        case Rc5Button_7:
        case Rc5Button_8:
        case Rc5Button_9:

            ConfigEnterCode *= 10;
            ConfigEnterCode += (uint16_t)(Rc5Command);
            if (ConfigEnterCode == 34567)
            {
                // Magic code to enter config mode.
                Stm.transitionTo(StmStateConfig);
            }
            else if (ConfigEnterCode > 34567)
            {
                ConfigEnterCode = 0;
            }
            Rc5NewData = false;
            break;
        case Rc5Button_F0: ConfigEnterCode = 0; break;
        }
    }
}

/***********************************************************************************************************************
 */
void StatePowerOn()
{
    bool TransmitSpeedDirectionData = false;

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
            case Rc5Button_0:
            case Rc5Button_1:
            case Rc5Button_2:
            case Rc5Button_3:
            case Rc5Button_4:
            case Rc5Button_5:
            case Rc5Button_6:
            case Rc5Button_7:
            case Rc5Button_8:
            case Rc5Button_9: Rc5NewData = false; break;
            case Rc5Button_Right:
                // Right button, change direction.
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x0)
                {
                    LocActualDirection = 0x80;
                }
                break;
            case Rc5Button_Left:
                // Left button, change direction.
                TransmitSpeedDirectionData = true;
                if (LocActualDirection == 0x80)
                {
                    LocActualDirection = 0x0;
                }
                break;
            case Rc5Button_LightOff:
                // Off (light) button
                LocActualFunctions &= ~(1 << 0);
                XPNet.setLocoFunc(locInfo.Address >> 8, locInfo.Address & 0xFF, LocActualFunctions & 0x01, 0);
                Rc5NewData       = false;
                LocUpdateTimeOut = millis();
                break;
            case Rc5Button_F0:
                // F0 (Light) button
                LocActualFunctions = LocToggleFunction(locInfo.Address, 0, 0);
                Rc5NewData         = false;
                break;
            case Rc5Button_F1:
            case Rc5Button_F2:
            case Rc5Button_F3:
            case Rc5Button_F4:
                // F1 .. F4 button toggle function
                LocActualFunctions = LocToggleFunction(locInfo.Address, Rc5Command - 20, FunctionOffset);
                Rc5NewData         = false;
                break;
            case Rc5Button_FPlus4:
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
            case Rc5Button_FPlus8:
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
            case Rc5Button_Plus:
                // + button
                if (AcOption == true)
                {
                    LocActualSpeed++;
                    TransmitSpeedDirectionData = true;
                }
                else
                {
                    TransmitSpeedDirectionData = true;
                    if (LocActualDirection == 0x80)
                    {
                        LocActualSpeed++;
                    }
                    else
                    {
                        if (LocActualSpeed > 0)
                        {
                            LocActualSpeed--;
                        }
                        else
                        {
                            // Speed 0, change direction.
                            LocActualDirection = 0x80;
                        }
                    }
                }
                break;
            case Rc5Button_Minus:
                // - button
                if (AcOption == true)
                {
                    if (LocActualSpeed > 0)
                    {
                        LocActualSpeed--;
                        TransmitSpeedDirectionData = true;
                    }
                }
                else
                {
                    TransmitSpeedDirectionData = true;
                    if (LocActualDirection == 0x80)
                    {
                        if (LocActualSpeed > 0)
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
                }
                break;
            case Rc5Button_TurnOutDiversing_1:
                // Turnout A red.
                SendTurnOutCommand(EepromAddressTurnoutA, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnoutForward_1:
                // Turnout A green.
                SendTurnOutCommand(EepromAddressTurnoutA, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnOutDiversing_2:
                // Turnout B red.
                SendTurnOutCommand(EepromAddressTurnoutB, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnoutForward_2:
                // Turnout B green.
                SendTurnOutCommand(EepromAddressTurnoutB, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnOutDiversing_3:
                // Turnout C red.
                SendTurnOutCommand(EepromAddressTurnoutC, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnoutForward_3:
                // Turnout C green.
                SendTurnOutCommand(EepromAddressTurnoutC, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnOutDiversing_4:
                // Turnout D red.
                SendTurnOutCommand(EepromAddressTurnoutD, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_TurnoutForward_4:
                // Turnout D green.
                SendTurnOutCommand(EepromAddressTurnoutD, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_A:
                LocSelectButtonAToD(EepromAddressLocA, 0);
                Rc5NewData = false;
                break;
            case Rc5Button_B:
                // B Button to select loc
                LocSelectButtonAToD(EepromAddressLocB, 1);
                Rc5NewData = false;
                break;
            case Rc5Button_C:
                // C Button to select loc
                LocSelectButtonAToD(EepromAddressLocC, 2);
                Rc5NewData = false;
                break;
            case Rc5Button_D:
                // D Button to select loc
                LocSelectButtonAToD(EepromAddressLocD, 3);
                Rc5NewData = false;
                break;
            default: break;
            }

            if (TransmitSpeedDirectionData == true)
            {
                LocUpdateTimeOut = millis();
                Rc5NewData       = false;

                switch (locInfo.Steps)
                {
                case 0:
                    // 14 speed decoder.
                    if (LocActualSpeed >= 14)
                    {
                        LocActualSpeed = 14;
                    }
                    XPNet.setLocoDrive(locInfo.Address >> 8, locInfo.Address & 0xFF, locInfo.Steps,
                        SpeedStep14TableToMMDcc[LocActualSpeed] | LocActualDirection);

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
            case Rc5Button_0:
            case Rc5Button_1:
            case Rc5Button_2:
            case Rc5Button_3:
            case Rc5Button_4:
            case Rc5Button_5:
            case Rc5Button_6:
            case Rc5Button_7:
            case Rc5Button_8:
            case Rc5Button_9:
                TurnOutAddress *= 10;
                TurnOutAddress += (uint16_t)(Rc5Command);

                if (TurnOutAddress > LocTurnoutAddressMax)
                {
                    TurnOutAddress = LocTurnoutAddressMin;
                }

                // Show address.
                display.fillRect(2, 13, 50, 22, SSD1306_BLACK);
                display.setTextSize(2);
                display.setCursor(3, 20);
                display.print(TurnOutAddress);
                display.display();
                Rc5NewData = false;
                break;
            case Rc5Button_F0:
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
            case Rc5Button_F1:
                // F1 to set turn out diversing without storing
                if (TurnOutAddress > 0)
                {
                    Rc5NewData       = false;
                    TurnOutDirection = 0;
                    SendTurnoutData  = true;
                }
                break;
            case Rc5Button_F4:
                // F4 to set turn out forward without storing
                if (TurnOutAddress > 0)
                {
                    Rc5NewData       = false;
                    TurnOutDirection = 1;
                    SendTurnoutData  = true;
                }
                break;
            case Rc5Button_TurnOutDiversing_1:
                // Red button above A, store turnout address
                TurnOutStoreAndUpdateIndication(EepromAddressTurnoutA, TurnOutAddress, "A");
                Rc5NewData       = false;
                SendTurnoutData  = true;
                TurnOutDirection = 0;
                break;
            case Rc5Button_TurnoutForward_1:
                // Green button above A, store turnout address
                TurnOutStoreAndUpdateIndication(EepromAddressTurnoutA, TurnOutAddress, "A");
                Rc5NewData       = false;
                SendTurnoutData  = true;
                TurnOutDirection = 0;
                break;
            case Rc5Button_TurnOutDiversing_2:
                // Red button above B, store turnout address
                TurnOutStoreAndUpdateIndication(EepromAddressTurnoutB, TurnOutAddress, "B");
                Rc5NewData       = false;
                SendTurnoutData  = true;
                TurnOutDirection = 0;
                break;
            case Rc5Button_TurnoutForward_2:
                // Green button above B, store turnout address
                Rc5NewData       = false;
                SendTurnoutData  = true;
                TurnOutDirection = 0;
                break;
            case Rc5Button_TurnOutDiversing_3:
                // Red button above C, store turnout address
                TurnOutStoreAndUpdateIndication(EepromAddressTurnoutC, TurnOutAddress, "C");
                Rc5NewData       = false;
                SendTurnoutData  = true;
                TurnOutDirection = 0;
                break;
            case Rc5Button_TurnoutForward_3:
                // Green button above C, store turnout address
                TurnOutStoreAndUpdateIndication(EepromAddressTurnoutC, TurnOutAddress, "C");
                Rc5NewData       = false;
                SendTurnoutData  = true;
                TurnOutDirection = 0;
                break;
            case Rc5Button_TurnOutDiversing_4:
                // Red button above D, store turnout address

                TurnOutStoreAndUpdateIndication(EepromAddressTurnoutD, TurnOutAddress, "D");
                Rc5NewData       = false;
                SendTurnoutData  = true;
                TurnOutDirection = 0;
                break;
            case Rc5Button_TurnoutForward_4:
                // Green button above D, store turnout address
                TurnOutStoreAndUpdateIndication(EepromAddressTurnoutD, TurnOutAddress, "D");
                Rc5NewData       = false;
                SendTurnoutData  = true;
                TurnOutDirection = 0;
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
        if (LocInfoChanged == true)
        {
            switch (locInfo.Steps)
            {
            case 0: LocActualSpeed = SpeedStep14TableFromMMDcc[locInfo.Speed]; break;
            case 2: LocActualSpeed = SpeedStep28TableFromDcc[locInfo.Speed]; break;
            default: LocActualSpeed = locInfo.Speed; break;
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
            case Rc5Button_0:
            case Rc5Button_1:
            case Rc5Button_2:
            case Rc5Button_3:
            case Rc5Button_4:
            case Rc5Button_5:
            case Rc5Button_6:
            case Rc5Button_7:
            case Rc5Button_8:
            case Rc5Button_9:
                // Increase selected address.
                LocAddressSelect *= 10;
                LocAddressSelect += (uint16_t)(Rc5Command);

                // Limit
                if (LocAddressSelect > LocTurnoutAddressMax)
                {
                    LocAddressSelect = LocTurnoutAddressMin;
                }

                // Display
                display.fillRect(74, 10, 51, 64, SSD1306_BLACK);
                display.setCursor(75, 14);
                display.setTextSize(2);
                display.print(LocAddressSelect);
                display.display();

                Rc5NewData = false;
                break;
            case Rc5Button_Loc:
                // Loc button to exit.
                Rc5NewData = false;
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                }
                Stm.transitionTo(StmStateGetLocInfo);
                break;
            case Rc5Button_F0:
                // Use F0 button to reset address.
                LocAddressSelect = 0;
                display.fillRect(74, 10, 51, 64, SSD1306_BLACK);
                display.setCursor(75, 14);
                display.setTextSize(2);
                display.print(LocAddressSelect);
                display.display();
                Rc5NewData = false;
                break;
            case Rc5Button_A:
                // A button, when loc address valid store loc address and update data on screen.
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                    LocLastSelected = 0;
                    LocStore(EepromAddressLocA, locInfo.Address, LocLastSelected);
                    ShowSelectecLocInStatusRow("A");
                }
                Rc5NewData = false;
                break;
            case Rc5Button_B:
                // B button, when loc address valid store loc address and update data on screen.
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                    LocLastSelected = 1;
                    LocStore(EepromAddressLocB, locInfo.Address, LocLastSelected);
                    ShowSelectecLocInStatusRow("B");
                }
                Rc5NewData = false;
                break;
            case Rc5Button_C:
                // C button, when loc address valid store loc address and update data on screen.
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                    LocLastSelected = 2;
                    LocStore(EepromAddressLocC, locInfo.Address, LocLastSelected);
                    ShowSelectecLocInStatusRow("C");
                }
                Rc5NewData = false;
                break;
            case Rc5Button_D:
                // D button, when loc address valid store loc address and update data on screen.
                if (LocAddressSelect > 0)
                {
                    locInfo.Address = LocAddressSelect;
                    LocLastSelected = 3;
                    LocStore(EepromAddressLocD, locInfo.Address, LocLastSelected);
                    ShowSelectecLocInStatusRow("D");
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
void StateConfig()
{
    if (Stm.executeOnce)
    {
        ConfigChanged = false;
        XpNetAddress  = 0;

        UpdateStatusRow(F("CONFIG"), true);

        LocAddressSelect = 0;
        display.fillRect(0, 8, 128, 56, SSD1306_BLACK);
        display.setTextSize(1);
        display.setCursor(5, 15);
        display.print("XPNet   :");
        display.setCursor(60, 15);
        display.print(XpNetAddress);

        // Show stop button behavior
        display.setCursor(5, 25);
        display.print("Stop    :");

        display.setCursor(60, 25);
        switch ((uint8_t)EEPROM.read(EepromAddressPower))
        {
        case 0: display.print("Power Off"); break;
        case 1: display.print("Emergency"); break;
        default:
            EEPROM.write(EepromAddressPower, 0);
            display.print("Power Off");
            break;
        }

        // Show behavior of up/down buttons
        display.setCursor(5, 35);
        display.print("Up/Down :");

        display.setCursor(60, 35);
        switch ((uint8_t)EEPROM.read(EepromAddressUpDown))
        {
        case 0: display.print("Step"); break;
        case 1: display.print("Continue"); break;
        default:
            EEPROM.write(EepromAddressUpDown, 0);
            display.print("Step");
            break;
        }

        // AcOption
        display.setCursor(5, 45);
        display.print("AcOption:");

        display.setCursor(60, 45);
        switch ((uint8_t)EEPROM.read(EepromAddressAcOption))
        {
        case 0: display.print("Off "); break;
        case 1: display.print("On      "); break;
        }

        display.display();
    }
    else
    {
        if (Rc5NewData == true)
        {
            switch (Rc5Command)
            {
            case Rc5Button_0:
            case Rc5Button_1:
            case Rc5Button_2:
            case Rc5Button_3:
            case Rc5Button_4:
            case Rc5Button_5:
            case Rc5Button_6:
            case Rc5Button_7:
            case Rc5Button_8:
            case Rc5Button_9:
                XpNetAddress *= 10;
                XpNetAddress += (uint16_t)(Rc5Command);

                // Limit
                if (XpNetAddress > 31)
                {
                    XpNetAddress = 0;
                }

                display.fillRect(60, 15, 51, 10, SSD1306_BLACK);
                display.setCursor(60, 15);
                display.print(XpNetAddress);
                display.display();

                // Only store addresses between 1 and 31
                if (XpNetAddress != 0)
                {
                    EEPROM.write(EepromXpNetAddress, XpNetAddress);
                    ConfigChanged = true;
                }

                Rc5NewData = false;
                break;
            case Rc5Button_FPlus4:
                // AcOption off
                AcOption = false;
                display.fillRect(60, 45, 51, 10, SSD1306_BLACK);
                display.setCursor(60, 45);
                display.print("Off ");
                display.display();

                EEPROM.write(EepromAddressAcOption, 0);
                ConfigChanged = true;
                Rc5NewData    = false;
                break;
            case Rc5Button_FPlus8:
                // AcOption on
                AcOption = true;
                display.fillRect(60, 45, 51, 10, SSD1306_BLACK);
                display.setCursor(60, 45);
                display.print("On");
                display.display();

                EEPROM.write(EepromAddressAcOption, 1);
                ConfigChanged = true;
                Rc5NewData    = false;
                break;
            case Rc5Button_F4:
                if (ConfigChanged == true)
                {
                    // Reset to activate new settings
                    UpdateStatusRow(F("RESET !!"), true);
                    wdt_enable(WDTO_15MS);
                }
                else
                {
                    Rc5NewData = false;
                    Stm.transitionTo(StmStatePowerOff);
                }
                break;
            case Rc5Button_OnOff:
                // Show stop button behavior
                display.fillRect(60, 25, 60, 10, SSD1306_BLACK);
                display.setCursor(60, 25);
                switch ((uint8_t)EEPROM.read(EepromAddressPower))
                {
                case 0:
                    display.print("Emergency");
                    EEPROM.write(EepromAddressPower, 1);
                    break;
                case 1:
                    display.print("Power Off");
                    EEPROM.write(EepromAddressPower, 0);
                    break;
                default:
                    EEPROM.write(EepromAddressPower, 0);
                    display.print("Power Off");
                    break;
                }
                display.display();
                Rc5NewData    = false;
                ConfigChanged = true;
                break;
            case Rc5Button_Plus:
            case Rc5Button_Minus:
                // Switch behavior of up/down buttons
                display.fillRect(60, 35, 51, 10, SSD1306_BLACK);
                display.setCursor(60, 35);
                switch ((uint8_t)EEPROM.read(EepromAddressUpDown))
                {
                case 0:
                    EEPROM.write(EepromAddressUpDown, 1);
                    display.print("Continue");
                    break;
                case 1:
                    EEPROM.write(EepromAddressUpDown, 0);
                    display.print("Step");
                    break;
                default:
                    EEPROM.write(EepromAddressUpDown, 0);
                    display.print("Step");
                    break;
                }
                display.display();
                Rc5NewData    = false;
                ConfigChanged = true;
                break;
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
        if (Rc5Command == Rc5Button_OnOff)
        {
            // Based on central station state transmit power on/off.
            // Based on the status change response the applicable state will be entered.
            // When calling setPower() in states generated sometimes run a rounds..
            switch (XpressNetPowerStat)
            {
            case csNormal:
                if (ButtonOnOffEmergency == false)
                {
                    XPNet.setPower(csTrackVoltageOff);
                }
                else
                {
                    XPNet.setPower(csEmergencyStop);
                }
                break;
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
        if (Rc5Command == Rc5Button_Loc)
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
        if (Rc5Command == Rc5Button_TurnOutSingle)
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
 */
bool transitionUpdateLocData()
{
    if ((millis() - LocInfoUpdateTimeOut) > 250)
    {
        LocInfoUpdateTimeOut = millis();
        XPNet.getLocoInfo(locInfo.Address >> 8, locInfo.Address & 0xFF);
        locInfoRefresh = true;
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
    // Init display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        for (;;)
            ; // Don't proceed, loop forever
    }

    ShowInitSreen();
    UpdateStatusRow(F("CONNECTING"), true);

    previousMillis        = millis();
    LocInfoUpdateTimeOut  = previousMillis;
    LocUpdateTimeOut      = previousMillis;
    TurnOutAddressTimeout = previousMillis;
    TurnOutSymbolTimeOut  = previousMillis;
    LocInfoUpdateTimeOut  = previousMillis;

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
    StmStatePowerOn->addTransition(&transitionUpdateLocData, StmStatePowerOn);

    StmStateEmergency->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateEmergency->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateEmergency->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateEmergency->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateEmergency->addTransition(&transitionRc5StopButton, StmStatePowerOff);
    StmStateEmergency->addTransition(&transitionFunctionOffSetReset, StmStatePowerOn);

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
    StmStateGetLocInfo->addTransition(&transitionRc5SelectLocButton, StmStateSelectLoc);
    StmStateGetLocInfo->addTransition(&transitionUpdateLocData, StmStateGetLocInfo);

    StmStateSelectLoc->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateSelectLoc->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateSelectLoc->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateSelectLoc->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateSelectLoc->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateSelectLoc->addTransition(&transitionRc5StopButton, StmStatePowerOff);

    StmStateTurnOut->addTransition(&transitionServiceMode, StmStateServiceMode);
    StmStateTurnOut->addTransition(&transitionPowerOff, StmStatePowerOff);
    StmStateTurnOut->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateTurnOut->addTransition(&transitionShortCircuit, StmStateShortCircuit);
    StmStateTurnOut->addTransition(&transitionRc5StopButton, StmStatePowerOff);
    StmStateTurnOut->addTransition(&transitionRc5TurnOutButton, StmStateGetLocInfo);
    StmStateTurnOut->addTransition(&transitionRc5SelectLocButton, StmStateGetLocInfo);
    StmStateTurnOut->addTransition(&transitionTurnOutDirectionShowDisable, StmStateTurnOut);

    StmStateConfig->addTransition(&transitionPowerOn, StmStateGetLocInfo);
    StmStateConfig->addTransition(&transitionEmergency, StmStateEmergency);
    StmStateConfig->addTransition(&transitionShortCircuit, StmStateShortCircuit);

    Serial.begin(57600);
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

            // Kick state machine after a button change.
            Stm.run();
            previousMillis = millis();
        }
        else
        {
            if (ButtonBehaviourPlusMinus == true)
            {
                // When Plus / Minus button pressed continuously and option enabled sent each plus / minus button
                // command.
                switch (Rc5Command)
                {
                case Rc5Button_Plus:
                case Rc5Button_Minus:
                    Rc5NewData        = true;
                    Rc5TogglePrevious = Rc5Toggle;

                    // Kick state machine after a button change.
                    Stm.run();
                    previousMillis = millis();
                    break;
                }
            }
        }
    }

    XPNet.receive();

    if (millis() - previousMillis >= 20)
    {
        // Update state machine every 20 msec when no other activity was present so the transition  functions are
        // kicked.
        Stm.run();
        previousMillis = millis();
    }
}

/***********************************************************************************************************************
 * XpressNet callback function for system status.
 */
void notifyXNetPower(uint8_t State) { XpressNetPowerStat = State; }

/***********************************************************************************************************************
 * XpressNet callback function for loc data.
 */
void notifyLokAll(uint8_t Adr_High, uint8_t Adr_Low, boolean Busy, uint8_t Steps, uint8_t Speed, uint8_t Direction,
    uint8_t F0, uint8_t F1, uint8_t F2, uint8_t F3, boolean Req)
{
    bool LocDataValid = true;

    uint16_t Address = ((uint16_t)(Adr_High) << 8) | (uint16_t)(Adr_Low);

    // Received loc address same as select then update data.
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

        Serial.println(Steps);
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
                // Convert received speed to "normal" speeds for the user.
                switch (locInfo.Steps)
                {
                case 0: break;
                case 2: LocActualSpeed = SpeedStep28TableFromDcc[Speed]; break;
                default: LocActualSpeed = Speed; break;
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

    // Non  used (for now).
    Busy = Busy;
    Req  = Req;
    F2   = F2;
    F3   = F3;
}
