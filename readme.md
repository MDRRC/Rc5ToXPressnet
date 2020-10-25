# Rc5 To XpressNet converter

The Rc5 to XpressNet convertor is an interface between an RC5 remote control and the [XpressNet bus](https://www.lenz-elektronik.de/digitalplus-xpressnet.php). 

It is based on: 

 * An Ardiuno Mega 2560 PRO Embed when the PCB is used.
 * An [Arduino Mega 2560](https://store.arduino.cc/arduino-mega-2560-rev3) will also work but the board is way bigger. 
 * MAX485 RS485 driver IC.
 * 128*64 pixels OLED SSD1306 I2C display. 
 * TSOP4836 IR receiver.
 
The Rc5ToXpressNet is intended to used with for example an [Uhlenbrock Iris infrared control](https://www.uhlenbrock.de/de_DE/produkte/digizen/I63D744D-001.htm!ArcEntryInfo=0004.9.I63D744D). Will also work with other RC5 remote controls for a TV but the layout and commands may be different, so adjustment of the RC5 codes by yourself may be required. Locomotives and accesoiry decoders (like turnout decoders) can be controlled. Programming of CV's for example is NOT supported.  
 
## Locomotive control
 * Control speed by pressing up / down button.
 * Change direction and stop locomotive by pressing left / right button.
 * Light on / off by pressing F0 / Off button
 * Change functions (F1..F12) of a locomotive (F11 and F12 are not shown, F10 is shown as 0). Functions F4..F8 and F9..F12 are changed after pressing button F+4 or F+8.
 * Select a preprogrammed locomotive by pressing button A..D (Only programmed locomotives can be controlled!).
 * Control preprogrammed turnouts by pressing the red/geen buttons above the A..D buttons. The turnout number and direction are shown for about 1.5 seconds in the status row. 
 
 ![](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/loccontrol1.JPG)
 
## Locomotive selection
 * After pressing the Locomotive button
 * Enter a address for a locomotive
 * Store the locomotive address by pressing button A..D.
 * Exit locomotive selection by pressing the Loc symbol button. 
 
 ![](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/select.JPG)

## Turnouts / accesoiry. 
 * After pressing the Single turnout button
 * Enter a address for a turnout
 * Press F1 or F4 to control the turnout without storing the turnout number.
 * Store the turnout address by pressing button A..D.
 
 ![](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/turnout.JPG)
 
  * Remark, the "double turnout" switch is NOT supported for entering turnout sequences.
 
## Configuration 
 * When OFF is display enter 34567 to enter the config mode. 
 * To change the XpressNet device address press button 0..9 set set a new addres. By default the XPressNet address is 30.
 * Press STOP button to toggle between power off and emergency mode.
 * Press +/- button to toggle between single step or continously update speed when +/- button is pressed. By default STEP is set.
 * Press F4+ / F8+ button to disable and enable the AC option. When the AC option is emabled the locomotive won't change direction when the actual speed is 0.
 * To exit press F4, when a change was done to the settings an automatic reset will be performed to activate the new settings.  
 ![](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/config.JPG)

## Tested with
 * [MDRRC-II (Lite)](https://robertdotevers.wordpress.com/). 
 * [Roco MultiMaus as master](https://www.roco.cc/en/product/5215-multimaus-0-0-0-0-0-004001-0/products.html). Only a quick test, needs more extensive testing...
 
## Connections

Below table with connections and here ![pdf of schematic](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/rx5xpressnetschematic.pdf)

| Ardiuno Mega  | Device         |
| ------------  | -------------- |
| IO 2          | TSOP4836 pin 1 |
| SCA 20        | SSD1306 SCA    |
| SCL 21        | SSD1306 SCL    |
| RXD1 19       | MAX485 pin 1   |
| TXD1 18       | MAX485 pin 4   |
| IO   22       | MAX485 pin 2/3 |

And of course put the VCC and GND of the SSD1306 / MAX485 / TSOP4836 to 3V3 of the Arduino Mega 2560 board (5V is also OK).

Below the PCB. The display and TSOP4836 IR receiver are mounted on the solder side! 

![](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/pcb_comp_side.JPG)

![](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/pcb_solder_side.JPG)

If your are interested in a PCB sent me a mail, PCB cost 4.50 Euro including post and package for the Netherland and 6.00 for EU countries (including post and package). 

## Used library's
If you want to build the code yourself or update / change the code following library's are required.
 * [Rc5](https://github.com/guyc/RC5) for IR RC5 receive.
 * [StateMachine](https://github.com/MDRRC/StateMachine) State machine forked from ([jrullan](https://github.com/jrullan/StateMachine))
 * [LinkedList for StateMachine](https://github.com/ivanseidel/LinkedList) for the [StateMachine](https://github.com/MDRRC/StateMachine). Probably you need to remove the test.cpp to avoid build errors.  
 * [XpressNet Client Library](http://pgahtow.de/wiki/index.php?title=XpressNet) for XpressNet protocol handling.
 * [Adafruit-SSD1306-Library](https://github.com/adafruit/Adafruit_SSD1306) to control the display.
 * [Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library) for graphic control of the display.
 
## Programming without building the code by yourself on Windows
Although the Arduino environment is easy in use, adding libraries and building / selecting the correct board might be difficult when someone starts with Arduino. So this part describes how to flash the Arduino Mega 2560 (Pro Embed) without building the code yourself.

 * First download and install the Arduino IDE.
 * Download the ZIP file from this project and unzip it.
 * Open flash.bat with a text editor and correct if required the location of the Arduino executable path and the comport. 
 * Save and close the file.
 * Double click on the batch file. At the end of the flash cycle a screen as shown below should be visible, if a lot of errors are shown check the avrdude location and / or com port. Also check if the "  are at the right locations. 
 ![](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/flashbatexample.JPG)
 * The Mega2560 should now be programmed and when ready the start screen appears on the display.

