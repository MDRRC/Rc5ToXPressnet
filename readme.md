# Rc5 To XPressNet converter (Work in progress!)

The Rc5 to XpressNet convertor is an interface between an RC5 remote control and the [XPressnet bus](www.lenzusa.com/1newsite1/Manuals/xpressnet.pdf). 

It is based on: 

 * An Ardiuno Mega 2560 Mini
 * An [Arduino Mega 2560](https://store.arduino.cc/arduino-mega-2560-rev3) will also work but the board is way bigger. 
 * MAX485 RS485 driver IC.
 * 128*64 pixels OLED SSD1306 I2C display. 
 * TSOP4836 IR receiver.
 
The Rc5ToXPressnet is intended to used with for example an [Uhlenbrock Iris infrared control](https://www.uhlenbrock.de/de_DE/produkte/digizen/I63D744D-001.htm!ArcEntryInfo=0004.9.I63D744D). Will also work with other RC5 remote controls for a TV but the layout and commands may e different, so adjustment of the code by yourself may be required.
 
## Functions
 * Control speed, direction and functions (F0..F12) of a locomotive (F5..F12 t.b.a and display of functions also).
 
 ![](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/loccontrol.JPG)
 
 * Select a locomotive on one of the 4 loc selection buttons (for now only one loc can be selected).
 
 ![](https://github.com/MDRRC/Rc5ToXPressnet/blob/master/Doc/select.JPG)
 
 * Control accesory decoders (turn outs for example) (t.b.a.)

## Tested with
 * [MDRRC-II (Lite)](https://robertdotevers.wordpress.com/). 
 * [Roco MultiMaus as master](https://www.roco.cc/en/product/5215-multimaus-0-0-0-0-0-004001-0/products.html). Only a quick test, needs more extensive testing...
 
## Connections

| Ardiuno Mega  | Device         |
| ------------  | -------------- |
| IO 7          | TSOP4836 pin 1 |
| SCA 20        | SSD1306 SCA    |
| SCL 21        | SSD1306 SCL    |
| RXD1 19       | MAX485 pin 1   |
| TXD1 18       | MAX485 pin 4   |
| IO   3        | MAX485 pin 2/3 |

And of course put the VCC and GND of the SSD1306 / MAX485 to 5V and TSOP4836 to 3V3 of the Arduino Mega 2560 board. 

## Used library's
If you want to build the code yourself or update / change the code following library's are required.
 * [Rc5](https://github.com/guyc/RC5) for IR RC5 receive.
 * [StateMachine](https://github.com/jrullan/StateMachine) for the state machine (requires some updates, see below!) 
 * [XpressNet Client Library](http://pgahtow.de/wiki/index.php?title=XpressNet) for XpressNet protocol handling.
 * [Adafruit-SSD1306-Library](https://github.com/adafruit/Adafruit_SSD1306) to control the display.
 * [Adafruit-GFX-Library](https://github.com/adafruit/Adafruit-GFX-Library) for graphic control of the display.

## State machine code update

 * Add code below to class StateMachine
  
```c
  bool transition = false;
```
  
 * Update run as shwon below
 
 ```c
 void StateMachine::run() {
  // Serial.println("StateMachine::run()");
  // Early exit, no states are defined
  if (this->stateList->size() == 0) return;

  // Initial condition
  if (this->currentState == -1) {
    this->currentState = 0;
  }

  // Execute state logic and return transitioned
  // to state number.
  int next = stateList->get(this->currentState)->execute();
  if (this->transition == false) {
    this->executeOnce = (this->currentState == next) ? false : true;
    this->currentState = next;
  } else {
    this->transition = false;
  }
}
```

* Update code as shown below

```c
/*
 * Jump to a state
 * given by a pointer to that state.
 */
State* StateMachine::transitionTo(State* s) {
  this->currentState = s->index;
  this->executeOnce = true;
  this->transition = true;
  return s;
}
```