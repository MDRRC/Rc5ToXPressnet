call "C:\Program Files (x86)\Arduino\hardware\tools\avr/bin/avrdude.exe" "-CC:\Program Files (x86)\Arduino\hardware\tools\avr/etc/avrdude.conf" -v -patmega2560 -cwiring -PCOM14 -b115200 -D -Uflash:w:Rc5XpNet.hex:i
pause