# MITSUBISHI MELBUS
Tested on mitsubishi Galant IX Rockford Head Unit </br>
* May also work on non Rockford Galant IX, Eclipse and COLT (untested)



### MITSUBISHI CD-changer emulator
Enables audio input on DIN 13pin socket.
Also listen to pressed buttons (prev & next) and sends UART command (P & N)


Parts list:
* Arduino Nano 5V
* Transistor that can handle more than 0,5A (for avoiding phantom power via Melbus pins which are always in high state)
* MC78M12CDTG or similar 10-12V voltage regulator to protect arduino & MELBUS in case of AMS1117 failure
* Resistors:
   * 3x 100Ω
   * 1x 1k
* A rectifier diode (> 0,5A e.g. 1N4004)
* Cables and optionally some connectors for fast removal of device from the car. (not included in schematics)
* Male DIN plug, 13 pin
* 3.5mm audio plug
* Ground loop isolator (in my case BA3121

Some knowledge of electronics, and you need to be good at soldering. You are responsible if things break! Not me! 

Shematics:

![CDC_MMC](https://github.com/user-attachments/assets/76138bf8-55f9-4812-a625-f611af472f64)


Credits to 
* [Visualapproach](https://github.com/visualapproach/Volvo-melbus/tree/master)
* [Karl Hagström](http://gizmosnack.blogspot.se/2015/11/aux-in-volvo-hu-xxxx-radio.html)
* http://volvo.wot.lv/wiki/doku.php?id=melbus.







