temperatureControl
==================

Arduino Temperature control code



You will need to recompile the code- please feel free to add a serial nterface to change the temperature!

Go to temperatureControl directory on pilotfish
cd arduino-nightly
sudo ln -sf /dev/ttyIFTray /dev/ttyUSB0
./arduino
Open the arduino project ~/temperatureControl/temperatureControllers/noiseDiodeTempControlPIDDallas_2Sensor21November2012/noiseDiodeTempControlPIDDallas_2Sensor21November2012.ino

Change the variable setPoint on line 113 to the temperature you want (Ive just checked and set it to 50C)

Set port to /dev/ttyUSB0
Arduino details: Arduino Nano
Chip: ATMega328
Check you can see stuff on the terminal.

Then compile and upload to the arduino
