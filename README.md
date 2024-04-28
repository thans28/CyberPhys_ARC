ECE 1188 Project 2: Autonomous Racing Challenge

This project can be run directly from code composer, many of the files start from the CC3100BOOST_MQTT-TwitterLED code as that is what was used for lab 6, but some additional files are necessary as well, like UART0.h for BLE, Reflectance.h for line stopping, and other includes originating from the wall follower code.

This code is enabled to control the MSP432 Bot with a Bluetooth Connection, and collect data from the bot using MQTT and a stable Wi-Fi connection. To get it to run, update the SSID_Name to your wifi name and the PASSKEY to the password. Then run our NodeRed flow and open our BLE controller through this link hosted on our GitHub https://thans28.github.io/CyberPhys_ARC/

To start the bot, line it up on a track, turn it on, make sure it is connected to the Wi-Fi and in the while loop to accept commands (indicated by the RED LED on P1). Once this happens, connect the BLE controller to your BLE module (in this code, the BLE module is named "Oday"), select your bias whether left wall following or right, and press go!

As the bot traverses the track, feel free to monitor the status in the MQTT app from our Node-Red Flow and finally, when the bot crosses a black line over white paper, the MQTT will tell you how long the bot ran for, how many collisions it had, the maximum RPM of both wheels, and finally a graph showing all three distance sensors through out the race.

Enjoy!
