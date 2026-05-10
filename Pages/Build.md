# Build
## Start of Journy
  We extend our gratitude to Professor Michael Zavlanos for generously providing us with the F1 Tenth prototype he crafted in 2020. This valuable machine came into our possession through the kind support of Professor Georgie Delagrammatikas in September 2023, marking the inception of Duke University's inaugural F1 Tenth team.
![The First Look of Our Car](/Images/First%20Look.jpg)
Since we didn't build the car from sketch, if you start from sketch, please read [F1 Tenth Official website](https://f1tenth.org/build.html#), or for other problems.

## Initial Condition
  However, the car was not in a runnable condition.   
1. Powerboard only has 1 output  
2. TX2 USB ports didn't work  
3. TX2 was using Ubuntu 18.04 and using unknown password  
4. Missing connector and adaptor for battery  
  We write this down to remind future project users if anything anything doesn't work exclude these, it could be something we didn't find out.

## Final Structure
  All material except something like bolts should be able to find in our [BOM](/BOM/Master%20BOM.xlsx) 

  Due to the limit budget, we only replace those neccesary components with certain reason  
1. Changed TX2 to NX. Since Ubuntu 18.04 is too old for our laptop and we didn't find the way to fix the usb port, we decided to buy a Jetson Xavier NX8. And we add a 256GB SSD and wifi module by ourselves.  
2. Rebuild 3 powerboards  
3. Add adaptor and connector for battery.  

### VESC
  The ESC is FSESC which is using VESC 6 architecture instead of VESC. This is what we got from Prof. Zavlanos and it works.
  ![FSESC](/Images/ESC.JPG)


### Power Board
  The powerboard has 5V and 12V outputs. We currently using V4. And the V9 files is [here](/powerboardV9) which is provided by Prof. Rahul Mangharam. The functionality doesn't change a lot.
  ![Power Board](/Images/Power%20board.JPG)


### Jetson NX
  And here is the connection on Jetson NX. There are a few spaces for further upgrades. CAUTION: The NX is really powerful which is a mini Linux computer. However it is expensive and perishable. The one we are using has been sent back to factory once since the USB all died. Besides, since the battery output power is smaller than charger, the NX can't run with full power, which is a potential upgrade option.
  ![NX Connections](/Images/NX%20Connections.JPG)  

  Since the developer kit is not longer available, we installed the Wifi modual and SSD manually. Addition SSD provide extra space for us to save our files, due to NX only has 16GB. For Wifi modual, it just need to ealiy screw in. For SSD, after insertion, check it [page](/Pages/SSD.md) if you need to replace a new SSD.
  ![Bottom of Jetson NX](/Images/SSD%20Wifi.jpg)      


### Overview
  Here is a picture to show the main component of our car.
  ![Overview](/Images/Overview.jpg)


## Final Look
And here is the final looks.
![Final Look 1](/Images/Final%20Look%201.JPG)
![Final Look 2](/Images/Final%20look%202.JPG)
