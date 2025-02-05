# SlacklineLab
BLE Sense code for slackline lab

Uses a BLE Sense as the peripheral. Sends 6DOF IMU data in bits. 
IMU sample rate determines BLE send rate (this should change in the future).

Central is intended to be another nano (BLE Sense or BLE IoT Nano) connected via Serial to a computer. Long term goal: use computer BLEand Python for more direct comms.

ToDo:
  - Complete central sketch
  - Test comms
  - Test wireless (batt power)
  - Add batteryMonitor service.
  - Add CSV write/storage?
