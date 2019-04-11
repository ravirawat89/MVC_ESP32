ESP-IDF: Multi valve controller using BLE GATT service
=======================================================

This project implements a clock service to maintain time and execute procedures at pre-recorded time points. The execution of the clock and checking alarms is enabled using the internal RTC based system clock.

The user interface is provided using a BLE GATT server. The settings and configuration are loaded using a GATT client running on an Android application.

Multiple solenoid valves can be turned ON/OFF as per pre-recorded time points set through Android application.
