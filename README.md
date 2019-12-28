# This repo is outdated. For the most recent code visit
https://github.com/sivanovbg/MQTT-SN_802.15.4_DeviceT1_THR_RLP
# MQTT-SN 802.15.4 Net
# End device Type 1 THR XLP (Low power)

An MQTT-SN Client implementation based on MRF24J40 and Arduino

Visit project website at https://sites.google.com/view/hobbyiot/projects/mqtt-sn-802-15-4

Twitter: @sivanovbg

This is the End device Type 1 THR Extra Low power Arduino code and shcematics. When sleeps it only consumes 4 uA from the power source (measured at VCC = 3.3V). Both DHT11 and MRF24J40MA are powered by Arduino I/Os and are being switched on only when needed.

Type 1 stands for Sensor node and THR means Temperature, Humidity and Reed sensors within.

The implementation is based ot MQTT-SN Specification Version 1.2. Arduino board used is Arduino Pro Mini @ 3.3 V. The 3.3V regulator has to be removed or disconnected as shown on the schematic.

Topics should be predefined on the client and gateway sides and are fixed to 2 positions alpha-numeric string.

Messages supported:

CONNECT

CONNACK

PINGREQ

PINGRESP

PUBLISH
