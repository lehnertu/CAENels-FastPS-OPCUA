# CAENels-FastPS-OPCUA

OPC UA server running on the CAENels FAST-PS
============================================

This is an OPC UA server running on the CAENels
[FAST-PS](http://www.caenels.com/products/fast-ps/) power supplies
for remote control of the devices.

It is based on the [Open62541](https://github.com/open62541/open62541/)
open source implementation of OPC UA.

Functionality
=============
- Provides an OPC-UA server at TCP/IP port 16664.
- Access to device data is handled via the provided TCP server (port 10001).
- Server configuration is loadad from file /etc/opcua.xml

Project status
==============
The server compiles and runs stabily on the power supply used for the tests.
All functionality necessary to user the supllies to power corrector coils
in an accelerator control system environment is provided. This does not
cover the whole functionality provided by the devices, only the essentials.

Build the server
================
The server is built with a cross-compiler running on a Linux system
for the ARM target CPU of the power supplies.

The OPC UA stack needds to be downloaded and built. This can be done on
the development system - there is no binary code produced at this stage.
The complete stack is obtained in two (amalgamated) files.
- open62541.h
- open62541.c

In addition the libxml2 library is required. It needs to be built
and installed into the cross-target tool chain.

A makefile is not yet provided, just a few lines are required to build the server.
- source ../tools/environment
- $CC -std=c99 -c open62541.c
- $CC -std=c99 -c -I $SDKTARGETSYSROOT/usr/include/libxml2/ OpcUaServer.c
- $CXX -o opcuaserver OpcUaServer.o open62541.o -lpthread -lxml2

Installation
============
For istallation a few files need to be copied onto the device:
- opcuaserver binary installed in /tmp/ for testing
- opcua.xml configuration file in /etc/
- libxml2.so.2 in /usr/lib/

The server can then be run by executing /tmp/opcuaserver.

