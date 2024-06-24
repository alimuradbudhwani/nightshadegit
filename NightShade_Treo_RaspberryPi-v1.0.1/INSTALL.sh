#!/bin/sh

# NightShade Treo Raspberry Pi Install Script v1.0

# Uninstall previous installation
rm -rf /usr/include/Treo
rm -f /usr/lib/libNightShade_Treo.so*
rm -f /usr/lib/python3.*/NightShade_Treo.so*

# Install C++ includes and shared library
cp -r ./usr/ /

#Install Python 3 module
cp ./python3/NightShade_Treo.so* /usr/lib/python3.*