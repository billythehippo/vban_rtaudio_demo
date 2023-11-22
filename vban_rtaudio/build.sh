#!/bin/bash
g++ `pkg-config --cflags --libs rtaudio` main.cpp udpsocket.cpp ringbuffer.cpp -lrtaudio -lpthread -lm -o0 -o vban_rtaudio
