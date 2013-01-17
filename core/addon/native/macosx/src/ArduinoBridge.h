/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef ArduinoBridge_ArduinoBridge_h
#define ArduinoBridge_ArduinoBridge_h

int openArduino(char* portname, char** error);

int digitalWrite(int pin, int value, char** error);
int digitalRead(int pin, int* valuePointer, char** error);

int analogWrite(int pin, int value, char** error);
int analogRead(int pin, int* valuePointer, char** error);

//max ontime and offtime is 32766. unit is microseconds
int pulse(int pin, int ontime, int offtime, char** error);

int pinMode(int pin, bool isOutput, char** error);

int delayMicroseconds(int value, char** error);

int closeArduino(void);

/* add by dokan for rx-28 */
int commandWrite(int valueA, int valueB, char** error);
int commandArduino(int valueA, int valueB, char** error);
void move_both(int angleA, int angleB);
void move(int mID, int angle);
void msleep(int msec);
void rx28_write(unsigned char* p);
unsigned char checksum(unsigned char* p);

void response(char* buffer);
int invoke(char* command, char* response, char** error, long length);

#endif
