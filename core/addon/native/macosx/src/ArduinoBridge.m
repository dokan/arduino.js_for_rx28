/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <stdio.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/IOBSD.h>
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#include <Foundation/Foundation.h>
#include "ArduinoBridge.h"
#include <unistd.h>

static int device_file_descriptor = -1;
static const int BUFFER_SIZE = 16;

void response(char* buffer) {
    NSLog(@"response");
	char readBuffer[BUFFER_SIZE]; // buffer for holding incoming data
	// this will loop unitl the serial port closes
	while(TRUE) {
		// read() blocks until some data is available or the port is closed
		long numBytes = read(device_file_descriptor, readBuffer, BUFFER_SIZE); // read up to the size of the buffer
		if(numBytes <= 0) {
			break; // Stop the thread if there is an error
        } else if (readBuffer[numBytes-1] == '\r') {
            strncat(buffer, readBuffer, numBytes-1);
            break;
		} else {
            strncat(buffer, readBuffer, numBytes);
		}
	}
}

int invoke(char* command, char* buffer, char** error, long length)
{
    NSLog(@"invoke");
    long commandLength;

    if ((int)length == 0){
        commandLength = strlen(command);
        NSLog(@"command: %s", command);
    }else{
        commandLength = length;
    }

    long wrotenLength = write(device_file_descriptor, command, commandLength);

    if (commandLength != wrotenLength) {
        sprintf(*error, "Error: coudln't write successfully %ld/%ld", wrotenLength, commandLength);
        return -1;
    }

    if ((int)length == 0){
        response(buffer);
    }
    return 0;
}

int openArduino(char* portname, char** error)
{
    if (device_file_descriptor != -1) {
        closeArduino();
    }
    
    NSLog(@"openArduino:[%s]",portname);
    
    const char *bsdPath = portname;
    struct termios gOriginalTTYAttrs;
    speed_t baudRate = 9600;
    unsigned long mics = 3;
    
	 int	serialFileDescriptor = open(bsdPath, O_RDWR | O_NOCTTY | O_NDELAY );
    device_file_descriptor = serialFileDescriptor;
    // TIOCEXCL causes blocking of non-root processes on this serial-port
    if ( ioctl(serialFileDescriptor, TIOCEXCL) == -1) {
        *error = "Error: couldn't obtain lock on serial port";
        NSLog(@"%s", *error);
        return -1;
    }
    if ( fcntl(serialFileDescriptor, F_SETFL, 0) == -1) { 
        // clear the O_NONBLOCK flag; all calls from here on out are blocking for non-root processes
        *error = "Error: couldn't obtain lock on serial port";
        NSLog(@"%s", *error);
        return -1;
    }
    // Get the current options and save them so we can restore the default settings later.
    if ( tcgetattr(serialFileDescriptor, &gOriginalTTYAttrs) == -1) { 
        *error = "Error: couldn't get serial attributes";
        NSLog(@"%s", *error);
        return -1;
    }
    // copy the old termios settings into the current
    //   you want to do this so that you get all the control characters assigned
	struct termios options = gOriginalTTYAttrs;
    /*
     cfmakeraw(&options) is equivilent to:
     options->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
     options->c_oflag &= ~OPOST;
     options->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
     options->c_cflag &= ~(CSIZE | PARENB);
     options->c_cflag |= CS8;
     */
    cfmakeraw(&options);
    
    // set tty attributes (raw-mode in this case)
    if ( tcsetattr(serialFileDescriptor, TCSANOW, &options) == -1) {
        *error = "Error: couldn't get serial attributes";
        NSLog(@"%s", *error);
        return -1;
    }
    // Set baud rate (any arbitrary baud rate can be set this way)
    if ( ioctl(serialFileDescriptor, IOSSIOSPEED, &baudRate) == -1) { 
        *error = "Error: Baud Rate out of bounds";
        NSLog(@"%s", *error);
        return -1;
    }
    // Set the receive latency (a.k.a. don't wait to buffer data)
    if ( ioctl(serialFileDescriptor, IOSSDATALAT, &mics) == -1) { 
        *error = "Error: coudln't set serial latency";
        NSLog(@"%s", *error);
        return -1;
    }

    //wait until Arduino finish the setup
    char buffer[BUFFER_SIZE];
    memset(buffer , '\0' , sizeof(buffer));
    //response(buffer);
    
    return 0;
}

int digitalWrite(int pin, int value, char** error)
{
    char command[16];
    sprintf(command, "d%d,%d\r", pin, value);
    char response[BUFFER_SIZE];
    memset(response , '\0' , sizeof(response));
    int result = invoke(command, response, error, 0);

//    NSLog(@"digitalWrite:");
//    NSLog(@"[%s]", response);

    return result;
}

int digitalRead(int pin, int* valuePointer, char** error)
{
    char command[16];
    sprintf(command, "D%d\r", pin);
    char response[4];
    memset(response , '\0' , sizeof(response));
    int result = invoke(command, response, error, 0);
    
//    NSLog(@"digitalRead:");
//    NSLog(@"[%s]", response);

    *valuePointer = atoi(response);
    
    return result;
}

int analogWrite(int pin, int value, char** error)
{
    char command[16];
    sprintf(command, "a%d,%d\r", pin, value);
    char response[BUFFER_SIZE];
    memset(response , '\0' , sizeof(response));
    int result = invoke(command, response, error, 0);
    
//    NSLog(@"analoglWrite:");
//    NSLog(@"[%s]", response);
    
    return result;
}

int analogRead(int pin, int* valuePointer, char** error)
{
    char command[16];
    sprintf(command, "A%d\r", pin);
    char response[4];
    memset(response , '\0' , sizeof(response));
    int result = invoke(command, response, error, 0);
    
//    NSLog(@"analogRead:");
//    NSLog(@"[%s]", response);
    *valuePointer = atoi(response);
    
    return result;
}

int pulse(int pin, int ontime, int offtime, char** error)
{
    char command[24];
    sprintf(command, "P%d,%d,%d\r", pin, ontime, offtime);
    char response[10];
    memset(response , '\0' , sizeof(response));
    int result = invoke(command, response, error, 0);
//    NSLog(@"analogRead:");
//    NSLog(@"[%s]", response);
    return result;
}

int pinMode(int pin, bool isOutput, char** error)
{
    char command[16];
    sprintf(command, "p%d,%d\r", pin, isOutput == true ? 0 : 1);
    char response[BUFFER_SIZE];
    int result = invoke(command, response, error, 0);
    
//    NSLog(@"pinMode:");
//    NSLog(@"[%s]", response);
    
    return result;
}

int delayMicroseconds(int value, char** error)
{
    char command[16];
    sprintf(command, "w%d\r", value);
    char response[BUFFER_SIZE];
    int result = invoke(command, response, error, 0);
    
//    NSLog(@"delayMicroseconds:");
//    NSLog(@"[%s]", response);
    
    return result;
}

int closeArduino(void)
{
//    NSLog(@"closeArduino");
    close(device_file_descriptor);
    return 0;
}

/* add by dokan for rx-28 */
int commandWrite(int valueA, int valueB, char** error)
{
    float degree = 512.0/150.0;
    NSLog(@"commandWrite");
    move_both(512, 512);
    
    int diff = valueA - valueB;
    
    if (diff > 0){
        msleep(valueB);
        move(2, 60 * degree);
        msleep(abs(diff));
        move(1, 60 * degree);
    }else{
        msleep(valueA);
        move(1, 60 * degree);
        msleep(abs(diff));
        move(2, 60 * degree);
    }
    
    return 0;
}

void msleep(int msec){
    usleep(msec * 1000);
}

/**
 * move both
 * @param angleA Aをangle度にする。
 * @param angleB Bをangle度にする。
 */
void move_both(int angleA, int angleB){
    NSLog(@"move_both %d %d", angleA, angleB);
    int speed = 0;
    
    unsigned char buf[15];
    
    buf[0] = (unsigned char) 0xFE;//ID
    buf[1] = (unsigned char) 14;//length
    buf[2] = (unsigned char) 0x83;//指示
    buf[3] = (unsigned char) 0x1e;//先頭アドレス
    buf[4] = (unsigned char) 0x04;//書込みbyte数
    buf[5] = (unsigned char) 0x01;//
    buf[6] = (unsigned char) (angleA & 0x00FF);
    buf[7] = (unsigned char) ((angleA & 0xFF00) >> 8); // 位置
    buf[8] = (unsigned char) (speed & 0x00FF); // 時間
    buf[9] = (unsigned char) ((speed & 0xFF00) >> 8);
    buf[10] = (unsigned char) 0x02;
    buf[11] = (unsigned char) (angleB & 0x00FF);
    buf[12] = (unsigned char) ((angleB & 0xFF00) >> 8); // 位置
    buf[13] = (unsigned char) (speed & 0x00FF); // 時間
    buf[14] = (unsigned char) ((speed & 0xFF00) >> 8);
    rx28_write(buf);
}

/**
 * move
 * @param mID targetのID。
 * @param angle angle度にする。
 */

void move(int mID, int angle){
    NSLog(@"move target:%02x %d", mID, angle);
    int speed = 0;
    
    unsigned char buf[8];
    
    buf[0] = (unsigned char) (mID & 0x00FF);//ID
    buf[1] = (unsigned char) 7;//length
    buf[2] = (unsigned char) 0x03;//指示
    buf[3] = (unsigned char) 0x1e;//先頭アドレス
    buf[4] = (unsigned char) (angle & 0x00FF);
    buf[5] = (unsigned char) ((angle & 0xFF00) >> 8); // 位置
    buf[6] = (unsigned char) (speed & 0x00FF); // 時間
    buf[7] = (unsigned char) ((speed & 0xFF00) >> 8);
    rx28_write(buf);
}

/*
 for debug
void move(int mID, int angle){
    NSLog(@"move target:%02x %d", mID, angle);
    //int speed = 0;
    
    unsigned char buf[5];
    
    buf[0] = (unsigned char) 0xFE;//ID
    buf[1] = (unsigned char) 4;//length
    buf[2] = (unsigned char) 0x03;//指示
    buf[3] = (unsigned char) 0x03;//先頭アドレス
    buf[4] = (unsigned char) 0x01;//
    rx28_write(buf);
}
*/


/**
 * rx28_write
 * @param p: IDへのポインタ
 **/
void rx28_write(unsigned char* p){
    char** error;
    unsigned char length;
    length = *(p+1);
    
    unsigned char buf[length + 4];
    buf[0] = (unsigned char) 0xFF;//HEADER
    buf[1] = (unsigned char) 0xFF;//HEADER
    
    for(int i = 0;i<length + 2;i++){
        buf[i+2] = *(p + i);
    }
    buf[3 + length] = checksum(&buf[2]);
    
    char response[BUFFER_SIZE];
    memset(response , '\0' , sizeof(response));
    
    for (int i = 0; i<length+4; i++){
        NSLog(@"printbuf buf[%02d]:0x%02x", i, buf[i]);
    }
    
    invoke((char*)buf, response, error, length+4);

}


/**
 * Checksum
 * @params IDが入っている所のアドレス
 * @return checksum
 **/
unsigned char checksum(unsigned char* p){
    unsigned char sum, length;
    
    sum =  *(p++); //read ID
    
    length = *(p++); // read LENGTH
    sum += length;
    
    for(int i = 0; i<length - 1;i++){
        sum += *(p++); //
    }
        
    return ~sum;
}

int commandArduino(int valueA, int valueB, char** error){
    char command[16];
    sprintf(command, "r%d,%d\r", valueA, valueB);
    char response[BUFFER_SIZE];
    memset(response , '\0' , sizeof(response));
    int result = invoke(command, response, error, StrLength(command));
    
    //    NSLog(@"commandArudino:");
    //    NSLog(@"[%s]", response);
    
    return result;
}