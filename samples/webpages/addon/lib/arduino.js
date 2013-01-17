/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

const {Cu, Ci, Cc} = require("chrome");
const { ctypes } = Cu.import("resource://gre/modules/ctypes.jsm");
const self = require("self");
const url = require("url");

const dylibURL = require("self").data.url(ctypes.libraryName('ArduinoBridge'));
const dylibPATH = url.toFilename(dylibURL).toString();
const dylib = ctypes.open(dylibPATH);  

var openArduino = dylib.declare("openArduino",       /* function name */  
                          ctypes.default_abi,    /* ABI type */  
                          ctypes.int32_t,       /* return type */  
                          ctypes.char.ptr, /* parameter */
                          new ctypes.PointerType(ctypes.char.ptr)
                          );
                          
var pinMode = dylib.declare("pinMode",
                          ctypes.default_abi,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          ctypes.bool,
                          new ctypes.PointerType(ctypes.char.ptr)
                          );

var digitalWrite = dylib.declare("digitalWrite",
                          ctypes.default_abi,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          new ctypes.PointerType(ctypes.char.ptr)
                          );

var digitalRead = dylib.declare("digitalRead",
                          ctypes.default_abi,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          new ctypes.PointerType(ctypes.int32_t),
                          new ctypes.PointerType(ctypes.char.ptr)
                          );
                          
var analogWrite = dylib.declare("analogWrite",
                          ctypes.default_abi,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          new ctypes.PointerType(ctypes.char.ptr)
                          );

var analogRead = dylib.declare("analogRead",
                          ctypes.default_abi,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          new ctypes.PointerType(ctypes.int32_t),
                          new ctypes.PointerType(ctypes.char.ptr)
                          );

var pulse = dylib.declare("pulse",
                          ctypes.default_abi,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          new ctypes.PointerType(ctypes.char.ptr)
                          );

var delayMicroseconds = dylib.declare("delayMicroseconds",
                          ctypes.default_abi,
                          ctypes.int32_t,
                          ctypes.int32_t,
                          new ctypes.PointerType(ctypes.char.ptr)
                          );

var closeArduino = dylib.declare("closeArduino",
                          ctypes.default_abi,
                          ctypes.int32_t
                          );

/* added by dokan for rx-28 */
//直接通信
var commandWrite = dylib.declare("commandWrite",
				 ctypes.default_abi,
				 ctypes.int32_t,
				 ctypes.int32_t,
				 ctypes.int32_t,
				 new ctypes.PointerType(ctypes.char.ptr)
				);

//arduinoに同じ値を渡すだけ。
var commandArduino = dylib.declare("commandArduino",
				 ctypes.default_abi,
				 ctypes.int32_t,
				 ctypes.int32_t,
				 ctypes.int32_t,
				 new ctypes.PointerType(ctypes.char.ptr)
				);



exports.open = function(portname) {
    var err = ctypes.char.ptr();
    if ( -1 == openArduino(portname, err.address()) ) {
        throw err.readString(); 
    }
};

exports.pinMode = function(pin, isOutputMode) {
    var err = ctypes.char.ptr();
    if ( -1 == pinMode(pin, isOutputMode, err.address()) ) {
        throw err.readString(); 
    }
};

exports.digitalWrite = function(pin, value) {
    var err = ctypes.char.ptr();
    if ( -1 == digitalWrite(pin, value, err.address()) ) {
        throw err.readString(); 
    }
};

exports.digitalRead = function(pin) {
    var value = ctypes.int32_t(-1);
    var err = ctypes.char.ptr();
    if ( -1 == digitalRead(pin, value.address(), err.address()) ) {
        throw err.readString(); 
    }
    return value.address().contents;
};

exports.analogWrite = function(pin, value) {
    var err = ctypes.char.ptr();
    if ( -1 == analogWrite(pin, value, err.address()) ) {
        throw err.readString(); 
    }
};

exports.analogRead = function(pin) {
    var value = ctypes.int32_t(-1);
    var err = ctypes.char.ptr();
    if ( -1 == analogRead(pin, value.address(), err.address()) ) {
        throw err.readString(); 
    }
    return value.address().contents;
};

exports.pulse = function(pin, ontime, offtime) {
    var err = ctypes.char.ptr();
    if ( -1 == pulse(pin, ontime, offtime, err.address()) ) {
        throw err.readString(); 
    }
};

exports.delayMicroseconds = function(value) {
    var err = ctypes.char.ptr();
    if ( -1 == delayMicroseconds(value, err.address()) ) {
        throw err.readString(); 
    }
};

exports.close = function() {
    closeArduino();
    //dylib.close();
};

/* add by dokan for rx-28*/
//直接通信
exports.commandWrite = function(valueA, valueB) {
    var err = ctypes.char.ptr();
    if ( -1 == commandWrite(valueA, valueB, err.address()) ) {
        throw err.readString(); 
    }
};

//arduinoに同じ値を渡すだけ。
exports.commandArduino = function(valueA, valueB) {
    var err = ctypes.char.ptr();
    if ( -1 == commandArduino(valueA, valueB, err.address()) ) {
        throw err.readString(); 
    }
};

//from Firefox 17 __exposedProps__
exports.__exposedProps__ = {
    open: "r",
    pinMode: "r",
    digitalWrite: "r",
    digitalRead: "r",
    analogWrite: "r",
    analogRead: "r",
    pulse: "r",
    delayMicroseconds: "r",
    close: "r",
    commandWrite: "r",
    commandArduino: "r"
};

