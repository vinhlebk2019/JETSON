// Auto-generated. Do not edit!

// (in-package aev_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class gui_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.msg_counter = null;
      this.userReqStart = null;
      this.userReqAutoRun = null;
      this.userReqStop = null;
      this.clearError = null;
      this.speedSetpoint = null;
      this.turnSignal = null;
      this.horn = null;
      this.frontLight = null;
      this.steeringLeftRight = null;
    }
    else {
      if (initObj.hasOwnProperty('msg_counter')) {
        this.msg_counter = initObj.msg_counter
      }
      else {
        this.msg_counter = 0;
      }
      if (initObj.hasOwnProperty('userReqStart')) {
        this.userReqStart = initObj.userReqStart
      }
      else {
        this.userReqStart = false;
      }
      if (initObj.hasOwnProperty('userReqAutoRun')) {
        this.userReqAutoRun = initObj.userReqAutoRun
      }
      else {
        this.userReqAutoRun = false;
      }
      if (initObj.hasOwnProperty('userReqStop')) {
        this.userReqStop = initObj.userReqStop
      }
      else {
        this.userReqStop = false;
      }
      if (initObj.hasOwnProperty('clearError')) {
        this.clearError = initObj.clearError
      }
      else {
        this.clearError = false;
      }
      if (initObj.hasOwnProperty('speedSetpoint')) {
        this.speedSetpoint = initObj.speedSetpoint
      }
      else {
        this.speedSetpoint = 0;
      }
      if (initObj.hasOwnProperty('turnSignal')) {
        this.turnSignal = initObj.turnSignal
      }
      else {
        this.turnSignal = 0;
      }
      if (initObj.hasOwnProperty('horn')) {
        this.horn = initObj.horn
      }
      else {
        this.horn = false;
      }
      if (initObj.hasOwnProperty('frontLight')) {
        this.frontLight = initObj.frontLight
      }
      else {
        this.frontLight = false;
      }
      if (initObj.hasOwnProperty('steeringLeftRight')) {
        this.steeringLeftRight = initObj.steeringLeftRight
      }
      else {
        this.steeringLeftRight = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gui_msg
    // Serialize message field [msg_counter]
    bufferOffset = _serializer.uint32(obj.msg_counter, buffer, bufferOffset);
    // Serialize message field [userReqStart]
    bufferOffset = _serializer.bool(obj.userReqStart, buffer, bufferOffset);
    // Serialize message field [userReqAutoRun]
    bufferOffset = _serializer.bool(obj.userReqAutoRun, buffer, bufferOffset);
    // Serialize message field [userReqStop]
    bufferOffset = _serializer.bool(obj.userReqStop, buffer, bufferOffset);
    // Serialize message field [clearError]
    bufferOffset = _serializer.bool(obj.clearError, buffer, bufferOffset);
    // Serialize message field [speedSetpoint]
    bufferOffset = _serializer.int16(obj.speedSetpoint, buffer, bufferOffset);
    // Serialize message field [turnSignal]
    bufferOffset = _serializer.uint8(obj.turnSignal, buffer, bufferOffset);
    // Serialize message field [horn]
    bufferOffset = _serializer.bool(obj.horn, buffer, bufferOffset);
    // Serialize message field [frontLight]
    bufferOffset = _serializer.bool(obj.frontLight, buffer, bufferOffset);
    // Serialize message field [steeringLeftRight]
    bufferOffset = _serializer.uint8(obj.steeringLeftRight, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gui_msg
    let len;
    let data = new gui_msg(null);
    // Deserialize message field [msg_counter]
    data.msg_counter = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [userReqStart]
    data.userReqStart = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [userReqAutoRun]
    data.userReqAutoRun = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [userReqStop]
    data.userReqStop = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [clearError]
    data.clearError = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [speedSetpoint]
    data.speedSetpoint = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [turnSignal]
    data.turnSignal = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [horn]
    data.horn = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [frontLight]
    data.frontLight = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [steeringLeftRight]
    data.steeringLeftRight = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aev_pkg/gui_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0288aa76c680cc5123702f434a849fe7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 	msg_counter
    bool 	userReqStart
    bool 	userReqAutoRun
    bool 	userReqStop
    bool 	clearError
    int16 	speedSetpoint
    uint8 	turnSignal
    bool 	horn
    bool 	frontLight
    uint8 steeringLeftRight
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gui_msg(null);
    if (msg.msg_counter !== undefined) {
      resolved.msg_counter = msg.msg_counter;
    }
    else {
      resolved.msg_counter = 0
    }

    if (msg.userReqStart !== undefined) {
      resolved.userReqStart = msg.userReqStart;
    }
    else {
      resolved.userReqStart = false
    }

    if (msg.userReqAutoRun !== undefined) {
      resolved.userReqAutoRun = msg.userReqAutoRun;
    }
    else {
      resolved.userReqAutoRun = false
    }

    if (msg.userReqStop !== undefined) {
      resolved.userReqStop = msg.userReqStop;
    }
    else {
      resolved.userReqStop = false
    }

    if (msg.clearError !== undefined) {
      resolved.clearError = msg.clearError;
    }
    else {
      resolved.clearError = false
    }

    if (msg.speedSetpoint !== undefined) {
      resolved.speedSetpoint = msg.speedSetpoint;
    }
    else {
      resolved.speedSetpoint = 0
    }

    if (msg.turnSignal !== undefined) {
      resolved.turnSignal = msg.turnSignal;
    }
    else {
      resolved.turnSignal = 0
    }

    if (msg.horn !== undefined) {
      resolved.horn = msg.horn;
    }
    else {
      resolved.horn = false
    }

    if (msg.frontLight !== undefined) {
      resolved.frontLight = msg.frontLight;
    }
    else {
      resolved.frontLight = false
    }

    if (msg.steeringLeftRight !== undefined) {
      resolved.steeringLeftRight = msg.steeringLeftRight;
    }
    else {
      resolved.steeringLeftRight = 0
    }

    return resolved;
    }
};

module.exports = gui_msg;
