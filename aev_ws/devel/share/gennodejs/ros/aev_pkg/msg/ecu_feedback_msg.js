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

class ecu_feedback_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.msg_counter = null;
      this.feedbackSpeed_b1 = null;
      this.feedbackSpeed_b2 = null;
      this.feedbackSpeed_b3 = null;
      this.feedbackSpeed_b4 = null;
      this.acceleratorLevel = null;
      this.acceleratorSwitch = null;
      this.brakeSwitch = null;
      this.movingDirection = null;
      this.turnSignal = null;
      this.horn = null;
      this.frontLight = null;
    }
    else {
      if (initObj.hasOwnProperty('msg_counter')) {
        this.msg_counter = initObj.msg_counter
      }
      else {
        this.msg_counter = 0;
      }
      if (initObj.hasOwnProperty('feedbackSpeed_b1')) {
        this.feedbackSpeed_b1 = initObj.feedbackSpeed_b1
      }
      else {
        this.feedbackSpeed_b1 = 0;
      }
      if (initObj.hasOwnProperty('feedbackSpeed_b2')) {
        this.feedbackSpeed_b2 = initObj.feedbackSpeed_b2
      }
      else {
        this.feedbackSpeed_b2 = 0;
      }
      if (initObj.hasOwnProperty('feedbackSpeed_b3')) {
        this.feedbackSpeed_b3 = initObj.feedbackSpeed_b3
      }
      else {
        this.feedbackSpeed_b3 = 0;
      }
      if (initObj.hasOwnProperty('feedbackSpeed_b4')) {
        this.feedbackSpeed_b4 = initObj.feedbackSpeed_b4
      }
      else {
        this.feedbackSpeed_b4 = 0;
      }
      if (initObj.hasOwnProperty('acceleratorLevel')) {
        this.acceleratorLevel = initObj.acceleratorLevel
      }
      else {
        this.acceleratorLevel = 0;
      }
      if (initObj.hasOwnProperty('acceleratorSwitch')) {
        this.acceleratorSwitch = initObj.acceleratorSwitch
      }
      else {
        this.acceleratorSwitch = false;
      }
      if (initObj.hasOwnProperty('brakeSwitch')) {
        this.brakeSwitch = initObj.brakeSwitch
      }
      else {
        this.brakeSwitch = false;
      }
      if (initObj.hasOwnProperty('movingDirection')) {
        this.movingDirection = initObj.movingDirection
      }
      else {
        this.movingDirection = false;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ecu_feedback_msg
    // Serialize message field [msg_counter]
    bufferOffset = _serializer.uint32(obj.msg_counter, buffer, bufferOffset);
    // Serialize message field [feedbackSpeed_b1]
    bufferOffset = _serializer.uint8(obj.feedbackSpeed_b1, buffer, bufferOffset);
    // Serialize message field [feedbackSpeed_b2]
    bufferOffset = _serializer.uint8(obj.feedbackSpeed_b2, buffer, bufferOffset);
    // Serialize message field [feedbackSpeed_b3]
    bufferOffset = _serializer.uint8(obj.feedbackSpeed_b3, buffer, bufferOffset);
    // Serialize message field [feedbackSpeed_b4]
    bufferOffset = _serializer.uint8(obj.feedbackSpeed_b4, buffer, bufferOffset);
    // Serialize message field [acceleratorLevel]
    bufferOffset = _serializer.uint8(obj.acceleratorLevel, buffer, bufferOffset);
    // Serialize message field [acceleratorSwitch]
    bufferOffset = _serializer.bool(obj.acceleratorSwitch, buffer, bufferOffset);
    // Serialize message field [brakeSwitch]
    bufferOffset = _serializer.bool(obj.brakeSwitch, buffer, bufferOffset);
    // Serialize message field [movingDirection]
    bufferOffset = _serializer.bool(obj.movingDirection, buffer, bufferOffset);
    // Serialize message field [turnSignal]
    bufferOffset = _serializer.uint8(obj.turnSignal, buffer, bufferOffset);
    // Serialize message field [horn]
    bufferOffset = _serializer.bool(obj.horn, buffer, bufferOffset);
    // Serialize message field [frontLight]
    bufferOffset = _serializer.bool(obj.frontLight, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ecu_feedback_msg
    let len;
    let data = new ecu_feedback_msg(null);
    // Deserialize message field [msg_counter]
    data.msg_counter = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [feedbackSpeed_b1]
    data.feedbackSpeed_b1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [feedbackSpeed_b2]
    data.feedbackSpeed_b2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [feedbackSpeed_b3]
    data.feedbackSpeed_b3 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [feedbackSpeed_b4]
    data.feedbackSpeed_b4 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [acceleratorLevel]
    data.acceleratorLevel = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [acceleratorSwitch]
    data.acceleratorSwitch = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [brakeSwitch]
    data.brakeSwitch = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [movingDirection]
    data.movingDirection = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [turnSignal]
    data.turnSignal = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [horn]
    data.horn = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [frontLight]
    data.frontLight = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aev_pkg/ecu_feedback_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4501f6c5918ccf5a041a7524a20b1561';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 	msg_counter
    uint8 feedbackSpeed_b1
    uint8 feedbackSpeed_b2
    uint8 feedbackSpeed_b3 
    uint8 feedbackSpeed_b4 
    uint8 	acceleratorLevel
    bool acceleratorSwitch
    bool brakeSwitch
    bool movingDirection
    uint8 	turnSignal
    bool 	horn
    bool 	frontLight
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ecu_feedback_msg(null);
    if (msg.msg_counter !== undefined) {
      resolved.msg_counter = msg.msg_counter;
    }
    else {
      resolved.msg_counter = 0
    }

    if (msg.feedbackSpeed_b1 !== undefined) {
      resolved.feedbackSpeed_b1 = msg.feedbackSpeed_b1;
    }
    else {
      resolved.feedbackSpeed_b1 = 0
    }

    if (msg.feedbackSpeed_b2 !== undefined) {
      resolved.feedbackSpeed_b2 = msg.feedbackSpeed_b2;
    }
    else {
      resolved.feedbackSpeed_b2 = 0
    }

    if (msg.feedbackSpeed_b3 !== undefined) {
      resolved.feedbackSpeed_b3 = msg.feedbackSpeed_b3;
    }
    else {
      resolved.feedbackSpeed_b3 = 0
    }

    if (msg.feedbackSpeed_b4 !== undefined) {
      resolved.feedbackSpeed_b4 = msg.feedbackSpeed_b4;
    }
    else {
      resolved.feedbackSpeed_b4 = 0
    }

    if (msg.acceleratorLevel !== undefined) {
      resolved.acceleratorLevel = msg.acceleratorLevel;
    }
    else {
      resolved.acceleratorLevel = 0
    }

    if (msg.acceleratorSwitch !== undefined) {
      resolved.acceleratorSwitch = msg.acceleratorSwitch;
    }
    else {
      resolved.acceleratorSwitch = false
    }

    if (msg.brakeSwitch !== undefined) {
      resolved.brakeSwitch = msg.brakeSwitch;
    }
    else {
      resolved.brakeSwitch = false
    }

    if (msg.movingDirection !== undefined) {
      resolved.movingDirection = msg.movingDirection;
    }
    else {
      resolved.movingDirection = false
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

    return resolved;
    }
};

module.exports = ecu_feedback_msg;
