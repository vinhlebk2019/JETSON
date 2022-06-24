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

class system_monitor_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.errorFlag = null;
      this.stopRequestFlag = null;
      this.errorInfo = null;
    }
    else {
      if (initObj.hasOwnProperty('errorFlag')) {
        this.errorFlag = initObj.errorFlag
      }
      else {
        this.errorFlag = false;
      }
      if (initObj.hasOwnProperty('stopRequestFlag')) {
        this.stopRequestFlag = initObj.stopRequestFlag
      }
      else {
        this.stopRequestFlag = false;
      }
      if (initObj.hasOwnProperty('errorInfo')) {
        this.errorInfo = initObj.errorInfo
      }
      else {
        this.errorInfo = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type system_monitor_msg
    // Serialize message field [errorFlag]
    bufferOffset = _serializer.bool(obj.errorFlag, buffer, bufferOffset);
    // Serialize message field [stopRequestFlag]
    bufferOffset = _serializer.bool(obj.stopRequestFlag, buffer, bufferOffset);
    // Serialize message field [errorInfo]
    bufferOffset = _serializer.uint32(obj.errorInfo, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type system_monitor_msg
    let len;
    let data = new system_monitor_msg(null);
    // Deserialize message field [errorFlag]
    data.errorFlag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stopRequestFlag]
    data.stopRequestFlag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [errorInfo]
    data.errorInfo = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aev_pkg/system_monitor_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7fae4553a12c1b5cf670af37bb199a65';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool 	errorFlag
    bool 	stopRequestFlag
    uint32 	errorInfo
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new system_monitor_msg(null);
    if (msg.errorFlag !== undefined) {
      resolved.errorFlag = msg.errorFlag;
    }
    else {
      resolved.errorFlag = false
    }

    if (msg.stopRequestFlag !== undefined) {
      resolved.stopRequestFlag = msg.stopRequestFlag;
    }
    else {
      resolved.stopRequestFlag = false
    }

    if (msg.errorInfo !== undefined) {
      resolved.errorInfo = msg.errorInfo;
    }
    else {
      resolved.errorInfo = 0
    }

    return resolved;
    }
};

module.exports = system_monitor_msg;
