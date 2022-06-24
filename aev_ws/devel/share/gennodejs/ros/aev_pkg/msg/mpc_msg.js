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

class mpc_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.msg_counter = null;
      this.SteeringAngle = null;
    }
    else {
      if (initObj.hasOwnProperty('msg_counter')) {
        this.msg_counter = initObj.msg_counter
      }
      else {
        this.msg_counter = 0;
      }
      if (initObj.hasOwnProperty('SteeringAngle')) {
        this.SteeringAngle = initObj.SteeringAngle
      }
      else {
        this.SteeringAngle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mpc_msg
    // Serialize message field [msg_counter]
    bufferOffset = _serializer.uint32(obj.msg_counter, buffer, bufferOffset);
    // Serialize message field [SteeringAngle]
    bufferOffset = _serializer.float32(obj.SteeringAngle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mpc_msg
    let len;
    let data = new mpc_msg(null);
    // Deserialize message field [msg_counter]
    data.msg_counter = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [SteeringAngle]
    data.SteeringAngle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aev_pkg/mpc_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab2da2dc4dc2caa0d3e85d831457bbdf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 	msg_counter
    float32 SteeringAngle 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mpc_msg(null);
    if (msg.msg_counter !== undefined) {
      resolved.msg_counter = msg.msg_counter;
    }
    else {
      resolved.msg_counter = 0
    }

    if (msg.SteeringAngle !== undefined) {
      resolved.SteeringAngle = msg.SteeringAngle;
    }
    else {
      resolved.SteeringAngle = 0.0
    }

    return resolved;
    }
};

module.exports = mpc_msg;
