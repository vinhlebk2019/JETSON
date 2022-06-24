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

class obstruction_imu_mgs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.msg_counter = null;
      this.obstruction = null;
      this.yaw_rate = null;
    }
    else {
      if (initObj.hasOwnProperty('msg_counter')) {
        this.msg_counter = initObj.msg_counter
      }
      else {
        this.msg_counter = 0;
      }
      if (initObj.hasOwnProperty('obstruction')) {
        this.obstruction = initObj.obstruction
      }
      else {
        this.obstruction = false;
      }
      if (initObj.hasOwnProperty('yaw_rate')) {
        this.yaw_rate = initObj.yaw_rate
      }
      else {
        this.yaw_rate = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type obstruction_imu_mgs
    // Serialize message field [msg_counter]
    bufferOffset = _serializer.uint32(obj.msg_counter, buffer, bufferOffset);
    // Serialize message field [obstruction]
    bufferOffset = _serializer.bool(obj.obstruction, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float64(obj.yaw_rate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type obstruction_imu_mgs
    let len;
    let data = new obstruction_imu_mgs(null);
    // Deserialize message field [msg_counter]
    data.msg_counter = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [obstruction]
    data.obstruction = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aev_pkg/obstruction_imu_mgs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d4c64278182e7c8fb2d6201af193e53';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 	msg_counter
    bool 	obstruction
    float64 yaw_rate
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new obstruction_imu_mgs(null);
    if (msg.msg_counter !== undefined) {
      resolved.msg_counter = msg.msg_counter;
    }
    else {
      resolved.msg_counter = 0
    }

    if (msg.obstruction !== undefined) {
      resolved.obstruction = msg.obstruction;
    }
    else {
      resolved.obstruction = false
    }

    if (msg.yaw_rate !== undefined) {
      resolved.yaw_rate = msg.yaw_rate;
    }
    else {
      resolved.yaw_rate = 0.0
    }

    return resolved;
    }
};

module.exports = obstruction_imu_mgs;
