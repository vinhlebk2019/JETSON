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

class lane_detection_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.msg_counter = null;
      this.centerOffset = null;
      this.curvature = null;
    }
    else {
      if (initObj.hasOwnProperty('msg_counter')) {
        this.msg_counter = initObj.msg_counter
      }
      else {
        this.msg_counter = 0;
      }
      if (initObj.hasOwnProperty('centerOffset')) {
        this.centerOffset = initObj.centerOffset
      }
      else {
        this.centerOffset = 0.0;
      }
      if (initObj.hasOwnProperty('curvature')) {
        this.curvature = initObj.curvature
      }
      else {
        this.curvature = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lane_detection_msg
    // Serialize message field [msg_counter]
    bufferOffset = _serializer.uint32(obj.msg_counter, buffer, bufferOffset);
    // Serialize message field [centerOffset]
    bufferOffset = _serializer.float32(obj.centerOffset, buffer, bufferOffset);
    // Serialize message field [curvature]
    bufferOffset = _serializer.float32(obj.curvature, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lane_detection_msg
    let len;
    let data = new lane_detection_msg(null);
    // Deserialize message field [msg_counter]
    data.msg_counter = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [centerOffset]
    data.centerOffset = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [curvature]
    data.curvature = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aev_pkg/lane_detection_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d45354acbb17a17b0a237f7d810945c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 	msg_counter
    float32 centerOffset
    float32 curvature
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lane_detection_msg(null);
    if (msg.msg_counter !== undefined) {
      resolved.msg_counter = msg.msg_counter;
    }
    else {
      resolved.msg_counter = 0
    }

    if (msg.centerOffset !== undefined) {
      resolved.centerOffset = msg.centerOffset;
    }
    else {
      resolved.centerOffset = 0.0
    }

    if (msg.curvature !== undefined) {
      resolved.curvature = msg.curvature;
    }
    else {
      resolved.curvature = 0.0
    }

    return resolved;
    }
};

module.exports = lane_detection_msg;
