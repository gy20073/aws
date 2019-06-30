// Auto-generated. Do not edit!

// (in-package path_follower.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Uout {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.acceleration = null;
      this.del = null;
    }
    else {
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = 0.0;
      }
      if (initObj.hasOwnProperty('del')) {
        this.del = initObj.del
      }
      else {
        this.del = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Uout
    // Serialize message field [acceleration]
    bufferOffset = _serializer.float64(obj.acceleration, buffer, bufferOffset);
    // Serialize message field [del]
    bufferOffset = _serializer.float64(obj.del, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Uout
    let len;
    let data = new Uout(null);
    // Deserialize message field [acceleration]
    data.acceleration = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [del]
    data.del = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'path_follower/Uout';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '73950a8bce4b9e8951518131a3cbe7f8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 acceleration
    float64 del
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Uout(null);
    if (msg.acceleration !== undefined) {
      resolved.acceleration = msg.acceleration;
    }
    else {
      resolved.acceleration = 0.0
    }

    if (msg.del !== undefined) {
      resolved.del = msg.del;
    }
    else {
      resolved.del = 0.0
    }

    return resolved;
    }
};

module.exports = Uout;
