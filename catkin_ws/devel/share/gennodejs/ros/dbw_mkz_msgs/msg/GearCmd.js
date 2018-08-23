// Auto-generated. Do not edit!

// (in-package dbw_mkz_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Gear = require('./Gear.js');

//-----------------------------------------------------------

class GearCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = new Gear();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GearCmd
    // Serialize message field [cmd]
    bufferOffset = Gear.serialize(obj.cmd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GearCmd
    let len;
    let data = new GearCmd(null);
    // Deserialize message field [cmd]
    data.cmd = Gear.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dbw_mkz_msgs/GearCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e4d2b0484c21e028e30e3c34f7f76ad3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Gear command enumeration
    Gear cmd
    
    ================================================================================
    MSG: dbw_mkz_msgs/Gear
    uint8 gear
    
    uint8 NONE=0
    uint8 PARK=1
    uint8 REVERSE=2
    uint8 NEUTRAL=3
    uint8 DRIVE=4
    uint8 LOW=5
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GearCmd(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = Gear.Resolve(msg.cmd)
    }
    else {
      resolved.cmd = new Gear()
    }

    return resolved;
    }
};

module.exports = GearCmd;
