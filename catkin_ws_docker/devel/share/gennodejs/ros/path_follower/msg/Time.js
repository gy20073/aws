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

class Time {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.t_start = null;
      this.t_end = null;
      this.mpc_start = null;
      this.t1 = null;
      this.t2 = null;
      this.t3 = null;
      this.t4 = null;
      this.t5 = null;
      this.t6 = null;
      this.t7 = null;
      this.t8 = null;
      this.t9 = null;
      this.solver_start = null;
      this.solver_end = null;
      this.mpc_end = null;
    }
    else {
      if (initObj.hasOwnProperty('t_start')) {
        this.t_start = initObj.t_start
      }
      else {
        this.t_start = 0.0;
      }
      if (initObj.hasOwnProperty('t_end')) {
        this.t_end = initObj.t_end
      }
      else {
        this.t_end = 0.0;
      }
      if (initObj.hasOwnProperty('mpc_start')) {
        this.mpc_start = initObj.mpc_start
      }
      else {
        this.mpc_start = 0.0;
      }
      if (initObj.hasOwnProperty('t1')) {
        this.t1 = initObj.t1
      }
      else {
        this.t1 = 0.0;
      }
      if (initObj.hasOwnProperty('t2')) {
        this.t2 = initObj.t2
      }
      else {
        this.t2 = 0.0;
      }
      if (initObj.hasOwnProperty('t3')) {
        this.t3 = initObj.t3
      }
      else {
        this.t3 = 0.0;
      }
      if (initObj.hasOwnProperty('t4')) {
        this.t4 = initObj.t4
      }
      else {
        this.t4 = 0.0;
      }
      if (initObj.hasOwnProperty('t5')) {
        this.t5 = initObj.t5
      }
      else {
        this.t5 = 0.0;
      }
      if (initObj.hasOwnProperty('t6')) {
        this.t6 = initObj.t6
      }
      else {
        this.t6 = 0.0;
      }
      if (initObj.hasOwnProperty('t7')) {
        this.t7 = initObj.t7
      }
      else {
        this.t7 = 0.0;
      }
      if (initObj.hasOwnProperty('t8')) {
        this.t8 = initObj.t8
      }
      else {
        this.t8 = 0.0;
      }
      if (initObj.hasOwnProperty('t9')) {
        this.t9 = initObj.t9
      }
      else {
        this.t9 = 0.0;
      }
      if (initObj.hasOwnProperty('solver_start')) {
        this.solver_start = initObj.solver_start
      }
      else {
        this.solver_start = 0.0;
      }
      if (initObj.hasOwnProperty('solver_end')) {
        this.solver_end = initObj.solver_end
      }
      else {
        this.solver_end = 0.0;
      }
      if (initObj.hasOwnProperty('mpc_end')) {
        this.mpc_end = initObj.mpc_end
      }
      else {
        this.mpc_end = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Time
    // Serialize message field [t_start]
    bufferOffset = _serializer.float64(obj.t_start, buffer, bufferOffset);
    // Serialize message field [t_end]
    bufferOffset = _serializer.float64(obj.t_end, buffer, bufferOffset);
    // Serialize message field [mpc_start]
    bufferOffset = _serializer.float64(obj.mpc_start, buffer, bufferOffset);
    // Serialize message field [t1]
    bufferOffset = _serializer.float64(obj.t1, buffer, bufferOffset);
    // Serialize message field [t2]
    bufferOffset = _serializer.float64(obj.t2, buffer, bufferOffset);
    // Serialize message field [t3]
    bufferOffset = _serializer.float64(obj.t3, buffer, bufferOffset);
    // Serialize message field [t4]
    bufferOffset = _serializer.float64(obj.t4, buffer, bufferOffset);
    // Serialize message field [t5]
    bufferOffset = _serializer.float64(obj.t5, buffer, bufferOffset);
    // Serialize message field [t6]
    bufferOffset = _serializer.float64(obj.t6, buffer, bufferOffset);
    // Serialize message field [t7]
    bufferOffset = _serializer.float64(obj.t7, buffer, bufferOffset);
    // Serialize message field [t8]
    bufferOffset = _serializer.float64(obj.t8, buffer, bufferOffset);
    // Serialize message field [t9]
    bufferOffset = _serializer.float64(obj.t9, buffer, bufferOffset);
    // Serialize message field [solver_start]
    bufferOffset = _serializer.float64(obj.solver_start, buffer, bufferOffset);
    // Serialize message field [solver_end]
    bufferOffset = _serializer.float64(obj.solver_end, buffer, bufferOffset);
    // Serialize message field [mpc_end]
    bufferOffset = _serializer.float64(obj.mpc_end, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Time
    let len;
    let data = new Time(null);
    // Deserialize message field [t_start]
    data.t_start = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t_end]
    data.t_end = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mpc_start]
    data.mpc_start = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t1]
    data.t1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t2]
    data.t2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t3]
    data.t3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t4]
    data.t4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t5]
    data.t5 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t6]
    data.t6 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t7]
    data.t7 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t8]
    data.t8 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t9]
    data.t9 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [solver_start]
    data.solver_start = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [solver_end]
    data.solver_end = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mpc_end]
    data.mpc_end = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 120;
  }

  static datatype() {
    // Returns string type for a message object
    return 'path_follower/Time';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0f100a1b959faa915aef817b8b8f13a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 t_start
    float64 t_end
    float64 mpc_start
    float64 t1
    float64 t2
    float64 t3
    float64 t4
    float64 t5
    float64 t6
    float64 t7
    float64 t8
    float64 t9
    float64 solver_start
    float64 solver_end
    float64 mpc_end
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Time(null);
    if (msg.t_start !== undefined) {
      resolved.t_start = msg.t_start;
    }
    else {
      resolved.t_start = 0.0
    }

    if (msg.t_end !== undefined) {
      resolved.t_end = msg.t_end;
    }
    else {
      resolved.t_end = 0.0
    }

    if (msg.mpc_start !== undefined) {
      resolved.mpc_start = msg.mpc_start;
    }
    else {
      resolved.mpc_start = 0.0
    }

    if (msg.t1 !== undefined) {
      resolved.t1 = msg.t1;
    }
    else {
      resolved.t1 = 0.0
    }

    if (msg.t2 !== undefined) {
      resolved.t2 = msg.t2;
    }
    else {
      resolved.t2 = 0.0
    }

    if (msg.t3 !== undefined) {
      resolved.t3 = msg.t3;
    }
    else {
      resolved.t3 = 0.0
    }

    if (msg.t4 !== undefined) {
      resolved.t4 = msg.t4;
    }
    else {
      resolved.t4 = 0.0
    }

    if (msg.t5 !== undefined) {
      resolved.t5 = msg.t5;
    }
    else {
      resolved.t5 = 0.0
    }

    if (msg.t6 !== undefined) {
      resolved.t6 = msg.t6;
    }
    else {
      resolved.t6 = 0.0
    }

    if (msg.t7 !== undefined) {
      resolved.t7 = msg.t7;
    }
    else {
      resolved.t7 = 0.0
    }

    if (msg.t8 !== undefined) {
      resolved.t8 = msg.t8;
    }
    else {
      resolved.t8 = 0.0
    }

    if (msg.t9 !== undefined) {
      resolved.t9 = msg.t9;
    }
    else {
      resolved.t9 = 0.0
    }

    if (msg.solver_start !== undefined) {
      resolved.solver_start = msg.solver_start;
    }
    else {
      resolved.solver_start = 0.0
    }

    if (msg.solver_end !== undefined) {
      resolved.solver_end = msg.solver_end;
    }
    else {
      resolved.solver_end = 0.0
    }

    if (msg.mpc_end !== undefined) {
      resolved.mpc_end = msg.mpc_end;
    }
    else {
      resolved.mpc_end = 0.0
    }

    return resolved;
    }
};

module.exports = Time;
