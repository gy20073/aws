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

class TrajectoryPoint2D {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.t = null;
      this.x = null;
      this.y = null;
      this.theta = null;
      this.kappa = null;
      this.kappa_dot = null;
      this.v = null;
      this.a = null;
      this.jerk = null;
      this.delta_theta = null;
      this.d = null;
      this.a_lat = null;
    }
    else {
      if (initObj.hasOwnProperty('t')) {
        this.t = initObj.t
      }
      else {
        this.t = 0.0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('theta')) {
        this.theta = initObj.theta
      }
      else {
        this.theta = 0.0;
      }
      if (initObj.hasOwnProperty('kappa')) {
        this.kappa = initObj.kappa
      }
      else {
        this.kappa = 0.0;
      }
      if (initObj.hasOwnProperty('kappa_dot')) {
        this.kappa_dot = initObj.kappa_dot
      }
      else {
        this.kappa_dot = 0.0;
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0.0;
      }
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = 0.0;
      }
      if (initObj.hasOwnProperty('jerk')) {
        this.jerk = initObj.jerk
      }
      else {
        this.jerk = 0.0;
      }
      if (initObj.hasOwnProperty('delta_theta')) {
        this.delta_theta = initObj.delta_theta
      }
      else {
        this.delta_theta = 0.0;
      }
      if (initObj.hasOwnProperty('d')) {
        this.d = initObj.d
      }
      else {
        this.d = 0.0;
      }
      if (initObj.hasOwnProperty('a_lat')) {
        this.a_lat = initObj.a_lat
      }
      else {
        this.a_lat = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryPoint2D
    // Serialize message field [t]
    bufferOffset = _serializer.float64(obj.t, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [theta]
    bufferOffset = _serializer.float64(obj.theta, buffer, bufferOffset);
    // Serialize message field [kappa]
    bufferOffset = _serializer.float64(obj.kappa, buffer, bufferOffset);
    // Serialize message field [kappa_dot]
    bufferOffset = _serializer.float64(obj.kappa_dot, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = _serializer.float64(obj.v, buffer, bufferOffset);
    // Serialize message field [a]
    bufferOffset = _serializer.float64(obj.a, buffer, bufferOffset);
    // Serialize message field [jerk]
    bufferOffset = _serializer.float64(obj.jerk, buffer, bufferOffset);
    // Serialize message field [delta_theta]
    bufferOffset = _serializer.float64(obj.delta_theta, buffer, bufferOffset);
    // Serialize message field [d]
    bufferOffset = _serializer.float64(obj.d, buffer, bufferOffset);
    // Serialize message field [a_lat]
    bufferOffset = _serializer.float64(obj.a_lat, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryPoint2D
    let len;
    let data = new TrajectoryPoint2D(null);
    // Deserialize message field [t]
    data.t = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta]
    data.theta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [kappa]
    data.kappa = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [kappa_dot]
    data.kappa_dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a]
    data.a = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [jerk]
    data.jerk = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_theta]
    data.delta_theta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d]
    data.d = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a_lat]
    data.a_lat = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'path_follower/TrajectoryPoint2D';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '63ab900fed4c2c35d54c1d98c787e72b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Header header  not sent directly
    float64 t
    float64 x
    float64 y
    float64 theta
    float64 kappa
    float64 kappa_dot
    float64 v
    float64 a
    float64 jerk
    float64 delta_theta     # heading misalignment with center line
    float64 d               # offset to center line
    float64 a_lat           # lateral (to traj not to center line!) acceleration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryPoint2D(null);
    if (msg.t !== undefined) {
      resolved.t = msg.t;
    }
    else {
      resolved.t = 0.0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.theta !== undefined) {
      resolved.theta = msg.theta;
    }
    else {
      resolved.theta = 0.0
    }

    if (msg.kappa !== undefined) {
      resolved.kappa = msg.kappa;
    }
    else {
      resolved.kappa = 0.0
    }

    if (msg.kappa_dot !== undefined) {
      resolved.kappa_dot = msg.kappa_dot;
    }
    else {
      resolved.kappa_dot = 0.0
    }

    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0.0
    }

    if (msg.a !== undefined) {
      resolved.a = msg.a;
    }
    else {
      resolved.a = 0.0
    }

    if (msg.jerk !== undefined) {
      resolved.jerk = msg.jerk;
    }
    else {
      resolved.jerk = 0.0
    }

    if (msg.delta_theta !== undefined) {
      resolved.delta_theta = msg.delta_theta;
    }
    else {
      resolved.delta_theta = 0.0
    }

    if (msg.d !== undefined) {
      resolved.d = msg.d;
    }
    else {
      resolved.d = 0.0
    }

    if (msg.a_lat !== undefined) {
      resolved.a_lat = msg.a_lat;
    }
    else {
      resolved.a_lat = 0.0
    }

    return resolved;
    }
};

module.exports = TrajectoryPoint2D;
