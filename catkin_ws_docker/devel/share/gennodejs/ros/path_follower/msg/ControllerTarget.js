// Auto-generated. Do not edit!

// (in-package path_follower.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ControllerTarget {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.target_velocity = null;
      this.target_steering_angle = null;
      this.cross_track_error = null;
      this.heading_error = null;
      this.timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('target_velocity')) {
        this.target_velocity = initObj.target_velocity
      }
      else {
        this.target_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('target_steering_angle')) {
        this.target_steering_angle = initObj.target_steering_angle
      }
      else {
        this.target_steering_angle = 0.0;
      }
      if (initObj.hasOwnProperty('cross_track_error')) {
        this.cross_track_error = initObj.cross_track_error
      }
      else {
        this.cross_track_error = 0.0;
      }
      if (initObj.hasOwnProperty('heading_error')) {
        this.heading_error = initObj.heading_error
      }
      else {
        this.heading_error = 0.0;
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerTarget
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [target_velocity]
    bufferOffset = _serializer.float64(obj.target_velocity, buffer, bufferOffset);
    // Serialize message field [target_steering_angle]
    bufferOffset = _serializer.float64(obj.target_steering_angle, buffer, bufferOffset);
    // Serialize message field [cross_track_error]
    bufferOffset = _serializer.float64(obj.cross_track_error, buffer, bufferOffset);
    // Serialize message field [heading_error]
    bufferOffset = _serializer.float64(obj.heading_error, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerTarget
    let len;
    let data = new ControllerTarget(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_velocity]
    data.target_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [target_steering_angle]
    data.target_steering_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cross_track_error]
    data.cross_track_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_error]
    data.heading_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'path_follower/ControllerTarget';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '775bd3cf3d1e910e04e7d60f982bcda1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 target_velocity           # target velocity, in m/s
    float64 target_steering_angle     # target steering angle, in degrees
    float64 cross_track_error         # perpendicular distance to intended trajectory, in meters
    float64 heading_error             # heading error, in degrees
    float64 timestamp 
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerTarget(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.target_velocity !== undefined) {
      resolved.target_velocity = msg.target_velocity;
    }
    else {
      resolved.target_velocity = 0.0
    }

    if (msg.target_steering_angle !== undefined) {
      resolved.target_steering_angle = msg.target_steering_angle;
    }
    else {
      resolved.target_steering_angle = 0.0
    }

    if (msg.cross_track_error !== undefined) {
      resolved.cross_track_error = msg.cross_track_error;
    }
    else {
      resolved.cross_track_error = 0.0
    }

    if (msg.heading_error !== undefined) {
      resolved.heading_error = msg.heading_error;
    }
    else {
      resolved.heading_error = 0.0
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    return resolved;
    }
};

module.exports = ControllerTarget;
