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

class Actuator {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.direction = null;
      this.steering_mode = null;
      this.steering_value = null;
      this.brake_pressure = null;
      this.throttle_fraction = null;
      this.timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = 0;
      }
      if (initObj.hasOwnProperty('steering_mode')) {
        this.steering_mode = initObj.steering_mode
      }
      else {
        this.steering_mode = 0;
      }
      if (initObj.hasOwnProperty('steering_value')) {
        this.steering_value = initObj.steering_value
      }
      else {
        this.steering_value = 0.0;
      }
      if (initObj.hasOwnProperty('brake_pressure')) {
        this.brake_pressure = initObj.brake_pressure
      }
      else {
        this.brake_pressure = 0.0;
      }
      if (initObj.hasOwnProperty('throttle_fraction')) {
        this.throttle_fraction = initObj.throttle_fraction
      }
      else {
        this.throttle_fraction = 0.0;
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
    // Serializes a message object of type Actuator
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = _serializer.uint8(obj.direction, buffer, bufferOffset);
    // Serialize message field [steering_mode]
    bufferOffset = _serializer.uint8(obj.steering_mode, buffer, bufferOffset);
    // Serialize message field [steering_value]
    bufferOffset = _serializer.float64(obj.steering_value, buffer, bufferOffset);
    // Serialize message field [brake_pressure]
    bufferOffset = _serializer.float64(obj.brake_pressure, buffer, bufferOffset);
    // Serialize message field [throttle_fraction]
    bufferOffset = _serializer.float64(obj.throttle_fraction, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Actuator
    let len;
    let data = new Actuator(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [steering_mode]
    data.steering_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [steering_value]
    data.steering_value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [brake_pressure]
    data.brake_pressure = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [throttle_fraction]
    data.throttle_fraction = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 34;
  }

  static datatype() {
    // Returns string type for a message object
    return 'path_follower/Actuator';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8164b580b4ff14b976d32e45c6c9f9d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint8 direction
    uint8 steering_mode
    float64 steering_value     # depending on steering_mode this is either angle or torque
    float64 brake_pressure
    float64 throttle_fraction
    float64 timestamp
    
      # for direction
    uint8 DIRECTION_FORWARD = 0
    uint8 DIRECTION_REVERSE = 1
    
      # for steering mode
    uint8 ANGLE_CONTROL = 0
    uint8 TORQUE_CONTROL = 1
    
    
    
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
    const resolved = new Actuator(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.direction !== undefined) {
      resolved.direction = msg.direction;
    }
    else {
      resolved.direction = 0
    }

    if (msg.steering_mode !== undefined) {
      resolved.steering_mode = msg.steering_mode;
    }
    else {
      resolved.steering_mode = 0
    }

    if (msg.steering_value !== undefined) {
      resolved.steering_value = msg.steering_value;
    }
    else {
      resolved.steering_value = 0.0
    }

    if (msg.brake_pressure !== undefined) {
      resolved.brake_pressure = msg.brake_pressure;
    }
    else {
      resolved.brake_pressure = 0.0
    }

    if (msg.throttle_fraction !== undefined) {
      resolved.throttle_fraction = msg.throttle_fraction;
    }
    else {
      resolved.throttle_fraction = 0.0
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

// Constants for message
Actuator.Constants = {
  DIRECTION_FORWARD: 0,
  DIRECTION_REVERSE: 1,
  ANGLE_CONTROL: 0,
  TORQUE_CONTROL: 1,
}

module.exports = Actuator;
