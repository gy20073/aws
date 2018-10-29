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

class ApplanixPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.timestamp = null;
      this.smooth_x = null;
      this.smooth_y = null;
      this.smooth_z = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
      this.vel_north = null;
      this.vel_east = null;
      this.vel_up = null;
      this.speed = null;
      this.track = null;
      this.roll = null;
      this.pitch = null;
      this.yaw = null;
      this.rate_roll = null;
      this.rate_pitch = null;
      this.rate_yaw = null;
      this.accel_x = null;
      this.accel_y = null;
      this.accel_z = null;
      this.wander = null;
      this.id = null;
      this.postprocess_code = null;
      this.hardware_timestamp = null;
      this.hardware_time_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('smooth_x')) {
        this.smooth_x = initObj.smooth_x
      }
      else {
        this.smooth_x = 0.0;
      }
      if (initObj.hasOwnProperty('smooth_y')) {
        this.smooth_y = initObj.smooth_y
      }
      else {
        this.smooth_y = 0.0;
      }
      if (initObj.hasOwnProperty('smooth_z')) {
        this.smooth_z = initObj.smooth_z
      }
      else {
        this.smooth_z = 0.0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('vel_north')) {
        this.vel_north = initObj.vel_north
      }
      else {
        this.vel_north = 0.0;
      }
      if (initObj.hasOwnProperty('vel_east')) {
        this.vel_east = initObj.vel_east
      }
      else {
        this.vel_east = 0.0;
      }
      if (initObj.hasOwnProperty('vel_up')) {
        this.vel_up = initObj.vel_up
      }
      else {
        this.vel_up = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('track')) {
        this.track = initObj.track
      }
      else {
        this.track = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('rate_roll')) {
        this.rate_roll = initObj.rate_roll
      }
      else {
        this.rate_roll = 0.0;
      }
      if (initObj.hasOwnProperty('rate_pitch')) {
        this.rate_pitch = initObj.rate_pitch
      }
      else {
        this.rate_pitch = 0.0;
      }
      if (initObj.hasOwnProperty('rate_yaw')) {
        this.rate_yaw = initObj.rate_yaw
      }
      else {
        this.rate_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('accel_x')) {
        this.accel_x = initObj.accel_x
      }
      else {
        this.accel_x = 0.0;
      }
      if (initObj.hasOwnProperty('accel_y')) {
        this.accel_y = initObj.accel_y
      }
      else {
        this.accel_y = 0.0;
      }
      if (initObj.hasOwnProperty('accel_z')) {
        this.accel_z = initObj.accel_z
      }
      else {
        this.accel_z = 0.0;
      }
      if (initObj.hasOwnProperty('wander')) {
        this.wander = initObj.wander
      }
      else {
        this.wander = 0.0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('postprocess_code')) {
        this.postprocess_code = initObj.postprocess_code
      }
      else {
        this.postprocess_code = 0;
      }
      if (initObj.hasOwnProperty('hardware_timestamp')) {
        this.hardware_timestamp = initObj.hardware_timestamp
      }
      else {
        this.hardware_timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('hardware_time_mode')) {
        this.hardware_time_mode = initObj.hardware_time_mode
      }
      else {
        this.hardware_time_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ApplanixPose
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [smooth_x]
    bufferOffset = _serializer.float64(obj.smooth_x, buffer, bufferOffset);
    // Serialize message field [smooth_y]
    bufferOffset = _serializer.float64(obj.smooth_y, buffer, bufferOffset);
    // Serialize message field [smooth_z]
    bufferOffset = _serializer.float64(obj.smooth_z, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float64(obj.altitude, buffer, bufferOffset);
    // Serialize message field [vel_north]
    bufferOffset = _serializer.float32(obj.vel_north, buffer, bufferOffset);
    // Serialize message field [vel_east]
    bufferOffset = _serializer.float32(obj.vel_east, buffer, bufferOffset);
    // Serialize message field [vel_up]
    bufferOffset = _serializer.float32(obj.vel_up, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [track]
    bufferOffset = _serializer.float32(obj.track, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float64(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float64(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [rate_roll]
    bufferOffset = _serializer.float64(obj.rate_roll, buffer, bufferOffset);
    // Serialize message field [rate_pitch]
    bufferOffset = _serializer.float64(obj.rate_pitch, buffer, bufferOffset);
    // Serialize message field [rate_yaw]
    bufferOffset = _serializer.float64(obj.rate_yaw, buffer, bufferOffset);
    // Serialize message field [accel_x]
    bufferOffset = _serializer.float64(obj.accel_x, buffer, bufferOffset);
    // Serialize message field [accel_y]
    bufferOffset = _serializer.float64(obj.accel_y, buffer, bufferOffset);
    // Serialize message field [accel_z]
    bufferOffset = _serializer.float64(obj.accel_z, buffer, bufferOffset);
    // Serialize message field [wander]
    bufferOffset = _serializer.float64(obj.wander, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [postprocess_code]
    bufferOffset = _serializer.int32(obj.postprocess_code, buffer, bufferOffset);
    // Serialize message field [hardware_timestamp]
    bufferOffset = _serializer.float64(obj.hardware_timestamp, buffer, bufferOffset);
    // Serialize message field [hardware_time_mode]
    bufferOffset = _serializer.int32(obj.hardware_time_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ApplanixPose
    let len;
    let data = new ApplanixPose(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [smooth_x]
    data.smooth_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [smooth_y]
    data.smooth_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [smooth_z]
    data.smooth_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_north]
    data.vel_north = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vel_east]
    data.vel_east = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vel_up]
    data.vel_up = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [track]
    data.track = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rate_roll]
    data.rate_roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rate_pitch]
    data.rate_pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rate_yaw]
    data.rate_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel_x]
    data.accel_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel_y]
    data.accel_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel_z]
    data.accel_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wander]
    data.wander = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [postprocess_code]
    data.postprocess_code = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [hardware_timestamp]
    data.hardware_timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [hardware_time_mode]
    data.hardware_time_mode = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 176;
  }

  static datatype() {
    // Returns string type for a message object
    return 'path_follower/ApplanixPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc005b38b12f428cb31db2f4286c261b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 timestamp
    float64 smooth_x
    float64 smooth_y
    float64 smooth_z
    float64 latitude
    float64 longitude
    float64 altitude
    float32 vel_north
    float32 vel_east
    float32 vel_up
    float32 speed
    float32 track
    float64 roll
    float64 pitch
    float64 yaw
    float64 rate_roll
    float64 rate_pitch
    float64 rate_yaw
    float64 accel_x
    float64 accel_y
    float64 accel_z
    float64 wander
    uint32  id
    int32   postprocess_code
    float64 hardware_timestamp
    int32   hardware_time_mode
    
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
    const resolved = new ApplanixPose(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    if (msg.smooth_x !== undefined) {
      resolved.smooth_x = msg.smooth_x;
    }
    else {
      resolved.smooth_x = 0.0
    }

    if (msg.smooth_y !== undefined) {
      resolved.smooth_y = msg.smooth_y;
    }
    else {
      resolved.smooth_y = 0.0
    }

    if (msg.smooth_z !== undefined) {
      resolved.smooth_z = msg.smooth_z;
    }
    else {
      resolved.smooth_z = 0.0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.vel_north !== undefined) {
      resolved.vel_north = msg.vel_north;
    }
    else {
      resolved.vel_north = 0.0
    }

    if (msg.vel_east !== undefined) {
      resolved.vel_east = msg.vel_east;
    }
    else {
      resolved.vel_east = 0.0
    }

    if (msg.vel_up !== undefined) {
      resolved.vel_up = msg.vel_up;
    }
    else {
      resolved.vel_up = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.track !== undefined) {
      resolved.track = msg.track;
    }
    else {
      resolved.track = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.rate_roll !== undefined) {
      resolved.rate_roll = msg.rate_roll;
    }
    else {
      resolved.rate_roll = 0.0
    }

    if (msg.rate_pitch !== undefined) {
      resolved.rate_pitch = msg.rate_pitch;
    }
    else {
      resolved.rate_pitch = 0.0
    }

    if (msg.rate_yaw !== undefined) {
      resolved.rate_yaw = msg.rate_yaw;
    }
    else {
      resolved.rate_yaw = 0.0
    }

    if (msg.accel_x !== undefined) {
      resolved.accel_x = msg.accel_x;
    }
    else {
      resolved.accel_x = 0.0
    }

    if (msg.accel_y !== undefined) {
      resolved.accel_y = msg.accel_y;
    }
    else {
      resolved.accel_y = 0.0
    }

    if (msg.accel_z !== undefined) {
      resolved.accel_z = msg.accel_z;
    }
    else {
      resolved.accel_z = 0.0
    }

    if (msg.wander !== undefined) {
      resolved.wander = msg.wander;
    }
    else {
      resolved.wander = 0.0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.postprocess_code !== undefined) {
      resolved.postprocess_code = msg.postprocess_code;
    }
    else {
      resolved.postprocess_code = 0
    }

    if (msg.hardware_timestamp !== undefined) {
      resolved.hardware_timestamp = msg.hardware_timestamp;
    }
    else {
      resolved.hardware_timestamp = 0.0
    }

    if (msg.hardware_time_mode !== undefined) {
      resolved.hardware_time_mode = msg.hardware_time_mode;
    }
    else {
      resolved.hardware_time_mode = 0
    }

    return resolved;
    }
};

module.exports = ApplanixPose;
