// Auto-generated. Do not edit!

// (in-package beginner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Person {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.user_id = null;
      this.direction = null;
      this.velocity = null;
      this.acc = null;
      this.threshold_upper = null;
      this.threshold_low = null;
      this.avg = null;
      this.signal = null;
      this.state = null;
      this.LCSS_state = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('user_id')) {
        this.user_id = initObj.user_id
      }
      else {
        this.user_id = '';
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = 0.0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = 0.0;
      }
      if (initObj.hasOwnProperty('threshold_upper')) {
        this.threshold_upper = initObj.threshold_upper
      }
      else {
        this.threshold_upper = 0.0;
      }
      if (initObj.hasOwnProperty('threshold_low')) {
        this.threshold_low = initObj.threshold_low
      }
      else {
        this.threshold_low = 0.0;
      }
      if (initObj.hasOwnProperty('avg')) {
        this.avg = initObj.avg
      }
      else {
        this.avg = 0.0;
      }
      if (initObj.hasOwnProperty('signal')) {
        this.signal = initObj.signal
      }
      else {
        this.signal = 0.0;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('LCSS_state')) {
        this.LCSS_state = initObj.LCSS_state
      }
      else {
        this.LCSS_state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Person
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [user_id]
    bufferOffset = _serializer.string(obj.user_id, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = _serializer.float64(obj.direction, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float64(obj.velocity, buffer, bufferOffset);
    // Serialize message field [acc]
    bufferOffset = _serializer.float64(obj.acc, buffer, bufferOffset);
    // Serialize message field [threshold_upper]
    bufferOffset = _serializer.float64(obj.threshold_upper, buffer, bufferOffset);
    // Serialize message field [threshold_low]
    bufferOffset = _serializer.float64(obj.threshold_low, buffer, bufferOffset);
    // Serialize message field [avg]
    bufferOffset = _serializer.float64(obj.avg, buffer, bufferOffset);
    // Serialize message field [signal]
    bufferOffset = _serializer.float64(obj.signal, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.int64(obj.state, buffer, bufferOffset);
    // Serialize message field [LCSS_state]
    bufferOffset = _serializer.int64(obj.LCSS_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Person
    let len;
    let data = new Person(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [user_id]
    data.user_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acc]
    data.acc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [threshold_upper]
    data.threshold_upper = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [threshold_low]
    data.threshold_low = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [avg]
    data.avg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [signal]
    data.signal = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [LCSS_state]
    data.LCSS_state = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.user_id.length;
    return length + 76;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner/Person';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9d6531b7d7f85370791373555eb7b8ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    ##string user_id
    ##float64 direction
    ##float64 velocity
    
    string user_id
    float64 direction
    float64 velocity
    float64 acc
    float64 threshold_upper
    float64 threshold_low
    float64 avg
    float64 signal
    int64 state
    int64 LCSS_state
    
    ##int64 Imu_state
    ##int64 Lidar_state
    
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
    const resolved = new Person(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.user_id !== undefined) {
      resolved.user_id = msg.user_id;
    }
    else {
      resolved.user_id = ''
    }

    if (msg.direction !== undefined) {
      resolved.direction = msg.direction;
    }
    else {
      resolved.direction = 0.0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = 0.0
    }

    if (msg.threshold_upper !== undefined) {
      resolved.threshold_upper = msg.threshold_upper;
    }
    else {
      resolved.threshold_upper = 0.0
    }

    if (msg.threshold_low !== undefined) {
      resolved.threshold_low = msg.threshold_low;
    }
    else {
      resolved.threshold_low = 0.0
    }

    if (msg.avg !== undefined) {
      resolved.avg = msg.avg;
    }
    else {
      resolved.avg = 0.0
    }

    if (msg.signal !== undefined) {
      resolved.signal = msg.signal;
    }
    else {
      resolved.signal = 0.0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.LCSS_state !== undefined) {
      resolved.LCSS_state = msg.LCSS_state;
    }
    else {
      resolved.LCSS_state = 0
    }

    return resolved;
    }
};

module.exports = Person;
