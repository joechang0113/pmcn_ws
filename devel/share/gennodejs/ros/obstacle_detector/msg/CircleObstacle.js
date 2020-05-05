// Auto-generated. Do not edit!

// (in-package obstacle_detector.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class CircleObstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.center = null;
      this.velocity = null;
      this.radius = null;
      this.true_radius = null;
      this.id = null;
      this.direction = null;
      this.state = null;
      this.LCSS_state = null;
      this.distance = null;
      this.action = null;
      this.state_list = null;
      this.direction_list = null;
      this.prev_direction = null;
    }
    else {
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('true_radius')) {
        this.true_radius = initObj.true_radius
      }
      else {
        this.true_radius = 0.0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = 0.0;
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
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0;
      }
      if (initObj.hasOwnProperty('action')) {
        this.action = initObj.action
      }
      else {
        this.action = '';
      }
      if (initObj.hasOwnProperty('state_list')) {
        this.state_list = initObj.state_list
      }
      else {
        this.state_list = [];
      }
      if (initObj.hasOwnProperty('direction_list')) {
        this.direction_list = initObj.direction_list
      }
      else {
        this.direction_list = [];
      }
      if (initObj.hasOwnProperty('prev_direction')) {
        this.prev_direction = initObj.prev_direction
      }
      else {
        this.prev_direction = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CircleObstacle
    // Serialize message field [center]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float64(obj.radius, buffer, bufferOffset);
    // Serialize message field [true_radius]
    bufferOffset = _serializer.float64(obj.true_radius, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = _serializer.float64(obj.direction, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.int64(obj.state, buffer, bufferOffset);
    // Serialize message field [LCSS_state]
    bufferOffset = _serializer.int64(obj.LCSS_state, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.int64(obj.distance, buffer, bufferOffset);
    // Serialize message field [action]
    bufferOffset = _serializer.string(obj.action, buffer, bufferOffset);
    // Serialize message field [state_list]
    bufferOffset = _arraySerializer.int64(obj.state_list, buffer, bufferOffset, null);
    // Serialize message field [direction_list]
    bufferOffset = _arraySerializer.int64(obj.direction_list, buffer, bufferOffset, null);
    // Serialize message field [prev_direction]
    bufferOffset = _serializer.int64(obj.prev_direction, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CircleObstacle
    let len;
    let data = new CircleObstacle(null);
    // Deserialize message field [center]
    data.center = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [true_radius]
    data.true_radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [LCSS_state]
    data.LCSS_state = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [action]
    data.action = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [state_list]
    data.state_list = _arrayDeserializer.int64(buffer, bufferOffset, null)
    // Deserialize message field [direction_list]
    data.direction_list = _arrayDeserializer.int64(buffer, bufferOffset, null)
    // Deserialize message field [prev_direction]
    data.prev_direction = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    length += object.action.length;
    length += 8 * object.state_list.length;
    length += 8 * object.direction_list.length;
    return length + 120;
  }

  static datatype() {
    // Returns string type for a message object
    return 'obstacle_detector/CircleObstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ee86047c4d5777f2884d730ab57547d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point center      # Central point [m]
    geometry_msgs/Vector3 velocity  # Linear velocity [m/s]
    float64 radius                  # Radius with added margin [m]
    float64 true_radius             # True measured radius [m]
    string id
    float64 direction
    
    int64 state
    int64 LCSS_state
    int64 distance
    string action
    int64[] state_list
    int64[] direction_list
    int64 prev_direction
    ##int32 possibility
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CircleObstacle(null);
    if (msg.center !== undefined) {
      resolved.center = geometry_msgs.msg.Point.Resolve(msg.center)
    }
    else {
      resolved.center = new geometry_msgs.msg.Point()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Vector3.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.true_radius !== undefined) {
      resolved.true_radius = msg.true_radius;
    }
    else {
      resolved.true_radius = 0.0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.direction !== undefined) {
      resolved.direction = msg.direction;
    }
    else {
      resolved.direction = 0.0
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

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0
    }

    if (msg.action !== undefined) {
      resolved.action = msg.action;
    }
    else {
      resolved.action = ''
    }

    if (msg.state_list !== undefined) {
      resolved.state_list = msg.state_list;
    }
    else {
      resolved.state_list = []
    }

    if (msg.direction_list !== undefined) {
      resolved.direction_list = msg.direction_list;
    }
    else {
      resolved.direction_list = []
    }

    if (msg.prev_direction !== undefined) {
      resolved.prev_direction = msg.prev_direction;
    }
    else {
      resolved.prev_direction = 0
    }

    return resolved;
    }
};

module.exports = CircleObstacle;
