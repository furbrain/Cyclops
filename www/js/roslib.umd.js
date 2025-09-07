(function (global, factory) {
  typeof exports === 'object' && typeof module !== 'undefined' ? factory(exports, require('cbor-js'), require('eventemitter3'), require('@xmldom/xmldom'), require('pngparse')) :
  typeof define === 'function' && define.amd ? define(['exports', 'cbor-js', 'eventemitter3', '@xmldom/xmldom', 'pngparse'], factory) :
  (global = typeof globalThis !== 'undefined' ? globalThis : global || self, factory(global.ROSLIB = {}, global.CBOR, global.eventemitter3, global.xmldom, global.pngparse));
})(this, (function (exports, CBOR, eventemitter3, xmldom, pngparse) { 'use strict';

  var UPPER32 = Math.pow(2, 32);

  var warnedPrecision = false;
  function warnPrecision() {
    if (!warnedPrecision) {
      warnedPrecision = true;
      console.warn(
        'CBOR 64-bit integer array values may lose precision. No further warnings.'
      );
    }
  }

  /**
   * Unpack 64-bit unsigned integer from byte array.
   * @param {Uint8Array} bytes
   */
  function decodeUint64LE(bytes) {
    warnPrecision();

    var byteLen = bytes.byteLength;
    var offset = bytes.byteOffset;
    var arrLen = byteLen / 8;

    var buffer = bytes.buffer.slice(offset, offset + byteLen);
    var uint32View = new Uint32Array(buffer);

    var arr = new Array(arrLen);
    for (var i = 0; i < arrLen; i++) {
      var si = i * 2;
      var lo = uint32View[si];
      var hi = uint32View[si + 1];
      arr[i] = lo + UPPER32 * hi;
    }

    return arr;
  }

  /**
   * Unpack 64-bit signed integer from byte array.
   * @param {Uint8Array} bytes
   */
  function decodeInt64LE(bytes) {
    warnPrecision();

    var byteLen = bytes.byteLength;
    var offset = bytes.byteOffset;
    var arrLen = byteLen / 8;

    var buffer = bytes.buffer.slice(offset, offset + byteLen);
    var uint32View = new Uint32Array(buffer);
    var int32View = new Int32Array(buffer);

    var arr = new Array(arrLen);
    for (var i = 0; i < arrLen; i++) {
      var si = i * 2;
      var lo = uint32View[si];
      var hi = int32View[si + 1];
      arr[i] = lo + UPPER32 * hi;
    }

    return arr;
  }

  /**
   * Unpack typed array from byte array.
   * @param {Uint8Array} bytes
   * @param {ArrayConstructor} ArrayType - Desired output array type
   */
  function decodeNativeArray(bytes, ArrayType) {
    var byteLen = bytes.byteLength;
    var offset = bytes.byteOffset;
    var buffer = bytes.buffer.slice(offset, offset + byteLen);
    return new ArrayType(buffer);
  }

  /**
   * Supports a subset of draft CBOR typed array tags:
   *     <https://tools.ietf.org/html/draft-ietf-cbor-array-tags-00>
   *
   * Only supports little-endian tags for now.
   */
  var nativeArrayTypes = {
    64: Uint8Array,
    69: Uint16Array,
    70: Uint32Array,
    72: Int8Array,
    77: Int16Array,
    78: Int32Array,
    85: Float32Array,
    86: Float64Array
  };

  /**
   * We can also decode 64-bit integer arrays, since ROS has these types.
   */
  var conversionArrayTypes = {
    71: decodeUint64LE,
    79: decodeInt64LE
  };

  /**
   * Handle CBOR typed array tags during decoding.
   * @param {Uint8Array} data
   * @param {Number} tag
   */
  function cborTypedArrayTagger(data, tag) {
    if (tag in nativeArrayTypes) {
      var arrayType = nativeArrayTypes[tag];
      return decodeNativeArray(data, arrayType);
    }
    if (tag in conversionArrayTypes) {
      return conversionArrayTypes[tag](data);
    }
    return data;
  }

  /**
   * Socket event handling utilities for handling events on either
   * WebSocket and TCP sockets
   *
   * Note to anyone reviewing this code: these functions are called
   * in the context of their parent object, unless bound
   * @fileOverview
   */

  var BSON = null;
  // @ts-expect-error -- Workarounds for not including BSON in bundle. need to revisit
  if (typeof bson !== 'undefined') {
    // @ts-expect-error -- Workarounds for not including BSON in bundle. need to revisit
    BSON = bson().BSON;
  }

  /**
   * Event listeners for a WebSocket or TCP socket to a JavaScript
   * ROS Client. Sets up Messages for a given topic to trigger an
   * event on the ROS client.
   *
   * @namespace SocketAdapter
   * @private
   */
  function SocketAdapter(client) {
    var decoder = null;
    if (client.transportOptions.decoder) {
      decoder = client.transportOptions.decoder;
    }

    function handleMessage(message) {
      if (message.op === 'publish') {
        client.emit(message.topic, message.msg);
      } else if (message.op === 'service_response') {
        client.emit(message.id, message);
      } else if (message.op === 'call_service') {
        client.emit(message.service, message);
      } else if (message.op === 'send_action_goal') {
        client.emit(message.action, message);
      } else if (message.op === 'cancel_action_goal') {
        client.emit(message.id, message);
      } else if (message.op === 'action_feedback') {
        client.emit(message.id, message);
      } else if (message.op === 'action_result') {
        client.emit(message.id, message);
      } else if (message.op === 'status') {
        if (message.id) {
          client.emit('status:' + message.id, message);
        } else {
          client.emit('status', message);
        }
      }
    }

    function handlePng(message, callback) {
      if (message.op === 'png') {
        // If in Node.js..
        if (typeof window === 'undefined') {
          Promise.resolve().then(function () { return decompressPng$3; }).then(({ default: decompressPng }) => decompressPng(message.data, callback));
        } else {
          // if in browser..
          Promise.resolve().then(function () { return decompressPng$1; }).then(({default: decompressPng}) => decompressPng(message.data, callback));
        }
      } else {
        callback(message);
      }
    }

    function decodeBSON(data, callback) {
      if (!BSON) {
        throw 'Cannot process BSON encoded message without BSON header.';
      }
      var reader = new FileReader();
      reader.onload = function () {
        // @ts-expect-error -- this doesn't seem right, but don't want to break current type coercion assumption
        var uint8Array = new Uint8Array(this.result);
        var msg = BSON.deserialize(uint8Array);
        callback(msg);
      };
      reader.readAsArrayBuffer(data);
    }

    return {
      /**
       * Emit a 'connection' event on WebSocket connection.
       *
       * @param {function} event - The argument to emit with the event.
       * @memberof SocketAdapter
       */
      onopen: function onOpen(event) {
        client.isConnected = true;
        client.emit('connection', event);
      },

      /**
       * Emit a 'close' event on WebSocket disconnection.
       *
       * @param {function} event - The argument to emit with the event.
       * @memberof SocketAdapter
       */
      onclose: function onClose(event) {
        client.isConnected = false;
        client.emit('close', event);
      },

      /**
       * Emit an 'error' event whenever there was an error.
       *
       * @param {function} event - The argument to emit with the event.
       * @memberof SocketAdapter
       */
      onerror: function onError(event) {
        client.emit('error', event);
      },

      /**
       * Parse message responses from rosbridge and send to the appropriate
       * topic, service, or param.
       *
       * @param {Object} data - The raw JSON message from rosbridge.
       * @memberof SocketAdapter
       */
      onmessage: function onMessage(data) {
        if (decoder) {
          decoder(data.data, function (message) {
            handleMessage(message);
          });
        } else if (typeof Blob !== 'undefined' && data.data instanceof Blob) {
          decodeBSON(data.data, function (message) {
            handlePng(message, handleMessage);
          });
        } else if (data.data instanceof ArrayBuffer) {
          var decoded = CBOR.decode(data.data, cborTypedArrayTagger);
          handleMessage(decoded);
        } else {
          var message = JSON.parse(typeof data === 'string' ? data : data.data);
          handlePng(message, handleMessage);
        }
      }
    };
  }

  /**
   * @fileOverview
   * @author Brandon Alexander - baalexander@gmail.com
   */


  /**
   * Publish and/or subscribe to a topic in ROS.
   *
   * Emits the following events:
   *  * 'warning' - If there are any warning during the Topic creation.
   *  * 'message' - The message data from rosbridge.
   * @template T
   */
  class Topic extends eventemitter3.EventEmitter {
    /** @type {boolean | undefined} */
    waitForReconnect = undefined;
    /** @type {(() => void) | undefined} */
    reconnectFunc = undefined;
    isAdvertised = false;
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} options.name - The topic name, like '/cmd_vel'.
     * @param {string} options.messageType - The message type, like 'std_msgs/String'.
     * @param {string} [options.compression=none] - The type of compression to use, like 'png', 'cbor', or 'cbor-raw'.
     * @param {number} [options.throttle_rate=0] - The rate (in ms in between messages) at which to throttle the topics.
     * @param {number} [options.queue_size=100] - The queue created at bridge side for re-publishing webtopics.
     * @param {boolean} [options.latch=false] - Latch the topic when publishing.
     * @param {number} [options.queue_length=0] - The queue length at bridge side used when subscribing.
     * @param {boolean} [options.reconnect_on_close=true] - The flag to enable resubscription and readvertisement on close event.
     */
    constructor(options) {
      super();
      this.ros = options.ros;
      this.name = options.name;
      this.messageType = options.messageType;
      this.compression = options.compression || 'none';
      this.throttle_rate = options.throttle_rate || 0;
      this.latch = options.latch || false;
      this.queue_size = options.queue_size || 100;
      this.queue_length = options.queue_length || 0;
      this.reconnect_on_close =
        options.reconnect_on_close !== undefined
          ? options.reconnect_on_close
          : true;

      // Check for valid compression types
      if (
        this.compression &&
        this.compression !== 'png' &&
        this.compression !== 'cbor' &&
        this.compression !== 'cbor-raw' &&
        this.compression !== 'none'
      ) {
        this.emit(
          'warning',
          this.compression +
            ' compression is not supported. No compression will be used.'
        );
        this.compression = 'none';
      }

      // Check if throttle rate is negative
      if (this.throttle_rate < 0) {
        this.emit('warning', this.throttle_rate + ' is not allowed. Set to 0');
        this.throttle_rate = 0;
      }

      if (this.reconnect_on_close) {
        this.callForSubscribeAndAdvertise = (message) => {
          this.ros.callOnConnection(message);

          this.waitForReconnect = false;
          this.reconnectFunc = () => {
            if (!this.waitForReconnect) {
              this.waitForReconnect = true;
              this.ros.callOnConnection(message);
              this.ros.once('connection', () => {
                this.waitForReconnect = false;
              });
            }
          };
          this.ros.on('close', this.reconnectFunc);
        };
      } else {
        this.callForSubscribeAndAdvertise = this.ros.callOnConnection;
      }
    }

    _messageCallback = (data) => {
      this.emit('message', data);
    };
    /**
     * @callback subscribeCallback
     * @param {T} message - The published message.
     */
    /**
     * Every time a message is published for the given topic, the callback
     * will be called with the message object.
     *
     * @param {subscribeCallback} callback - Function with the following params:
     */
    subscribe(callback) {
      if (typeof callback === 'function') {
        this.on('message', callback);
      }

      if (this.subscribeId) {
        return;
      }
      this.ros.on(this.name, this._messageCallback);
      this.subscribeId =
        'subscribe:' + this.name + ':' + (++this.ros.idCounter).toString();

      this.callForSubscribeAndAdvertise({
        op: 'subscribe',
        id: this.subscribeId,
        type: this.messageType,
        topic: this.name,
        compression: this.compression,
        throttle_rate: this.throttle_rate,
        queue_length: this.queue_length
      });
    }
    /**
     * Unregister as a subscriber for the topic. Unsubscribing will stop
     * and remove all subscribe callbacks. To remove a callback, you must
     * explicitly pass the callback function in.
     *
     * @param {import('eventemitter3').EventEmitter.ListenerFn} [callback] - The callback to unregister, if
     *     provided and other listeners are registered the topic won't
     *     unsubscribe, just stop emitting to the passed listener.
     */
    unsubscribe(callback) {
      if (callback) {
        this.off('message', callback);
        // If there is any other callbacks still subscribed don't unsubscribe
        if (this.listeners('message').length) {
          return;
        }
      }
      if (!this.subscribeId) {
        return;
      }
      // Note: Don't call this.removeAllListeners, allow client to handle that themselves
      this.ros.off(this.name, this._messageCallback);
      if (this.reconnect_on_close) {
        this.ros.off('close', this.reconnectFunc);
      }
      this.emit('unsubscribe');
      this.ros.callOnConnection({
        op: 'unsubscribe',
        id: this.subscribeId,
        topic: this.name
      });
      this.subscribeId = null;
    }
    /**
     * Register as a publisher for the topic.
     */
    advertise() {
      if (this.isAdvertised) {
        return;
      }
      this.advertiseId =
        'advertise:' + this.name + ':' + (++this.ros.idCounter).toString();
      this.callForSubscribeAndAdvertise({
        op: 'advertise',
        id: this.advertiseId,
        type: this.messageType,
        topic: this.name,
        latch: this.latch,
        queue_size: this.queue_size
      });
      this.isAdvertised = true;

      if (!this.reconnect_on_close) {
        this.ros.on('close', () => {
          this.isAdvertised = false;
        });
      }
    }
    /**
     * Unregister as a publisher for the topic.
     */
    unadvertise() {
      if (!this.isAdvertised) {
        return;
      }
      if (this.reconnect_on_close) {
        this.ros.off('close', this.reconnectFunc);
      }
      this.emit('unadvertise');
      this.ros.callOnConnection({
        op: 'unadvertise',
        id: this.advertiseId,
        topic: this.name
      });
      this.isAdvertised = false;
    }
    /**
     * Publish the message.
     *
     * @param {T} message - The message to publish.
     */
    publish(message) {
      if (!this.isAdvertised) {
        this.advertise();
      }

      this.ros.idCounter++;
      var call = {
        op: 'publish',
        id: 'publish:' + this.name + ':' + this.ros.idCounter,
        topic: this.name,
        msg: message,
        latch: this.latch
      };
      this.ros.callOnConnection(call);
    }
  }

  /**
   * @fileOverview
   * @author Brandon Alexander - baalexander@gmail.com
   */


  /**
   * A ROS service client.
   * @template TRequest, TResponse
   */
  class Service extends eventemitter3.EventEmitter {
    /**
       * Stores a reference to the most recent service callback advertised so it can be removed from the EventEmitter during un-advertisement
       * @private
       * @type {((rosbridgeRequest) => any) | null}
       */
    _serviceCallback = null;
    isAdvertised = false;
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} options.name - The service name, like '/add_two_ints'.
     * @param {string} options.serviceType - The service type, like 'rospy_tutorials/AddTwoInts'.
     */
    constructor(options) {
      super();
      this.ros = options.ros;
      this.name = options.name;
      this.serviceType = options.serviceType;
    }
    /**
     * @callback callServiceCallback
     *  @param {TResponse} response - The response from the service request.
     */
    /**
     * @callback callServiceFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Call the service. Returns the service response in the
     * callback. Does nothing if this service is currently advertised.
     *
     * @param {TRequest} request - The service request to send.
     * @param {callServiceCallback} [callback] - Function with the following params:
     * @param {callServiceFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    callService(request, callback, failedCallback) {
      if (this.isAdvertised) {
        return;
      }

      var serviceCallId =
        'call_service:' + this.name + ':' + (++this.ros.idCounter).toString();

      if (callback || failedCallback) {
        this.ros.once(serviceCallId, function (message) {
          if (message.result !== undefined && message.result === false) {
            if (typeof failedCallback === 'function') {
              failedCallback(message.values);
            }
          } else if (typeof callback === 'function') {
            callback(message.values);
          }
        });
      }

      var call = {
        op: 'call_service',
        id: serviceCallId,
        service: this.name,
        type: this.serviceType,
        args: request
      };
      this.ros.callOnConnection(call);
    }
    /**
     * @callback advertiseCallback
     * @param {TRequest} request - The service request.
     * @param {Partial<TResponse>} response - An empty dictionary. Take care not to overwrite this. Instead, only modify the values within.
     * @returns {boolean} true if the service has finished successfully, i.e., without any fatal errors.
     */
    /**
     * Advertise the service. This turns the Service object from a client
     * into a server. The callback will be called with every request
     * that's made on this service.
     *
     * @param {advertiseCallback} callback - This works similarly to the callback for a C++ service and should take the following params
     */
    advertise(callback) {
      if (this.isAdvertised) {
        throw new Error('Cannot advertise the same Service twice!');
      }

      // Store the new callback for removal during un-advertisement
      this._serviceCallback = (rosbridgeRequest) => {
        var response = {};
        var success = callback(rosbridgeRequest.args, response);

        var call = {
          op: 'service_response',
          service: this.name,
          values: response,
          result: success
        };

        if (rosbridgeRequest.id) {
          call.id = rosbridgeRequest.id;
        }

        this.ros.callOnConnection(call);
      };

      this.ros.on(this.name, this._serviceCallback);
      this.ros.callOnConnection({
        op: 'advertise_service',
        type: this.serviceType,
        service: this.name
      });
      this.isAdvertised = true;
    }

    unadvertise() {
      if (!this.isAdvertised) {
        throw new Error(`Tried to un-advertise service ${this.name}, but it was not advertised!`);
      }
      this.ros.callOnConnection({
        op: 'unadvertise_service',
        service: this.name
      });
      // Remove the registered callback
      if (this._serviceCallback) {
        this.ros.off(this.name, this._serviceCallback);
      }
      this.isAdvertised = false;
    }

    /**
     * An alternate form of Service advertisement that supports a modern Promise-based interface for use with async/await.
     * @param {(request: TRequest) => Promise<TResponse>} callback An asynchronous callback processing the request and returning a response.
     */
    advertiseAsync(callback) {
      if (this.isAdvertised) {
        throw new Error('Cannot advertise the same Service twice!');
      }
      this._serviceCallback = async (rosbridgeRequest) => {
        /** @type {{op: string, service: string, values?: TResponse, result: boolean, id?: string}} */
        let rosbridgeResponse = {
          op: 'service_response',
          service: this.name,
          result: false
        };
        try {
          rosbridgeResponse.values = await callback(rosbridgeRequest.args);
          rosbridgeResponse.result = true;
        } finally {
          if (rosbridgeRequest.id) {
            rosbridgeResponse.id = rosbridgeRequest.id;
          }
          this.ros.callOnConnection(rosbridgeResponse);
        }
      };
      this.ros.on(this.name, this._serviceCallback);
      this.ros.callOnConnection({
        op: 'advertise_service',
        type: this.serviceType,
        service: this.name
      });
      this.isAdvertised = true;
    }
  }

  /**
   * @fileOverview
   * @author Brandon Alexander - baalexander@gmail.com
   */


  /**
   * A ROS parameter.
   */
  class Param {
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} options.name - The param name, like max_vel_x.
     */
    constructor(options) {
      this.ros = options.ros;
      this.name = options.name;
    }
    /**
     * @callback getCallback
     * @param {Object} value - The value of the param from ROS.
     */
    /**
     * @callback getFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Fetch the value of the param.
     *
     * @param {getCallback} callback - The callback function.
     * @param {getFailedCallback} [failedCallback] - The callback function when the service call failed.
     */
    get(callback, failedCallback) {
      var paramClient = new Service({
        ros: this.ros,
        name: '/rosapi/get_param',
        serviceType: 'rosapi/GetParam'
      });

      var request = {name: this.name};

      paramClient.callService(
        request,
        function (result) {
          var value = JSON.parse(result.value);
          callback(value);
        },
        failedCallback
      );
    }
    /**
     * @callback setParamCallback
     * @param {Object} response - The response from the service request.
     */
    /**
     * @callback setParamFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Set the value of the param in ROS.
     *
     * @param {Object} value - The value to set param to.
     * @param {setParamCallback} [callback] - The callback function.
     * @param {setParamFailedCallback} [failedCallback] - The callback function when the service call failed.
     */
    set(value, callback, failedCallback) {
      var paramClient = new Service({
        ros: this.ros,
        name: '/rosapi/set_param',
        serviceType: 'rosapi/SetParam'
      });

      var request = {
        name: this.name,
        value: JSON.stringify(value)
      };

      paramClient.callService(request, callback, failedCallback);
    }
    /**
     * Delete this parameter on the ROS server.
     *
     * @param {setParamCallback} callback - The callback function.
     * @param {setParamFailedCallback} [failedCallback] - The callback function when the service call failed.
     */
    delete(callback, failedCallback) {
      var paramClient = new Service({
        ros: this.ros,
        name: '/rosapi/delete_param',
        serviceType: 'rosapi/DeleteParam'
      });

      var request = {
        name: this.name
      };

      paramClient.callService(request, callback, failedCallback);
    }
  }

  /**
   * @fileOverview
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * An actionlib action client.
   *
   * Emits the following events:
   *  * 'timeout' - If a timeout occurred while sending a goal.
   *  * 'status' - The status messages received from the action server.
   *  * 'feedback' - The feedback messages received from the action server.
   *  * 'result' - The result returned from the action server.
   *
   */
  class ActionClient extends eventemitter3.EventEmitter {
    goals = {};
    /** flag to check if a status has been received */
    receivedStatus = false
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} options.serverName - The action server name, like '/fibonacci'.
     * @param {string} options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
     * @param {number} [options.timeout] - The timeout length when connecting to the action server.
     * @param {boolean} [options.omitFeedback] - The flag to indicate whether to omit the feedback channel or not.
     * @param {boolean} [options.omitStatus] - The flag to indicate whether to omit the status channel or not.
     * @param {boolean} [options.omitResult] - The flag to indicate whether to omit the result channel or not.
     */
    constructor(options) {
      super();
      this.ros = options.ros;
      this.serverName = options.serverName;
      this.actionName = options.actionName;
      this.timeout = options.timeout;
      this.omitFeedback = options.omitFeedback;
      this.omitStatus = options.omitStatus;
      this.omitResult = options.omitResult;

      // create the topics associated with actionlib
      this.feedbackListener = new Topic({
        ros: this.ros,
        name: this.serverName + '/feedback',
        messageType: this.actionName + 'Feedback'
      });

      this.statusListener = new Topic({
        ros: this.ros,
        name: this.serverName + '/status',
        messageType: 'actionlib_msgs/GoalStatusArray'
      });

      this.resultListener = new Topic({
        ros: this.ros,
        name: this.serverName + '/result',
        messageType: this.actionName + 'Result'
      });

      this.goalTopic = new Topic({
        ros: this.ros,
        name: this.serverName + '/goal',
        messageType: this.actionName + 'Goal'
      });

      this.cancelTopic = new Topic({
        ros: this.ros,
        name: this.serverName + '/cancel',
        messageType: 'actionlib_msgs/GoalID'
      });

      // advertise the goal and cancel topics
      this.goalTopic.advertise();
      this.cancelTopic.advertise();

      // subscribe to the status topic
      if (!this.omitStatus) {
        this.statusListener.subscribe((statusMessage) => {
          this.receivedStatus = true;
          statusMessage.status_list.forEach((status) => {
            var goal = this.goals[status.goal_id.id];
            if (goal) {
              goal.emit('status', status);
            }
          });
        });
      }

      // subscribe the the feedback topic
      if (!this.omitFeedback) {
        this.feedbackListener.subscribe((feedbackMessage) => {
          var goal = this.goals[feedbackMessage.status.goal_id.id];
          if (goal) {
            goal.emit('status', feedbackMessage.status);
            goal.emit('feedback', feedbackMessage.feedback);
          }
        });
      }

      // subscribe to the result topic
      if (!this.omitResult) {
        this.resultListener.subscribe((resultMessage) => {
          var goal = this.goals[resultMessage.status.goal_id.id];

          if (goal) {
            goal.emit('status', resultMessage.status);
            goal.emit('result', resultMessage.result);
          }
        });
      }

      // If timeout specified, emit a 'timeout' event if the action server does not respond
      if (this.timeout) {
        setTimeout(() => {
          if (!this.receivedStatus) {
            this.emit('timeout');
          }
        }, this.timeout);
      }
    }
    /**
     * Cancel all goals associated with this ActionClient.
     */
    cancel() {
      var cancelMessage = {};
      this.cancelTopic.publish(cancelMessage);
    }
    /**
     * Unsubscribe and unadvertise all topics associated with this ActionClient.
     */
    dispose() {
      this.goalTopic.unadvertise();
      this.cancelTopic.unadvertise();
      if (!this.omitStatus) {
        this.statusListener.unsubscribe();
      }
      if (!this.omitFeedback) {
        this.feedbackListener.unsubscribe();
      }
      if (!this.omitResult) {
        this.resultListener.unsubscribe();
      }
    }
  }

  /**
   * @fileOverview
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * An actionlib goal that is associated with an action server.
   *
   * Emits the following events:
   *  * 'timeout' - If a timeout occurred while sending a goal.
   */
  class Goal extends eventemitter3.EventEmitter {
    isFinished = false;
    status = undefined;
    result = undefined;
    feedback = undefined;
    // Create a random ID
    goalID = 'goal_' + Math.random() + '_' + new Date().getTime();
    /**
     * @param {Object} options
     * @param {ActionClient} options.actionClient - The ROSLIB.ActionClient to use with this goal.
     * @param {Object} options.goalMessage - The JSON object containing the goal for the action server.
     */
    constructor(options) {
      super();
      this.actionClient = options.actionClient;

      // Fill in the goal message
      this.goalMessage = {
        goal_id: {
          stamp: {
            secs: 0,
            nsecs: 0
          },
          id: this.goalID
        },
        goal: options.goalMessage
      };

      this.on('status', (status) => {
        this.status = status;
      });

      this.on('result', (result) => {
        this.isFinished = true;
        this.result = result;
      });

      this.on('feedback', (feedback) => {
        this.feedback = feedback;
      });

      // Add the goal
      this.actionClient.goals[this.goalID] = this;
    }
    /**
     * Send the goal to the action server.
     *
     * @param {number} [timeout] - A timeout length for the goal's result.
     */
    send(timeout) {
      this.actionClient.goalTopic.publish(this.goalMessage);
      if (timeout) {
        setTimeout(() => {
          if (!this.isFinished) {
            this.emit('timeout');
          }
        }, timeout);
      }
    }
    /**
     * Cancel the current goal.
     */
    cancel() {
      var cancelMessage = {
        id: this.goalID
      };
      this.actionClient.cancelTopic.publish(cancelMessage);
    }
  }

  /**
   * @fileOverview
   * @author David Gossow - dgossow@willowgarage.com
   */

  /**
   * A Quaternion.
   */
  class Quaternion {
    /**
     * @param {Object} [options]
     * @param {number|null} [options.x=0] - The x value.
     * @param {number|null} [options.y=0] - The y value.
     * @param {number|null} [options.z=0] - The z value.
     * @param {number|null} [options.w=1] - The w value.
     */
    constructor(options) {
      options = options || {};
      this.x = options.x || 0;
      this.y = options.y || 0;
      this.z = options.z || 0;
      this.w = typeof options.w === 'number' ? options.w : 1;
    }
    /**
     * Perform a conjugation on this quaternion.
     */
    conjugate() {
      this.x *= -1;
      this.y *= -1;
      this.z *= -1;
    }
    /**
     * Return the norm of this quaternion.
     */
    norm() {
      return Math.sqrt(
        this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w
      );
    }
    /**
     * Perform a normalization on this quaternion.
     */
    normalize() {
      var l = Math.sqrt(
        this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w
      );
      if (l === 0) {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.w = 1;
      } else {
        l = 1 / l;
        this.x = this.x * l;
        this.y = this.y * l;
        this.z = this.z * l;
        this.w = this.w * l;
      }
    }
    /**
     * Convert this quaternion into its inverse.
     */
    invert() {
      this.conjugate();
      this.normalize();
    }
    /**
     * Set the values of this quaternion to the product of itself and the given quaternion.
     *
     * @param {Quaternion} q - The quaternion to multiply with.
     */
    multiply(q) {
      var newX = this.x * q.w + this.y * q.z - this.z * q.y + this.w * q.x;
      var newY = -this.x * q.z + this.y * q.w + this.z * q.x + this.w * q.y;
      var newZ = this.x * q.y - this.y * q.x + this.z * q.w + this.w * q.z;
      var newW = -this.x * q.x - this.y * q.y - this.z * q.z + this.w * q.w;
      this.x = newX;
      this.y = newY;
      this.z = newZ;
      this.w = newW;
    }
    /**
     * Clone a copy of this quaternion.
     *
     * @returns {Quaternion} The cloned quaternion.
     */
    clone() {
      return new Quaternion(this);
    }
  }

  /**
   * @fileOverview
   * @author David Gossow - dgossow@willowgarage.com
   */


  /**
   * A 3D vector.
   */
  class Vector3 {
    /**
     * @param {Object} [options]
     * @param {number} [options.x=0] - The x value.
     * @param {number} [options.y=0] - The y value.
     * @param {number} [options.z=0] - The z value.
     */
    constructor(options) {
      options = options || {};
      this.x = options.x || 0;
      this.y = options.y || 0;
      this.z = options.z || 0;
    }
    /**
     * Set the values of this vector to the sum of itself and the given vector.
     *
     * @param {Vector3} v - The vector to add with.
     */
    add(v) {
      this.x += v.x;
      this.y += v.y;
      this.z += v.z;
    }
    /**
     * Set the values of this vector to the difference of itself and the given vector.
     *
     * @param {Vector3} v - The vector to subtract with.
     */
    subtract(v) {
      this.x -= v.x;
      this.y -= v.y;
      this.z -= v.z;
    }
    /**
     * Multiply the given Quaternion with this vector.
     *
     * @param {Quaternion} q - The quaternion to multiply with.
     */
    multiplyQuaternion(q) {
      var ix = q.w * this.x + q.y * this.z - q.z * this.y;
      var iy = q.w * this.y + q.z * this.x - q.x * this.z;
      var iz = q.w * this.z + q.x * this.y - q.y * this.x;
      var iw = -q.x * this.x - q.y * this.y - q.z * this.z;
      this.x = ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y;
      this.y = iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z;
      this.z = iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x;
    }
    /**
     * Clone a copy of this vector.
     *
     * @returns {Vector3} The cloned vector.
     */
    clone() {
      return new Vector3(this);
    }
  }

  /**
   * @fileOverview
   * @author David Gossow - dgossow@willowgarage.com
   */


  /**
   * A Transform in 3-space. Values are copied into this object.
   */
  class Transform {
    /**
     * @param {Object} options
     * @param {Vector3} options.translation - The ROSLIB.Vector3 describing the translation.
     * @param {Quaternion} options.rotation - The ROSLIB.Quaternion describing the rotation.
     */
    constructor(options) {
      // Copy the values into this object if they exist
      this.translation = new Vector3(options.translation);
      this.rotation = new Quaternion(options.rotation);
    }
    /**
     * Clone a copy of this transform.
     *
     * @returns {Transform} The cloned transform.
     */
    clone() {
      return new Transform(this);
    }
  }

  /**
   * @fileOverview
   * @author David Gossow - dgossow@willowgarage.com
   */


  /**
   * A TF Client that listens to TFs from tf2_web_republisher.
   */
  class TFClient extends eventemitter3.EventEmitter {
    /** @type {Goal|false} */
    currentGoal = false;
    /** @type {Topic|false} */
    currentTopic = false;
    frameInfos = {};
    republisherUpdateRequested = false;
    /** @type {((tf: any) => any) | undefined} */
    _subscribeCB = undefined;
    _isDisposed = false;
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} [options.fixedFrame=base_link] - The fixed frame.
     * @param {number} [options.angularThres=2.0] - The angular threshold for the TF republisher.
     * @param {number} [options.transThres=0.01] - The translation threshold for the TF republisher.
     * @param {number} [options.rate=10.0] - The rate for the TF republisher.
     * @param {number} [options.updateDelay=50] - The time (in ms) to wait after a new subscription
     *     to update the TF republisher's list of TFs.
     * @param {number} [options.topicTimeout=2.0] - The timeout parameter for the TF republisher.
     * @param {string} [options.serverName="/tf2_web_republisher"] - The name of the tf2_web_republisher server.
     * @param {string} [options.repubServiceName="/republish_tfs"] - The name of the republish_tfs service (non groovy compatibility mode only).
     */
    constructor(options) {
      super();
      this.ros = options.ros;
      this.fixedFrame = options.fixedFrame || 'base_link';
      this.angularThres = options.angularThres || 2.0;
      this.transThres = options.transThres || 0.01;
      this.rate = options.rate || 10.0;
      this.updateDelay = options.updateDelay || 50;
      var seconds = options.topicTimeout || 2.0;
      var secs = Math.floor(seconds);
      var nsecs = Math.floor((seconds - secs) * 1000000000);
      this.topicTimeout = {
        secs: secs,
        nsecs: nsecs
      };
      this.serverName = options.serverName || '/tf2_web_republisher';
      this.repubServiceName = options.repubServiceName || '/republish_tfs';

      // Create an Action Client
      this.actionClient = new ActionClient({
        ros: options.ros,
        serverName: this.serverName,
        actionName: 'tf2_web_republisher/TFSubscriptionAction',
        omitStatus: true,
        omitResult: true
      });

      // Create a Service Client
      this.serviceClient = new Service({
        ros: options.ros,
        name: this.repubServiceName,
        serviceType: 'tf2_web_republisher/RepublishTFs'
      });
    }
    /**
     * Process the incoming TF message and send them out using the callback
     * functions.
     *
     * @param {Object} tf - The TF message from the server.
     */
    processTFArray(tf) {
      tf.transforms.forEach((transform) => {
        var frameID = transform.child_frame_id;
        if (frameID[0] === '/') {
          frameID = frameID.substring(1);
        }
        var info = this.frameInfos[frameID];
        if (info) {
          info.transform = new Transform({
            translation: transform.transform.translation,
            rotation: transform.transform.rotation
          });
          info.cbs.forEach((cb) => {
            cb(info.transform);
          });
        }
      }, this);
    }
    /**
     * Create and send a new goal (or service request) to the tf2_web_republisher
     * based on the current list of TFs.
     */
    updateGoal() {
      var goalMessage = {
        source_frames: Object.keys(this.frameInfos),
        target_frame: this.fixedFrame,
        angular_thres: this.angularThres,
        trans_thres: this.transThres,
        rate: this.rate
      };

      // if we're running in groovy compatibility mode (the default)
      // then use the action interface to tf2_web_republisher
      if (this.ros.groovyCompatibility) {
        if (this.currentGoal) {
          this.currentGoal.cancel();
        }
        this.currentGoal = new Goal({
          actionClient: this.actionClient,
          goalMessage: goalMessage
        });

        this.currentGoal.on('feedback', this.processTFArray.bind(this));
        this.currentGoal.send();
      } else {
        // otherwise, use the service interface
        // The service interface has the same parameters as the action,
        // plus the timeout
        goalMessage.timeout = this.topicTimeout;
        this.serviceClient.callService(goalMessage, this.processResponse.bind(this));
      }

      this.republisherUpdateRequested = false;
    }
    /**
     * Process the service response and subscribe to the tf republisher
     * topic.
     *
     * @param {Object} response - The service response containing the topic name.
     */
    processResponse(response) {
      // Do not setup a topic subscription if already disposed. Prevents a race condition where
      // The dispose() function is called before the service call receives a response.
      if (this._isDisposed) {
        return;
      }

      // if we subscribed to a topic before, unsubscribe so
      // the republisher stops publishing it
      if (this.currentTopic) {
        this.currentTopic.unsubscribe(this._subscribeCB);
      }

      this.currentTopic = new Topic({
        ros: this.ros,
        name: response.topic_name,
        messageType: 'tf2_web_republisher/TFArray'
      });
      this._subscribeCB = this.processTFArray.bind(this);
      this.currentTopic.subscribe(this._subscribeCB);
    }
    /**
     * @callback subscribeCallback
     * @param {Transform} callback.transform - The transform data.
     */
    /**
     * Subscribe to the given TF frame.
     *
     * @param {string} frameID - The TF frame to subscribe to.
     * @param {subscribeCallback} callback - Function with the following params:
     */
    subscribe(frameID, callback) {
      // remove leading slash, if it's there
      if (frameID[0] === '/') {
        frameID = frameID.substring(1);
      }
      // if there is no callback registered for the given frame, create empty callback list
      if (!this.frameInfos[frameID]) {
        this.frameInfos[frameID] = {
          cbs: []
        };
        if (!this.republisherUpdateRequested) {
          setTimeout(this.updateGoal.bind(this), this.updateDelay);
          this.republisherUpdateRequested = true;
        }
      }

      // if we already have a transform, callback immediately
      else if (this.frameInfos[frameID].transform) {
        callback(this.frameInfos[frameID].transform);
      }
      this.frameInfos[frameID].cbs.push(callback);
    }
    /**
     * Unsubscribe from the given TF frame.
     *
     * @param {string} frameID - The TF frame to unsubscribe from.
     * @param {function} callback - The callback function to remove.
     */
    unsubscribe(frameID, callback) {
      // remove leading slash, if it's there
      if (frameID[0] === '/') {
        frameID = frameID.substring(1);
      }
      var info = this.frameInfos[frameID];
      for (var cbs = (info && info.cbs) || [], idx = cbs.length; idx--; ) {
        if (cbs[idx] === callback) {
          cbs.splice(idx, 1);
        }
      }
      if (!callback || cbs.length === 0) {
        delete this.frameInfos[frameID];
      }
    }
    /**
     * Unsubscribe and unadvertise all topics associated with this TFClient.
     */
    dispose() {
      this._isDisposed = true;
      this.actionClient.dispose();
      if (this.currentTopic) {
        this.currentTopic.unsubscribe(this._subscribeCB);
      }
    }
  }

  /**
   * @fileOverview
   * @author Laura Lindzey - lindzey@gmail.com
   */


  /**
   * An actionlib action server client.
   *
   * Emits the following events:
   *  * 'goal' - Goal sent by action client.
   *  * 'cancel' - Action client has canceled the request.
   */
  class SimpleActionServer extends eventemitter3.EventEmitter {
    // needed for handling preemption prompted by a new goal being received
    /** @type {{goal_id: {id: any, stamp: any}, goal: any} | null} */
    currentGoal = null; // currently tracked goal
    /** @type {{goal_id: {id: any, stamp: any}, goal: any} | null} */
    nextGoal = null; // the one this'll be preempting
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} options.serverName - The action server name, like '/fibonacci'.
     * @param {string} options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
     */
    constructor(options) {
      super();
      this.ros = options.ros;
      this.serverName = options.serverName;
      this.actionName = options.actionName;

      // create and advertise publishers
      this.feedbackPublisher = new Topic({
        ros: this.ros,
        name: this.serverName + '/feedback',
        messageType: this.actionName + 'Feedback'
      });
      this.feedbackPublisher.advertise();

      var statusPublisher = new Topic({
        ros: this.ros,
        name: this.serverName + '/status',
        messageType: 'actionlib_msgs/GoalStatusArray'
      });
      statusPublisher.advertise();

      this.resultPublisher = new Topic({
        ros: this.ros,
        name: this.serverName + '/result',
        messageType: this.actionName + 'Result'
      });
      this.resultPublisher.advertise();

      // create and subscribe to listeners
      var goalListener = new Topic({
        ros: this.ros,
        name: this.serverName + '/goal',
        messageType: this.actionName + 'Goal'
      });

      var cancelListener = new Topic({
        ros: this.ros,
        name: this.serverName + '/cancel',
        messageType: 'actionlib_msgs/GoalID'
      });

      // Track the goals and their status in order to publish status...
      this.statusMessage = {
        header: {
          stamp: { secs: 0, nsecs: 100 },
          frame_id: ''
        },
        /** @type {{goal_id: any, status: number}[]} */
        status_list: []
      };

      goalListener.subscribe((goalMessage) => {
        if (this.currentGoal) {
          this.nextGoal = goalMessage;
          // needs to happen AFTER rest is set up
          this.emit('cancel');
        } else {
          this.statusMessage.status_list = [{ goal_id: goalMessage.goal_id, status: 1 }];
          this.currentGoal = goalMessage;
          this.emit('goal', goalMessage.goal);
        }
      });

      // helper function to determine ordering of timestamps
      // returns t1 < t2
      var isEarlier = function (t1, t2) {
        if (t1.secs > t2.secs) {
          return false;
        } else if (t1.secs < t2.secs) {
          return true;
        } else if (t1.nsecs < t2.nsecs) {
          return true;
        } else {
          return false;
        }
      };

      // TODO: this may be more complicated than necessary, since I'm
      // not sure if the callbacks can ever wind up with a scenario
      // where we've been preempted by a next goal, it hasn't finished
      // processing, and then we get a cancel message
      cancelListener.subscribe((cancelMessage) => {
        // cancel ALL goals if both empty
        if (
          cancelMessage.stamp.secs === 0 &&
          cancelMessage.stamp.secs === 0 &&
          cancelMessage.id === ''
        ) {
          this.nextGoal = null;
          if (this.currentGoal) {
            this.emit('cancel');
          }
        } else {
          // treat id and stamp independently
          if (
            this.currentGoal &&
            cancelMessage.id === this.currentGoal.goal_id.id
          ) {
            this.emit('cancel');
          } else if (
            this.nextGoal &&
            cancelMessage.id === this.nextGoal.goal_id.id
          ) {
            this.nextGoal = null;
          }

          if (
            this.nextGoal &&
            isEarlier(this.nextGoal.goal_id.stamp, cancelMessage.stamp)
          ) {
            this.nextGoal = null;
          }
          if (
            this.currentGoal &&
            isEarlier(this.currentGoal.goal_id.stamp, cancelMessage.stamp)
          ) {
            this.emit('cancel');
          }
        }
      });

      // publish status at pseudo-fixed rate; required for clients to know they've connected
      setInterval(() => {
        var currentTime = new Date();
        var secs = Math.floor(currentTime.getTime() / 1000);
        var nsecs = Math.round(
          1000000000 * (currentTime.getTime() / 1000 - secs)
        );
        this.statusMessage.header.stamp.secs = secs;
        this.statusMessage.header.stamp.nsecs = nsecs;
        statusPublisher.publish(this.statusMessage);
      }, 500); // publish every 500ms
    }
    /**
     * Set action state to succeeded and return to client.
     *
     * @param {Object} result - The result to return to the client.
     */
    setSucceeded(result) {
      if (this.currentGoal !== null) {
        var resultMessage = {
          status: { goal_id: this.currentGoal.goal_id, status: 3 },
          result: result
        };
        this.resultPublisher.publish(resultMessage);
    
        this.statusMessage.status_list = [];
        if (this.nextGoal) {
          this.currentGoal = this.nextGoal;
          this.nextGoal = null;
          this.emit('goal', this.currentGoal.goal);
        } else {
          this.currentGoal = null;
        }
      }
    }
    /**
     * Set action state to aborted and return to client.
     *
     * @param {Object} result - The result to return to the client.
     */
    setAborted(result) {
      if (this.currentGoal !== null) {
        var resultMessage = {
          status: { goal_id: this.currentGoal.goal_id, status: 4 },
          result: result
        };
        this.resultPublisher.publish(resultMessage);
    
        this.statusMessage.status_list = [];
        if (this.nextGoal) {
          this.currentGoal = this.nextGoal;
          this.nextGoal = null;
          this.emit('goal', this.currentGoal.goal);
        } else {
          this.currentGoal = null;
        }
      }
    }
    /**
     * Send a feedback message.
     *
     * @param {Object} feedback - The feedback to send to the client.
     */
    sendFeedback(feedback) {
      if (this.currentGoal !== null) {
        var feedbackMessage = {
          status: { goal_id: this.currentGoal.goal_id, status: 1 },
          feedback: feedback
        };
        this.feedbackPublisher.publish(feedbackMessage);
      }
    }
    /**
     * Handle case where client requests preemption.
     */
    setPreempted() {
      if (this.currentGoal !== null) { 
        this.statusMessage.status_list = [];
        var resultMessage = {
          status: { goal_id: this.currentGoal.goal_id, status: 2 }
        };
        this.resultPublisher.publish(resultMessage);
    
        if (this.nextGoal) {
          this.currentGoal = this.nextGoal;
          this.nextGoal = null;
          this.emit('goal', this.currentGoal.goal);
        } else {
          this.currentGoal = null;
        }
      }
    }
  }

  /**
   * @fileOverview
   * @author Brandon Alexander - baalexander@gmail.com
   */


  /**
   * Manages connection to the server and all interactions with ROS.
   *
   * Emits the following events:
   *  * 'error' - There was an error with ROS.
   *  * 'connection' - Connected to the WebSocket server.
   *  * 'close' - Disconnected to the WebSocket server.
   *  * &#60;topicName&#62; - A message came from rosbridge with the given topic name.
   *  * &#60;serviceID&#62; - A service response came from rosbridge with the given ID.
   */
  class Ros extends eventemitter3.EventEmitter {
    /** @type {WebSocket | import("ws").WebSocket | null} */
    socket = null;
    idCounter = 0;
    isConnected = false;
    groovyCompatibility = true;
    /**
     * @param {Object} [options]
     * @param {string} [options.url] - The WebSocket URL for rosbridge. Can be specified later with `connect`.
     * @param {boolean} [options.groovyCompatibility=true] - Don't use interfaces that changed after the last groovy release or rosbridge_suite and related tools.
     * @param {'websocket'|RTCPeerConnection} [options.transportLibrary='websocket'] - 'websocket', or an RTCPeerConnection instance controlling how the connection is created in `connect`.
     * @param {Object} [options.transportOptions={}] - The options to use when creating a connection. Currently only used if `transportLibrary` is RTCPeerConnection.
     */
    constructor(options) {
      super();
      options = options || {};
      this.transportLibrary = options.transportLibrary || 'websocket';
      this.transportOptions = options.transportOptions || {};
      this.groovyCompatibility = options.groovyCompatibility ?? true;

      // begin by checking if a URL was given
      if (options.url) {
        this.connect(options.url);
      }
    }
    /**
     * Connect to the specified WebSocket.
     *
     * @param {string} url - WebSocket URL or RTCDataChannel label for rosbridge.
     */
    connect(url) {
      if (this.transportLibrary.constructor.name === 'RTCPeerConnection') {
        this.socket = Object.assign(
          // @ts-expect-error -- this is kinda wild. `this.transportLibrary` can either be a string or an RTCDataChannel. This needs fixing.
          this.transportLibrary.createDataChannel(url, this.transportOptions),
          SocketAdapter(this)
        );
      } else if (this.transportLibrary === 'websocket') {
        if (!this.socket || this.socket.readyState === WebSocket.CLOSED) {
          // Detect if in browser vs in NodeJS
          if (typeof window !== 'undefined') {
            const sock = new WebSocket(url);
            sock.binaryType = 'arraybuffer';
            this.socket = Object.assign(sock, SocketAdapter(this));
          } else {
            // if in Node.js, import ws to replace browser WebSocket API
            import('ws').then((ws) => {
              const sock = new ws.WebSocket(url);
              sock.binaryType = 'arraybuffer';
              this.socket = Object.assign(sock, SocketAdapter(this));
            });
          }
        }
      } else {
        throw 'Unknown transportLibrary: ' + this.transportLibrary.toString();
      }
    }
    /**
     * Disconnect from the WebSocket server.
     */
    close() {
      if (this.socket) {
        this.socket.close();
      }
    }
    /**
     * Send an authorization request to the server.
     *
     * @param {string} mac - MAC (hash) string given by the trusted source.
     * @param {string} client - IP of the client.
     * @param {string} dest - IP of the destination.
     * @param {string} rand - Random string given by the trusted source.
     * @param {Object} t - Time of the authorization request.
     * @param {string} level - User level as a string given by the client.
     * @param {Object} end - End time of the client's session.
     */
    authenticate(mac, client, dest, rand, t, level, end) {
      // create the request
      var auth = {
        op: 'auth',
        mac: mac,
        client: client,
        dest: dest,
        rand: rand,
        t: t,
        level: level,
        end: end
      };
      // send the request
      this.callOnConnection(auth);
    }
    /**
     * Send an encoded message over the WebSocket.
     *
     * @param {Object} messageEncoded - The encoded message to be sent.
     */
    sendEncodedMessage(messageEncoded) {
      if (!this.isConnected) {
        this.once('connection', () => {
          if (this.socket !== null) {
            this.socket.send(messageEncoded);
          }
        });
      } else {
        if (this.socket !== null) {
          this.socket.send(messageEncoded);
        }
      }
    }
    /**
     * Send the message over the WebSocket, but queue the message up if not yet
     * connected.
     *
     * @param {Object} message - The message to be sent.
     */
    callOnConnection(message) {
      if (this.transportOptions.encoder) {
        this.transportOptions.encoder(message, this.sendEncodedMessage);
      } else {
        this.sendEncodedMessage(JSON.stringify(message));
      }
    }
    /**
     * Send a set_level request to the server.
     *
     * @param {string} level - Status level (none, error, warning, info).
     * @param {number} [id] - Operation ID to change status level on.
     */
    setStatusLevel(level, id) {
      var levelMsg = {
        op: 'set_level',
        level: level,
        id: id
      };

      this.callOnConnection(levelMsg);
    }
    /**
     * @callback getActionServersCallback
     * @param {string[]} actionservers - Array of action server names.
     */
    /**
     * @callback getActionServersFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve a list of action servers in ROS as an array of string.
     *
     * @param {getActionServersCallback} callback - Function with the following params:
     * @param {getActionServersFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getActionServers(callback, failedCallback) {
      /** @satisfies {Service<any, any>} */
      var getActionServers = new Service({
        ros: this,
        name: '/rosapi/action_servers',
        serviceType: 'rosapi/GetActionServers'
      });

      var request = {};
      if (typeof failedCallback === 'function') {
        getActionServers.callService(
          request,
          function (result) {
            callback(result.action_servers);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        getActionServers.callService(request, function (result) {
          callback(result.action_servers);
        });
      }
    }
    /**
     * @callback getTopicsCallback
     * @param {Object} result - The result object with the following params:
     * @param {string[]} result.topics - Array of topic names.
     * @param {string[]} result.types - Array of message type names.
     */
    /**
     * @callback getTopicsFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve a list of topics in ROS as an array.
     *
     * @param {getTopicsCallback} callback - Function with the following params:
     * @param {getTopicsFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getTopics(callback, failedCallback) {
      var topicsClient = new Service({
        ros: this,
        name: '/rosapi/topics',
        serviceType: 'rosapi/Topics'
      });

      var request = {};
      if (typeof failedCallback === 'function') {
        topicsClient.callService(
          request,
          function (result) {
            callback(result);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        topicsClient.callService(request, function (result) {
          callback(result);
        });
      }
    }
    /**
     * @callback getTopicsForTypeCallback
     * @param {string[]} topics - Array of topic names.
     */
    /**
     * @callback getTopicsForTypeFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve a list of topics in ROS as an array of a specific type.
     *
     * @param {string} topicType - The topic type to find.
     * @param {getTopicsForTypeCallback} callback - Function with the following params:
     * @param {getTopicsForTypeFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getTopicsForType(topicType, callback, failedCallback) {
      var topicsForTypeClient = new Service({
        ros: this,
        name: '/rosapi/topics_for_type',
        serviceType: 'rosapi/TopicsForType'
      });

      var request = {
        type: topicType
      };
      if (typeof failedCallback === 'function') {
        topicsForTypeClient.callService(
          request,
          function (result) {
            callback(result.topics);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        topicsForTypeClient.callService(request, function (result) {
          callback(result.topics);
        });
      }
    }
    /**
     * @callback getServicesCallback
     * @param {string[]} services - Array of service names.
     */
    /**
     * @callback getServicesFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve a list of active service names in ROS.
     *
     * @param {getServicesCallback} callback - Function with the following params:
     * @param {getServicesFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getServices(callback, failedCallback) {
      var servicesClient = new Service({
        ros: this,
        name: '/rosapi/services',
        serviceType: 'rosapi/Services'
      });

      var request = {};
      if (typeof failedCallback === 'function') {
        servicesClient.callService(
          request,
          function (result) {
            callback(result.services);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        servicesClient.callService(request, function (result) {
          callback(result.services);
        });
      }
    }
    /**
     * @callback getServicesForTypeCallback
     * @param {string[]} topics - Array of service names.
     */
    /**
     * @callback getServicesForTypeFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve a list of services in ROS as an array as specific type.
     *
     * @param {string} serviceType - The service type to find.
     * @param {getServicesForTypeCallback} callback - Function with the following params:
     * @param {getServicesForTypeFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getServicesForType(serviceType, callback, failedCallback) {
      var servicesForTypeClient = new Service({
        ros: this,
        name: '/rosapi/services_for_type',
        serviceType: 'rosapi/ServicesForType'
      });

      var request = {
        type: serviceType
      };
      if (typeof failedCallback === 'function') {
        servicesForTypeClient.callService(
          request,
          function (result) {
            callback(result.services);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        servicesForTypeClient.callService(request, function (result) {
          callback(result.services);
        });
      }
    }
    /**
     * @callback getServiceRequestDetailsCallback
     * @param {Object} result - The result object with the following params:
     * @param {string[]} result.typedefs - An array containing the details of the service request.
     */
    /**
     * @callback getServiceRequestDetailsFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve the details of a ROS service request.
     *
     * @param {string} type - The type of the service.
     * @param {getServiceRequestDetailsCallback} callback - Function with the following params:
     * @param {getServiceRequestDetailsFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getServiceRequestDetails(type, callback, failedCallback) {
      var serviceTypeClient = new Service({
        ros: this,
        name: '/rosapi/service_request_details',
        serviceType: 'rosapi/ServiceRequestDetails'
      });
      var request = {
        type: type
      };

      if (typeof failedCallback === 'function') {
        serviceTypeClient.callService(
          request,
          function (result) {
            callback(result);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        serviceTypeClient.callService(request, function (result) {
          callback(result);
        });
      }
    }
    /**
     * @callback getServiceResponseDetailsCallback
     * @param {{typedefs: string[]}} result - The result object with the following params:
     */
    /**
     * @callback getServiceResponseDetailsFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve the details of a ROS service response.
     *
     * @param {string} type - The type of the service.
     * @param {getServiceResponseDetailsCallback} callback - Function with the following params:
     * @param {getServiceResponseDetailsFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getServiceResponseDetails(type, callback, failedCallback) {
      /** @satisfies {Service<{},{typedefs: string[]}>} */
      var serviceTypeClient = new Service({
        ros: this,
        name: '/rosapi/service_response_details',
        serviceType: 'rosapi/ServiceResponseDetails'
      });
      var request = {
        type: type
      };

      if (typeof failedCallback === 'function') {
        serviceTypeClient.callService(
          request,
          function (result) {
            callback(result);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        serviceTypeClient.callService(request, function (result) {
          callback(result);
        });
      }
    }
    /**
     * @callback getNodesCallback
     * @param {string[]} nodes - Array of node names.
     */
    /**
     * @callback getNodesFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve a list of active node names in ROS.
     *
     * @param {getNodesCallback} callback - Function with the following params:
     * @param {getNodesFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getNodes(callback, failedCallback) {
      var nodesClient = new Service({
        ros: this,
        name: '/rosapi/nodes',
        serviceType: 'rosapi/Nodes'
      });

      var request = {};
      if (typeof failedCallback === 'function') {
        nodesClient.callService(
          request,
          function (result) {
            callback(result.nodes);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        nodesClient.callService(request, function (result) {
          callback(result.nodes);
        });
      }
    }
    /**
     * @callback getNodeDetailsCallback
     * @param {string[]} subscriptions - Array of subscribed topic names.
     * @param {string[]} publications - Array of published topic names.
     * @param {string[]} services - Array of service names hosted.
     */
    /**
     * @callback getNodeDetailsFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * @callback getNodeDetailsLegacyCallback
     * @param {Object} result - The result object with the following params:
     * @param {string[]} result.subscribing - Array of subscribed topic names.
     * @param {string[]} result.publishing - Array of published topic names.
     * @param {string[]} result.services - Array of service names hosted.
     */
    /**
     * Retrieve a list of subscribed topics, publishing topics and services of a specific node.
     * <br>
     * These are the parameters if failedCallback is <strong>defined</strong>.
     *
     * @param {string} node - Name of the node.
     * @param {getNodeDetailsCallback} callback - Function with the following params:
     * @param {getNodeDetailsFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     *
     * @also
     *
     * Retrieve a list of subscribed topics, publishing topics and services of a specific node.
     * <br>
     * These are the parameters if failedCallback is <strong>undefined</strong>.
     *
     * @param {string} node - Name of the node.
     * @param {getNodeDetailsLegacyCallback} callback - Function with the following params:
     * @param {getNodeDetailsFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getNodeDetails(node, callback, failedCallback) {
      var nodesClient = new Service({
        ros: this,
        name: '/rosapi/node_details',
        serviceType: 'rosapi/NodeDetails'
      });

      var request = {
        node: node
      };
      if (typeof failedCallback === 'function') {
        nodesClient.callService(
          request,
          function (result) {
            callback(result.subscribing, result.publishing, result.services);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        nodesClient.callService(request, function (result) {
          // @ts-expect-error -- callback parameter polymorphism, see JSDoc comment above
          callback(result);
        });
      }
    }
    /**
     * @callback getParamsCallback
     * @param {string[]} params - Array of param names.
     */
    /**
     * @callback getParamsFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve a list of parameter names from the ROS Parameter Server.
     *
     * @param {getParamsCallback} callback - Function with the following params:
     * @param {getParamsFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getParams(callback, failedCallback) {
      var paramsClient = new Service({
        ros: this,
        name: '/rosapi/get_param_names',
        serviceType: 'rosapi/GetParamNames'
      });
      var request = {};
      if (typeof failedCallback === 'function') {
        paramsClient.callService(
          request,
          function (result) {
            callback(result.names);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        paramsClient.callService(request, function (result) {
          callback(result.names);
        });
      }
    }
    /**
     * @callback getTopicTypeCallback
     * @param {string} type - The type of the topic.
     */
    /**
     * @callback getTopicTypeFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve the type of a ROS topic.
     *
     * @param {string} topic - Name of the topic.
     * @param {getTopicTypeCallback} callback - Function with the following params:
     * @param {getTopicTypeFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getTopicType(topic, callback, failedCallback) {
      var topicTypeClient = new Service({
        ros: this,
        name: '/rosapi/topic_type',
        serviceType: 'rosapi/TopicType'
      });
      var request = {
        topic: topic
      };

      if (typeof failedCallback === 'function') {
        topicTypeClient.callService(
          request,
          function (result) {
            callback(result.type);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        topicTypeClient.callService(request, function (result) {
          callback(result.type);
        });
      }
    }
    /**
     * @callback getServiceTypeCallback
     * @param {string} type - The type of the service.
     */
    /**
     * @callback getServiceTypeFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve the type of a ROS service.
     *
     * @param {string} service - Name of the service.
     * @param {getServiceTypeCallback} callback - Function with the following params:
     * @param {getServiceTypeFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getServiceType(service, callback, failedCallback) {
      var serviceTypeClient = new Service({
        ros: this,
        name: '/rosapi/service_type',
        serviceType: 'rosapi/ServiceType'
      });
      var request = {
        service: service
      };

      if (typeof failedCallback === 'function') {
        serviceTypeClient.callService(
          request,
          function (result) {
            callback(result.type);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        serviceTypeClient.callService(request, function (result) {
          callback(result.type);
        });
      }
    }
    /**
     * @callback getMessageDetailsCallback
     * @param {string} details - An array of the message details.
     */
    /**
     * @callback getMessageDetailsFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve the details of a ROS message.
     *
     * @param {string} message - The name of the message type.
     * @param {getMessageDetailsCallback} callback - Function with the following params:
     * @param {getMessageDetailsFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getMessageDetails(message, callback, failedCallback) {
      var messageDetailClient = new Service({
        ros: this,
        name: '/rosapi/message_details',
        serviceType: 'rosapi/MessageDetails'
      });
      var request = {
        type: message
      };

      if (typeof failedCallback === 'function') {
        messageDetailClient.callService(
          request,
          function (result) {
            callback(result.typedefs);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        messageDetailClient.callService(request, function (result) {
          callback(result.typedefs);
        });
      }
    }
    /**
     * Decode a typedef array into a dictionary like `rosmsg show foo/bar`.
     *
     * @param {Object[]} defs - Array of type_def dictionary.
     */
    decodeTypeDefs(defs) {
      var decodeTypeDefsRec = (theType, hints) => {
        // calls itself recursively to resolve type definition using hints.
        var typeDefDict = {};
        for (var i = 0; i < theType.fieldnames.length; i++) {
          var arrayLen = theType.fieldarraylen[i];
          var fieldName = theType.fieldnames[i];
          var fieldType = theType.fieldtypes[i];
          if (fieldType.indexOf('/') === -1) {
            // check the fieldType includes '/' or not
            if (arrayLen === -1) {
              typeDefDict[fieldName] = fieldType;
            } else {
              typeDefDict[fieldName] = [fieldType];
            }
          } else {
            // lookup the name
            var sub = false;
            for (var j = 0; j < hints.length; j++) {
              if (hints[j].type.toString() === fieldType.toString()) {
                sub = hints[j];
                break;
              }
            }
            if (sub) {
              var subResult = decodeTypeDefsRec(sub, hints);
              if (arrayLen === -1) {
                typeDefDict[fieldName] = subResult; // add this decoding result to dictionary
              } else {
                typeDefDict[fieldName] = [subResult];
              }
            } else {
              this.emit(
                'error',
                'Cannot find ' + fieldType + ' in decodeTypeDefs'
              );
            }
          }
        }
        return typeDefDict;
      };

      return decodeTypeDefsRec(defs[0], defs);
    }
    /**
     * @callback getTopicsAndRawTypesCallback
     * @param {Object} result - The result object with the following params:
     * @param {string[]} result.topics - Array of topic names.
     * @param {string[]} result.types - Array of message type names.
     * @param {string[]} result.typedefs_full_text - Array of full definitions of message types, similar to `gendeps --cat`.
     */
    /**
     * @callback getTopicsAndRawTypesFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Retrieve a list of topics and their associated type definitions.
     *
     * @param {getTopicsAndRawTypesCallback} callback - Function with the following params:
     * @param {getTopicsAndRawTypesFailedCallback} [failedCallback] - The callback function when the service call failed with params:
     */
    getTopicsAndRawTypes(callback, failedCallback) {
      var topicsAndRawTypesClient = new Service({
        ros: this,
        name: '/rosapi/topics_and_raw_types',
        serviceType: 'rosapi/TopicsAndRawTypes'
      });

      var request = {};
      if (typeof failedCallback === 'function') {
        topicsAndRawTypesClient.callService(
          request,
          function (result) {
            callback(result);
          },
          function (message) {
            failedCallback(message);
          }
        );
      } else {
        topicsAndRawTypesClient.callService(request, function (result) {
          callback(result);
        });
      }
    }
    Topic(options) {
      return new Topic({ ros: this, ...options });
    }
    Param(options) {
      return new Param({ ros: this, ...options });
    }
    Service(options) {
      return new Service({ ros: this, ...options });
    }
    TFClient(options) {
      return new TFClient({ ros: this, ...options });
    }
    ActionClient(options) {
      return new ActionClient({ ros: this, ...options });
    }
    SimpleActionServer(options) {
      return new SimpleActionServer({ ros: this, ...options });
    }
  }

  /**
   * @fileOverview
   * @author Sebastian Castro - sebastian.castro@picknik.ai
   */


  /**
   * A ROS 2 action client.
   * @template TGoal, TFeedback, TResult
   */
  class Action extends eventemitter3.EventEmitter {
    isAdvertised = false;
    /**
     * @callback advertiseActionCallback
     * @param {TGoal} goal - The action goal.
     * @param {string} id - The ID of the action goal to execute.
     */
    /**
     * @private
     * @type {advertiseActionCallback | null}
     */
    _actionCallback = null;
    /**
     * @callback advertiseCancelCallback
     * @param {string} id - The ID of the action goal to cancel.
     */
    /**
     * @private
     * @type {advertiseCancelCallback | null}
     */
    _cancelCallback = null;
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} options.name - The action name, like '/fibonacci'.
     * @param {string} options.actionType - The action type, like 'action_tutorials_interfaces/Fibonacci'.
     */
    constructor(options) {
      super();
      this.ros = options.ros;
      this.name = options.name;
      this.actionType = options.actionType;
    }

    /**
     * @callback sendGoalResultCallback
     * @param {TResult} result - The result from the action.
     */
    /**
     * @callback sendGoalFeedbackCallback
     * @param {TFeedback} feedback - The feedback from the action.
     */
    /**
     * @callback sendGoalFailedCallback
     * @param {string} error - The error message reported by ROS.
     */
    /**
     * Sends an action goal. Returns the feedback in the feedback callback while the action is running
     * and the result in the result callback when the action is completed.
     * Does nothing if this action is currently advertised.
     *
     * @param {TGoal} goal - The action goal to send.
     * @param {sendGoalResultCallback} resultCallback - The callback function when the action is completed.
     * @param {sendGoalFeedbackCallback} [feedbackCallback] - The callback function when the action pulishes feedback.
     * @param {sendGoalFailedCallback} [failedCallback] - The callback function when the action failed.
     */
    sendGoal(goal, resultCallback, feedbackCallback, failedCallback) {
      if (this.isAdvertised) {
        return;
      }

      var actionGoalId =
        'send_action_goal:' + this.name + ':' + ++this.ros.idCounter;

      if (resultCallback || failedCallback) {
        this.ros.on(actionGoalId, function (message) {
          if (message.result !== undefined && message.result === false) {
            if (typeof failedCallback === 'function') {
              failedCallback(message.values);
            }
          } else if (
            message.op === 'action_feedback' &&
            typeof feedbackCallback === 'function'
          ) {
            feedbackCallback(message.values);
          } else if (
            message.op === 'action_result' &&
            typeof resultCallback === 'function'
          ) {
            resultCallback(message.values);
          }
        });
      }

      var call = {
        op: 'send_action_goal',
        id: actionGoalId,
        action: this.name,
        action_type: this.actionType,
        args: goal,
        feedback: true,
      };
      this.ros.callOnConnection(call);

      return actionGoalId;
    }

    /**
     * Cancels an action goal.
     *
     * @param {string} id - The ID of the action goal to cancel.
     */
    cancelGoal(id) {
      var call = {
        op: 'cancel_action_goal',
        id: id,
        action: this.name
      };
      this.ros.callOnConnection(call);
    }

    /**
     * Advertise the action. This turns the Action object from a client
     * into a server. The callback will be called with every goal sent to this action.
     *
     * @param {advertiseActionCallback} actionCallback - This works similarly to the callback for a C++ action.
     * @param {advertiseCancelCallback} cancelCallback - A callback function to execute when the action is canceled.
     */
    advertise(actionCallback, cancelCallback) {
      if (this.isAdvertised || typeof actionCallback !== 'function') {
        return;
      }

      this._actionCallback = actionCallback;
      this._cancelCallback = cancelCallback;
      this.ros.on(this.name, this._executeAction.bind(this));
      this.ros.callOnConnection({
        op: 'advertise_action',
        type: this.actionType,
        action: this.name
      });
      this.isAdvertised = true;
    }

    /**
     * Unadvertise a previously advertised action.
     */
    unadvertise() {
      if (!this.isAdvertised) {
        return;
      }
      this.ros.callOnConnection({
        op: 'unadvertise_action',
        action: this.name
      });
      this.isAdvertised = false;
    }

    /**
     * Helper function that executes an action by calling the provided
     * action callback with the auto-generated ID as a user-accessible input.
     * Should not be called manually.
     *
     * @param {Object} rosbridgeRequest - The rosbridge request containing the action goal to send and its ID.
     * @param {string} rosbridgeRequest.id - The ID of the action goal.
     * @param {TGoal} rosbridgeRequest.args - The arguments of the action goal.
     */
    _executeAction(rosbridgeRequest) {
      var id = rosbridgeRequest.id;

      // If a cancellation callback exists, call it when a cancellation event is emitted.
      if (typeof id === 'string') {
        this.ros.on(id, (message) => {
          if (
            message.op === 'cancel_action_goal' &&
            typeof this._cancelCallback === 'function'
          ) {
            this._cancelCallback(id);
          }
        });
      }

      // Call the action goal execution function provided.
      if (typeof this._actionCallback === 'function') {
        this._actionCallback(rosbridgeRequest.args, id);
      }
    }

    /**
     * Helper function to send action feedback inside an action handler.
     *
     * @param {string} id - The action goal ID.
     * @param {TFeedback} feedback - The feedback to send.
     */
    sendFeedback(id, feedback) {
      var call = {
        op: 'action_feedback',
        id: id,
        action: this.name,
        values: feedback
      };
      this.ros.callOnConnection(call);
    }

    /**
     * Helper function to set an action as succeeded.
     *
     * @param {string} id - The action goal ID.
     * @param {TResult} result - The result to set.
     */
    setSucceeded(id, result) {
      var call = {
        op: 'action_result',
        id: id,
        action: this.name,
        values: result,
        result: true
      };
      this.ros.callOnConnection(call);
    }

    /**
     * Helper function to set an action as failed.
     *
     * @param {string} id - The action goal ID.
     */
    setFailed(id) {
      var call = {
        op: 'action_result',
        id: id,
        action: this.name,
        result: false
      };
      this.ros.callOnConnection(call);
    }
  }

  var Core = /*#__PURE__*/Object.freeze({
    __proto__: null,
    Action: Action,
    Param: Param,
    Ros: Ros,
    Service: Service,
    Topic: Topic
  });

  /**
   * @fileOverview
   * @author Justin Young - justin@oodar.com.au
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * An actionlib action listener.
   *
   * Emits the following events:
   *  * 'status' - The status messages received from the action server.
   *  * 'feedback' - The feedback messages received from the action server.
   *  * 'result' - The result returned from the action server.
   *

   */
  class ActionListener extends eventemitter3.EventEmitter {
    /**
     * @param {Object} options
     * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
     * @param {string} options.serverName - The action server name, like '/fibonacci'.
     * @param {string} options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
     */
    constructor(options) {
      super();
      this.ros = options.ros;
      this.serverName = options.serverName;
      this.actionName = options.actionName;

      // create the topics associated with actionlib
      var goalListener = new Topic({
        ros: this.ros,
        name: this.serverName + '/goal',
        messageType: this.actionName + 'Goal'
      });

      var feedbackListener = new Topic({
        ros: this.ros,
        name: this.serverName + '/feedback',
        messageType: this.actionName + 'Feedback'
      });

      var statusListener = new Topic({
        ros: this.ros,
        name: this.serverName + '/status',
        messageType: 'actionlib_msgs/GoalStatusArray'
      });

      var resultListener = new Topic({
        ros: this.ros,
        name: this.serverName + '/result',
        messageType: this.actionName + 'Result'
      });

      goalListener.subscribe((goalMessage) => {
        this.emit('goal', goalMessage);
      });

      statusListener.subscribe((statusMessage) => {
        statusMessage.status_list.forEach((status) => {
          this.emit('status', status);
        });
      });

      feedbackListener.subscribe((feedbackMessage) => {
        this.emit('status', feedbackMessage.status);
        this.emit('feedback', feedbackMessage.feedback);
      });

      // subscribe to the result topic
      resultListener.subscribe((resultMessage) => {
        this.emit('status', resultMessage.status);
        this.emit('result', resultMessage.result);
      });
    }
  }

  var ActionLib = /*#__PURE__*/Object.freeze({
    __proto__: null,
    ActionClient: ActionClient,
    ActionListener: ActionListener,
    Goal: Goal,
    SimpleActionServer: SimpleActionServer
  });

  /**
   * @fileOverview
   * @author David Gossow - dgossow@willowgarage.com
   */


  /**
   * A Pose in 3D space. Values are copied into this object.
   */
  class Pose {
    /**
     * @param {Object} [options]
     * @param {Vector3} [options.position] - The ROSLIB.Vector3 describing the position.
     * @param {Quaternion} [options.orientation] - The ROSLIB.Quaternion describing the orientation.
     */
    constructor(options) {
      options = options || {};
      // copy the values into this object if they exist
      options = options || {};
      this.position = new Vector3(options.position);
      this.orientation = new Quaternion(options.orientation);
    }
    /**
     * Apply a transform against this pose.
     *
     * @param {Transform} tf - The transform to be applied.
     */
    applyTransform(tf) {
      this.position.multiplyQuaternion(tf.rotation);
      this.position.add(tf.translation);
      var tmp = tf.rotation.clone();
      tmp.multiply(this.orientation);
      this.orientation = tmp;
    }
    /**
     * Clone a copy of this pose.
     *
     * @returns {Pose} The cloned pose.
     */
    clone() {
      return new Pose(this);
    }
    /**
     * Multiply this pose with another pose without altering this pose.
     *
     * @returns {Pose} The result of the multiplication.
     */
    multiply(pose) {
      var p = pose.clone();
      p.applyTransform({
        rotation: this.orientation,
        translation: this.position
      });
      return p;
    }
    /**
     * Compute the inverse of this pose.
     *
     * @returns {Pose} The inverse of the pose.
     */
    getInverse() {
      var inverse = this.clone();
      inverse.orientation.invert();
      inverse.position.multiplyQuaternion(inverse.orientation);
      inverse.position.x *= -1;
      inverse.position.y *= -1;
      inverse.position.z *= -1;
      return inverse;
    }
  }

  var Math$1 = /*#__PURE__*/Object.freeze({
    __proto__: null,
    Pose: Pose,
    Quaternion: Quaternion,
    Transform: Transform,
    Vector3: Vector3
  });

  /**
   * A TF Client that listens to TFs from tf2_web_republisher.
   */
  class ROS2TFClient extends eventemitter3.EventEmitter {
      /**
       * @param {Object} options
       * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
       * @param {string} [options.fixedFrame=base_link] - The fixed frame.
       * @param {number} [options.angularThres=2.0] - The angular threshold for the TF republisher.
       * @param {number} [options.transThres=0.01] - The translation threshold for the TF republisher.
       * @param {number} [options.rate=10.0] - The rate for the TF republisher.
       * @param {number} [options.updateDelay=50] - The time (in ms) to wait after a new subscription
       *     to update the TF republisher's list of TFs.
       * @param {number} [options.topicTimeout=2.0] - The timeout parameter for the TF republisher.
       * @param {string} [options.serverName="/tf2_web_republisher"] - The name of the tf2_web_republisher server.
       * @param {string} [options.repubServiceName="/republish_tfs"] - The name of the republish_tfs service (non groovy compatibility mode only).
       */
      constructor(options) {
          super();
          this.ros = options.ros;
          this.fixedFrame = options.fixedFrame || 'base_link';
          this.angularThres = options.angularThres || 2.0;
          this.transThres = options.transThres || 0.01;
          this.rate = options.rate || 10.0;
          this.updateDelay = options.updateDelay || 50;
          const seconds = options.topicTimeout || 2.0;
          const secs = Math.floor(seconds);
          const nsecs = Math.floor((seconds - secs) * 1E9);
          this.topicTimeout = {
              secs: secs,
              nsecs: nsecs
          };
          this.serverName = options.serverName || '/tf2_web_republisher';
          this.goal_id = '';
          this.frameInfos = {};
          this.republisherUpdateRequested = false;
          this._subscribeCB = undefined;
          this._isDisposed = false;

          // Create an Action Client
          this.actionClient = new Action({
              ros: options.ros,
              name: this.serverName,
              actionType: 'tf2_web_republisher_msgs/TFSubscription',
          });

      }

      /**
       * Process the incoming TF message and send them out using the callback
       * functions.
       *
       * @param {Object} tf - The TF message from the server.
       */
      processTFArray(tf) {
          let that = this;
          tf.transforms.forEach(function (transform) {
              let frameID = transform.child_frame_id;
              if (frameID[0] === '/') {
                  frameID = frameID.substring(1);
              }
              const info = that.frameInfos[frameID];
              if (info) {
                  info.transform = new Transform({
                      translation: transform.transform.translation,
                      rotation: transform.transform.rotation
                  });
                  info.cbs.forEach(function (cb) {
                      cb(info.transform);
                  });
              }
          }, this);
      }

      /**
       * Create and send a new goal (or service request) to the tf2_web_republisher
       * based on the current list of TFs.
       */
      updateGoal() {
          const goalMessage = {
              source_frames: Object.keys(this.frameInfos),
              target_frame: this.fixedFrame,
              angular_thres: this.angularThres,
              trans_thres: this.transThres,
              rate: this.rate
          };

          if (this.goal_id !== '') {
              this.actionClient.cancelGoal(this.goal_id);
          }
          this.currentGoal = goalMessage;

          const id = this.actionClient.sendGoal(goalMessage,
              (result) => {
              },
              (feedback) => {
                  this.processTFArray(feedback);
              },
          );
          if (typeof id === 'string') {
              this.goal_id = id;
          }

          this.republisherUpdateRequested = false;
      }

      /**
       * @callback subscribeCallback
       * @param {Transform} callback.transform - The transform data.
       */
      /**
       * Subscribe to the given TF frame.
       *
       * @param {string} frameID - The TF frame to subscribe to.
       * @param {subscribeCallback} callback - Function with the following params:
       */
      subscribe(frameID, callback) {
          // remove leading slash, if it's there
          if (frameID[0] === '/') {
              frameID = frameID.substring(1);
          }
          // if there is no callback registered for the given frame, create empty callback list
          if (!this.frameInfos[frameID]) {
              this.frameInfos[frameID] = {
                  cbs: []
              };
              if (!this.republisherUpdateRequested) {
                  setTimeout(this.updateGoal.bind(this), this.updateDelay);
                  this.republisherUpdateRequested = true;
              }
          }

          // if we already have a transform, callback immediately
          else if (this.frameInfos[frameID].transform) {
              callback(this.frameInfos[frameID].transform);
          }
          this.frameInfos[frameID].cbs.push(callback);
      }

      /**
       * Unsubscribe from the given TF frame.
       *
       * @param {string} frameID - The TF frame to unsubscribe from.
       * @param {function} callback - The callback function to remove.
       */
      unsubscribe(frameID, callback) {
          // remove leading slash, if it's there
          if (frameID[0] === '/') {
              frameID = frameID.substring(1);
          }
          const info = this.frameInfos[frameID];
          for (var cbs = (info && info.cbs) || [], idx = cbs.length; idx--;) {
              if (cbs[idx] === callback) {
                  cbs.splice(idx, 1);
              }
          }
          if (!callback || cbs.length === 0) {
              delete this.frameInfos[frameID];
          }
      }

      /**
       * Unsubscribe and unadvertise all topics associated with this TFClient.
       */
      dispose() {
          this._isDisposed = true;
      }
  }

  var Tf = /*#__PURE__*/Object.freeze({
    __proto__: null,
    ROS2TFClient: ROS2TFClient,
    TFClient: TFClient
  });

  const URDF_SPHERE = 0;
  const URDF_BOX = 1;
  const URDF_CYLINDER = 2;
  const URDF_MESH = 3;

  /**
   * @fileOverview
   * @author Benjamin Pitzer - ben.pitzer@gmail.com
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * A Box element in a URDF.
   */
  class UrdfBox {
    /** @type {Vector3 | null} */
    dimension;
    /**
     * @param {Object} options
     * @param {Element} options.xml - The XML element to parse.
     */
    constructor(options) {
      this.type = URDF_BOX;

      // Parse the xml string
      var xyz = options.xml.getAttribute('size')?.split(' ');
      if (xyz) {
        this.dimension = new Vector3({
          x: parseFloat(xyz[0]),
          y: parseFloat(xyz[1]),
          z: parseFloat(xyz[2])
        });
      } else {
        this.dimension = null;
      }
    }
  }

  /**
   * @fileOverview
   * @author Benjamin Pitzer - ben.pitzer@gmail.com
   * @author Russell Toris - rctoris@wpi.edu
   */

  /**
   * A Color element in a URDF.
   */
  class UrdfColor {
    /**
     * @param {Object} options
     * @param {Element} options.xml - The XML element to parse.
     */
    constructor(options) {
      // Parse the xml string
      var rgba = options.xml.getAttribute('rgba')?.split(' ');
      if (rgba) {
        this.r = parseFloat(rgba[0]);
        this.g = parseFloat(rgba[1]);
        this.b = parseFloat(rgba[2]);
        this.a = parseFloat(rgba[3]);
      }
    }
  }

  /**
   * @fileOverview
   * @author Benjamin Pitzer - ben.pitzer@gmail.com
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * A Cylinder element in a URDF.
   */
  class UrdfCylinder {
    /**
     * @param {Object} options
     * @param {Element} options.xml - The XML element to parse.
     */
    constructor(options) {
      this.type = URDF_CYLINDER;
      // @ts-expect-error -- possibly null
      this.length = parseFloat(options.xml.getAttribute('length'));
      // @ts-expect-error -- possibly null
      this.radius = parseFloat(options.xml.getAttribute('radius'));
    }
  }

  /**
   * @fileOverview
   * @author Benjamin Pitzer - ben.pitzer@gmail.com
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * A Material element in a URDF.
   */
  class UrdfMaterial {
    /** @type {string | null} */
    textureFilename = null;
    /** @type {UrdfColor | null} */
    color = null;
    /**
     * @param {Object} options
     * @param {Element} options.xml - The XML element to parse.
     */
    constructor(options) {

      this.name = options.xml.getAttribute('name');

      // Texture
      var textures = options.xml.getElementsByTagName('texture');
      if (textures.length > 0) {
        this.textureFilename = textures[0].getAttribute('filename');
      }

      // Color
      var colors = options.xml.getElementsByTagName('color');
      if (colors.length > 0) {
        // Parse the RBGA string
        this.color = new UrdfColor({
          xml: colors[0]
        });
      }
    }
    isLink() {
      return this.color === null && this.textureFilename === null;
    }
    assign(obj) {
      return Object.assign(this, obj);
    }
  }

  /**
   * @fileOverview
   * @author Benjamin Pitzer - ben.pitzer@gmail.com
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * A Mesh element in a URDF.
   */
  class UrdfMesh {
    /** @type {Vector3 | null} */
    scale = null;
    /**
     * @param {Object} options
     * @param {Element} options.xml - The XML element to parse.
     */
    constructor(options) {
      this.type = URDF_MESH;
      this.filename = options.xml.getAttribute('filename');

      // Check for a scale
      var scale = options.xml.getAttribute('scale');
      if (scale) {
        // Get the XYZ
        var xyz = scale.split(' ');
        this.scale = new Vector3({
          x: parseFloat(xyz[0]),
          y: parseFloat(xyz[1]),
          z: parseFloat(xyz[2])
        });
      }
    }
  }

  /**
   * @fileOverview
   * @author Benjamin Pitzer - ben.pitzer@gmail.com
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * A Sphere element in a URDF.
   */
  class UrdfSphere {
    /**
     * @param {Object} options
     * @param {Element} options.xml - The XML element to parse.
     */
    constructor(options) {
      this.type = URDF_SPHERE;
      this.radius = parseFloat(options.xml.getAttribute('radius') || 'NaN');
    }
  }

  /**
   * @fileOverview
   * @author Benjamin Pitzer - ben.pitzer@gmail.com
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * A Visual element in a URDF.
   */
  class UrdfVisual {
    /** @type {Pose | null} */
    origin = null;
    /** @type {UrdfMesh | UrdfSphere | UrdfBox | UrdfCylinder | null} */
    geometry = null;
    /** @type {UrdfMaterial | null} */
    material = null;
    /**
     * @param {Object} options
     * @param {Element} options.xml - The XML element to parse.
     */
    constructor(options) {
      var xml = options.xml;
      this.name = options.xml.getAttribute('name');

      // Origin
      var origins = xml.getElementsByTagName('origin');
      if (origins.length === 0) {
        // use the identity as the default
        this.origin = new Pose();
      } else {
        // Check the XYZ
        var xyzValue = origins[0].getAttribute('xyz');
        var position = new Vector3();
        if (xyzValue) {
          var xyz = xyzValue.split(' ');
          position = new Vector3({
            x: parseFloat(xyz[0]),
            y: parseFloat(xyz[1]),
            z: parseFloat(xyz[2])
          });
        }

        // Check the RPY
        var rpyValue = origins[0].getAttribute('rpy');
        var orientation = new Quaternion();
        if (rpyValue) {
          var rpy = rpyValue.split(' ');
          // Convert from RPY
          var roll = parseFloat(rpy[0]);
          var pitch = parseFloat(rpy[1]);
          var yaw = parseFloat(rpy[2]);
          var phi = roll / 2.0;
          var the = pitch / 2.0;
          var psi = yaw / 2.0;
          var x =
            Math.sin(phi) * Math.cos(the) * Math.cos(psi) -
            Math.cos(phi) * Math.sin(the) * Math.sin(psi);
          var y =
            Math.cos(phi) * Math.sin(the) * Math.cos(psi) +
            Math.sin(phi) * Math.cos(the) * Math.sin(psi);
          var z =
            Math.cos(phi) * Math.cos(the) * Math.sin(psi) -
            Math.sin(phi) * Math.sin(the) * Math.cos(psi);
          var w =
            Math.cos(phi) * Math.cos(the) * Math.cos(psi) +
            Math.sin(phi) * Math.sin(the) * Math.sin(psi);

          orientation = new Quaternion({
            x: x,
            y: y,
            z: z,
            w: w
          });
          orientation.normalize();
        }
        this.origin = new Pose({
          position: position,
          orientation: orientation
        });
      }

      // Geometry
      var geoms = xml.getElementsByTagName('geometry');
      if (geoms.length > 0) {
        var geom = geoms[0];
        var shape = null;
        // Check for the shape
        for (var i = 0; i < geom.childNodes.length; i++) {
          /** @type {Element} */
          // @ts-expect-error -- unknown why this doesn't work properly.
          var node = geom.childNodes[i];
          if (node.nodeType === 1) {
            shape = node;
            break;
          }
        }
        if (shape) {
          // Check the type
          var type = shape.nodeName;
          if (type === 'sphere') {
            this.geometry = new UrdfSphere({
              xml: shape
            });
          } else if (type === 'box') {
            this.geometry = new UrdfBox({
              xml: shape
            });
          } else if (type === 'cylinder') {
            this.geometry = new UrdfCylinder({
              xml: shape
            });
          } else if (type === 'mesh') {
            this.geometry = new UrdfMesh({
              xml: shape
            });
          } else {
            console.warn('Unknown geometry type ' + type);
          }
        }
      }

      // Material
      var materials = xml.getElementsByTagName('material');
      if (materials.length > 0) {
        this.material = new UrdfMaterial({
          xml: materials[0]
        });
      }
    }
  }

  /**
   * @fileOverview
   * @author Benjamin Pitzer - ben.pitzer@gmail.com
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * A Link element in a URDF.
   */
  class UrdfLink {
    /**
     * @param {Object} options
     * @param {Element} options.xml - The XML element to parse.
     */
    constructor(options) {
      this.name = options.xml.getAttribute('name');
      this.visuals = [];
      var visuals = options.xml.getElementsByTagName('visual');

      for (var i = 0; i < visuals.length; i++) {
        this.visuals.push(
          new UrdfVisual({
            xml: visuals[i]
          })
        );
      }
    }
  }

  /**
   * @fileOverview
   * @author David V. Lu!! - davidvlu@gmail.com
   */


  /**
   * A Joint element in a URDF.
   */
  class UrdfJoint {
    /**
     * @param {Object} options
     * @param {Element} options.xml - The XML element to parse.
     */
    constructor(options) {
      this.name = options.xml.getAttribute('name');
      this.type = options.xml.getAttribute('type');

      var parents = options.xml.getElementsByTagName('parent');
      if (parents.length > 0) {
        this.parent = parents[0].getAttribute('link');
      }

      var children = options.xml.getElementsByTagName('child');
      if (children.length > 0) {
        this.child = children[0].getAttribute('link');
      }

      var limits = options.xml.getElementsByTagName('limit');
      if (limits.length > 0) {
        this.minval = parseFloat(limits[0].getAttribute('lower') || 'NaN');
        this.maxval = parseFloat(limits[0].getAttribute('upper') || 'NaN');
      }

      // Origin
      var origins = options.xml.getElementsByTagName('origin');
      if (origins.length === 0) {
        // use the identity as the default
        this.origin = new Pose();
      } else {
        // Check the XYZ
        var xyzValue = origins[0].getAttribute('xyz');
        var position = new Vector3();
        if (xyzValue) {
          var xyz = xyzValue.split(' ');
          position = new Vector3({
            x: parseFloat(xyz[0]),
            y: parseFloat(xyz[1]),
            z: parseFloat(xyz[2])
          });
        }

        // Check the RPY
        var rpyValue = origins[0].getAttribute('rpy');
        var orientation = new Quaternion();
        if (rpyValue) {
          var rpy = rpyValue.split(' ');
          // Convert from RPY
          var roll = parseFloat(rpy[0]);
          var pitch = parseFloat(rpy[1]);
          var yaw = parseFloat(rpy[2]);
          var phi = roll / 2.0;
          var the = pitch / 2.0;
          var psi = yaw / 2.0;
          var x =
            Math.sin(phi) * Math.cos(the) * Math.cos(psi) -
            Math.cos(phi) * Math.sin(the) * Math.sin(psi);
          var y =
            Math.cos(phi) * Math.sin(the) * Math.cos(psi) +
            Math.sin(phi) * Math.cos(the) * Math.sin(psi);
          var z =
            Math.cos(phi) * Math.cos(the) * Math.sin(psi) -
            Math.sin(phi) * Math.sin(the) * Math.cos(psi);
          var w =
            Math.cos(phi) * Math.cos(the) * Math.cos(psi) +
            Math.sin(phi) * Math.sin(the) * Math.sin(psi);

          orientation = new Quaternion({
            x: x,
            y: y,
            z: z,
            w: w
          });
          orientation.normalize();
        }
        this.origin = new Pose({
          position: position,
          orientation: orientation
        });
      }
    }
  }

  /**
   * @fileOverview
   * @author Benjamin Pitzer - ben.pitzer@gmail.com
   * @author Russell Toris - rctoris@wpi.edu
   */


  /**
   * A URDF Model can be used to parse a given URDF into the appropriate elements.
   */
  class UrdfModel {
    materials = {};
    links = {};
    joints = {};
    /**
     * @param {Object} options
     * @param {Element} [options.xml] - The XML element to parse.
     * @param {string} [options.string] - The XML element to parse as a string.
     */
    constructor(options) {
      var xmlDoc = options.xml;
      var string = options.string;

      // Check if we are using a string or an XML element
      if (string) {
        // Parse the string
        var parser = new xmldom.DOMParser();
        xmlDoc = parser.parseFromString(string, 'text/xml').documentElement;
      }
      if (!xmlDoc) {
        throw new Error('No URDF document parsed!');
      }

      // Initialize the model with the given XML node.
      // Get the robot tag
      var robotXml = xmlDoc;

      // Get the robot name
      this.name = robotXml.getAttribute('name');

      // Parse all the visual elements we need
      for (var nodes = robotXml.childNodes, i = 0; i < nodes.length; i++) {
        /** @type {Element} */
        // @ts-expect-error -- unknown why this doesn't work properly.
        var node = nodes[i];
        if (node.tagName === 'material') {
          var material = new UrdfMaterial({
            xml: node
          });
          // Make sure this is unique
          if (this.materials[material.name] !== void 0) {
            if (this.materials[material.name].isLink()) {
              this.materials[material.name].assign(material);
            } else {
              console.warn('Material ' + material.name + 'is not unique.');
            }
          } else {
            this.materials[material.name] = material;
          }
        } else if (node.tagName === 'link') {
          var link = new UrdfLink({
            xml: node
          });
          // Make sure this is unique
          if (this.links[link.name] !== void 0) {
            console.warn('Link ' + link.name + ' is not unique.');
          } else {
            // Check for a material
            for (var j = 0; j < link.visuals.length; j++) {
              var mat = link.visuals[j].material;
              if (mat !== null && mat.name) {
                if (this.materials[mat.name] !== void 0) {
                  link.visuals[j].material = this.materials[mat.name];
                } else {
                  this.materials[mat.name] = mat;
                }
              }
            }

            // Add the link
            this.links[link.name] = link;
          }
        } else if (node.tagName === 'joint') {
          var joint = new UrdfJoint({
            xml: node
          });
          this.joints[joint.name] = joint;
        }
      }
    }
  }

  var Urdf = /*#__PURE__*/Object.freeze({
    __proto__: null,
    URDF_BOX: URDF_BOX,
    URDF_CYLINDER: URDF_CYLINDER,
    URDF_MESH: URDF_MESH,
    URDF_SPHERE: URDF_SPHERE,
    UrdfBox: UrdfBox,
    UrdfColor: UrdfColor,
    UrdfCylinder: UrdfCylinder,
    UrdfLink: UrdfLink,
    UrdfMaterial: UrdfMaterial,
    UrdfMesh: UrdfMesh,
    UrdfModel: UrdfModel,
    UrdfSphere: UrdfSphere,
    UrdfVisual: UrdfVisual
  });

  /**
   * @fileOverview
   * @author Russell Toris - rctoris@wpi.edu
   */

  /** @description Library version */
  const REVISION = '1.4.1';

  // Add to global namespace for in-browser support (i.e. CDN)
  globalThis.ROSLIB = {
    REVISION,
    ...Core,
    ...ActionLib,
    ...Math$1,
    ...Tf,
    ...Urdf
  };

  /**
   * @fileOverview
   * @author Ramon Wijnands - rayman747@hotmail.com
   */


  /**
   * @callback decompressPngCallback
   * @param data - The uncompressed data.
   */
  /**
   * If a message was compressed as a PNG image (a compression hack since
   * gzipping over WebSockets * is not supported yet), this function decodes
   * the "image" as a Base64 string.
   *
   * @private
   * @param data - An object containing the PNG data.
   * @param {decompressPngCallback} callback - Function with the following params:
   */
  function decompressPng$2(data, callback) {
    var buffer = new Buffer(data, 'base64');

    pngparse.parse(buffer, function (err, data) {
      if (err) {
        console.warn('Cannot process PNG encoded message ');
      } else {
        var jsonData = data.data.toString();
        callback(JSON.parse(jsonData));
      }
    });
  }

  var decompressPng$3 = /*#__PURE__*/Object.freeze({
    __proto__: null,
    default: decompressPng$2
  });

  /**
   * @fileOverview
   * @author Graeme Yeates - github.com/megawac
   */

  /**
   * @callback decompressPngCallback
   * @param data - The uncompressed data.
   */
  /**
   * If a message was compressed as a PNG image (a compression hack since
   * gzipping over WebSockets * is not supported yet), this function places the
   * "image" in a canvas element then decodes the * "image" as a Base64 string.
   *
   * @private
   * @param data - An object containing the PNG data.
   * @param {decompressPngCallback} callback - Function with the following params:
   */
  function decompressPng(data, callback) {
    // Uncompresses the data before sending it through (use image/canvas to do so).
    var image = new Image();
    // When the image loads, extracts the raw data (JSON message).
    image.onload = function () {
      // Creates a local canvas to draw on.
      var canvas = document.createElement('canvas');
      var context = canvas.getContext('2d');

      if (!context) {
        throw new Error('Failed to create Canvas context!');
      }

      // Sets width and height.
      canvas.width = image.width;
      canvas.height = image.height;

      // Prevents anti-aliasing and loosing data
      context.imageSmoothingEnabled = false;

      // Puts the data into the image.
      context.drawImage(image, 0, 0);
      // Grabs the raw, uncompressed data.
      var imageData = context.getImageData(0, 0, image.width, image.height).data;

      // Constructs the JSON.
      var jsonData = '';
      for (var i = 0; i < imageData.length; i += 4) {
        // RGB
        jsonData += String.fromCharCode(
          imageData[i],
          imageData[i + 1],
          imageData[i + 2]
        );
      }
      callback(JSON.parse(jsonData));
    };
    // Sends the image data to load.
    image.src = 'data:image/png;base64,' + data;
  }

  var decompressPng$1 = /*#__PURE__*/Object.freeze({
    __proto__: null,
    default: decompressPng
  });

  exports.Action = Action;
  exports.ActionClient = ActionClient;
  exports.ActionListener = ActionListener;
  exports.Goal = Goal;
  exports.Param = Param;
  exports.Pose = Pose;
  exports.Quaternion = Quaternion;
  exports.REVISION = REVISION;
  exports.ROS2TFClient = ROS2TFClient;
  exports.Ros = Ros;
  exports.Service = Service;
  exports.SimpleActionServer = SimpleActionServer;
  exports.TFClient = TFClient;
  exports.Topic = Topic;
  exports.Transform = Transform;
  exports.URDF_BOX = URDF_BOX;
  exports.URDF_CYLINDER = URDF_CYLINDER;
  exports.URDF_MESH = URDF_MESH;
  exports.URDF_SPHERE = URDF_SPHERE;
  exports.UrdfBox = UrdfBox;
  exports.UrdfColor = UrdfColor;
  exports.UrdfCylinder = UrdfCylinder;
  exports.UrdfLink = UrdfLink;
  exports.UrdfMaterial = UrdfMaterial;
  exports.UrdfMesh = UrdfMesh;
  exports.UrdfModel = UrdfModel;
  exports.UrdfSphere = UrdfSphere;
  exports.UrdfVisual = UrdfVisual;
  exports.Vector3 = Vector3;

}));
