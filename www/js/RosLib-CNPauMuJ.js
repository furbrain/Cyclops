import { EventEmitter as Oe } from "eventemitter3";
function ur(o) {
  return o && o.__esModule && Object.prototype.hasOwnProperty.call(o, "default") ? o.default : o;
}
function xr(o) {
  if (Object.prototype.hasOwnProperty.call(o, "__esModule")) return o;
  var t = o.default;
  if (typeof t == "function") {
    var u = function s() {
      var a = !1;
      try {
        a = this instanceof s;
      } catch {
      }
      return a ? Reflect.construct(t, arguments, this.constructor) : t.apply(this, arguments);
    };
    u.prototype = t.prototype;
  } else u = {};
  return Object.defineProperty(u, "__esModule", { value: !0 }), Object.keys(o).forEach(function(s) {
    var a = Object.getOwnPropertyDescriptor(o, s);
    Object.defineProperty(u, s, a.get ? a : {
      enumerable: !0,
      get: function() {
        return o[s];
      }
    });
  }), u;
}
var pt = { exports: {} }, ir = pt.exports, St;
function nr() {
  return St || (St = 1, function(o) {
    (function(t, u) {
      var s = Math.pow(2, -24), a = Math.pow(2, 32), n = Math.pow(2, 53);
      function l(O) {
        var A = new ArrayBuffer(256), f = new DataView(A), M, S = 0;
        function k(v) {
          for (var U = A.byteLength, C = S + v; U < C; )
            U *= 2;
          if (U !== A.byteLength) {
            var L = f;
            A = new ArrayBuffer(U), f = new DataView(A);
            for (var h = S + 3 >> 2, p = 0; p < h; ++p)
              f.setUint32(p * 4, L.getUint32(p * 4));
          }
          return M = v, f;
        }
        function Y() {
          S += M;
        }
        function te(v) {
          Y(k(8).setFloat64(S, v));
        }
        function x(v) {
          Y(k(1).setUint8(S, v));
        }
        function H(v) {
          for (var U = k(v.length), C = 0; C < v.length; ++C)
            U.setUint8(S + C, v[C]);
          Y();
        }
        function j(v) {
          Y(k(2).setUint16(S, v));
        }
        function re(v) {
          Y(k(4).setUint32(S, v));
        }
        function ie(v) {
          var U = v % a, C = (v - U) / a, L = k(8);
          L.setUint32(S, C), L.setUint32(S + 4, U), Y();
        }
        function m(v, U) {
          U < 24 ? x(v << 5 | U) : U < 256 ? (x(v << 5 | 24), x(U)) : U < 65536 ? (x(v << 5 | 25), j(U)) : U < 4294967296 ? (x(v << 5 | 26), re(U)) : (x(v << 5 | 27), ie(U));
        }
        function N(v) {
          var U;
          if (v === !1)
            return x(244);
          if (v === !0)
            return x(245);
          if (v === null)
            return x(246);
          if (v === u)
            return x(247);
          switch (typeof v) {
            case "number":
              if (Math.floor(v) === v) {
                if (0 <= v && v <= n)
                  return m(0, v);
                if (-n <= v && v < 0)
                  return m(1, -(v + 1));
              }
              return x(251), te(v);
            case "string":
              var C = [];
              for (U = 0; U < v.length; ++U) {
                var L = v.charCodeAt(U);
                L < 128 ? C.push(L) : L < 2048 ? (C.push(192 | L >> 6), C.push(128 | L & 63)) : L < 55296 ? (C.push(224 | L >> 12), C.push(128 | L >> 6 & 63), C.push(128 | L & 63)) : (L = (L & 1023) << 10, L |= v.charCodeAt(++U) & 1023, L += 65536, C.push(240 | L >> 18), C.push(128 | L >> 12 & 63), C.push(128 | L >> 6 & 63), C.push(128 | L & 63));
              }
              return m(3, C.length), H(C);
            default:
              var h;
              if (Array.isArray(v))
                for (h = v.length, m(4, h), U = 0; U < h; ++U)
                  N(v[U]);
              else if (v instanceof Uint8Array)
                m(2, v.length), H(v);
              else {
                var p = Object.keys(v);
                for (h = p.length, m(5, h), U = 0; U < h; ++U) {
                  var d = p[U];
                  N(d), N(v[d]);
                }
              }
          }
        }
        if (N(O), "slice" in A)
          return A.slice(0, S);
        for (var P = new ArrayBuffer(S), G = new DataView(P), W = 0; W < S; ++W)
          G.setUint8(W, f.getUint8(W));
        return P;
      }
      function g(O, A, f) {
        var M = new DataView(O), S = 0;
        typeof A != "function" && (A = function(C) {
          return C;
        }), typeof f != "function" && (f = function() {
          return u;
        });
        function k(C, L) {
          return S += L, C;
        }
        function Y(C) {
          return k(new Uint8Array(O, S, C), C);
        }
        function te() {
          var C = new ArrayBuffer(4), L = new DataView(C), h = re(), p = h & 32768, d = h & 31744, T = h & 1023;
          if (d === 31744)
            d = 261120;
          else if (d !== 0)
            d += 114688;
          else if (T !== 0)
            return T * s;
          return L.setUint32(0, p << 16 | d << 13 | T << 13), L.getFloat32(0);
        }
        function x() {
          return k(M.getFloat32(S), 4);
        }
        function H() {
          return k(M.getFloat64(S), 8);
        }
        function j() {
          return k(M.getUint8(S), 1);
        }
        function re() {
          return k(M.getUint16(S), 2);
        }
        function ie() {
          return k(M.getUint32(S), 4);
        }
        function m() {
          return ie() * a + ie();
        }
        function N() {
          return M.getUint8(S) !== 255 ? !1 : (S += 1, !0);
        }
        function P(C) {
          if (C < 24)
            return C;
          if (C === 24)
            return j();
          if (C === 25)
            return re();
          if (C === 26)
            return ie();
          if (C === 27)
            return m();
          if (C === 31)
            return -1;
          throw "Invalid length encoding";
        }
        function G(C) {
          var L = j();
          if (L === 255)
            return -1;
          var h = P(L & 31);
          if (h < 0 || L >> 5 !== C)
            throw "Invalid indefinite length element";
          return h;
        }
        function W(C, L) {
          for (var h = 0; h < L; ++h) {
            var p = j();
            p & 128 && (p < 224 ? (p = (p & 31) << 6 | j() & 63, L -= 1) : p < 240 ? (p = (p & 15) << 12 | (j() & 63) << 6 | j() & 63, L -= 2) : (p = (p & 15) << 18 | (j() & 63) << 12 | (j() & 63) << 6 | j() & 63, L -= 3)), p < 65536 ? C.push(p) : (p -= 65536, C.push(55296 | p >> 10), C.push(56320 | p & 1023));
          }
        }
        function v() {
          var C = j(), L = C >> 5, h = C & 31, p, d;
          if (L === 7)
            switch (h) {
              case 25:
                return te();
              case 26:
                return x();
              case 27:
                return H();
            }
          if (d = P(h), d < 0 && (L < 2 || 6 < L))
            throw "Invalid length";
          switch (L) {
            case 0:
              return d;
            case 1:
              return -1 - d;
            case 2:
              if (d < 0) {
                for (var T = [], I = 0; (d = G(L)) >= 0; )
                  I += d, T.push(Y(d));
                var D = new Uint8Array(I), R = 0;
                for (p = 0; p < T.length; ++p)
                  D.set(T[p], R), R += T[p].length;
                return D;
              }
              return Y(d);
            case 3:
              var X = [];
              if (d < 0)
                for (; (d = G(L)) >= 0; )
                  W(X, d);
              else
                W(X, d);
              return String.fromCharCode.apply(null, X);
            case 4:
              var B;
              if (d < 0)
                for (B = []; !N(); )
                  B.push(v());
              else
                for (B = new Array(d), p = 0; p < d; ++p)
                  B[p] = v();
              return B;
            case 5:
              var y = {};
              for (p = 0; p < d || d < 0 && !N(); ++p) {
                var w = v();
                y[w] = v();
              }
              return y;
            case 6:
              return A(v(), d);
            case 7:
              switch (d) {
                case 20:
                  return !1;
                case 21:
                  return !0;
                case 22:
                  return null;
                case 23:
                  return u;
                default:
                  return f(d);
              }
          }
        }
        var U = v();
        if (S !== O.byteLength)
          throw "Remaining bytes";
        return U;
      }
      var b = { encode: l, decode: g };
      o.exports ? o.exports = b : t.CBOR || (t.CBOR = b);
    })(ir);
  }(pt)), pt.exports;
}
var sr = nr();
const ar = /* @__PURE__ */ ur(sr);
var Gt = Math.pow(2, 32), Ot = !1;
function Vt() {
  Ot || (Ot = !0, console.warn(
    "CBOR 64-bit integer array values may lose precision. No further warnings."
  ));
}
function or(o) {
  Vt();
  for (var t = o.byteLength, u = o.byteOffset, s = t / 8, a = o.buffer.slice(u, u + t), n = new Uint32Array(a), l = new Array(s), g = 0; g < s; g++) {
    var b = g * 2, O = n[b], A = n[b + 1];
    l[g] = O + Gt * A;
  }
  return l;
}
function cr(o) {
  Vt();
  for (var t = o.byteLength, u = o.byteOffset, s = t / 8, a = o.buffer.slice(u, u + t), n = new Uint32Array(a), l = new Int32Array(a), g = new Array(s), b = 0; b < s; b++) {
    var O = b * 2, A = n[O], f = l[O + 1];
    g[b] = A + Gt * f;
  }
  return g;
}
function lr(o, t) {
  var u = o.byteLength, s = o.byteOffset, a = o.buffer.slice(s, s + u);
  return new t(a);
}
var xt = {
  64: Uint8Array,
  69: Uint16Array,
  70: Uint32Array,
  72: Int8Array,
  77: Int16Array,
  78: Int32Array,
  85: Float32Array,
  86: Float64Array
}, It = {
  71: or,
  79: cr
};
function hr(o, t) {
  if (t in xt) {
    var u = xt[t];
    return lr(o, u);
  }
  return t in It ? It[t](o) : o;
}
var gt = null;
typeof bson < "u" && (gt = bson().BSON);
function vt(o) {
  var t = null;
  o.transportOptions.decoder && (t = o.transportOptions.decoder);
  function u(n) {
    n.op === "publish" ? o.emit(n.topic, n.msg) : n.op === "service_response" ? o.emit(n.id, n) : n.op === "call_service" ? o.emit(n.service, n) : n.op === "send_action_goal" ? o.emit(n.action, n) : n.op === "cancel_action_goal" || n.op === "action_feedback" || n.op === "action_result" ? o.emit(n.id, n) : n.op === "status" && (n.id ? o.emit("status:" + n.id, n) : o.emit("status", n));
  }
  function s(n, l) {
    n.op === "png" ? typeof window > "u" ? import("./decompressPng-C2VNa7pW.js").then(({ default: g }) => g(n.data, l)) : import("./decompressPng-DB2EQ_f0.js").then(({ default: g }) => g(n.data, l)) : l(n);
  }
  function a(n, l) {
    if (!gt)
      throw "Cannot process BSON encoded message without BSON header.";
    var g = new FileReader();
    g.onload = function() {
      var b = new Uint8Array(this.result), O = gt.deserialize(b);
      l(O);
    }, g.readAsArrayBuffer(n);
  }
  return {
    /**
     * Emit a 'connection' event on WebSocket connection.
     *
     * @param {function} event - The argument to emit with the event.
     * @memberof SocketAdapter
     */
    onopen: function(l) {
      o.isConnected = !0, o.emit("connection", l);
    },
    /**
     * Emit a 'close' event on WebSocket disconnection.
     *
     * @param {function} event - The argument to emit with the event.
     * @memberof SocketAdapter
     */
    onclose: function(l) {
      o.isConnected = !1, o.emit("close", l);
    },
    /**
     * Emit an 'error' event whenever there was an error.
     *
     * @param {function} event - The argument to emit with the event.
     * @memberof SocketAdapter
     */
    onerror: function(l) {
      o.emit("error", l);
    },
    /**
     * Parse message responses from rosbridge and send to the appropriate
     * topic, service, or param.
     *
     * @param {Object} data - The raw JSON message from rosbridge.
     * @memberof SocketAdapter
     */
    onmessage: function(l) {
      if (t)
        t(l.data, function(O) {
          u(O);
        });
      else if (typeof Blob < "u" && l.data instanceof Blob)
        a(l.data, function(O) {
          s(O, u);
        });
      else if (l.data instanceof ArrayBuffer) {
        var g = ar.decode(l.data, hr);
        u(g);
      } else {
        var b = JSON.parse(typeof l == "string" ? l : l.data);
        s(b, u);
      }
    }
  };
}
class me extends Oe {
  /** @type {boolean | undefined} */
  waitForReconnect = void 0;
  /** @type {(() => void) | undefined} */
  reconnectFunc = void 0;
  isAdvertised = !1;
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
  constructor(t) {
    super(), this.ros = t.ros, this.name = t.name, this.messageType = t.messageType, this.compression = t.compression || "none", this.throttle_rate = t.throttle_rate || 0, this.latch = t.latch || !1, this.queue_size = t.queue_size || 100, this.queue_length = t.queue_length || 0, this.reconnect_on_close = t.reconnect_on_close !== void 0 ? t.reconnect_on_close : !0, this.compression && this.compression !== "png" && this.compression !== "cbor" && this.compression !== "cbor-raw" && this.compression !== "none" && (this.emit(
      "warning",
      this.compression + " compression is not supported. No compression will be used."
    ), this.compression = "none"), this.throttle_rate < 0 && (this.emit("warning", this.throttle_rate + " is not allowed. Set to 0"), this.throttle_rate = 0), this.reconnect_on_close ? this.callForSubscribeAndAdvertise = (u) => {
      this.ros.callOnConnection(u), this.waitForReconnect = !1, this.reconnectFunc = () => {
        this.waitForReconnect || (this.waitForReconnect = !0, this.ros.callOnConnection(u), this.ros.once("connection", () => {
          this.waitForReconnect = !1;
        }));
      }, this.ros.on("close", this.reconnectFunc);
    } : this.callForSubscribeAndAdvertise = this.ros.callOnConnection;
  }
  _messageCallback = (t) => {
    this.emit("message", t);
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
  subscribe(t) {
    typeof t == "function" && this.on("message", t), !this.subscribeId && (this.ros.on(this.name, this._messageCallback), this.subscribeId = "subscribe:" + this.name + ":" + (++this.ros.idCounter).toString(), this.callForSubscribeAndAdvertise({
      op: "subscribe",
      id: this.subscribeId,
      type: this.messageType,
      topic: this.name,
      compression: this.compression,
      throttle_rate: this.throttle_rate,
      queue_length: this.queue_length
    }));
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
  unsubscribe(t) {
    t && (this.off("message", t), this.listeners("message").length) || this.subscribeId && (this.ros.off(this.name, this._messageCallback), this.reconnect_on_close && this.ros.off("close", this.reconnectFunc), this.emit("unsubscribe"), this.ros.callOnConnection({
      op: "unsubscribe",
      id: this.subscribeId,
      topic: this.name
    }), this.subscribeId = null);
  }
  /**
   * Register as a publisher for the topic.
   */
  advertise() {
    this.isAdvertised || (this.advertiseId = "advertise:" + this.name + ":" + (++this.ros.idCounter).toString(), this.callForSubscribeAndAdvertise({
      op: "advertise",
      id: this.advertiseId,
      type: this.messageType,
      topic: this.name,
      latch: this.latch,
      queue_size: this.queue_size
    }), this.isAdvertised = !0, this.reconnect_on_close || this.ros.on("close", () => {
      this.isAdvertised = !1;
    }));
  }
  /**
   * Unregister as a publisher for the topic.
   */
  unadvertise() {
    this.isAdvertised && (this.reconnect_on_close && this.ros.off("close", this.reconnectFunc), this.emit("unadvertise"), this.ros.callOnConnection({
      op: "unadvertise",
      id: this.advertiseId,
      topic: this.name
    }), this.isAdvertised = !1);
  }
  /**
   * Publish the message.
   *
   * @param {T} message - The message to publish.
   */
  publish(t) {
    this.isAdvertised || this.advertise(), this.ros.idCounter++;
    var u = {
      op: "publish",
      id: "publish:" + this.name + ":" + this.ros.idCounter,
      topic: this.name,
      msg: t,
      latch: this.latch
    };
    this.ros.callOnConnection(u);
  }
}
class se extends Oe {
  /**
     * Stores a reference to the most recent service callback advertised so it can be removed from the EventEmitter during un-advertisement
     * @private
     * @type {((rosbridgeRequest) => any) | null}
     */
  _serviceCallback = null;
  isAdvertised = !1;
  /**
   * Queue for serializing advertise/unadvertise operations to prevent race conditions
   * @private
   */
  _operationQueue = Promise.resolve();
  /**
   * Track if an unadvertise operation is pending to prevent double operations
   * @private
   */
  _pendingUnadvertise = !1;
  /**
   * @param {Object} options
   * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
   * @param {string} options.name - The service name, like '/add_two_ints'.
   * @param {string} options.serviceType - The service type, like 'rospy_tutorials/AddTwoInts'.
   */
  constructor(t) {
    super(), this.ros = t.ros, this.name = t.name, this.serviceType = t.serviceType;
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
   * @param {number} [timeout] - Optional timeout, in seconds, for the service call. A non-positive value means no timeout.
   *                             If not provided, the rosbridge server will use its default value.
  */
  callService(t, u, s, a) {
    if (!this.isAdvertised) {
      var n = "call_service:" + this.name + ":" + (++this.ros.idCounter).toString();
      (u || s) && this.ros.once(n, function(g) {
        g.result !== void 0 && g.result === !1 ? typeof s == "function" && s(g.values) : typeof u == "function" && u(g.values);
      });
      var l = {
        op: "call_service",
        id: n,
        service: this.name,
        type: this.serviceType,
        args: t,
        timeout: a
      };
      this.ros.callOnConnection(l);
    }
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
  advertise(t) {
    return this._operationQueue = this._operationQueue.then(async () => {
      this.isAdvertised && await this._doUnadvertise(), this._serviceCallback = (u) => {
        var s = {}, a = t(u.args, s), n = {
          op: "service_response",
          service: this.name,
          values: s,
          result: a
        };
        u.id && (n.id = u.id), this.ros.callOnConnection(n);
      }, this.ros.on(this.name, this._serviceCallback), this.ros.callOnConnection({
        op: "advertise_service",
        type: this.serviceType,
        service: this.name
      }), this.isAdvertised = !0;
    }).catch((u) => {
      throw this.emit("error", u), u;
    }), this._operationQueue;
  }
  /**
   * Internal method to perform unadvertisement without queueing
   * @private
   */
  async _doUnadvertise() {
    if (!(!this.isAdvertised || this._pendingUnadvertise)) {
      this._pendingUnadvertise = !0;
      try {
        this.isAdvertised = !1, this._serviceCallback && (this.ros.off(this.name, this._serviceCallback), this._serviceCallback = null), this.ros.callOnConnection({
          op: "unadvertise_service",
          service: this.name
        });
      } finally {
        this._pendingUnadvertise = !1;
      }
    }
  }
  unadvertise() {
    return this._operationQueue = this._operationQueue.then(async () => {
      await this._doUnadvertise();
    }).catch((t) => {
      throw this.emit("error", t), t;
    }), this._operationQueue;
  }
  /**
   * An alternate form of Service advertisement that supports a modern Promise-based interface for use with async/await.
   * @param {(request: TRequest) => Promise<TResponse>} callback An asynchronous callback processing the request and returning a response.
   */
  advertiseAsync(t) {
    return this._operationQueue = this._operationQueue.then(async () => {
      this.isAdvertised && await this._doUnadvertise(), this._serviceCallback = async (u) => {
        let s = {
          op: "service_response",
          service: this.name,
          result: !1
        };
        try {
          s.values = await t(u.args), s.result = !0;
        } finally {
          u.id && (s.id = u.id), this.ros.callOnConnection(s);
        }
      }, this.ros.on(this.name, this._serviceCallback), this.ros.callOnConnection({
        op: "advertise_service",
        type: this.serviceType,
        service: this.name
      }), this.isAdvertised = !0;
    }).catch((u) => {
      throw this.emit("error", u), u;
    }), this._operationQueue;
  }
}
class zt {
  /**
   * @param {Object} options
   * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
   * @param {string} options.name - The param name, like max_vel_x.
   */
  constructor(t) {
    this.ros = t.ros, this.name = t.name;
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
  get(t, u) {
    var s = new se({
      ros: this.ros,
      name: "rosapi/get_param",
      serviceType: "rosapi/GetParam"
    }), a = { name: this.name };
    s.callService(
      a,
      function(n) {
        var l = JSON.parse(n.value);
        t(l);
      },
      u
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
  set(t, u, s) {
    var a = new se({
      ros: this.ros,
      name: "rosapi/set_param",
      serviceType: "rosapi/SetParam"
    }), n = {
      name: this.name,
      value: JSON.stringify(t)
    };
    a.callService(n, u, s);
  }
  /**
   * Delete this parameter on the ROS server.
   *
   * @param {setParamCallback} callback - The callback function.
   * @param {setParamFailedCallback} [failedCallback] - The callback function when the service call failed.
   */
  delete(t, u) {
    var s = new se({
      ros: this.ros,
      name: "rosapi/delete_param",
      serviceType: "rosapi/DeleteParam"
    }), a = {
      name: this.name
    };
    s.callService(a, t, u);
  }
}
class Tt extends Oe {
  goals = {};
  /** flag to check if a status has been received */
  receivedStatus = !1;
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
  constructor(t) {
    super(), this.ros = t.ros, this.serverName = t.serverName, this.actionName = t.actionName, this.timeout = t.timeout, this.omitFeedback = t.omitFeedback, this.omitStatus = t.omitStatus, this.omitResult = t.omitResult, this.feedbackListener = new me({
      ros: this.ros,
      name: this.serverName + "/feedback",
      messageType: this.actionName + "Feedback"
    }), this.statusListener = new me({
      ros: this.ros,
      name: this.serverName + "/status",
      messageType: "actionlib_msgs/GoalStatusArray"
    }), this.resultListener = new me({
      ros: this.ros,
      name: this.serverName + "/result",
      messageType: this.actionName + "Result"
    }), this.goalTopic = new me({
      ros: this.ros,
      name: this.serverName + "/goal",
      messageType: this.actionName + "Goal"
    }), this.cancelTopic = new me({
      ros: this.ros,
      name: this.serverName + "/cancel",
      messageType: "actionlib_msgs/GoalID"
    }), this.goalTopic.advertise(), this.cancelTopic.advertise(), this.omitStatus || this.statusListener.subscribe((u) => {
      this.receivedStatus = !0, u.status_list.forEach((s) => {
        var a = this.goals[s.goal_id.id];
        a && a.emit("status", s);
      });
    }), this.omitFeedback || this.feedbackListener.subscribe((u) => {
      var s = this.goals[u.status.goal_id.id];
      s && (s.emit("status", u.status), s.emit("feedback", u.feedback));
    }), this.omitResult || this.resultListener.subscribe((u) => {
      var s = this.goals[u.status.goal_id.id];
      s && (s.emit("status", u.status), s.emit("result", u.result));
    }), this.timeout && setTimeout(() => {
      this.receivedStatus || this.emit("timeout");
    }, this.timeout);
  }
  /**
   * Cancel all goals associated with this ActionClient.
   */
  cancel() {
    var t = {};
    this.cancelTopic.publish(t);
  }
  /**
   * Unsubscribe and unadvertise all topics associated with this ActionClient.
   */
  dispose() {
    this.goalTopic.unadvertise(), this.cancelTopic.unadvertise(), this.omitStatus || this.statusListener.unsubscribe(), this.omitFeedback || this.feedbackListener.unsubscribe(), this.omitResult || this.resultListener.unsubscribe();
  }
}
class Ht extends Oe {
  isFinished = !1;
  status = void 0;
  result = void 0;
  feedback = void 0;
  // Create a random ID
  goalID = "goal_" + Math.random() + "_" + (/* @__PURE__ */ new Date()).getTime();
  /**
   * @param {Object} options
   * @param {ActionClient} options.actionClient - The ROSLIB.ActionClient to use with this goal.
   * @param {Object} options.goalMessage - The JSON object containing the goal for the action server.
   */
  constructor(t) {
    super(), this.actionClient = t.actionClient, this.goalMessage = {
      goal_id: {
        stamp: {
          secs: 0,
          nsecs: 0
        },
        id: this.goalID
      },
      goal: t.goalMessage
    }, this.on("status", (u) => {
      this.status = u;
    }), this.on("result", (u) => {
      this.isFinished = !0, this.result = u;
    }), this.on("feedback", (u) => {
      this.feedback = u;
    }), this.actionClient.goals[this.goalID] = this;
  }
  /**
   * Send the goal to the action server.
   *
   * @param {number} [timeout] - A timeout length for the goal's result.
   */
  send(t) {
    this.actionClient.goalTopic.publish(this.goalMessage), t && setTimeout(() => {
      this.isFinished || this.emit("timeout");
    }, t);
  }
  /**
   * Cancel the current goal.
   */
  cancel() {
    var t = {
      id: this.goalID
    };
    this.actionClient.cancelTopic.publish(t);
  }
}
class Me {
  constructor(t) {
    this.x = t?.x ?? 0, this.y = t?.y ?? 0, this.z = t?.z ?? 0;
  }
  /**
   * Set the values of this vector to the sum of itself and the given vector.
   *
   * @param {Vector3} v - The vector to add with.
   */
  add(t) {
    this.x += t.x, this.y += t.y, this.z += t.z;
  }
  /**
   * Set the values of this vector to the difference of itself and the given vector.
   *
   * @param {Vector3} v - The vector to subtract with.
   */
  subtract(t) {
    this.x -= t.x, this.y -= t.y, this.z -= t.z;
  }
  /**
   * Multiply the given Quaternion with this vector.
   *
   * @param {Quaternion} q - The quaternion to multiply with.
   */
  multiplyQuaternion(t) {
    const u = t.w * this.x + t.y * this.z - t.z * this.y, s = t.w * this.y + t.z * this.x - t.x * this.z, a = t.w * this.z + t.x * this.y - t.y * this.x, n = -t.x * this.x - t.y * this.y - t.z * this.z;
    this.x = u * t.w + n * -t.x + s * -t.z - a * -t.y, this.y = s * t.w + n * -t.y + a * -t.x - u * -t.z, this.z = a * t.w + n * -t.z + u * -t.y - s * -t.x;
  }
  /**
   * Clone a copy of this vector.
   *
   * @returns {Vector3} The cloned vector.
   */
  clone() {
    return new Me(this);
  }
}
class Ue {
  constructor(t) {
    this.x = t?.x ?? 0, this.y = t?.y ?? 0, this.z = t?.z ?? 0, this.w = typeof t?.w == "number" ? t.w : 1;
  }
  /**
   * Perform a conjugation on this quaternion.
   */
  conjugate() {
    this.x *= -1, this.y *= -1, this.z *= -1;
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
    let t = Math.sqrt(
      this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w
    );
    t === 0 ? (this.x = 0, this.y = 0, this.z = 0, this.w = 1) : (t = 1 / t, this.x = this.x * t, this.y = this.y * t, this.z = this.z * t, this.w = this.w * t);
  }
  /**
   * Convert this quaternion into its inverse.
   */
  invert() {
    this.conjugate(), this.normalize();
  }
  /**
   * Set the values of this quaternion to the product of itself and the given quaternion.
   *
   * @param {IQuaternion} q - The quaternion to multiply with.
   */
  multiply(t) {
    const u = this.x * t.w + this.y * t.z - this.z * t.y + this.w * t.x, s = -this.x * t.z + this.y * t.w + this.z * t.x + this.w * t.y, a = this.x * t.y - this.y * t.x + this.z * t.w + this.w * t.z, n = -this.x * t.x - this.y * t.y - this.z * t.z + this.w * t.w;
    this.x = u, this.y = s, this.z = a, this.w = n;
  }
  /**
   * Clone a copy of this quaternion.
   *
   * @returns {Quaternion} The cloned quaternion.
   */
  clone() {
    return new Ue(this);
  }
}
class ht {
  constructor(t) {
    this.translation = new Me(t.translation), this.rotation = new Ue(t.rotation);
  }
  /**
   * Clone a copy of this transform.
   *
   * @returns {Transform} The cloned transform.
   */
  clone() {
    return new ht(this);
  }
}
class Yt extends Oe {
  /** @type {Goal|false} */
  currentGoal = !1;
  /** @type {Topic|false} */
  currentTopic = !1;
  frameInfos = {};
  republisherUpdateRequested = !1;
  /** @type {((tf: any) => any) | undefined} */
  _subscribeCB = void 0;
  _isDisposed = !1;
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
  constructor(t) {
    super(), this.ros = t.ros, this.fixedFrame = t.fixedFrame || "base_link", this.angularThres = t.angularThres || 2, this.transThres = t.transThres || 0.01, this.rate = t.rate || 10, this.updateDelay = t.updateDelay || 50;
    var u = t.topicTimeout || 2, s = Math.floor(u), a = Math.floor((u - s) * 1e9);
    this.topicTimeout = {
      secs: s,
      nsecs: a
    }, this.serverName = t.serverName || "/tf2_web_republisher", this.repubServiceName = t.repubServiceName || "/republish_tfs", this.actionClient = new Tt({
      ros: t.ros,
      serverName: this.serverName,
      actionName: "tf2_web_republisher/TFSubscriptionAction",
      omitStatus: !0,
      omitResult: !0
    }), this.serviceClient = new se({
      ros: t.ros,
      name: this.repubServiceName,
      serviceType: "tf2_web_republisher/RepublishTFs"
    });
  }
  /**
   * Process the incoming TF message and send them out using the callback
   * functions.
   *
   * @param {Object} tf - The TF message from the server.
   */
  processTFArray(t) {
    t.transforms.forEach((u) => {
      var s = u.child_frame_id;
      s[0] === "/" && (s = s.substring(1));
      var a = this.frameInfos[s];
      a && (a.transform = new ht({
        translation: u.transform.translation,
        rotation: u.transform.rotation
      }), a.cbs.forEach((n) => {
        n(a.transform);
      }));
    }, this);
  }
  /**
   * Create and send a new goal (or service request) to the tf2_web_republisher
   * based on the current list of TFs.
   */
  updateGoal() {
    var t = {
      source_frames: Object.keys(this.frameInfos),
      target_frame: this.fixedFrame,
      angular_thres: this.angularThres,
      trans_thres: this.transThres,
      rate: this.rate
    };
    this.ros.groovyCompatibility ? (this.currentGoal && this.currentGoal.cancel(), this.currentGoal = new Ht({
      actionClient: this.actionClient,
      goalMessage: t
    }), this.currentGoal.on("feedback", this.processTFArray.bind(this)), this.currentGoal.send()) : (t.timeout = this.topicTimeout, this.serviceClient.callService(t, this.processResponse.bind(this))), this.republisherUpdateRequested = !1;
  }
  /**
   * Process the service response and subscribe to the tf republisher
   * topic.
   *
   * @param {Object} response - The service response containing the topic name.
   */
  processResponse(t) {
    this._isDisposed || (this.currentTopic && this.currentTopic.unsubscribe(this._subscribeCB), this.currentTopic = new me({
      ros: this.ros,
      name: t.topic_name,
      messageType: "tf2_web_republisher/TFArray"
    }), this._subscribeCB = this.processTFArray.bind(this), this.currentTopic.subscribe(this._subscribeCB));
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
  subscribe(t, u) {
    t[0] === "/" && (t = t.substring(1)), this.frameInfos[t] ? this.frameInfos[t].transform && u(this.frameInfos[t].transform) : (this.frameInfos[t] = {
      cbs: []
    }, this.republisherUpdateRequested || (setTimeout(this.updateGoal.bind(this), this.updateDelay), this.republisherUpdateRequested = !0)), this.frameInfos[t].cbs.push(u);
  }
  /**
   * Unsubscribe from the given TF frame.
   *
   * @param {string} frameID - The TF frame to unsubscribe from.
   * @param {function} callback - The callback function to remove.
   */
  unsubscribe(t, u) {
    t[0] === "/" && (t = t.substring(1));
    for (var s = this.frameInfos[t], a = s && s.cbs || [], n = a.length; n--; )
      a[n] === u && a.splice(n, 1);
    (!u || a.length === 0) && delete this.frameInfos[t];
  }
  /**
   * Unsubscribe and unadvertise all topics associated with this TFClient.
   */
  dispose() {
    this._isDisposed = !0, this.actionClient.dispose(), this.currentTopic && this.currentTopic.unsubscribe(this._subscribeCB);
  }
}
class jt extends Oe {
  // needed for handling preemption prompted by a new goal being received
  /** @type {{goal_id: {id: any, stamp: any}, goal: any} | null} */
  currentGoal = null;
  // currently tracked goal
  /** @type {{goal_id: {id: any, stamp: any}, goal: any} | null} */
  nextGoal = null;
  // the one this'll be preempting
  /**
   * @param {Object} options
   * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
   * @param {string} options.serverName - The action server name, like '/fibonacci'.
   * @param {string} options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
   */
  constructor(t) {
    super(), this.ros = t.ros, this.serverName = t.serverName, this.actionName = t.actionName, this.feedbackPublisher = new me({
      ros: this.ros,
      name: this.serverName + "/feedback",
      messageType: this.actionName + "Feedback"
    }), this.feedbackPublisher.advertise();
    var u = new me({
      ros: this.ros,
      name: this.serverName + "/status",
      messageType: "actionlib_msgs/GoalStatusArray"
    });
    u.advertise(), this.resultPublisher = new me({
      ros: this.ros,
      name: this.serverName + "/result",
      messageType: this.actionName + "Result"
    }), this.resultPublisher.advertise();
    var s = new me({
      ros: this.ros,
      name: this.serverName + "/goal",
      messageType: this.actionName + "Goal"
    }), a = new me({
      ros: this.ros,
      name: this.serverName + "/cancel",
      messageType: "actionlib_msgs/GoalID"
    });
    this.statusMessage = {
      header: {
        stamp: { secs: 0, nsecs: 100 },
        frame_id: ""
      },
      /** @type {{goal_id: any, status: number}[]} */
      status_list: []
    }, s.subscribe((l) => {
      this.currentGoal ? (this.nextGoal = l, this.emit("cancel")) : (this.statusMessage.status_list = [{ goal_id: l.goal_id, status: 1 }], this.currentGoal = l, this.emit("goal", l.goal));
    });
    var n = function(l, g) {
      return l.secs > g.secs ? !1 : l.secs < g.secs ? !0 : l.nsecs < g.nsecs;
    };
    a.subscribe((l) => {
      l.stamp.secs === 0 && l.stamp.secs === 0 && l.id === "" ? (this.nextGoal = null, this.currentGoal && this.emit("cancel")) : (this.currentGoal && l.id === this.currentGoal.goal_id.id ? this.emit("cancel") : this.nextGoal && l.id === this.nextGoal.goal_id.id && (this.nextGoal = null), this.nextGoal && n(this.nextGoal.goal_id.stamp, l.stamp) && (this.nextGoal = null), this.currentGoal && n(this.currentGoal.goal_id.stamp, l.stamp) && this.emit("cancel"));
    }), setInterval(() => {
      var l = /* @__PURE__ */ new Date(), g = Math.floor(l.getTime() / 1e3), b = Math.round(
        1e9 * (l.getTime() / 1e3 - g)
      );
      this.statusMessage.header.stamp.secs = g, this.statusMessage.header.stamp.nsecs = b, u.publish(this.statusMessage);
    }, 500);
  }
  /**
   * Set action state to succeeded and return to client.
   *
   * @param {Object} result - The result to return to the client.
   */
  setSucceeded(t) {
    if (this.currentGoal !== null) {
      var u = {
        status: { goal_id: this.currentGoal.goal_id, status: 3 },
        result: t
      };
      this.resultPublisher.publish(u), this.statusMessage.status_list = [], this.nextGoal ? (this.currentGoal = this.nextGoal, this.nextGoal = null, this.emit("goal", this.currentGoal.goal)) : this.currentGoal = null;
    }
  }
  /**
   * Set action state to aborted and return to client.
   *
   * @param {Object} result - The result to return to the client.
   */
  setAborted(t) {
    if (this.currentGoal !== null) {
      var u = {
        status: { goal_id: this.currentGoal.goal_id, status: 4 },
        result: t
      };
      this.resultPublisher.publish(u), this.statusMessage.status_list = [], this.nextGoal ? (this.currentGoal = this.nextGoal, this.nextGoal = null, this.emit("goal", this.currentGoal.goal)) : this.currentGoal = null;
    }
  }
  /**
   * Send a feedback message.
   *
   * @param {Object} feedback - The feedback to send to the client.
   */
  sendFeedback(t) {
    if (this.currentGoal !== null) {
      var u = {
        status: { goal_id: this.currentGoal.goal_id, status: 1 },
        feedback: t
      };
      this.feedbackPublisher.publish(u);
    }
  }
  /**
   * Handle case where client requests preemption.
   */
  setPreempted() {
    if (this.currentGoal !== null) {
      this.statusMessage.status_list = [];
      var t = {
        status: { goal_id: this.currentGoal.goal_id, status: 2 }
      };
      this.resultPublisher.publish(t), this.nextGoal ? (this.currentGoal = this.nextGoal, this.nextGoal = null, this.emit("goal", this.currentGoal.goal)) : this.currentGoal = null;
    }
  }
}
class fr extends Oe {
  /** @type {WebSocket | import("ws").WebSocket | null} */
  socket = null;
  idCounter = 0;
  isConnected = !1;
  groovyCompatibility = !0;
  /**
   * @param {Object} [options]
   * @param {string} [options.url] - The WebSocket URL for rosbridge. Can be specified later with `connect`.
   * @param {boolean} [options.groovyCompatibility=true] - Don't use interfaces that changed after the last groovy release or rosbridge_suite and related tools.
   * @param {'websocket'|RTCPeerConnection} [options.transportLibrary='websocket'] - 'websocket', or an RTCPeerConnection instance controlling how the connection is created in `connect`.
   * @param {Object} [options.transportOptions={}] - The options to use when creating a connection. Currently only used if `transportLibrary` is RTCPeerConnection.
   */
  constructor(t) {
    super(), t = t || {}, this.transportLibrary = t.transportLibrary || "websocket", this.transportOptions = t.transportOptions || {}, this.groovyCompatibility = t.groovyCompatibility ?? !0, t.url && this.connect(t.url);
  }
  /**
   * Connect to the specified WebSocket.
   *
   * @param {string} url - WebSocket URL or RTCDataChannel label for rosbridge.
   */
  connect(t) {
    if (this.transportLibrary.constructor.name === "RTCPeerConnection")
      this.socket = Object.assign(
        // @ts-expect-error -- this is kinda wild. `this.transportLibrary` can either be a string or an RTCDataChannel. This needs fixing.
        this.transportLibrary.createDataChannel(t, this.transportOptions),
        vt(this)
      );
    else if (this.transportLibrary === "websocket")
      if (typeof WebSocket == "function") {
        if (!this.socket || this.socket.readyState === WebSocket.CLOSED) {
          const u = new WebSocket(t);
          u.binaryType = "arraybuffer", this.socket = Object.assign(u, vt(this));
        }
      } else
        import("ws").then((u) => {
          if (!this.socket || this.socket.readyState === u.WebSocket.CLOSED) {
            const s = new u.WebSocket(t);
            s.binaryType = "arraybuffer", this.socket = Object.assign(s, vt(this));
          }
        });
    else
      throw "Unknown transportLibrary: " + this.transportLibrary.toString();
  }
  /**
   * Disconnect from the WebSocket server.
   */
  close() {
    this.socket && this.socket.close();
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
  authenticate(t, u, s, a, n, l, g) {
    var b = {
      op: "auth",
      mac: t,
      client: u,
      dest: s,
      rand: a,
      t: n,
      level: l,
      end: g
    };
    this.callOnConnection(b);
  }
  /**
   * Send an encoded message over the WebSocket.
   *
   * @param {Object} messageEncoded - The encoded message to be sent.
   */
  sendEncodedMessage(t) {
    this.isConnected ? this.socket !== null && this.socket.send(t) : this.once("connection", () => {
      this.socket !== null && this.socket.send(t);
    });
  }
  /**
   * Send the message over the WebSocket, but queue the message up if not yet
   * connected.
   *
   * @param {Object} message - The message to be sent.
   */
  callOnConnection(t) {
    this.transportOptions.encoder ? this.transportOptions.encoder(t, this.sendEncodedMessage) : this.sendEncodedMessage(JSON.stringify(t));
  }
  /**
   * Send a set_level request to the server.
   *
   * @param {string} level - Status level (none, error, warning, info).
   * @param {number} [id] - Operation ID to change status level on.
   */
  setStatusLevel(t, u) {
    var s = {
      op: "set_level",
      level: t,
      id: u
    };
    this.callOnConnection(s);
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
  getActionServers(t, u) {
    var s = new se({
      ros: this,
      name: "rosapi/action_servers",
      serviceType: "rosapi/GetActionServers"
    }), a = {};
    typeof u == "function" ? s.callService(
      a,
      function(n) {
        t(n.action_servers);
      },
      function(n) {
        u(n);
      }
    ) : s.callService(a, function(n) {
      t(n.action_servers);
    });
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
  getTopics(t, u) {
    var s = new se({
      ros: this,
      name: "rosapi/topics",
      serviceType: "rosapi/Topics"
    }), a = {};
    typeof u == "function" ? s.callService(
      a,
      function(n) {
        t(n);
      },
      function(n) {
        u(n);
      }
    ) : s.callService(a, function(n) {
      t(n);
    });
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
  getTopicsForType(t, u, s) {
    var a = new se({
      ros: this,
      name: "rosapi/topics_for_type",
      serviceType: "rosapi/TopicsForType"
    }), n = {
      type: t
    };
    typeof s == "function" ? a.callService(
      n,
      function(l) {
        u(l.topics);
      },
      function(l) {
        s(l);
      }
    ) : a.callService(n, function(l) {
      u(l.topics);
    });
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
  getServices(t, u) {
    var s = new se({
      ros: this,
      name: "rosapi/services",
      serviceType: "rosapi/Services"
    }), a = {};
    typeof u == "function" ? s.callService(
      a,
      function(n) {
        t(n.services);
      },
      function(n) {
        u(n);
      }
    ) : s.callService(a, function(n) {
      t(n.services);
    });
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
  getServicesForType(t, u, s) {
    var a = new se({
      ros: this,
      name: "rosapi/services_for_type",
      serviceType: "rosapi/ServicesForType"
    }), n = {
      type: t
    };
    typeof s == "function" ? a.callService(
      n,
      function(l) {
        u(l.services);
      },
      function(l) {
        s(l);
      }
    ) : a.callService(n, function(l) {
      u(l.services);
    });
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
  getServiceRequestDetails(t, u, s) {
    var a = new se({
      ros: this,
      name: "rosapi/service_request_details",
      serviceType: "rosapi/ServiceRequestDetails"
    }), n = {
      type: t
    };
    typeof s == "function" ? a.callService(
      n,
      function(l) {
        u(l);
      },
      function(l) {
        s(l);
      }
    ) : a.callService(n, function(l) {
      u(l);
    });
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
  getServiceResponseDetails(t, u, s) {
    var a = new se({
      ros: this,
      name: "rosapi/service_response_details",
      serviceType: "rosapi/ServiceResponseDetails"
    }), n = {
      type: t
    };
    typeof s == "function" ? a.callService(
      n,
      function(l) {
        u(l);
      },
      function(l) {
        s(l);
      }
    ) : a.callService(n, function(l) {
      u(l);
    });
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
  getNodes(t, u) {
    var s = new se({
      ros: this,
      name: "rosapi/nodes",
      serviceType: "rosapi/Nodes"
    }), a = {};
    typeof u == "function" ? s.callService(
      a,
      function(n) {
        t(n.nodes);
      },
      function(n) {
        u(n);
      }
    ) : s.callService(a, function(n) {
      t(n.nodes);
    });
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
  getNodeDetails(t, u, s) {
    var a = new se({
      ros: this,
      name: "rosapi/node_details",
      serviceType: "rosapi/NodeDetails"
    }), n = {
      node: t
    };
    typeof s == "function" ? a.callService(
      n,
      function(l) {
        u(l.subscribing, l.publishing, l.services);
      },
      function(l) {
        s(l);
      }
    ) : a.callService(n, function(l) {
      u(l);
    });
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
  getParams(t, u) {
    var s = new se({
      ros: this,
      name: "rosapi/get_param_names",
      serviceType: "rosapi/GetParamNames"
    }), a = {};
    typeof u == "function" ? s.callService(
      a,
      function(n) {
        t(n.names);
      },
      function(n) {
        u(n);
      }
    ) : s.callService(a, function(n) {
      t(n.names);
    });
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
  getTopicType(t, u, s) {
    var a = new se({
      ros: this,
      name: "rosapi/topic_type",
      serviceType: "rosapi/TopicType"
    }), n = {
      topic: t
    };
    typeof s == "function" ? a.callService(
      n,
      function(l) {
        u(l.type);
      },
      function(l) {
        s(l);
      }
    ) : a.callService(n, function(l) {
      u(l.type);
    });
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
  getServiceType(t, u, s) {
    var a = new se({
      ros: this,
      name: "rosapi/service_type",
      serviceType: "rosapi/ServiceType"
    }), n = {
      service: t
    };
    typeof s == "function" ? a.callService(
      n,
      function(l) {
        u(l.type);
      },
      function(l) {
        s(l);
      }
    ) : a.callService(n, function(l) {
      u(l.type);
    });
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
  getMessageDetails(t, u, s) {
    var a = new se({
      ros: this,
      name: "rosapi/message_details",
      serviceType: "rosapi/MessageDetails"
    }), n = {
      type: t
    };
    typeof s == "function" ? a.callService(
      n,
      function(l) {
        u(l.typedefs);
      },
      function(l) {
        s(l);
      }
    ) : a.callService(n, function(l) {
      u(l.typedefs);
    });
  }
  /**
   * Decode a typedef array into a dictionary like `rosmsg show foo/bar`.
   *
   * @param {Object[]} defs - Array of type_def dictionary.
   */
  decodeTypeDefs(t) {
    var u = (s, a) => {
      for (var n = {}, l = 0; l < s.fieldnames.length; l++) {
        var g = s.fieldarraylen[l], b = s.fieldnames[l], O = s.fieldtypes[l];
        if (O.indexOf("/") === -1)
          g === -1 ? n[b] = O : n[b] = [O];
        else {
          for (var A = !1, f = 0; f < a.length; f++)
            if (a[f].type.toString() === O.toString()) {
              A = a[f];
              break;
            }
          if (A) {
            var M = u(A, a);
            g === -1 ? n[b] = M : n[b] = [M];
          } else
            this.emit(
              "error",
              "Cannot find " + O + " in decodeTypeDefs"
            );
        }
      }
      return n;
    };
    return u(t[0], t);
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
  getTopicsAndRawTypes(t, u) {
    var s = new se({
      ros: this,
      name: "rosapi/topics_and_raw_types",
      serviceType: "rosapi/TopicsAndRawTypes"
    }), a = {};
    typeof u == "function" ? s.callService(
      a,
      function(n) {
        t(n);
      },
      function(n) {
        u(n);
      }
    ) : s.callService(a, function(n) {
      t(n);
    });
  }
  Topic(t) {
    return new me({ ros: this, ...t });
  }
  Param(t) {
    return new zt({ ros: this, ...t });
  }
  Service(t) {
    return new se({ ros: this, ...t });
  }
  TFClient(t) {
    return new Yt({ ros: this, ...t });
  }
  ActionClient(t) {
    return new Tt({ ros: this, ...t });
  }
  SimpleActionServer(t) {
    return new jt({ ros: this, ...t });
  }
}
var mt = /* @__PURE__ */ ((o) => (o[o.STATUS_UNKNOWN = 0] = "STATUS_UNKNOWN", o[o.STATUS_ACCEPTED = 1] = "STATUS_ACCEPTED", o[o.STATUS_EXECUTING = 2] = "STATUS_EXECUTING", o[o.STATUS_CANCELING = 3] = "STATUS_CANCELING", o[o.STATUS_SUCCEEDED = 4] = "STATUS_SUCCEEDED", o[o.STATUS_CANCELED = 5] = "STATUS_CANCELED", o[o.STATUS_ABORTED = 6] = "STATUS_ABORTED", o))(mt || {});
class Xt extends Oe {
  isAdvertised = !1;
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
  constructor(t) {
    super(), this.ros = t.ros, this.name = t.name, this.actionType = t.actionType;
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
  sendGoal(t, u, s, a) {
    if (!this.isAdvertised) {
      var n = "send_action_goal:" + this.name + ":" + ++this.ros.idCounter;
      (u || a) && this.ros.on(n, function(g) {
        g.result !== void 0 && g.result === !1 ? typeof a == "function" && a(g.values) : g.op === "action_feedback" && typeof s == "function" ? s(g.values) : g.op === "action_result" && typeof u == "function" && u(g.values);
      });
      var l = {
        op: "send_action_goal",
        id: n,
        action: this.name,
        action_type: this.actionType,
        args: t,
        feedback: !0
      };
      return this.ros.callOnConnection(l), n;
    }
  }
  /**
   * Cancels an action goal.
   *
   * @param {string} id - The ID of the action goal to cancel.
   */
  cancelGoal(t) {
    var u = {
      op: "cancel_action_goal",
      id: t,
      action: this.name
    };
    this.ros.callOnConnection(u);
  }
  /**
   * Advertise the action. This turns the Action object from a client
   * into a server. The callback will be called with every goal sent to this action.
   *
   * @param {advertiseActionCallback} actionCallback - This works similarly to the callback for a C++ action.
   * @param {advertiseCancelCallback} cancelCallback - A callback function to execute when the action is canceled.
   */
  advertise(t, u) {
    this.isAdvertised || typeof t != "function" || (this._actionCallback = t, this._cancelCallback = u, this.ros.on(this.name, this._executeAction.bind(this)), this.ros.callOnConnection({
      op: "advertise_action",
      type: this.actionType,
      action: this.name
    }), this.isAdvertised = !0);
  }
  /**
   * Unadvertise a previously advertised action.
   */
  unadvertise() {
    this.isAdvertised && (this.ros.callOnConnection({
      op: "unadvertise_action",
      action: this.name
    }), this.isAdvertised = !1);
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
  _executeAction(t) {
    var u = t.id;
    typeof u == "string" && this.ros.on(u, (s) => {
      s.op === "cancel_action_goal" && typeof this._cancelCallback == "function" && this._cancelCallback(u);
    }), typeof this._actionCallback == "function" && this._actionCallback(t.args, u);
  }
  /**
   * Helper function to send action feedback inside an action handler.
   *
   * @param {string} id - The action goal ID.
   * @param {TFeedback} feedback - The feedback to send.
   */
  sendFeedback(t, u) {
    var s = {
      op: "action_feedback",
      id: t,
      action: this.name,
      values: u
    };
    this.ros.callOnConnection(s);
  }
  /**
   * Helper function to set an action as succeeded.
   *
   * @param {string} id - The action goal ID.
   * @param {TResult} result - The result to set.
   */
  setSucceeded(t, u) {
    var s = {
      op: "action_result",
      id: t,
      action: this.name,
      values: u,
      status: mt.STATUS_SUCCEEDED,
      result: !0
    };
    this.ros.callOnConnection(s);
  }
  /**
   * Helper function to set an action as canceled.
   *
   * @param {string} id - The action goal ID.
   * @param {TResult} result - The result to set.
   */
  setCanceled(t, u) {
    var s = {
      op: "action_result",
      id: t,
      action: this.name,
      values: u,
      status: mt.STATUS_CANCELED,
      result: !0
    };
    this.ros.callOnConnection(s);
  }
  /**
   * Helper function to set an action as failed.
   *
   * @param {string} id - The action goal ID.
   */
  setFailed(t) {
    var u = {
      op: "action_result",
      id: t,
      action: this.name,
      status: mt.STATUS_ABORTED,
      result: !1
    };
    this.ros.callOnConnection(u);
  }
}
const pr = /* @__PURE__ */ Object.freeze(/* @__PURE__ */ Object.defineProperty({
  __proto__: null,
  Action: Xt,
  Param: zt,
  Ros: fr,
  Service: se,
  Topic: me
}, Symbol.toStringTag, { value: "Module" }));
class mr extends Oe {
  /**
   * @param {Object} options
   * @param {Ros} options.ros - The ROSLIB.Ros connection handle.
   * @param {string} options.serverName - The action server name, like '/fibonacci'.
   * @param {string} options.actionName - The action message name, like 'actionlib_tutorials/FibonacciAction'.
   */
  constructor(t) {
    super(), this.ros = t.ros, this.serverName = t.serverName, this.actionName = t.actionName;
    var u = new me({
      ros: this.ros,
      name: this.serverName + "/goal",
      messageType: this.actionName + "Goal"
    }), s = new me({
      ros: this.ros,
      name: this.serverName + "/feedback",
      messageType: this.actionName + "Feedback"
    }), a = new me({
      ros: this.ros,
      name: this.serverName + "/status",
      messageType: "actionlib_msgs/GoalStatusArray"
    }), n = new me({
      ros: this.ros,
      name: this.serverName + "/result",
      messageType: this.actionName + "Result"
    });
    u.subscribe((l) => {
      this.emit("goal", l);
    }), a.subscribe((l) => {
      l.status_list.forEach((g) => {
        this.emit("status", g);
      });
    }), s.subscribe((l) => {
      this.emit("status", l.status), this.emit("feedback", l.feedback);
    }), n.subscribe((l) => {
      this.emit("status", l.status), this.emit("result", l.result);
    });
  }
}
const dr = /* @__PURE__ */ Object.freeze(/* @__PURE__ */ Object.defineProperty({
  __proto__: null,
  ActionClient: Tt,
  ActionListener: mr,
  Goal: Ht,
  SimpleActionServer: jt
}, Symbol.toStringTag, { value: "Module" }));
class ut {
  constructor(t) {
    this.position = new Me(t?.position), this.orientation = new Ue(t?.orientation);
  }
  /**
   * Apply a transform against this pose.
   *
   * @param {ITransform} tf - The transform to be applied.
   */
  applyTransform(t) {
    this.position.multiplyQuaternion(t.rotation), this.position.add(t.translation);
    const u = new Ue(t.rotation);
    u.multiply(this.orientation), this.orientation = u;
  }
  /**
   * Clone a copy of this pose.
   *
   * @returns {Pose} The cloned pose.
   */
  clone() {
    return new ut(this);
  }
  /**
   * Multiply this pose with another pose without altering this pose.
   *
   * @returns {Pose} The result of the multiplication.
   */
  multiply(t) {
    const u = t.clone();
    return u.applyTransform({
      rotation: this.orientation,
      translation: this.position
    }), u;
  }
  /**
   * Compute the inverse of this pose.
   *
   * @returns {Pose} The inverse of the pose.
   */
  getInverse() {
    const t = this.clone();
    return t.orientation.invert(), t.position.multiplyQuaternion(t.orientation), t.position.x *= -1, t.position.y *= -1, t.position.z *= -1, t;
  }
}
const Er = /* @__PURE__ */ Object.freeze(/* @__PURE__ */ Object.defineProperty({
  __proto__: null,
  Pose: ut,
  Quaternion: Ue,
  Transform: ht,
  Vector3: Me
}, Symbol.toStringTag, { value: "Module" }));
class Dr extends Oe {
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
  constructor(t) {
    super(), this.ros = t.ros, this.fixedFrame = t.fixedFrame || "base_link", this.angularThres = t.angularThres || 2, this.transThres = t.transThres || 0.01, this.rate = t.rate || 10, this.updateDelay = t.updateDelay || 50;
    const u = t.topicTimeout || 2, s = Math.floor(u), a = Math.floor((u - s) * 1e9);
    this.topicTimeout = {
      secs: s,
      nsecs: a
    }, this.serverName = t.serverName || "/tf2_web_republisher", this.goal_id = "", this.frameInfos = {}, this.republisherUpdateRequested = !1, this._subscribeCB = void 0, this._isDisposed = !1, this.actionClient = new Xt({
      ros: t.ros,
      name: this.serverName,
      actionType: "tf2_web_republisher_msgs/TFSubscription"
    });
  }
  /**
   * Process the incoming TF message and send them out using the callback
   * functions.
   *
   * @param {Object} tf - The TF message from the server.
   */
  processTFArray(t) {
    let u = this;
    t.transforms.forEach(function(s) {
      let a = s.child_frame_id;
      a[0] === "/" && (a = a.substring(1));
      const n = u.frameInfos[a];
      n && (n.transform = new ht({
        translation: s.transform.translation,
        rotation: s.transform.rotation
      }), n.cbs.forEach(function(l) {
        l(n.transform);
      }));
    }, this);
  }
  /**
   * Create and send a new goal (or service request) to the tf2_web_republisher
   * based on the current list of TFs.
   */
  updateGoal() {
    const t = {
      source_frames: Object.keys(this.frameInfos),
      target_frame: this.fixedFrame,
      angular_thres: this.angularThres,
      trans_thres: this.transThres,
      rate: this.rate
    };
    this.goal_id !== "" && this.actionClient.cancelGoal(this.goal_id), this.currentGoal = t;
    const u = this.actionClient.sendGoal(
      t,
      (s) => {
      },
      (s) => {
        this.processTFArray(s);
      }
    );
    typeof u == "string" && (this.goal_id = u), this.republisherUpdateRequested = !1;
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
  subscribe(t, u) {
    t[0] === "/" && (t = t.substring(1)), this.frameInfos[t] ? this.frameInfos[t].transform && u(this.frameInfos[t].transform) : (this.frameInfos[t] = {
      cbs: []
    }, this.republisherUpdateRequested || (setTimeout(this.updateGoal.bind(this), this.updateDelay), this.republisherUpdateRequested = !0)), this.frameInfos[t].cbs.push(u);
  }
  /**
   * Unsubscribe from the given TF frame.
   *
   * @param {string} frameID - The TF frame to unsubscribe from.
   * @param {function} callback - The callback function to remove.
   */
  unsubscribe(t, u) {
    t[0] === "/" && (t = t.substring(1));
    const s = this.frameInfos[t];
    for (var a = s && s.cbs || [], n = a.length; n--; )
      a[n] === u && a.splice(n, 1);
    (!u || a.length === 0) && delete this.frameInfos[t];
  }
  /**
   * Unsubscribe and unadvertise all topics associated with this TFClient.
   */
  dispose() {
    this._isDisposed = !0;
  }
}
const vr = /* @__PURE__ */ Object.freeze(/* @__PURE__ */ Object.defineProperty({
  __proto__: null,
  ROS2TFClient: Dr,
  TFClient: Yt
}, Symbol.toStringTag, { value: "Module" }));
var it = /* @__PURE__ */ ((o) => (o[o.SPHERE = 0] = "SPHERE", o[o.BOX = 1] = "BOX", o[o.CYLINDER = 2] = "CYLINDER", o[o.MESH = 3] = "MESH", o))(it || {}), $ = /* @__PURE__ */ ((o) => (o.Name = "name", o.Type = "type", o.Parent = "parent", o.Link = "link", o.Child = "child", o.Limit = "limit", o.Upper = "upper", o.Lower = "lower", o.Origin = "origin", o.Xyz = "xyz", o.Rpy = "rpy", o.Size = "size", o.Rgba = "rgba", o.Length = "length", o.Radius = "radius", o.Visuals = "visual", o.Texture = "texture", o.Filename = "filename", o.Color = "color", o.Geometry = "geometry", o.Material = "material", o.Scale = "scale", o))($ || {});
class Qt {
  constructor({ xml: t }) {
    this.dimension = null, this.type = it.BOX;
    const u = t.getAttribute($.Size)?.split(" ");
    !u || u.length !== 3 || (this.dimension = new Me({
      x: parseFloat(u[0]),
      y: parseFloat(u[1]),
      z: parseFloat(u[2])
    }));
  }
}
class Wt {
  constructor({ xml: t }) {
    this.r = 0, this.g = 0, this.b = 0, this.a = 1;
    const u = t.getAttribute($.Rgba)?.split(" ");
    !u || u.length !== 4 || (this.r = parseFloat(u[0]), this.g = parseFloat(u[1]), this.b = parseFloat(u[2]), this.a = parseFloat(u[3]));
  }
}
class Jt {
  constructor({ xml: t }) {
    this.type = it.CYLINDER, this.length = parseFloat(t.getAttribute($.Length) ?? "NaN"), this.radius = parseFloat(t.getAttribute($.Radius) ?? "NaN");
  }
}
class Ct {
  constructor({ xml: t }) {
    this.textureFilename = null, this.color = null, this.name = t.getAttribute($.Name) ?? "unknown_name";
    const u = t.getElementsByTagName($.Texture);
    u.length > 0 && (this.textureFilename = u[0].getAttribute($.Filename));
    const s = t.getElementsByTagName($.Color);
    s.length > 0 && (this.color = new Wt({
      xml: s[0]
    }));
  }
  isLink() {
    return this.color === null && this.textureFilename === null;
  }
  assign(t) {
    return Object.assign(this, t);
  }
}
class Zt {
  /**
   * @param {Object} options
   * @param {Element} options.xml - The XML element to parse.
   */
  constructor({ xml: t }) {
    this.scale = null, this.type = it.MESH, this.filename = t.getAttribute($.Filename);
    const u = t.getAttribute($.Scale)?.split(" ");
    !u || u.length !== 3 || (this.scale = new Me({
      x: parseFloat(u[0]),
      y: parseFloat(u[1]),
      z: parseFloat(u[2])
    }));
  }
}
class $t {
  constructor({ xml: t }) {
    this.radius = NaN, this.type = it.SPHERE, this.radius = parseFloat(t.getAttribute($.Radius) ?? "NaN");
  }
}
function bt(o) {
  const t = o.getAttribute($.Xyz)?.split(" ");
  let u = new Me();
  t && t.length === 3 && (u = new Me({
    x: parseFloat(t[0]),
    y: parseFloat(t[1]),
    z: parseFloat(t[2])
  }));
  const s = o.getAttribute($.Rpy)?.split(" ");
  let a = new Ue();
  if (s && s.length === 3) {
    const n = parseFloat(s[0]), l = parseFloat(s[1]), g = parseFloat(s[2]), b = n / 2, O = l / 2, A = g / 2, f = Math.sin(b) * Math.cos(O) * Math.cos(A) - Math.cos(b) * Math.sin(O) * Math.sin(A), M = Math.cos(b) * Math.sin(O) * Math.cos(A) + Math.sin(b) * Math.cos(O) * Math.sin(A), S = Math.cos(b) * Math.cos(O) * Math.sin(A) - Math.sin(b) * Math.sin(O) * Math.cos(A), k = Math.cos(b) * Math.cos(O) * Math.cos(A) + Math.sin(b) * Math.sin(O) * Math.sin(A);
    a = new Ue({
      x: f,
      y: M,
      z: S,
      w: k
    }), a.normalize();
  }
  return new ut({
    position: u,
    orientation: a
  });
}
function yt(o) {
  return o.nodeType === 1;
}
function Ar(o) {
  let t = null;
  for (const s of o.childNodes)
    if (yt(s)) {
      t = s;
      break;
    }
  if (!t)
    return null;
  const u = {
    xml: t
  };
  switch (t.nodeName) {
    case "sphere":
      return new $t(u);
    case "box":
      return new Qt(u);
    case "cylinder":
      return new Jt(u);
    case "mesh":
      return new Zt(u);
    default:
      return console.warn(`Unknown geometry type ${t.nodeName}`), null;
  }
}
class Kt {
  constructor({ xml: t }) {
    this.origin = new ut(), this.geometry = null, this.material = null, this.name = t.getAttribute($.Name);
    const u = t.getElementsByTagName($.Origin);
    u.length > 0 && (this.origin = bt(u[0]));
    const s = t.getElementsByTagName($.Geometry);
    s.length > 0 && (this.geometry = Ar(s[0]));
    const a = t.getElementsByTagName($.Material);
    a.length > 0 && (this.material = new Ct({
      xml: a[0]
    }));
  }
}
class er {
  constructor({ xml: t }) {
    this.visuals = [], this.name = t.getAttribute($.Name) ?? "unknown_name";
    const u = t.getElementsByTagName($.Visuals);
    for (const s of u)
      this.visuals.push(
        new Kt({
          xml: s
        })
      );
  }
}
var J = {}, le = {}, Rt;
function nt() {
  if (Rt) return le;
  Rt = 1;
  function o(x, H, j) {
    if (j === void 0 && (j = Array.prototype), x && typeof j.find == "function")
      return j.find.call(x, H);
    for (var re = 0; re < x.length; re++)
      if (u(x, re)) {
        var ie = x[re];
        if (H.call(void 0, ie, re, x))
          return ie;
      }
  }
  function t(x, H) {
    return H === void 0 && (H = Object), H && typeof H.getOwnPropertyDescriptors == "function" && (x = H.create(null, H.getOwnPropertyDescriptors(x))), H && typeof H.freeze == "function" ? H.freeze(x) : x;
  }
  function u(x, H) {
    return Object.prototype.hasOwnProperty.call(x, H);
  }
  function s(x, H) {
    if (x === null || typeof x != "object")
      throw new TypeError("target is not an object");
    for (var j in H)
      u(H, j) && (x[j] = H[j]);
    return x;
  }
  var a = t({
    allowfullscreen: !0,
    async: !0,
    autofocus: !0,
    autoplay: !0,
    checked: !0,
    controls: !0,
    default: !0,
    defer: !0,
    disabled: !0,
    formnovalidate: !0,
    hidden: !0,
    ismap: !0,
    itemscope: !0,
    loop: !0,
    multiple: !0,
    muted: !0,
    nomodule: !0,
    novalidate: !0,
    open: !0,
    playsinline: !0,
    readonly: !0,
    required: !0,
    reversed: !0,
    selected: !0
  });
  function n(x) {
    return u(a, x.toLowerCase());
  }
  var l = t({
    area: !0,
    base: !0,
    br: !0,
    col: !0,
    embed: !0,
    hr: !0,
    img: !0,
    input: !0,
    link: !0,
    meta: !0,
    param: !0,
    source: !0,
    track: !0,
    wbr: !0
  });
  function g(x) {
    return u(l, x.toLowerCase());
  }
  var b = t({
    script: !1,
    style: !1,
    textarea: !0,
    title: !0
  });
  function O(x) {
    var H = x.toLowerCase();
    return u(b, H) && !b[H];
  }
  function A(x) {
    var H = x.toLowerCase();
    return u(b, H) && b[H];
  }
  function f(x) {
    return x === S.HTML;
  }
  function M(x) {
    return f(x) || x === S.XML_XHTML_APPLICATION;
  }
  var S = t({
    /**
     * `text/html`, the only mime type that triggers treating an XML document as HTML.
     *
     * @see https://www.iana.org/assignments/media-types/text/html IANA MimeType registration
     * @see https://en.wikipedia.org/wiki/HTML Wikipedia
     * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMParser/parseFromString MDN
     * @see https://html.spec.whatwg.org/multipage/dynamic-markup-insertion.html#dom-domparser-parsefromstring
     *      WHATWG HTML Spec
     */
    HTML: "text/html",
    /**
     * `application/xml`, the standard mime type for XML documents.
     *
     * @see https://www.iana.org/assignments/media-types/application/xml IANA MimeType
     *      registration
     * @see https://tools.ietf.org/html/rfc7303#section-9.1 RFC 7303
     * @see https://en.wikipedia.org/wiki/XML_and_MIME Wikipedia
     */
    XML_APPLICATION: "application/xml",
    /**
     * `text/xml`, an alias for `application/xml`.
     *
     * @see https://tools.ietf.org/html/rfc7303#section-9.2 RFC 7303
     * @see https://www.iana.org/assignments/media-types/text/xml IANA MimeType registration
     * @see https://en.wikipedia.org/wiki/XML_and_MIME Wikipedia
     */
    XML_TEXT: "text/xml",
    /**
     * `application/xhtml+xml`, indicates an XML document that has the default HTML namespace,
     * but is parsed as an XML document.
     *
     * @see https://www.iana.org/assignments/media-types/application/xhtml+xml IANA MimeType
     *      registration
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-createdocument WHATWG DOM Spec
     * @see https://en.wikipedia.org/wiki/XHTML Wikipedia
     */
    XML_XHTML_APPLICATION: "application/xhtml+xml",
    /**
     * `image/svg+xml`,
     *
     * @see https://www.iana.org/assignments/media-types/image/svg+xml IANA MimeType registration
     * @see https://www.w3.org/TR/SVG11/ W3C SVG 1.1
     * @see https://en.wikipedia.org/wiki/Scalable_Vector_Graphics Wikipedia
     */
    XML_SVG_IMAGE: "image/svg+xml"
  }), k = Object.keys(S).map(function(x) {
    return S[x];
  });
  function Y(x) {
    return k.indexOf(x) > -1;
  }
  var te = t({
    /**
     * The XHTML namespace.
     *
     * @see http://www.w3.org/1999/xhtml
     */
    HTML: "http://www.w3.org/1999/xhtml",
    /**
     * The SVG namespace.
     *
     * @see http://www.w3.org/2000/svg
     */
    SVG: "http://www.w3.org/2000/svg",
    /**
     * The `xml:` namespace.
     *
     * @see http://www.w3.org/XML/1998/namespace
     */
    XML: "http://www.w3.org/XML/1998/namespace",
    /**
     * The `xmlns:` namespace.
     *
     * @see https://www.w3.org/2000/xmlns/
     */
    XMLNS: "http://www.w3.org/2000/xmlns/"
  });
  return le.assign = s, le.find = o, le.freeze = t, le.HTML_BOOLEAN_ATTRIBUTES = a, le.HTML_RAW_TEXT_ELEMENTS = b, le.HTML_VOID_ELEMENTS = l, le.hasDefaultHTMLNamespace = M, le.hasOwn = u, le.isHTMLBooleanAttribute = n, le.isHTMLRawTextElement = O, le.isHTMLEscapableRawTextElement = A, le.isHTMLMimeType = f, le.isHTMLVoidElement = g, le.isValidMimeType = Y, le.MIME_TYPE = S, le.NAMESPACE = te, le;
}
var rt = {}, Bt;
function dt() {
  if (Bt) return rt;
  Bt = 1;
  var o = nt();
  function t(M, S) {
    M.prototype = Object.create(Error.prototype, {
      constructor: { value: M },
      name: { value: M.name, enumerable: !0, writable: S }
    });
  }
  var u = o.freeze({
    /**
     * the default value as defined by the spec
     */
    Error: "Error",
    /**
     * @deprecated
     * Use RangeError instead.
     */
    IndexSizeError: "IndexSizeError",
    /**
     * @deprecated
     * Just to match the related static code, not part of the spec.
     */
    DomstringSizeError: "DomstringSizeError",
    HierarchyRequestError: "HierarchyRequestError",
    WrongDocumentError: "WrongDocumentError",
    InvalidCharacterError: "InvalidCharacterError",
    /**
     * @deprecated
     * Just to match the related static code, not part of the spec.
     */
    NoDataAllowedError: "NoDataAllowedError",
    NoModificationAllowedError: "NoModificationAllowedError",
    NotFoundError: "NotFoundError",
    NotSupportedError: "NotSupportedError",
    InUseAttributeError: "InUseAttributeError",
    InvalidStateError: "InvalidStateError",
    SyntaxError: "SyntaxError",
    InvalidModificationError: "InvalidModificationError",
    NamespaceError: "NamespaceError",
    /**
     * @deprecated
     * Use TypeError for invalid arguments,
     * "NotSupportedError" DOMException for unsupported operations,
     * and "NotAllowedError" DOMException for denied requests instead.
     */
    InvalidAccessError: "InvalidAccessError",
    /**
     * @deprecated
     * Just to match the related static code, not part of the spec.
     */
    ValidationError: "ValidationError",
    /**
     * @deprecated
     * Use TypeError instead.
     */
    TypeMismatchError: "TypeMismatchError",
    SecurityError: "SecurityError",
    NetworkError: "NetworkError",
    AbortError: "AbortError",
    /**
     * @deprecated
     * Just to match the related static code, not part of the spec.
     */
    URLMismatchError: "URLMismatchError",
    QuotaExceededError: "QuotaExceededError",
    TimeoutError: "TimeoutError",
    InvalidNodeTypeError: "InvalidNodeTypeError",
    DataCloneError: "DataCloneError",
    EncodingError: "EncodingError",
    NotReadableError: "NotReadableError",
    UnknownError: "UnknownError",
    ConstraintError: "ConstraintError",
    DataError: "DataError",
    TransactionInactiveError: "TransactionInactiveError",
    ReadOnlyError: "ReadOnlyError",
    VersionError: "VersionError",
    OperationError: "OperationError",
    NotAllowedError: "NotAllowedError",
    OptOutError: "OptOutError"
  }), s = Object.keys(u);
  function a(M) {
    return typeof M == "number" && M >= 1 && M <= 25;
  }
  function n(M) {
    return typeof M == "string" && M.substring(M.length - u.Error.length) === u.Error;
  }
  function l(M, S) {
    a(M) ? (this.name = s[M], this.message = S || "") : (this.message = M, this.name = n(S) ? S : u.Error), Error.captureStackTrace && Error.captureStackTrace(this, l);
  }
  t(l, !0), Object.defineProperties(l.prototype, {
    code: {
      enumerable: !0,
      get: function() {
        var M = s.indexOf(this.name);
        return a(M) ? M : 0;
      }
    }
  });
  for (var g = {
    INDEX_SIZE_ERR: 1,
    DOMSTRING_SIZE_ERR: 2,
    HIERARCHY_REQUEST_ERR: 3,
    WRONG_DOCUMENT_ERR: 4,
    INVALID_CHARACTER_ERR: 5,
    NO_DATA_ALLOWED_ERR: 6,
    NO_MODIFICATION_ALLOWED_ERR: 7,
    NOT_FOUND_ERR: 8,
    NOT_SUPPORTED_ERR: 9,
    INUSE_ATTRIBUTE_ERR: 10,
    INVALID_STATE_ERR: 11,
    SYNTAX_ERR: 12,
    INVALID_MODIFICATION_ERR: 13,
    NAMESPACE_ERR: 14,
    INVALID_ACCESS_ERR: 15,
    VALIDATION_ERR: 16,
    TYPE_MISMATCH_ERR: 17,
    SECURITY_ERR: 18,
    NETWORK_ERR: 19,
    ABORT_ERR: 20,
    URL_MISMATCH_ERR: 21,
    QUOTA_EXCEEDED_ERR: 22,
    TIMEOUT_ERR: 23,
    INVALID_NODE_TYPE_ERR: 24,
    DATA_CLONE_ERR: 25
  }, b = Object.entries(g), O = 0; O < b.length; O++) {
    var A = b[O][0];
    l[A] = b[O][1];
  }
  function f(M, S) {
    this.message = M, this.locator = S, Error.captureStackTrace && Error.captureStackTrace(this, f);
  }
  return t(f), rt.DOMException = l, rt.DOMExceptionName = u, rt.ExceptionCode = g, rt.ParseError = f, rt;
}
var ue = {}, z = {}, Mt;
function tr() {
  if (Mt) return z;
  Mt = 1;
  function o(ne) {
    try {
      typeof ne != "function" && (ne = RegExp);
      var pe = new ne("𝌆", "u").exec("𝌆");
      return !!pe && pe[0].length === 2;
    } catch {
    }
    return !1;
  }
  var t = o();
  function u(ne) {
    if (ne.source[0] !== "[")
      throw new Error(ne + " can not be used with chars");
    return ne.source.slice(1, ne.source.lastIndexOf("]"));
  }
  function s(ne, pe) {
    if (ne.source[0] !== "[")
      throw new Error("/" + ne.source + "/ can not be used with chars_without");
    if (!pe || typeof pe != "string")
      throw new Error(JSON.stringify(pe) + " is not a valid search");
    if (ne.source.indexOf(pe) === -1)
      throw new Error('"' + pe + '" is not is /' + ne.source + "/");
    if (pe === "-" && ne.source.indexOf(pe) !== 1)
      throw new Error('"' + pe + '" is not at the first postion of /' + ne.source + "/");
    return new RegExp(ne.source.replace(pe, ""), t ? "u" : "");
  }
  function a(ne) {
    var pe = this;
    return new RegExp(
      Array.prototype.slice.call(arguments).map(function(Ie) {
        var Re = typeof Ie == "string";
        if (Re && pe === void 0 && Ie === "|")
          throw new Error("use regg instead of reg to wrap expressions with `|`!");
        return Re ? Ie : Ie.source;
      }).join(""),
      t ? "mu" : "m"
    );
  }
  function n(ne) {
    if (arguments.length === 0)
      throw new Error("no parameters provided");
    return a.apply(n, ["(?:"].concat(Array.prototype.slice.call(arguments), [")"]));
  }
  var l = "�", g = /[-\x09\x0A\x0D\x20-\x2C\x2E-\uD7FF\uE000-\uFFFD]/;
  t && (g = a("[", u(g), "\\u{10000}-\\u{10FFFF}", "]"));
  var b = /[\x20\x09\x0D\x0A]/, O = u(b), A = a(b, "+"), f = a(b, "*"), M = /[:_a-zA-Z\xC0-\xD6\xD8-\xF6\xF8-\u02FF\u0370-\u1FFF\u200C-\u200D\u2070-\u218F\u2C00-\u2FEF\u3001-\uD7FF\uF900-\uFDCF\uFDF0-\uFFFD]/;
  t && (M = a("[", u(M), "\\u{10000}-\\u{10FFFF}", "]"));
  var S = u(M), k = a("[", S, u(/[-.0-9\xB7]/), u(/[\u0300-\u036F\u203F-\u2040]/), "]"), Y = a(M, k, "*"), te = a(k, "+"), x = a("&", Y, ";"), H = n(/&#[0-9]+;|&#x[0-9a-fA-F]+;/), j = n(x, "|", H), re = a("%", Y, ";"), ie = n(
    a('"', n(/[^%&"]/, "|", re, "|", j), "*", '"'),
    "|",
    a("'", n(/[^%&']/, "|", re, "|", j), "*", "'")
  ), m = n('"', n(/[^<&"]/, "|", j), "*", '"', "|", "'", n(/[^<&']/, "|", j), "*", "'"), N = s(M, ":"), P = s(k, ":"), G = a(N, P, "*"), W = a(G, n(":", G), "?"), v = a("^", W, "$"), U = a("(", W, ")"), C = n(/"[^"]*"|'[^']*'/), L = a(/^<\?/, "(", Y, ")", n(A, "(", g, "*?)"), "?", /\?>/), h = /[\x20\x0D\x0Aa-zA-Z0-9-'()+,./:=?;!*#@$_%]/, p = n('"', h, '*"', "|", "'", s(h, "'"), "*'"), d = "<!--", T = "-->", I = a(d, n(s(g, "-"), "|", a("-", s(g, "-"))), "*", T), D = "#PCDATA", R = n(
    a(/\(/, f, D, n(f, /\|/, f, W), "*", f, /\)\*/),
    "|",
    a(/\(/, f, D, f, /\)/)
  ), X = /[?*+]?/, B = a(
    /\([^>]+\)/,
    X
    /*regg(choice, '|', seq), _children_quantity*/
  ), y = n("EMPTY", "|", "ANY", "|", R, "|", B), w = "<!ELEMENT", q = a(w, A, n(W, "|", re), A, n(y, "|", re), f, ">"), Q = a("NOTATION", A, /\(/, f, Y, n(f, /\|/, f, Y), "*", f, /\)/), he = a(/\(/, f, te, n(f, /\|/, f, te), "*", f, /\)/), be = n(Q, "|", he), ve = n(/CDATA|ID|IDREF|IDREFS|ENTITY|ENTITIES|NMTOKEN|NMTOKENS/, "|", be), ae = n(/#REQUIRED|#IMPLIED/, "|", n(n("#FIXED", A), "?", m)), F = n(A, Y, A, ve, A, ae), Le = "<!ATTLIST", we = a(Le, A, Y, F, "*", f, ">"), fe = "about:legacy-compat", Fe = n('"' + fe + '"', "|", "'" + fe + "'"), _e = "SYSTEM", ye = "PUBLIC", xe = n(n(_e, A, C), "|", n(ye, A, p, A, C)), ke = a(
    "^",
    n(
      n(_e, A, "(?<SystemLiteralOnly>", C, ")"),
      "|",
      n(ye, A, "(?<PubidLiteral>", p, ")", A, "(?<SystemLiteral>", C, ")")
    )
  ), qe = n(A, "NDATA", A, Y), Te = n(ie, "|", n(xe, qe, "?")), K = "<!ENTITY", Pe = a(K, A, Y, A, Te, f, ">"), oe = n(ie, "|", xe), Ge = a(K, A, "%", A, Y, A, oe, f, ">"), st = n(Pe, "|", Ge), Ve = a(ye, A, p), ze = a("<!NOTATION", A, Y, A, n(xe, "|", Ve), f, ">"), V = a(f, "=", f), ee = /1[.]\d+/, Ae = a(A, "version", V, n("'", ee, "'", "|", '"', ee, '"')), ge = /[A-Za-z][-A-Za-z0-9._]*/, He = n(A, "encoding", V, n('"', ge, '"', "|", "'", ge, "'")), Je = n(A, "standalone", V, n("'", n("yes", "|", "no"), "'", "|", '"', n("yes", "|", "no"), '"')), Ze = a(/^<\?xml/, Ae, He, "?", Je, "?", f, /\?>/), $e = "<!DOCTYPE", at = "<![CDATA[", ot = "]]>", Ke = /<!\[CDATA\[/, Ye = /\]\]>/, et = a(g, "*?", Ye), ft = a(Ke, et);
  return z.chars = u, z.chars_without = s, z.detectUnicodeSupport = o, z.reg = a, z.regg = n, z.ABOUT_LEGACY_COMPAT = fe, z.ABOUT_LEGACY_COMPAT_SystemLiteral = Fe, z.AttlistDecl = we, z.CDATA_START = at, z.CDATA_END = ot, z.CDSect = ft, z.Char = g, z.Comment = I, z.COMMENT_START = d, z.COMMENT_END = T, z.DOCTYPE_DECL_START = $e, z.elementdecl = q, z.EntityDecl = st, z.EntityValue = ie, z.ExternalID = xe, z.ExternalID_match = ke, z.Name = Y, z.NotationDecl = ze, z.Reference = j, z.PEReference = re, z.PI = L, z.PUBLIC = ye, z.PubidLiteral = p, z.QName = W, z.QName_exact = v, z.QName_group = U, z.S = A, z.SChar_s = O, z.S_OPT = f, z.SYSTEM = _e, z.SystemLiteral = C, z.UNICODE_REPLACEMENT_CHARACTER = l, z.UNICODE_SUPPORT = t, z.XMLDecl = Ze, z;
}
var Lt;
function rr() {
  if (Lt) return ue;
  Lt = 1;
  var o = nt(), t = o.find, u = o.hasDefaultHTMLNamespace, s = o.hasOwn, a = o.isHTMLMimeType, n = o.isHTMLRawTextElement, l = o.isHTMLVoidElement, g = o.MIME_TYPE, b = o.NAMESPACE, O = Symbol(), A = dt(), f = A.DOMException, M = A.DOMExceptionName, S = tr();
  function k(e) {
    if (e !== O)
      throw new TypeError("Illegal constructor");
  }
  function Y(e) {
    return e !== "";
  }
  function te(e) {
    return e ? e.split(/[\t\n\f\r ]+/).filter(Y) : [];
  }
  function x(e, r) {
    return s(e, r) || (e[r] = !0), e;
  }
  function H(e) {
    if (!e) return [];
    var r = te(e);
    return Object.keys(r.reduce(x, {}));
  }
  function j(e) {
    return function(r) {
      return e && e.indexOf(r) !== -1;
    };
  }
  function re(e) {
    if (!S.QName_exact.test(e))
      throw new f(f.INVALID_CHARACTER_ERR, 'invalid character in qualified name "' + e + '"');
  }
  function ie(e, r) {
    re(r), e = e || null;
    var i = null, c = r;
    if (r.indexOf(":") >= 0) {
      var E = r.split(":");
      i = E[0], c = E[1];
    }
    if (i !== null && e === null)
      throw new f(f.NAMESPACE_ERR, "prefix is non-null and namespace is null");
    if (i === "xml" && e !== o.NAMESPACE.XML)
      throw new f(f.NAMESPACE_ERR, 'prefix is "xml" and namespace is not the XML namespace');
    if ((i === "xmlns" || r === "xmlns") && e !== o.NAMESPACE.XMLNS)
      throw new f(
        f.NAMESPACE_ERR,
        'either qualifiedName or prefix is "xmlns" and namespace is not the XMLNS namespace'
      );
    if (e === o.NAMESPACE.XMLNS && i !== "xmlns" && r !== "xmlns")
      throw new f(
        f.NAMESPACE_ERR,
        'namespace is the XMLNS namespace and neither qualifiedName nor prefix is "xmlns"'
      );
    return [e, i, c];
  }
  function m(e, r) {
    for (var i in e)
      s(e, i) && (r[i] = e[i]);
  }
  function N(e, r) {
    var i = e.prototype;
    if (!(i instanceof r)) {
      let c = function() {
      };
      c.prototype = r.prototype, c = new c(), m(i, c), e.prototype = i = c;
    }
    i.constructor != e && (typeof e != "function" && console.error("unknown Class:" + e), i.constructor = e);
  }
  var P = {}, G = P.ELEMENT_NODE = 1, W = P.ATTRIBUTE_NODE = 2, v = P.TEXT_NODE = 3, U = P.CDATA_SECTION_NODE = 4, C = P.ENTITY_REFERENCE_NODE = 5, L = P.ENTITY_NODE = 6, h = P.PROCESSING_INSTRUCTION_NODE = 7, p = P.COMMENT_NODE = 8, d = P.DOCUMENT_NODE = 9, T = P.DOCUMENT_TYPE_NODE = 10, I = P.DOCUMENT_FRAGMENT_NODE = 11, D = P.NOTATION_NODE = 12, R = o.freeze({
    DOCUMENT_POSITION_DISCONNECTED: 1,
    DOCUMENT_POSITION_PRECEDING: 2,
    DOCUMENT_POSITION_FOLLOWING: 4,
    DOCUMENT_POSITION_CONTAINS: 8,
    DOCUMENT_POSITION_CONTAINED_BY: 16,
    DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC: 32
  });
  function X(e, r) {
    if (r.length < e.length) return X(r, e);
    var i = null;
    for (var c in e) {
      if (e[c] !== r[c]) return i;
      i = e[c];
    }
    return i;
  }
  function B(e) {
    return e.guid || (e.guid = Math.random()), e.guid;
  }
  function y() {
  }
  y.prototype = {
    /**
     * The number of nodes in the list. The range of valid child node indices is 0 to length-1
     * inclusive.
     *
     * @type {number}
     */
    length: 0,
    /**
     * Returns the item at `index`. If index is greater than or equal to the number of nodes in
     * the list, this returns null.
     *
     * @param index
     * Unsigned long Index into the collection.
     * @returns {Node | null}
     * The node at position `index` in the NodeList,
     * or null if that is not a valid index.
     */
    item: function(e) {
      return e >= 0 && e < this.length ? this[e] : null;
    },
    /**
     * Returns a string representation of the NodeList.
     *
     * @param {unknown} nodeFilter
     * __A filter function? Not implemented according to the spec?__.
     * @returns {string}
     * A string representation of the NodeList.
     */
    toString: function(e) {
      for (var r = [], i = 0; i < this.length; i++)
        Re(this[i], r, e);
      return r.join("");
    },
    /**
     * Filters the NodeList based on a predicate.
     *
     * @param {function(Node): boolean} predicate
     * - A predicate function to filter the NodeList.
     * @returns {Node[]}
     * An array of nodes that satisfy the predicate.
     * @private
     */
    filter: function(e) {
      return Array.prototype.filter.call(this, e);
    },
    /**
     * Returns the first index at which a given node can be found in the NodeList, or -1 if it is
     * not present.
     *
     * @param {Node} item
     * - The Node item to locate in the NodeList.
     * @returns {number}
     * The first index of the node in the NodeList; -1 if not found.
     * @private
     */
    indexOf: function(e) {
      return Array.prototype.indexOf.call(this, e);
    }
  }, y.prototype[Symbol.iterator] = function() {
    var e = this, r = 0;
    return {
      next: function() {
        return r < e.length ? {
          value: e[r++],
          done: !1
        } : {
          done: !0
        };
      },
      return: function() {
        return {
          done: !0
        };
      }
    };
  };
  function w(e, r) {
    this._node = e, this._refresh = r, q(this);
  }
  function q(e) {
    var r = e._node._inc || e._node.ownerDocument._inc;
    if (e._inc !== r) {
      var i = e._refresh(e._node);
      if (wt(e, "length", i.length), !e.$$length || i.length < e.$$length)
        for (var c = i.length; c in e; c++)
          s(e, c) && delete e[c];
      m(i, e), e._inc = r;
    }
  }
  w.prototype.item = function(e) {
    return q(this), this[e] || null;
  }, N(w, y);
  function Q() {
  }
  function he(e, r) {
    for (var i = 0; i < e.length; ) {
      if (e[i] === r)
        return i;
      i++;
    }
  }
  function be(e, r, i, c) {
    if (c ? r[he(r, c)] = i : (r[r.length] = i, r.length++), e) {
      i.ownerElement = e;
      var E = e.ownerDocument;
      E && (c && _e(E, e, c), Fe(E, e, i));
    }
  }
  function ve(e, r, i) {
    var c = he(r, i);
    if (c >= 0) {
      for (var E = r.length - 1; c <= E; )
        r[c] = r[++c];
      if (r.length = E, e) {
        var _ = e.ownerDocument;
        _ && _e(_, e, i), i.ownerElement = null;
      }
    }
  }
  Q.prototype = {
    length: 0,
    item: y.prototype.item,
    /**
     * Get an attribute by name. Note: Name is in lower case in case of HTML namespace and
     * document.
     *
     * @param {string} localName
     * The local name of the attribute.
     * @returns {Attr | null}
     * The attribute with the given local name, or null if no such attribute exists.
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-get-by-name
     */
    getNamedItem: function(e) {
      this._ownerElement && this._ownerElement._isInHTMLDocumentAndNamespace() && (e = e.toLowerCase());
      for (var r = 0; r < this.length; ) {
        var i = this[r];
        if (i.nodeName === e)
          return i;
        r++;
      }
      return null;
    },
    /**
     * Set an attribute.
     *
     * @param {Attr} attr
     * The attribute to set.
     * @returns {Attr | null}
     * The old attribute with the same local name and namespace URI as the new one, or null if no
     * such attribute exists.
     * @throws {DOMException}
     * With code:
     * - {@link INUSE_ATTRIBUTE_ERR} - If the attribute is already an attribute of another
     * element.
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-set
     */
    setNamedItem: function(e) {
      var r = e.ownerElement;
      if (r && r !== this._ownerElement)
        throw new f(f.INUSE_ATTRIBUTE_ERR);
      var i = this.getNamedItemNS(e.namespaceURI, e.localName);
      return i === e ? e : (be(this._ownerElement, this, e, i), i);
    },
    /**
     * Set an attribute, replacing an existing attribute with the same local name and namespace
     * URI if one exists.
     *
     * @param {Attr} attr
     * The attribute to set.
     * @returns {Attr | null}
     * The old attribute with the same local name and namespace URI as the new one, or null if no
     * such attribute exists.
     * @throws {DOMException}
     * Throws a DOMException with the name "InUseAttributeError" if the attribute is already an
     * attribute of another element.
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-set
     */
    setNamedItemNS: function(e) {
      return this.setNamedItem(e);
    },
    /**
     * Removes an attribute specified by the local name.
     *
     * @param {string} localName
     * The local name of the attribute to be removed.
     * @returns {Attr}
     * The attribute node that was removed.
     * @throws {DOMException}
     * With code:
     * - {@link DOMException.NOT_FOUND_ERR} if no attribute with the given name is found.
     * @see https://dom.spec.whatwg.org/#dom-namednodemap-removenameditem
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-remove-by-name
     */
    removeNamedItem: function(e) {
      var r = this.getNamedItem(e);
      if (!r)
        throw new f(f.NOT_FOUND_ERR, e);
      return ve(this._ownerElement, this, r), r;
    },
    /**
     * Removes an attribute specified by the namespace and local name.
     *
     * @param {string | null} namespaceURI
     * The namespace URI of the attribute to be removed.
     * @param {string} localName
     * The local name of the attribute to be removed.
     * @returns {Attr}
     * The attribute node that was removed.
     * @throws {DOMException}
     * With code:
     * - {@link DOMException.NOT_FOUND_ERR} if no attribute with the given namespace URI and local
     * name is found.
     * @see https://dom.spec.whatwg.org/#dom-namednodemap-removenameditemns
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-remove-by-namespace
     */
    removeNamedItemNS: function(e, r) {
      var i = this.getNamedItemNS(e, r);
      if (!i)
        throw new f(f.NOT_FOUND_ERR, e ? e + " : " + r : r);
      return ve(this._ownerElement, this, i), i;
    },
    /**
     * Get an attribute by namespace and local name.
     *
     * @param {string | null} namespaceURI
     * The namespace URI of the attribute.
     * @param {string} localName
     * The local name of the attribute.
     * @returns {Attr | null}
     * The attribute with the given namespace URI and local name, or null if no such attribute
     * exists.
     * @see https://dom.spec.whatwg.org/#concept-element-attributes-get-by-namespace
     */
    getNamedItemNS: function(e, r) {
      e || (e = null);
      for (var i = 0; i < this.length; ) {
        var c = this[i];
        if (c.localName === r && c.namespaceURI === e)
          return c;
        i++;
      }
      return null;
    }
  }, Q.prototype[Symbol.iterator] = function() {
    var e = this, r = 0;
    return {
      next: function() {
        return r < e.length ? {
          value: e[r++],
          done: !1
        } : {
          done: !0
        };
      },
      return: function() {
        return {
          done: !0
        };
      }
    };
  };
  function ae() {
  }
  ae.prototype = {
    /**
     * Test if the DOM implementation implements a specific feature and version, as specified in
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/core.html#DOMFeatures DOM Features}.
     *
     * The DOMImplementation.hasFeature() method returns a Boolean flag indicating if a given
     * feature is supported. The different implementations fairly diverged in what kind of
     * features were reported. The latest version of the spec settled to force this method to
     * always return true, where the functionality was accurate and in use.
     *
     * @deprecated
     * It is deprecated and modern browsers return true in all cases.
     * @function DOMImplementation#hasFeature
     * @param {string} feature
     * The name of the feature to test.
     * @param {string} [version]
     * This is the version number of the feature to test.
     * @returns {boolean}
     * Always returns true.
     * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMImplementation/hasFeature MDN
     * @see https://www.w3.org/TR/REC-DOM-Level-1/level-one-core.html#ID-5CED94D7 DOM Level 1 Core
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-hasfeature DOM Living Standard
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#ID-5CED94D7 DOM Level 3 Core
     */
    hasFeature: function(e, r) {
      return !0;
    },
    /**
     * Creates a DOM Document object of the specified type with its document element. Note that
     * based on the {@link DocumentType}
     * given to create the document, the implementation may instantiate specialized
     * {@link Document} objects that support additional features than the "Core", such as "HTML"
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#DOM2HTML DOM Level 2 HTML}.
     * On the other hand, setting the {@link DocumentType} after the document was created makes
     * this very unlikely to happen. Alternatively, specialized {@link Document} creation methods,
     * such as createHTMLDocument
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#DOM2HTML DOM Level 2 HTML},
     * can be used to obtain specific types of {@link Document} objects.
     *
     * __It behaves slightly different from the description in the living standard__:
     * - There is no interface/class `XMLDocument`, it returns a `Document`
     * instance (with it's `type` set to `'xml'`).
     * - `encoding`, `mode`, `origin`, `url` fields are currently not declared.
     *
     * @function DOMImplementation.createDocument
     * @param {string | null} namespaceURI
     * The
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/glossary.html#dt-namespaceURI namespace URI}
     * of the document element to create or null.
     * @param {string | null} qualifiedName
     * The
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/glossary.html#dt-qualifiedname qualified name}
     * of the document element to be created or null.
     * @param {DocumentType | null} [doctype=null]
     * The type of document to be created or null. When doctype is not null, its
     * {@link Node#ownerDocument} attribute is set to the document being created. Default is
     * `null`
     * @returns {Document}
     * A new {@link Document} object with its document element. If the NamespaceURI,
     * qualifiedName, and doctype are null, the returned {@link Document} is empty with no
     * document element.
     * @throws {DOMException}
     * With code:
     *
     * - `INVALID_CHARACTER_ERR`: Raised if the specified qualified name is not an XML name
     * according to {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#XML XML 1.0}.
     * - `NAMESPACE_ERR`: Raised if the qualifiedName is malformed, if the qualifiedName has a
     * prefix and the namespaceURI is null, or if the qualifiedName is null and the namespaceURI
     * is different from null, or if the qualifiedName has a prefix that is "xml" and the
     * namespaceURI is different from "{@link http://www.w3.org/XML/1998/namespace}"
     * {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#Namespaces XML Namespaces},
     * or if the DOM implementation does not support the "XML" feature but a non-null namespace
     * URI was provided, since namespaces were defined by XML.
     * - `WRONG_DOCUMENT_ERR`: Raised if doctype has already been used with a different document
     * or was created from a different implementation.
     * - `NOT_SUPPORTED_ERR`: May be raised if the implementation does not support the feature
     * "XML" and the language exposed through the Document does not support XML Namespaces (such
     * as {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#HTML40 HTML 4.01}).
     * @since DOM Level 2.
     * @see {@link #createHTMLDocument}
     * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMImplementation/createDocument MDN
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-createdocument DOM Living Standard
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Level-2-Core-DOM-createDocument DOM
     *      Level 3 Core
     * @see https://www.w3.org/TR/DOM-Level-2-Core/core.html#Level-2-Core-DOM-createDocument DOM
     *      Level 2 Core (initial)
     */
    createDocument: function(e, r, i) {
      var c = g.XML_APPLICATION;
      e === b.HTML ? c = g.XML_XHTML_APPLICATION : e === b.SVG && (c = g.XML_SVG_IMAGE);
      var E = new fe(O, { contentType: c });
      if (E.implementation = this, E.childNodes = new y(), E.doctype = i || null, i && E.appendChild(i), r) {
        var _ = E.createElementNS(e, r);
        E.appendChild(_);
      }
      return E;
    },
    /**
     * Creates an empty DocumentType node. Entity declarations and notations are not made
     * available. Entity reference expansions and default attribute additions do not occur.
     *
     * **This behavior is slightly different from the one in the specs**:
     * - `encoding`, `mode`, `origin`, `url` fields are currently not declared.
     * - `publicId` and `systemId` contain the raw data including any possible quotes,
     *   so they can always be serialized back to the original value
     * - `internalSubset` contains the raw string between `[` and `]` if present,
     *   but is not parsed or validated in any form.
     *
     * @function DOMImplementation#createDocumentType
     * @param {string} qualifiedName
     * The {@link https://www.w3.org/TR/DOM-Level-3-Core/glossary.html#dt-qualifiedname qualified
     * name} of the document type to be created.
     * @param {string} [publicId]
     * The external subset public identifier.
     * @param {string} [systemId]
     * The external subset system identifier.
     * @param {string} [internalSubset]
     * the internal subset or an empty string if it is not present
     * @returns {DocumentType}
     * A new {@link DocumentType} node with {@link Node#ownerDocument} set to null.
     * @throws {DOMException}
     * With code:
     *
     * - `INVALID_CHARACTER_ERR`: Raised if the specified qualified name is not an XML name
     * according to {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#XML XML 1.0}.
     * - `NAMESPACE_ERR`: Raised if the qualifiedName is malformed.
     * - `NOT_SUPPORTED_ERR`: May be raised if the implementation does not support the feature
     * "XML" and the language exposed through the Document does not support XML Namespaces (such
     * as {@link https://www.w3.org/TR/DOM-Level-3-Core/references.html#HTML40 HTML 4.01}).
     * @since DOM Level 2.
     * @see https://developer.mozilla.org/en-US/docs/Web/API/DOMImplementation/createDocumentType
     *      MDN
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-createdocumenttype DOM Living
     *      Standard
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Level-3-Core-DOM-createDocType DOM
     *      Level 3 Core
     * @see https://www.w3.org/TR/DOM-Level-2-Core/core.html#Level-2-Core-DOM-createDocType DOM
     *      Level 2 Core
     * @see https://github.com/xmldom/xmldom/blob/master/CHANGELOG.md#050
     * @see https://www.w3.org/TR/DOM-Level-2-Core/#core-ID-Core-DocType-internalSubset
     * @prettierignore
     */
    createDocumentType: function(e, r, i, c) {
      re(e);
      var E = new $e(O);
      return E.name = e, E.nodeName = e, E.publicId = r || "", E.systemId = i || "", E.internalSubset = c || "", E.childNodes = new y(), E;
    },
    /**
     * Returns an HTML document, that might already have a basic DOM structure.
     *
     * __It behaves slightly different from the description in the living standard__:
     * - If the first argument is `false` no initial nodes are added (steps 3-7 in the specs are
     * omitted)
     * - `encoding`, `mode`, `origin`, `url` fields are currently not declared.
     *
     * @param {string | false} [title]
     * A string containing the title to give the new HTML document.
     * @returns {Document}
     * The HTML document.
     * @since WHATWG Living Standard.
     * @see {@link #createDocument}
     * @see https://dom.spec.whatwg.org/#dom-domimplementation-createhtmldocument
     * @see https://dom.spec.whatwg.org/#html-document
     */
    createHTMLDocument: function(e) {
      var r = new fe(O, { contentType: g.HTML });
      if (r.implementation = this, r.childNodes = new y(), e !== !1) {
        r.doctype = this.createDocumentType("html"), r.doctype.ownerDocument = r, r.appendChild(r.doctype);
        var i = r.createElement("html");
        r.appendChild(i);
        var c = r.createElement("head");
        if (i.appendChild(c), typeof e == "string") {
          var E = r.createElement("title");
          E.appendChild(r.createTextNode(e)), c.appendChild(E);
        }
        i.appendChild(r.createElement("body"));
      }
      return r;
    }
  };
  function F(e) {
    k(e);
  }
  F.prototype = {
    /**
     * The first child of this node.
     *
     * @type {Node | null}
     */
    firstChild: null,
    /**
     * The last child of this node.
     *
     * @type {Node | null}
     */
    lastChild: null,
    /**
     * The previous sibling of this node.
     *
     * @type {Node | null}
     */
    previousSibling: null,
    /**
     * The next sibling of this node.
     *
     * @type {Node | null}
     */
    nextSibling: null,
    /**
     * The parent node of this node.
     *
     * @type {Node | null}
     */
    parentNode: null,
    /**
     * The parent element of this node.
     *
     * @type {Element | null}
     */
    get parentElement() {
      return this.parentNode && this.parentNode.nodeType === this.ELEMENT_NODE ? this.parentNode : null;
    },
    /**
     * The child nodes of this node.
     *
     * @type {NodeList}
     */
    childNodes: null,
    /**
     * The document object associated with this node.
     *
     * @type {Document | null}
     */
    ownerDocument: null,
    /**
     * The value of this node.
     *
     * @type {string | null}
     */
    nodeValue: null,
    /**
     * The namespace URI of this node.
     *
     * @type {string | null}
     */
    namespaceURI: null,
    /**
     * The prefix of the namespace for this node.
     *
     * @type {string | null}
     */
    prefix: null,
    /**
     * The local part of the qualified name of this node.
     *
     * @type {string | null}
     */
    localName: null,
    /**
     * The baseURI is currently always `about:blank`,
     * since that's what happens when you create a document from scratch.
     *
     * @type {'about:blank'}
     */
    baseURI: "about:blank",
    /**
     * Is true if this node is part of a document.
     *
     * @type {boolean}
     */
    get isConnected() {
      var e = this.getRootNode();
      return e && e.nodeType === e.DOCUMENT_NODE;
    },
    /**
     * Checks whether `other` is an inclusive descendant of this node.
     *
     * @param {Node | null | undefined} other
     * The node to check.
     * @returns {boolean}
     * True if `other` is an inclusive descendant of this node; false otherwise.
     * @see https://dom.spec.whatwg.org/#dom-node-contains
     */
    contains: function(e) {
      if (!e) return !1;
      var r = e;
      do {
        if (this === r) return !0;
        r = e.parentNode;
      } while (r);
      return !1;
    },
    /**
     * @typedef GetRootNodeOptions
     * @property {boolean} [composed=false]
     */
    /**
     * Searches for the root node of this node.
     *
     * **This behavior is slightly different from the in the specs**:
     * - ignores `options.composed`, since `ShadowRoot`s are unsupported, always returns root.
     *
     * @param {GetRootNodeOptions} [options]
     * @returns {Node}
     * Root node.
     * @see https://dom.spec.whatwg.org/#dom-node-getrootnode
     * @see https://dom.spec.whatwg.org/#concept-shadow-including-root
     */
    getRootNode: function(e) {
      var r = this;
      do {
        if (!r.parentNode)
          return r;
        r = r.parentNode;
      } while (r);
    },
    /**
     * Checks whether the given node is equal to this node.
     *
     * @param {Node} [otherNode]
     * @see https://dom.spec.whatwg.org/#concept-node-equals
     */
    isEqualNode: function(e) {
      if (!e || this.nodeType !== e.nodeType) return !1;
      switch (this.nodeType) {
        case this.DOCUMENT_TYPE_NODE:
          if (this.name !== e.name || this.publicId !== e.publicId || this.systemId !== e.systemId) return !1;
          break;
        case this.ELEMENT_NODE:
          if (this.namespaceURI !== e.namespaceURI || this.prefix !== e.prefix || this.localName !== e.localName || this.attributes.length !== e.attributes.length) return !1;
          for (var r = 0; r < this.attributes.length; r++) {
            var i = this.attributes.item(r);
            if (!i.isEqualNode(e.getAttributeNodeNS(i.namespaceURI, i.localName)))
              return !1;
          }
          break;
        case this.ATTRIBUTE_NODE:
          if (this.namespaceURI !== e.namespaceURI || this.localName !== e.localName || this.value !== e.value) return !1;
          break;
        case this.PROCESSING_INSTRUCTION_NODE:
          if (this.target !== e.target || this.data !== e.data)
            return !1;
          break;
        case this.TEXT_NODE:
        case this.COMMENT_NODE:
          if (this.data !== e.data) return !1;
          break;
      }
      if (this.childNodes.length !== e.childNodes.length)
        return !1;
      for (var r = 0; r < this.childNodes.length; r++)
        if (!this.childNodes[r].isEqualNode(e.childNodes[r]))
          return !1;
      return !0;
    },
    /**
     * Checks whether or not the given node is this node.
     *
     * @param {Node} [otherNode]
     */
    isSameNode: function(e) {
      return this === e;
    },
    /**
     * Inserts a node before a reference node as a child of this node.
     *
     * @param {Node} newChild
     * The new child node to be inserted.
     * @param {Node | null} refChild
     * The reference node before which newChild will be inserted.
     * @returns {Node}
     * The new child node successfully inserted.
     * @throws {DOMException}
     * Throws a DOMException if inserting the node would result in a DOM tree that is not
     * well-formed, or if `child` is provided but is not a child of `parent`.
     * See {@link _insertBefore} for more details.
     * @since Modified in DOM L2
     */
    insertBefore: function(e, r) {
      return V(this, e, r);
    },
    /**
     * Replaces an old child node with a new child node within this node.
     *
     * @param {Node} newChild
     * The new node that is to replace the old node.
     * If it already exists in the DOM, it is removed from its original position.
     * @param {Node} oldChild
     * The existing child node to be replaced.
     * @returns {Node}
     * Returns the replaced child node.
     * @throws {DOMException}
     * Throws a DOMException if replacing the node would result in a DOM tree that is not
     * well-formed, or if `oldChild` is not a child of `this`.
     * This can also occur if the pre-replacement validity assertion fails.
     * See {@link _insertBefore}, {@link Node.removeChild}, and
     * {@link assertPreReplacementValidityInDocument} for more details.
     * @see https://dom.spec.whatwg.org/#concept-node-replace
     */
    replaceChild: function(e, r) {
      V(this, e, r, ze), r && this.removeChild(r);
    },
    /**
     * Removes an existing child node from this node.
     *
     * @param {Node} oldChild
     * The child node to be removed.
     * @returns {Node}
     * Returns the removed child node.
     * @throws {DOMException}
     * Throws a DOMException if `oldChild` is not a child of `this`.
     * See {@link _removeChild} for more details.
     */
    removeChild: function(e) {
      return xe(this, e);
    },
    /**
     * Appends a child node to this node.
     *
     * @param {Node} newChild
     * The child node to be appended to this node.
     * If it already exists in the DOM, it is removed from its original position.
     * @returns {Node}
     * Returns the appended child node.
     * @throws {DOMException}
     * Throws a DOMException if appending the node would result in a DOM tree that is not
     * well-formed, or if `newChild` is not a valid Node.
     * See {@link insertBefore} for more details.
     */
    appendChild: function(e) {
      return this.insertBefore(e, null);
    },
    /**
     * Determines whether this node has any child nodes.
     *
     * @returns {boolean}
     * Returns true if this node has any child nodes, and false otherwise.
     */
    hasChildNodes: function() {
      return this.firstChild != null;
    },
    /**
     * Creates a copy of the calling node.
     *
     * @param {boolean} deep
     * If true, the contents of the node are recursively copied.
     * If false, only the node itself (and its attributes, if it is an element) are copied.
     * @returns {Node}
     * Returns the newly created copy of the node.
     * @throws {DOMException}
     * May throw a DOMException if operations within {@link Element#setAttributeNode} or
     * {@link Node#appendChild} (which are potentially invoked in this method) do not meet their
     * specific constraints.
     * @see {@link cloneNode}
     */
    cloneNode: function(e) {
      return Et(this.ownerDocument || this, this, e);
    },
    /**
     * Puts the specified node and all of its subtree into a "normalized" form. In a normalized
     * subtree, no text nodes in the subtree are empty and there are no adjacent text nodes.
     *
     * Specifically, this method merges any adjacent text nodes (i.e., nodes for which `nodeType`
     * is `TEXT_NODE`) into a single node with the combined data. It also removes any empty text
     * nodes.
     *
     * This method operates recursively, so it also normalizes any and all descendent nodes within
     * the subtree.
     *
     * @throws {DOMException}
     * May throw a DOMException if operations within removeChild or appendData (which are
     * potentially invoked in this method) do not meet their specific constraints.
     * @since Modified in DOM Level 2
     * @see {@link Node.removeChild}
     * @see {@link CharacterData.appendData}
     */
    normalize: function() {
      for (var e = this.firstChild; e; ) {
        var r = e.nextSibling;
        r && r.nodeType == v && e.nodeType == v ? (this.removeChild(r), e.appendData(r.data)) : (e.normalize(), e = r);
      }
    },
    /**
     * Checks whether the DOM implementation implements a specific feature and its version.
     *
     * @deprecated
     * Since `DOMImplementation.hasFeature` is deprecated and always returns true.
     * @param {string} feature
     * The package name of the feature to test. This is the same name that can be passed to the
     * method `hasFeature` on `DOMImplementation`.
     * @param {string} version
     * This is the version number of the package name to test.
     * @returns {boolean}
     * Returns true in all cases in the current implementation.
     * @since Introduced in DOM Level 2
     * @see {@link DOMImplementation.hasFeature}
     */
    isSupported: function(e, r) {
      return this.ownerDocument.implementation.hasFeature(e, r);
    },
    /**
     * Look up the prefix associated to the given namespace URI, starting from this node.
     * **The default namespace declarations are ignored by this method.**
     * See Namespace Prefix Lookup for details on the algorithm used by this method.
     *
     * **This behavior is different from the in the specs**:
     * - no node type specific handling
     * - uses the internal attribute _nsMap for resolving namespaces that is updated when changing attributes
     *
     * @param {string | null} namespaceURI
     * The namespace URI for which to find the associated prefix.
     * @returns {string | null}
     * The associated prefix, if found; otherwise, null.
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Node3-lookupNamespacePrefix
     * @see https://www.w3.org/TR/DOM-Level-3-Core/namespaces-algorithms.html#lookupNamespacePrefixAlgo
     * @see https://dom.spec.whatwg.org/#dom-node-lookupprefix
     * @see https://github.com/xmldom/xmldom/issues/322
     * @prettierignore
     */
    lookupPrefix: function(e) {
      for (var r = this; r; ) {
        var i = r._nsMap;
        if (i) {
          for (var c in i)
            if (s(i, c) && i[c] === e)
              return c;
        }
        r = r.nodeType == W ? r.ownerDocument : r.parentNode;
      }
      return null;
    },
    /**
     * This function is used to look up the namespace URI associated with the given prefix,
     * starting from this node.
     *
     * **This behavior is different from the in the specs**:
     * - no node type specific handling
     * - uses the internal attribute _nsMap for resolving namespaces that is updated when changing attributes
     *
     * @param {string | null} prefix
     * The prefix for which to find the associated namespace URI.
     * @returns {string | null}
     * The associated namespace URI, if found; otherwise, null.
     * @since DOM Level 3
     * @see https://dom.spec.whatwg.org/#dom-node-lookupnamespaceuri
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Node3-lookupNamespaceURI
     * @prettierignore
     */
    lookupNamespaceURI: function(e) {
      for (var r = this; r; ) {
        var i = r._nsMap;
        if (i && s(i, e))
          return i[e];
        r = r.nodeType == W ? r.ownerDocument : r.parentNode;
      }
      return null;
    },
    /**
     * Determines whether the given namespace URI is the default namespace.
     *
     * The function works by looking up the prefix associated with the given namespace URI. If no
     * prefix is found (i.e., the namespace URI is not registered in the namespace map of this
     * node or any of its ancestors), it returns `true`, implying the namespace URI is considered
     * the default.
     *
     * **This behavior is different from the in the specs**:
     * - no node type specific handling
     * - uses the internal attribute _nsMap for resolving namespaces that is updated when changing attributes
     *
     * @param {string | null} namespaceURI
     * The namespace URI to be checked.
     * @returns {boolean}
     * Returns true if the given namespace URI is the default namespace, false otherwise.
     * @since DOM Level 3
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#Node3-isDefaultNamespace
     * @see https://dom.spec.whatwg.org/#dom-node-isdefaultnamespace
     * @prettierignore
     */
    isDefaultNamespace: function(e) {
      var r = this.lookupPrefix(e);
      return r == null;
    },
    /**
     * Compares the reference node with a node with regard to their position in the document and
     * according to the document order.
     *
     * @param {Node} other
     * The node to compare the reference node to.
     * @returns {number}
     * Returns how the node is positioned relatively to the reference node according to the
     * bitmask. 0 if reference node and given node are the same.
     * @since DOM Level 3
     * @see https://www.w3.org/TR/2004/REC-DOM-Level-3-Core-20040407/core.html#Node3-compare
     * @see https://dom.spec.whatwg.org/#dom-node-comparedocumentposition
     */
    compareDocumentPosition: function(e) {
      if (this === e) return 0;
      var r = e, i = this, c = null, E = null;
      if (r instanceof Ae && (c = r, r = c.ownerElement), i instanceof Ae && (E = i, i = E.ownerElement, c && r && i === r))
        for (var _ = 0, Z; Z = i.attributes[_]; _++) {
          if (Z === c)
            return R.DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC + R.DOCUMENT_POSITION_PRECEDING;
          if (Z === E)
            return R.DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC + R.DOCUMENT_POSITION_FOLLOWING;
        }
      if (!r || !i || i.ownerDocument !== r.ownerDocument)
        return R.DOCUMENT_POSITION_DISCONNECTED + R.DOCUMENT_POSITION_IMPLEMENTATION_SPECIFIC + (B(i.ownerDocument) > B(r.ownerDocument) ? R.DOCUMENT_POSITION_FOLLOWING : R.DOCUMENT_POSITION_PRECEDING);
      if (E && r === i)
        return R.DOCUMENT_POSITION_CONTAINS + R.DOCUMENT_POSITION_PRECEDING;
      if (c && r === i)
        return R.DOCUMENT_POSITION_CONTAINED_BY + R.DOCUMENT_POSITION_FOLLOWING;
      for (var ce = [], de = r.parentNode; de; ) {
        if (!E && de === i)
          return R.DOCUMENT_POSITION_CONTAINED_BY + R.DOCUMENT_POSITION_FOLLOWING;
        ce.push(de), de = de.parentNode;
      }
      ce.reverse();
      for (var Ce = [], De = i.parentNode; De; ) {
        if (!c && De === r)
          return R.DOCUMENT_POSITION_CONTAINS + R.DOCUMENT_POSITION_PRECEDING;
        Ce.push(De), De = De.parentNode;
      }
      Ce.reverse();
      var je = X(ce, Ce);
      for (var Be in je.childNodes) {
        var Ne = je.childNodes[Be];
        if (Ne === i) return R.DOCUMENT_POSITION_FOLLOWING;
        if (Ne === r) return R.DOCUMENT_POSITION_PRECEDING;
        if (Ce.indexOf(Ne) >= 0) return R.DOCUMENT_POSITION_FOLLOWING;
        if (ce.indexOf(Ne) >= 0) return R.DOCUMENT_POSITION_PRECEDING;
      }
      return 0;
    }
  };
  function Le(e) {
    return e == "<" && "&lt;" || e == ">" && "&gt;" || e == "&" && "&amp;" || e == '"' && "&quot;" || "&#" + e.charCodeAt() + ";";
  }
  m(P, F), m(P, F.prototype), m(R, F), m(R, F.prototype);
  function we(e, r) {
    if (r(e))
      return !0;
    if (e = e.firstChild)
      do
        if (we(e, r))
          return !0;
      while (e = e.nextSibling);
  }
  function fe(e, r) {
    k(e);
    var i = r || {};
    this.ownerDocument = this, this.contentType = i.contentType || g.XML_APPLICATION, this.type = a(this.contentType) ? "html" : "xml";
  }
  function Fe(e, r, i) {
    e && e._inc++;
    var c = i.namespaceURI;
    c === b.XMLNS && (r._nsMap[i.prefix ? i.localName : ""] = i.value);
  }
  function _e(e, r, i, c) {
    e && e._inc++;
    var E = i.namespaceURI;
    E === b.XMLNS && delete r._nsMap[i.prefix ? i.localName : ""];
  }
  function ye(e, r, i) {
    if (e && e._inc) {
      e._inc++;
      var c = r.childNodes;
      if (i && !i.nextSibling)
        c[c.length++] = i;
      else {
        for (var E = r.firstChild, _ = 0; E; )
          c[_++] = E, E = E.nextSibling;
        c.length = _, delete c[c.length];
      }
    }
  }
  function xe(e, r) {
    if (e !== r.parentNode)
      throw new f(f.NOT_FOUND_ERR, "child's parent is not parent");
    var i = r.previousSibling, c = r.nextSibling;
    return i ? i.nextSibling = c : e.firstChild = c, c ? c.previousSibling = i : e.lastChild = i, ye(e.ownerDocument, e), r.parentNode = null, r.previousSibling = null, r.nextSibling = null, r;
  }
  function ke(e) {
    return e && (e.nodeType === F.DOCUMENT_NODE || e.nodeType === F.DOCUMENT_FRAGMENT_NODE || e.nodeType === F.ELEMENT_NODE);
  }
  function qe(e) {
    return e && (e.nodeType === F.CDATA_SECTION_NODE || e.nodeType === F.COMMENT_NODE || e.nodeType === F.DOCUMENT_FRAGMENT_NODE || e.nodeType === F.DOCUMENT_TYPE_NODE || e.nodeType === F.ELEMENT_NODE || e.nodeType === F.PROCESSING_INSTRUCTION_NODE || e.nodeType === F.TEXT_NODE);
  }
  function Te(e) {
    return e && e.nodeType === F.DOCUMENT_TYPE_NODE;
  }
  function K(e) {
    return e && e.nodeType === F.ELEMENT_NODE;
  }
  function Pe(e) {
    return e && e.nodeType === F.TEXT_NODE;
  }
  function oe(e, r) {
    var i = e.childNodes || [];
    if (t(i, K) || Te(r))
      return !1;
    var c = t(i, Te);
    return !(r && c && i.indexOf(c) > i.indexOf(r));
  }
  function Ge(e, r) {
    var i = e.childNodes || [];
    function c(_) {
      return K(_) && _ !== r;
    }
    if (t(i, c))
      return !1;
    var E = t(i, Te);
    return !(r && E && i.indexOf(E) > i.indexOf(r));
  }
  function st(e, r, i) {
    if (!ke(e))
      throw new f(f.HIERARCHY_REQUEST_ERR, "Unexpected parent node type " + e.nodeType);
    if (i && i.parentNode !== e)
      throw new f(f.NOT_FOUND_ERR, "child not in parent");
    if (
      // 4. If `node` is not a DocumentFragment, DocumentType, Element, or CharacterData node, then throw a "HierarchyRequestError" DOMException.
      !qe(r) || // 5. If either `node` is a Text node and `parent` is a document,
      // the sax parser currently adds top level text nodes, this will be fixed in 0.9.0
      // || (node.nodeType === Node.TEXT_NODE && parent.nodeType === Node.DOCUMENT_NODE)
      // or `node` is a doctype and `parent` is not a document, then throw a "HierarchyRequestError" DOMException.
      Te(r) && e.nodeType !== F.DOCUMENT_NODE
    )
      throw new f(
        f.HIERARCHY_REQUEST_ERR,
        "Unexpected node type " + r.nodeType + " for parent node type " + e.nodeType
      );
  }
  function Ve(e, r, i) {
    var c = e.childNodes || [], E = r.childNodes || [];
    if (r.nodeType === F.DOCUMENT_FRAGMENT_NODE) {
      var _ = E.filter(K);
      if (_.length > 1 || t(E, Pe))
        throw new f(f.HIERARCHY_REQUEST_ERR, "More than one element or text in fragment");
      if (_.length === 1 && !oe(e, i))
        throw new f(f.HIERARCHY_REQUEST_ERR, "Element in fragment can not be inserted before doctype");
    }
    if (K(r) && !oe(e, i))
      throw new f(f.HIERARCHY_REQUEST_ERR, "Only one element can be added and only after doctype");
    if (Te(r)) {
      if (t(c, Te))
        throw new f(f.HIERARCHY_REQUEST_ERR, "Only one doctype is allowed");
      var Z = t(c, K);
      if (i && c.indexOf(Z) < c.indexOf(i))
        throw new f(f.HIERARCHY_REQUEST_ERR, "Doctype can only be inserted before an element");
      if (!i && Z)
        throw new f(f.HIERARCHY_REQUEST_ERR, "Doctype can not be appended since element is present");
    }
  }
  function ze(e, r, i) {
    var c = e.childNodes || [], E = r.childNodes || [];
    if (r.nodeType === F.DOCUMENT_FRAGMENT_NODE) {
      var _ = E.filter(K);
      if (_.length > 1 || t(E, Pe))
        throw new f(f.HIERARCHY_REQUEST_ERR, "More than one element or text in fragment");
      if (_.length === 1 && !Ge(e, i))
        throw new f(f.HIERARCHY_REQUEST_ERR, "Element in fragment can not be inserted before doctype");
    }
    if (K(r) && !Ge(e, i))
      throw new f(f.HIERARCHY_REQUEST_ERR, "Only one element can be added and only after doctype");
    if (Te(r)) {
      if (t(c, function(de) {
        return Te(de) && de !== i;
      }))
        throw new f(f.HIERARCHY_REQUEST_ERR, "Only one doctype is allowed");
      var Z = t(c, K);
      if (i && c.indexOf(Z) < c.indexOf(i))
        throw new f(f.HIERARCHY_REQUEST_ERR, "Doctype can only be inserted before an element");
    }
  }
  function V(e, r, i, c) {
    st(e, r, i), e.nodeType === F.DOCUMENT_NODE && (c || Ve)(e, r, i);
    var E = r.parentNode;
    if (E && E.removeChild(r), r.nodeType === I) {
      var _ = r.firstChild;
      if (_ == null)
        return r;
      var Z = r.lastChild;
    } else
      _ = Z = r;
    var ce = i ? i.previousSibling : e.lastChild;
    _.previousSibling = ce, Z.nextSibling = i, ce ? ce.nextSibling = _ : e.firstChild = _, i == null ? e.lastChild = Z : i.previousSibling = Z;
    do
      _.parentNode = e;
    while (_ !== Z && (_ = _.nextSibling));
    return ye(e.ownerDocument || e, e, r), r.nodeType == I && (r.firstChild = r.lastChild = null), r;
  }
  fe.prototype = {
    /**
     * The implementation that created this document.
     *
     * @type DOMImplementation
     * @readonly
     */
    implementation: null,
    nodeName: "#document",
    nodeType: d,
    /**
     * The DocumentType node of the document.
     *
     * @type DocumentType
     * @readonly
     */
    doctype: null,
    documentElement: null,
    _inc: 1,
    insertBefore: function(e, r) {
      if (e.nodeType === I) {
        for (var i = e.firstChild; i; ) {
          var c = i.nextSibling;
          this.insertBefore(i, r), i = c;
        }
        return e;
      }
      return V(this, e, r), e.ownerDocument = this, this.documentElement === null && e.nodeType === G && (this.documentElement = e), e;
    },
    removeChild: function(e) {
      var r = xe(this, e);
      return r === this.documentElement && (this.documentElement = null), r;
    },
    replaceChild: function(e, r) {
      V(this, e, r, ze), e.ownerDocument = this, r && this.removeChild(r), K(e) && (this.documentElement = e);
    },
    // Introduced in DOM Level 2:
    importNode: function(e, r) {
      return Nt(this, e, r);
    },
    // Introduced in DOM Level 2:
    getElementById: function(e) {
      var r = null;
      return we(this.documentElement, function(i) {
        if (i.nodeType == G && i.getAttribute("id") == e)
          return r = i, !0;
      }), r;
    },
    /**
     * Creates a new `Element` that is owned by this `Document`.
     * In HTML Documents `localName` is the lower cased `tagName`,
     * otherwise no transformation is being applied.
     * When `contentType` implies the HTML namespace, it will be set as `namespaceURI`.
     *
     * __This implementation differs from the specification:__ - The provided name is not checked
     * against the `Name` production,
     * so no related error will be thrown.
     * - There is no interface `HTMLElement`, it is always an `Element`.
     * - There is no support for a second argument to indicate using custom elements.
     *
     * @param {string} tagName
     * @returns {Element}
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Document/createElement
     * @see https://dom.spec.whatwg.org/#dom-document-createelement
     * @see https://dom.spec.whatwg.org/#concept-create-element
     */
    createElement: function(e) {
      var r = new ee(O);
      r.ownerDocument = this, this.type === "html" && (e = e.toLowerCase()), u(this.contentType) && (r.namespaceURI = b.HTML), r.nodeName = e, r.tagName = e, r.localName = e, r.childNodes = new y();
      var i = r.attributes = new Q();
      return i._ownerElement = r, r;
    },
    /**
     * @returns {DocumentFragment}
     */
    createDocumentFragment: function() {
      var e = new Ye(O);
      return e.ownerDocument = this, e.childNodes = new y(), e;
    },
    /**
     * @param {string} data
     * @returns {Text}
     */
    createTextNode: function(e) {
      var r = new He(O);
      return r.ownerDocument = this, r.childNodes = new y(), r.appendData(e), r;
    },
    /**
     * @param {string} data
     * @returns {Comment}
     */
    createComment: function(e) {
      var r = new Je(O);
      return r.ownerDocument = this, r.childNodes = new y(), r.appendData(e), r;
    },
    /**
     * @param {string} data
     * @returns {CDATASection}
     */
    createCDATASection: function(e) {
      var r = new Ze(O);
      return r.ownerDocument = this, r.childNodes = new y(), r.appendData(e), r;
    },
    /**
     * @param {string} target
     * @param {string} data
     * @returns {ProcessingInstruction}
     */
    createProcessingInstruction: function(e, r) {
      var i = new et(O);
      return i.ownerDocument = this, i.childNodes = new y(), i.nodeName = i.target = e, i.nodeValue = i.data = r, i;
    },
    /**
     * Creates an `Attr` node that is owned by this document.
     * In HTML Documents `localName` is the lower cased `name`,
     * otherwise no transformation is being applied.
     *
     * __This implementation differs from the specification:__ - The provided name is not checked
     * against the `Name` production,
     * so no related error will be thrown.
     *
     * @param {string} name
     * @returns {Attr}
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Document/createAttribute
     * @see https://dom.spec.whatwg.org/#dom-document-createattribute
     */
    createAttribute: function(e) {
      if (!S.QName_exact.test(e))
        throw new f(f.INVALID_CHARACTER_ERR, 'invalid character in name "' + e + '"');
      return this.type === "html" && (e = e.toLowerCase()), this._createAttribute(e);
    },
    _createAttribute: function(e) {
      var r = new Ae(O);
      return r.ownerDocument = this, r.childNodes = new y(), r.name = e, r.nodeName = e, r.localName = e, r.specified = !0, r;
    },
    /**
     * Creates an EntityReference object.
     * The current implementation does not fill the `childNodes` with those of the corresponding
     * `Entity`
     *
     * @deprecated
     * In DOM Level 4.
     * @param {string} name
     * The name of the entity to reference. No namespace well-formedness checks are performed.
     * @returns {EntityReference}
     * @throws {DOMException}
     * With code `INVALID_CHARACTER_ERR` when `name` is not valid.
     * @throws {DOMException}
     * with code `NOT_SUPPORTED_ERR` when the document is of type `html`
     * @see https://www.w3.org/TR/DOM-Level-3-Core/core.html#ID-392B75AE
     */
    createEntityReference: function(e) {
      if (!S.Name.test(e))
        throw new f(f.INVALID_CHARACTER_ERR, 'not a valid xml name "' + e + '"');
      if (this.type === "html")
        throw new f("document is an html document", M.NotSupportedError);
      var r = new Ke(O);
      return r.ownerDocument = this, r.childNodes = new y(), r.nodeName = e, r;
    },
    // Introduced in DOM Level 2:
    /**
     * @param {string} namespaceURI
     * @param {string} qualifiedName
     * @returns {Element}
     */
    createElementNS: function(e, r) {
      var i = ie(e, r), c = new ee(O), E = c.attributes = new Q();
      return c.childNodes = new y(), c.ownerDocument = this, c.nodeName = r, c.tagName = r, c.namespaceURI = i[0], c.prefix = i[1], c.localName = i[2], E._ownerElement = c, c;
    },
    // Introduced in DOM Level 2:
    /**
     * @param {string} namespaceURI
     * @param {string} qualifiedName
     * @returns {Attr}
     */
    createAttributeNS: function(e, r) {
      var i = ie(e, r), c = new Ae(O);
      return c.ownerDocument = this, c.childNodes = new y(), c.nodeName = r, c.name = r, c.specified = !0, c.namespaceURI = i[0], c.prefix = i[1], c.localName = i[2], c;
    }
  }, N(fe, F);
  function ee(e) {
    k(e), this._nsMap = /* @__PURE__ */ Object.create(null);
  }
  ee.prototype = {
    nodeType: G,
    /**
     * The attributes of this element.
     *
     * @type {NamedNodeMap | null}
     */
    attributes: null,
    getQualifiedName: function() {
      return this.prefix ? this.prefix + ":" + this.localName : this.localName;
    },
    _isInHTMLDocumentAndNamespace: function() {
      return this.ownerDocument.type === "html" && this.namespaceURI === b.HTML;
    },
    /**
     * Implementaton of Level2 Core function hasAttributes.
     *
     * @returns {boolean}
     * True if attribute list is not empty.
     * @see https://www.w3.org/TR/DOM-Level-2-Core/#core-ID-NodeHasAttrs
     */
    hasAttributes: function() {
      return !!(this.attributes && this.attributes.length);
    },
    hasAttribute: function(e) {
      return !!this.getAttributeNode(e);
    },
    /**
     * Returns element’s first attribute whose qualified name is `name`, and `null`
     * if there is no such attribute.
     *
     * @param {string} name
     * @returns {string | null}
     */
    getAttribute: function(e) {
      var r = this.getAttributeNode(e);
      return r ? r.value : null;
    },
    getAttributeNode: function(e) {
      return this._isInHTMLDocumentAndNamespace() && (e = e.toLowerCase()), this.attributes.getNamedItem(e);
    },
    /**
     * Sets the value of element’s first attribute whose qualified name is qualifiedName to value.
     *
     * @param {string} name
     * @param {string} value
     */
    setAttribute: function(e, r) {
      this._isInHTMLDocumentAndNamespace() && (e = e.toLowerCase());
      var i = this.getAttributeNode(e);
      i ? i.value = i.nodeValue = "" + r : (i = this.ownerDocument._createAttribute(e), i.value = i.nodeValue = "" + r, this.setAttributeNode(i));
    },
    removeAttribute: function(e) {
      var r = this.getAttributeNode(e);
      r && this.removeAttributeNode(r);
    },
    setAttributeNode: function(e) {
      return this.attributes.setNamedItem(e);
    },
    setAttributeNodeNS: function(e) {
      return this.attributes.setNamedItemNS(e);
    },
    removeAttributeNode: function(e) {
      return this.attributes.removeNamedItem(e.nodeName);
    },
    //get real attribute name,and remove it by removeAttributeNode
    removeAttributeNS: function(e, r) {
      var i = this.getAttributeNodeNS(e, r);
      i && this.removeAttributeNode(i);
    },
    hasAttributeNS: function(e, r) {
      return this.getAttributeNodeNS(e, r) != null;
    },
    /**
     * Returns element’s attribute whose namespace is `namespaceURI` and local name is
     * `localName`,
     * or `null` if there is no such attribute.
     *
     * @param {string} namespaceURI
     * @param {string} localName
     * @returns {string | null}
     */
    getAttributeNS: function(e, r) {
      var i = this.getAttributeNodeNS(e, r);
      return i ? i.value : null;
    },
    /**
     * Sets the value of element’s attribute whose namespace is `namespaceURI` and local name is
     * `localName` to value.
     *
     * @param {string} namespaceURI
     * @param {string} qualifiedName
     * @param {string} value
     * @see https://dom.spec.whatwg.org/#dom-element-setattributens
     */
    setAttributeNS: function(e, r, i) {
      var c = ie(e, r), E = c[2], _ = this.getAttributeNodeNS(e, E);
      _ ? _.value = _.nodeValue = "" + i : (_ = this.ownerDocument.createAttributeNS(e, r), _.value = _.nodeValue = "" + i, this.setAttributeNode(_));
    },
    getAttributeNodeNS: function(e, r) {
      return this.attributes.getNamedItemNS(e, r);
    },
    /**
     * Returns a LiveNodeList of all child elements which have **all** of the given class name(s).
     *
     * Returns an empty list if `classNames` is an empty string or only contains HTML white space
     * characters.
     *
     * Warning: This returns a live LiveNodeList.
     * Changes in the DOM will reflect in the array as the changes occur.
     * If an element selected by this array no longer qualifies for the selector,
     * it will automatically be removed. Be aware of this for iteration purposes.
     *
     * @param {string} classNames
     * Is a string representing the class name(s) to match; multiple class names are separated by
     * (ASCII-)whitespace.
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Element/getElementsByClassName
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Document/getElementsByClassName
     * @see https://dom.spec.whatwg.org/#concept-getelementsbyclassname
     */
    getElementsByClassName: function(e) {
      var r = H(e);
      return new w(this, function(i) {
        var c = [];
        return r.length > 0 && we(i, function(E) {
          if (E !== i && E.nodeType === G) {
            var _ = E.getAttribute("class");
            if (_) {
              var Z = e === _;
              if (!Z) {
                var ce = H(_);
                Z = r.every(j(ce));
              }
              Z && c.push(E);
            }
          }
        }), c;
      });
    },
    /**
     * Returns a LiveNodeList of elements with the given qualifiedName.
     * Searching for all descendants can be done by passing `*` as `qualifiedName`.
     *
     * All descendants of the specified element are searched, but not the element itself.
     * The returned list is live, which means it updates itself with the DOM tree automatically.
     * Therefore, there is no need to call `Element.getElementsByTagName()`
     * with the same element and arguments repeatedly if the DOM changes in between calls.
     *
     * When called on an HTML element in an HTML document,
     * `getElementsByTagName` lower-cases the argument before searching for it.
     * This is undesirable when trying to match camel-cased SVG elements (such as
     * `<linearGradient>`) in an HTML document.
     * Instead, use `Element.getElementsByTagNameNS()`,
     * which preserves the capitalization of the tag name.
     *
     * `Element.getElementsByTagName` is similar to `Document.getElementsByTagName()`,
     * except that it only searches for elements that are descendants of the specified element.
     *
     * @param {string} qualifiedName
     * @returns {LiveNodeList}
     * @see https://developer.mozilla.org/en-US/docs/Web/API/Element/getElementsByTagName
     * @see https://dom.spec.whatwg.org/#concept-getelementsbytagname
     */
    getElementsByTagName: function(e) {
      var r = (this.nodeType === d ? this : this.ownerDocument).type === "html", i = e.toLowerCase();
      return new w(this, function(c) {
        var E = [];
        return we(c, function(_) {
          if (!(_ === c || _.nodeType !== G))
            if (e === "*")
              E.push(_);
            else {
              var Z = _.getQualifiedName(), ce = r && _.namespaceURI === b.HTML ? i : e;
              Z === ce && E.push(_);
            }
        }), E;
      });
    },
    getElementsByTagNameNS: function(e, r) {
      return new w(this, function(i) {
        var c = [];
        return we(i, function(E) {
          E !== i && E.nodeType === G && (e === "*" || E.namespaceURI === e) && (r === "*" || E.localName == r) && c.push(E);
        }), c;
      });
    }
  }, fe.prototype.getElementsByClassName = ee.prototype.getElementsByClassName, fe.prototype.getElementsByTagName = ee.prototype.getElementsByTagName, fe.prototype.getElementsByTagNameNS = ee.prototype.getElementsByTagNameNS, N(ee, F);
  function Ae(e) {
    k(e), this.namespaceURI = null, this.prefix = null, this.ownerElement = null;
  }
  Ae.prototype.nodeType = W, N(Ae, F);
  function ge(e) {
    k(e);
  }
  ge.prototype = {
    data: "",
    substringData: function(e, r) {
      return this.data.substring(e, e + r);
    },
    appendData: function(e) {
      e = this.data + e, this.nodeValue = this.data = e, this.length = e.length;
    },
    insertData: function(e, r) {
      this.replaceData(e, 0, r);
    },
    deleteData: function(e, r) {
      this.replaceData(e, r, "");
    },
    replaceData: function(e, r, i) {
      var c = this.data.substring(0, e), E = this.data.substring(e + r);
      i = c + i + E, this.nodeValue = this.data = i, this.length = i.length;
    }
  }, N(ge, F);
  function He(e) {
    k(e);
  }
  He.prototype = {
    nodeName: "#text",
    nodeType: v,
    splitText: function(e) {
      var r = this.data, i = r.substring(e);
      r = r.substring(0, e), this.data = this.nodeValue = r, this.length = r.length;
      var c = this.ownerDocument.createTextNode(i);
      return this.parentNode && this.parentNode.insertBefore(c, this.nextSibling), c;
    }
  }, N(He, ge);
  function Je(e) {
    k(e);
  }
  Je.prototype = {
    nodeName: "#comment",
    nodeType: p
  }, N(Je, ge);
  function Ze(e) {
    k(e);
  }
  Ze.prototype = {
    nodeName: "#cdata-section",
    nodeType: U
  }, N(Ze, He);
  function $e(e) {
    k(e);
  }
  $e.prototype.nodeType = T, N($e, F);
  function at(e) {
    k(e);
  }
  at.prototype.nodeType = D, N(at, F);
  function ot(e) {
    k(e);
  }
  ot.prototype.nodeType = L, N(ot, F);
  function Ke(e) {
    k(e);
  }
  Ke.prototype.nodeType = C, N(Ke, F);
  function Ye(e) {
    k(e);
  }
  Ye.prototype.nodeName = "#document-fragment", Ye.prototype.nodeType = I, N(Ye, F);
  function et(e) {
    k(e);
  }
  et.prototype.nodeType = h, N(et, ge);
  function ft() {
  }
  ft.prototype.serializeToString = function(e, r) {
    return ne.call(e, r);
  }, F.prototype.toString = ne;
  function ne(e) {
    var r = [], i = this.nodeType === d && this.documentElement || this, c = i.prefix, E = i.namespaceURI;
    if (E && c == null) {
      var c = i.lookupPrefix(E);
      if (c == null)
        var _ = [
          { namespace: E, prefix: null }
          //{namespace:uri,prefix:''}
        ];
    }
    return Re(this, r, e, _), r.join("");
  }
  function pe(e, r, i) {
    var c = e.prefix || "", E = e.namespaceURI;
    if (!E || c === "xml" && E === b.XML || E === b.XMLNS)
      return !1;
    for (var _ = i.length; _--; ) {
      var Z = i[_];
      if (Z.prefix === c)
        return Z.namespace !== E;
    }
    return !0;
  }
  function Ie(e, r, i) {
    e.push(" ", r, '="', i.replace(/[<>&"\t\n\r]/g, Le), '"');
  }
  function Re(e, r, i, c) {
    c || (c = []);
    var E = e.nodeType === d ? e : e.ownerDocument, _ = E.type === "html";
    if (i)
      if (e = i(e), e) {
        if (typeof e == "string") {
          r.push(e);
          return;
        }
      } else
        return;
    switch (e.nodeType) {
      case G:
        var Z = e.attributes, ce = Z.length, Ee = e.firstChild, de = e.tagName, Ce = de;
        if (!_ && !e.prefix && e.namespaceURI) {
          for (var De, je = 0; je < Z.length; je++)
            if (Z.item(je).name === "xmlns") {
              De = Z.item(je).value;
              break;
            }
          if (!De)
            for (var Be = c.length - 1; Be >= 0; Be--) {
              var Ne = c[Be];
              if (Ne.prefix === "" && Ne.namespace === e.namespaceURI) {
                De = Ne.namespace;
                break;
              }
            }
          if (De !== e.namespaceURI)
            for (var Be = c.length - 1; Be >= 0; Be--) {
              var Ne = c[Be];
              if (Ne.namespace === e.namespaceURI) {
                Ne.prefix && (Ce = Ne.prefix + ":" + de);
                break;
              }
            }
        }
        r.push("<", Ce);
        for (var Xe = 0; Xe < ce; Xe++) {
          var Se = Z.item(Xe);
          Se.prefix == "xmlns" ? c.push({
            prefix: Se.localName,
            namespace: Se.value
          }) : Se.nodeName == "xmlns" && c.push({ prefix: "", namespace: Se.value });
        }
        for (var Xe = 0; Xe < ce; Xe++) {
          var Se = Z.item(Xe);
          if (pe(Se, _, c)) {
            var Qe = Se.prefix || "", ct = Se.namespaceURI;
            Ie(r, Qe ? "xmlns:" + Qe : "xmlns", ct), c.push({ prefix: Qe, namespace: ct });
          }
          Re(Se, r, i, c);
        }
        if (de === Ce && pe(e, _, c)) {
          var Qe = e.prefix || "", ct = e.namespaceURI;
          Ie(r, Qe ? "xmlns:" + Qe : "xmlns", ct), c.push({ prefix: Qe, namespace: ct });
        }
        var Dt = !Ee;
        if (Dt && (_ || e.namespaceURI === b.HTML) && (Dt = l(de)), Dt)
          r.push("/>");
        else {
          if (r.push(">"), _ && n(de))
            for (; Ee; )
              Ee.data ? r.push(Ee.data) : Re(Ee, r, i, c.slice()), Ee = Ee.nextSibling;
          else
            for (; Ee; )
              Re(Ee, r, i, c.slice()), Ee = Ee.nextSibling;
          r.push("</", Ce, ">");
        }
        return;
      case d:
      case I:
        for (var Ee = e.firstChild; Ee; )
          Re(Ee, r, i, c.slice()), Ee = Ee.nextSibling;
        return;
      case W:
        return Ie(r, e.name, e.value);
      case v:
        return r.push(e.data.replace(/[<&>]/g, Le));
      case U:
        return r.push(S.CDATA_START, e.data, S.CDATA_END);
      case p:
        return r.push(S.COMMENT_START, e.data, S.COMMENT_END);
      case T:
        var _t = e.publicId, tt = e.systemId;
        r.push(S.DOCTYPE_DECL_START, " ", e.name), _t ? (r.push(" ", S.PUBLIC, " ", _t), tt && tt !== "." && r.push(" ", tt)) : tt && tt !== "." && r.push(" ", S.SYSTEM, " ", tt), e.internalSubset && r.push(" [", e.internalSubset, "]"), r.push(">");
        return;
      case h:
        return r.push("<?", e.target, " ", e.data, "?>");
      case C:
        return r.push("&", e.nodeName, ";");
      //case ENTITY_NODE:
      //case NOTATION_NODE:
      default:
        r.push("??", e.nodeName);
    }
  }
  function Nt(e, r, i) {
    var c;
    switch (r.nodeType) {
      case G:
        c = r.cloneNode(!1), c.ownerDocument = e;
      //var attrs = node2.attributes;
      //var len = attrs.length;
      //for(var i=0;i<len;i++){
      //node2.setAttributeNodeNS(importNode(doc,attrs.item(i),deep));
      //}
      case I:
        break;
      case W:
        i = !0;
        break;
    }
    if (c || (c = r.cloneNode(!1)), c.ownerDocument = e, c.parentNode = null, i)
      for (var E = r.firstChild; E; )
        c.appendChild(Nt(e, E, i)), E = E.nextSibling;
    return c;
  }
  function Et(e, r, i) {
    var c = new r.constructor(O);
    for (var E in r)
      if (s(r, E)) {
        var _ = r[E];
        typeof _ != "object" && _ != c[E] && (c[E] = _);
      }
    switch (r.childNodes && (c.childNodes = new y()), c.ownerDocument = e, c.nodeType) {
      case G:
        var Z = r.attributes, ce = c.attributes = new Q(), de = Z.length;
        ce._ownerElement = c;
        for (var Ce = 0; Ce < de; Ce++)
          c.setAttributeNode(Et(e, Z.item(Ce), !0));
        break;
      case W:
        i = !0;
    }
    if (i)
      for (var De = r.firstChild; De; )
        c.appendChild(Et(e, De, i)), De = De.nextSibling;
    return c;
  }
  function wt(e, r, i) {
    e[r] = i;
  }
  try {
    if (Object.defineProperty) {
      let e = function(r) {
        switch (r.nodeType) {
          case G:
          case I:
            var i = [];
            for (r = r.firstChild; r; )
              r.nodeType !== 7 && r.nodeType !== 8 && i.push(e(r)), r = r.nextSibling;
            return i.join("");
          default:
            return r.nodeValue;
        }
      };
      Object.defineProperty(w.prototype, "length", {
        get: function() {
          return q(this), this.$$length;
        }
      }), Object.defineProperty(F.prototype, "textContent", {
        get: function() {
          return e(this);
        },
        set: function(r) {
          switch (this.nodeType) {
            case G:
            case I:
              for (; this.firstChild; )
                this.removeChild(this.firstChild);
              (r || String(r)) && this.appendChild(this.ownerDocument.createTextNode(r));
              break;
            default:
              this.data = r, this.value = r, this.nodeValue = r;
          }
        }
      }), wt = function(r, i, c) {
        r["$$" + i] = c;
      };
    }
  } catch {
  }
  return ue._updateLiveList = q, ue.Attr = Ae, ue.CDATASection = Ze, ue.CharacterData = ge, ue.Comment = Je, ue.Document = fe, ue.DocumentFragment = Ye, ue.DocumentType = $e, ue.DOMImplementation = ae, ue.Element = ee, ue.Entity = ot, ue.EntityReference = Ke, ue.LiveNodeList = w, ue.NamedNodeMap = Q, ue.Node = F, ue.NodeList = y, ue.Notation = at, ue.Text = He, ue.ProcessingInstruction = et, ue.XMLSerializer = ft, ue;
}
var We = {}, At = {}, Ft;
function gr() {
  return Ft || (Ft = 1, function(o) {
    var t = nt().freeze;
    o.XML_ENTITIES = t({
      amp: "&",
      apos: "'",
      gt: ">",
      lt: "<",
      quot: '"'
    }), o.HTML_ENTITIES = t({
      Aacute: "Á",
      aacute: "á",
      Abreve: "Ă",
      abreve: "ă",
      ac: "∾",
      acd: "∿",
      acE: "∾̳",
      Acirc: "Â",
      acirc: "â",
      acute: "´",
      Acy: "А",
      acy: "а",
      AElig: "Æ",
      aelig: "æ",
      af: "⁡",
      Afr: "𝔄",
      afr: "𝔞",
      Agrave: "À",
      agrave: "à",
      alefsym: "ℵ",
      aleph: "ℵ",
      Alpha: "Α",
      alpha: "α",
      Amacr: "Ā",
      amacr: "ā",
      amalg: "⨿",
      AMP: "&",
      amp: "&",
      And: "⩓",
      and: "∧",
      andand: "⩕",
      andd: "⩜",
      andslope: "⩘",
      andv: "⩚",
      ang: "∠",
      ange: "⦤",
      angle: "∠",
      angmsd: "∡",
      angmsdaa: "⦨",
      angmsdab: "⦩",
      angmsdac: "⦪",
      angmsdad: "⦫",
      angmsdae: "⦬",
      angmsdaf: "⦭",
      angmsdag: "⦮",
      angmsdah: "⦯",
      angrt: "∟",
      angrtvb: "⊾",
      angrtvbd: "⦝",
      angsph: "∢",
      angst: "Å",
      angzarr: "⍼",
      Aogon: "Ą",
      aogon: "ą",
      Aopf: "𝔸",
      aopf: "𝕒",
      ap: "≈",
      apacir: "⩯",
      apE: "⩰",
      ape: "≊",
      apid: "≋",
      apos: "'",
      ApplyFunction: "⁡",
      approx: "≈",
      approxeq: "≊",
      Aring: "Å",
      aring: "å",
      Ascr: "𝒜",
      ascr: "𝒶",
      Assign: "≔",
      ast: "*",
      asymp: "≈",
      asympeq: "≍",
      Atilde: "Ã",
      atilde: "ã",
      Auml: "Ä",
      auml: "ä",
      awconint: "∳",
      awint: "⨑",
      backcong: "≌",
      backepsilon: "϶",
      backprime: "‵",
      backsim: "∽",
      backsimeq: "⋍",
      Backslash: "∖",
      Barv: "⫧",
      barvee: "⊽",
      Barwed: "⌆",
      barwed: "⌅",
      barwedge: "⌅",
      bbrk: "⎵",
      bbrktbrk: "⎶",
      bcong: "≌",
      Bcy: "Б",
      bcy: "б",
      bdquo: "„",
      becaus: "∵",
      Because: "∵",
      because: "∵",
      bemptyv: "⦰",
      bepsi: "϶",
      bernou: "ℬ",
      Bernoullis: "ℬ",
      Beta: "Β",
      beta: "β",
      beth: "ℶ",
      between: "≬",
      Bfr: "𝔅",
      bfr: "𝔟",
      bigcap: "⋂",
      bigcirc: "◯",
      bigcup: "⋃",
      bigodot: "⨀",
      bigoplus: "⨁",
      bigotimes: "⨂",
      bigsqcup: "⨆",
      bigstar: "★",
      bigtriangledown: "▽",
      bigtriangleup: "△",
      biguplus: "⨄",
      bigvee: "⋁",
      bigwedge: "⋀",
      bkarow: "⤍",
      blacklozenge: "⧫",
      blacksquare: "▪",
      blacktriangle: "▴",
      blacktriangledown: "▾",
      blacktriangleleft: "◂",
      blacktriangleright: "▸",
      blank: "␣",
      blk12: "▒",
      blk14: "░",
      blk34: "▓",
      block: "█",
      bne: "=⃥",
      bnequiv: "≡⃥",
      bNot: "⫭",
      bnot: "⌐",
      Bopf: "𝔹",
      bopf: "𝕓",
      bot: "⊥",
      bottom: "⊥",
      bowtie: "⋈",
      boxbox: "⧉",
      boxDL: "╗",
      boxDl: "╖",
      boxdL: "╕",
      boxdl: "┐",
      boxDR: "╔",
      boxDr: "╓",
      boxdR: "╒",
      boxdr: "┌",
      boxH: "═",
      boxh: "─",
      boxHD: "╦",
      boxHd: "╤",
      boxhD: "╥",
      boxhd: "┬",
      boxHU: "╩",
      boxHu: "╧",
      boxhU: "╨",
      boxhu: "┴",
      boxminus: "⊟",
      boxplus: "⊞",
      boxtimes: "⊠",
      boxUL: "╝",
      boxUl: "╜",
      boxuL: "╛",
      boxul: "┘",
      boxUR: "╚",
      boxUr: "╙",
      boxuR: "╘",
      boxur: "└",
      boxV: "║",
      boxv: "│",
      boxVH: "╬",
      boxVh: "╫",
      boxvH: "╪",
      boxvh: "┼",
      boxVL: "╣",
      boxVl: "╢",
      boxvL: "╡",
      boxvl: "┤",
      boxVR: "╠",
      boxVr: "╟",
      boxvR: "╞",
      boxvr: "├",
      bprime: "‵",
      Breve: "˘",
      breve: "˘",
      brvbar: "¦",
      Bscr: "ℬ",
      bscr: "𝒷",
      bsemi: "⁏",
      bsim: "∽",
      bsime: "⋍",
      bsol: "\\",
      bsolb: "⧅",
      bsolhsub: "⟈",
      bull: "•",
      bullet: "•",
      bump: "≎",
      bumpE: "⪮",
      bumpe: "≏",
      Bumpeq: "≎",
      bumpeq: "≏",
      Cacute: "Ć",
      cacute: "ć",
      Cap: "⋒",
      cap: "∩",
      capand: "⩄",
      capbrcup: "⩉",
      capcap: "⩋",
      capcup: "⩇",
      capdot: "⩀",
      CapitalDifferentialD: "ⅅ",
      caps: "∩︀",
      caret: "⁁",
      caron: "ˇ",
      Cayleys: "ℭ",
      ccaps: "⩍",
      Ccaron: "Č",
      ccaron: "č",
      Ccedil: "Ç",
      ccedil: "ç",
      Ccirc: "Ĉ",
      ccirc: "ĉ",
      Cconint: "∰",
      ccups: "⩌",
      ccupssm: "⩐",
      Cdot: "Ċ",
      cdot: "ċ",
      cedil: "¸",
      Cedilla: "¸",
      cemptyv: "⦲",
      cent: "¢",
      CenterDot: "·",
      centerdot: "·",
      Cfr: "ℭ",
      cfr: "𝔠",
      CHcy: "Ч",
      chcy: "ч",
      check: "✓",
      checkmark: "✓",
      Chi: "Χ",
      chi: "χ",
      cir: "○",
      circ: "ˆ",
      circeq: "≗",
      circlearrowleft: "↺",
      circlearrowright: "↻",
      circledast: "⊛",
      circledcirc: "⊚",
      circleddash: "⊝",
      CircleDot: "⊙",
      circledR: "®",
      circledS: "Ⓢ",
      CircleMinus: "⊖",
      CirclePlus: "⊕",
      CircleTimes: "⊗",
      cirE: "⧃",
      cire: "≗",
      cirfnint: "⨐",
      cirmid: "⫯",
      cirscir: "⧂",
      ClockwiseContourIntegral: "∲",
      CloseCurlyDoubleQuote: "”",
      CloseCurlyQuote: "’",
      clubs: "♣",
      clubsuit: "♣",
      Colon: "∷",
      colon: ":",
      Colone: "⩴",
      colone: "≔",
      coloneq: "≔",
      comma: ",",
      commat: "@",
      comp: "∁",
      compfn: "∘",
      complement: "∁",
      complexes: "ℂ",
      cong: "≅",
      congdot: "⩭",
      Congruent: "≡",
      Conint: "∯",
      conint: "∮",
      ContourIntegral: "∮",
      Copf: "ℂ",
      copf: "𝕔",
      coprod: "∐",
      Coproduct: "∐",
      COPY: "©",
      copy: "©",
      copysr: "℗",
      CounterClockwiseContourIntegral: "∳",
      crarr: "↵",
      Cross: "⨯",
      cross: "✗",
      Cscr: "𝒞",
      cscr: "𝒸",
      csub: "⫏",
      csube: "⫑",
      csup: "⫐",
      csupe: "⫒",
      ctdot: "⋯",
      cudarrl: "⤸",
      cudarrr: "⤵",
      cuepr: "⋞",
      cuesc: "⋟",
      cularr: "↶",
      cularrp: "⤽",
      Cup: "⋓",
      cup: "∪",
      cupbrcap: "⩈",
      CupCap: "≍",
      cupcap: "⩆",
      cupcup: "⩊",
      cupdot: "⊍",
      cupor: "⩅",
      cups: "∪︀",
      curarr: "↷",
      curarrm: "⤼",
      curlyeqprec: "⋞",
      curlyeqsucc: "⋟",
      curlyvee: "⋎",
      curlywedge: "⋏",
      curren: "¤",
      curvearrowleft: "↶",
      curvearrowright: "↷",
      cuvee: "⋎",
      cuwed: "⋏",
      cwconint: "∲",
      cwint: "∱",
      cylcty: "⌭",
      Dagger: "‡",
      dagger: "†",
      daleth: "ℸ",
      Darr: "↡",
      dArr: "⇓",
      darr: "↓",
      dash: "‐",
      Dashv: "⫤",
      dashv: "⊣",
      dbkarow: "⤏",
      dblac: "˝",
      Dcaron: "Ď",
      dcaron: "ď",
      Dcy: "Д",
      dcy: "д",
      DD: "ⅅ",
      dd: "ⅆ",
      ddagger: "‡",
      ddarr: "⇊",
      DDotrahd: "⤑",
      ddotseq: "⩷",
      deg: "°",
      Del: "∇",
      Delta: "Δ",
      delta: "δ",
      demptyv: "⦱",
      dfisht: "⥿",
      Dfr: "𝔇",
      dfr: "𝔡",
      dHar: "⥥",
      dharl: "⇃",
      dharr: "⇂",
      DiacriticalAcute: "´",
      DiacriticalDot: "˙",
      DiacriticalDoubleAcute: "˝",
      DiacriticalGrave: "`",
      DiacriticalTilde: "˜",
      diam: "⋄",
      Diamond: "⋄",
      diamond: "⋄",
      diamondsuit: "♦",
      diams: "♦",
      die: "¨",
      DifferentialD: "ⅆ",
      digamma: "ϝ",
      disin: "⋲",
      div: "÷",
      divide: "÷",
      divideontimes: "⋇",
      divonx: "⋇",
      DJcy: "Ђ",
      djcy: "ђ",
      dlcorn: "⌞",
      dlcrop: "⌍",
      dollar: "$",
      Dopf: "𝔻",
      dopf: "𝕕",
      Dot: "¨",
      dot: "˙",
      DotDot: "⃜",
      doteq: "≐",
      doteqdot: "≑",
      DotEqual: "≐",
      dotminus: "∸",
      dotplus: "∔",
      dotsquare: "⊡",
      doublebarwedge: "⌆",
      DoubleContourIntegral: "∯",
      DoubleDot: "¨",
      DoubleDownArrow: "⇓",
      DoubleLeftArrow: "⇐",
      DoubleLeftRightArrow: "⇔",
      DoubleLeftTee: "⫤",
      DoubleLongLeftArrow: "⟸",
      DoubleLongLeftRightArrow: "⟺",
      DoubleLongRightArrow: "⟹",
      DoubleRightArrow: "⇒",
      DoubleRightTee: "⊨",
      DoubleUpArrow: "⇑",
      DoubleUpDownArrow: "⇕",
      DoubleVerticalBar: "∥",
      DownArrow: "↓",
      Downarrow: "⇓",
      downarrow: "↓",
      DownArrowBar: "⤓",
      DownArrowUpArrow: "⇵",
      DownBreve: "̑",
      downdownarrows: "⇊",
      downharpoonleft: "⇃",
      downharpoonright: "⇂",
      DownLeftRightVector: "⥐",
      DownLeftTeeVector: "⥞",
      DownLeftVector: "↽",
      DownLeftVectorBar: "⥖",
      DownRightTeeVector: "⥟",
      DownRightVector: "⇁",
      DownRightVectorBar: "⥗",
      DownTee: "⊤",
      DownTeeArrow: "↧",
      drbkarow: "⤐",
      drcorn: "⌟",
      drcrop: "⌌",
      Dscr: "𝒟",
      dscr: "𝒹",
      DScy: "Ѕ",
      dscy: "ѕ",
      dsol: "⧶",
      Dstrok: "Đ",
      dstrok: "đ",
      dtdot: "⋱",
      dtri: "▿",
      dtrif: "▾",
      duarr: "⇵",
      duhar: "⥯",
      dwangle: "⦦",
      DZcy: "Џ",
      dzcy: "џ",
      dzigrarr: "⟿",
      Eacute: "É",
      eacute: "é",
      easter: "⩮",
      Ecaron: "Ě",
      ecaron: "ě",
      ecir: "≖",
      Ecirc: "Ê",
      ecirc: "ê",
      ecolon: "≕",
      Ecy: "Э",
      ecy: "э",
      eDDot: "⩷",
      Edot: "Ė",
      eDot: "≑",
      edot: "ė",
      ee: "ⅇ",
      efDot: "≒",
      Efr: "𝔈",
      efr: "𝔢",
      eg: "⪚",
      Egrave: "È",
      egrave: "è",
      egs: "⪖",
      egsdot: "⪘",
      el: "⪙",
      Element: "∈",
      elinters: "⏧",
      ell: "ℓ",
      els: "⪕",
      elsdot: "⪗",
      Emacr: "Ē",
      emacr: "ē",
      empty: "∅",
      emptyset: "∅",
      EmptySmallSquare: "◻",
      emptyv: "∅",
      EmptyVerySmallSquare: "▫",
      emsp: " ",
      emsp13: " ",
      emsp14: " ",
      ENG: "Ŋ",
      eng: "ŋ",
      ensp: " ",
      Eogon: "Ę",
      eogon: "ę",
      Eopf: "𝔼",
      eopf: "𝕖",
      epar: "⋕",
      eparsl: "⧣",
      eplus: "⩱",
      epsi: "ε",
      Epsilon: "Ε",
      epsilon: "ε",
      epsiv: "ϵ",
      eqcirc: "≖",
      eqcolon: "≕",
      eqsim: "≂",
      eqslantgtr: "⪖",
      eqslantless: "⪕",
      Equal: "⩵",
      equals: "=",
      EqualTilde: "≂",
      equest: "≟",
      Equilibrium: "⇌",
      equiv: "≡",
      equivDD: "⩸",
      eqvparsl: "⧥",
      erarr: "⥱",
      erDot: "≓",
      Escr: "ℰ",
      escr: "ℯ",
      esdot: "≐",
      Esim: "⩳",
      esim: "≂",
      Eta: "Η",
      eta: "η",
      ETH: "Ð",
      eth: "ð",
      Euml: "Ë",
      euml: "ë",
      euro: "€",
      excl: "!",
      exist: "∃",
      Exists: "∃",
      expectation: "ℰ",
      ExponentialE: "ⅇ",
      exponentiale: "ⅇ",
      fallingdotseq: "≒",
      Fcy: "Ф",
      fcy: "ф",
      female: "♀",
      ffilig: "ﬃ",
      fflig: "ﬀ",
      ffllig: "ﬄ",
      Ffr: "𝔉",
      ffr: "𝔣",
      filig: "ﬁ",
      FilledSmallSquare: "◼",
      FilledVerySmallSquare: "▪",
      fjlig: "fj",
      flat: "♭",
      fllig: "ﬂ",
      fltns: "▱",
      fnof: "ƒ",
      Fopf: "𝔽",
      fopf: "𝕗",
      ForAll: "∀",
      forall: "∀",
      fork: "⋔",
      forkv: "⫙",
      Fouriertrf: "ℱ",
      fpartint: "⨍",
      frac12: "½",
      frac13: "⅓",
      frac14: "¼",
      frac15: "⅕",
      frac16: "⅙",
      frac18: "⅛",
      frac23: "⅔",
      frac25: "⅖",
      frac34: "¾",
      frac35: "⅗",
      frac38: "⅜",
      frac45: "⅘",
      frac56: "⅚",
      frac58: "⅝",
      frac78: "⅞",
      frasl: "⁄",
      frown: "⌢",
      Fscr: "ℱ",
      fscr: "𝒻",
      gacute: "ǵ",
      Gamma: "Γ",
      gamma: "γ",
      Gammad: "Ϝ",
      gammad: "ϝ",
      gap: "⪆",
      Gbreve: "Ğ",
      gbreve: "ğ",
      Gcedil: "Ģ",
      Gcirc: "Ĝ",
      gcirc: "ĝ",
      Gcy: "Г",
      gcy: "г",
      Gdot: "Ġ",
      gdot: "ġ",
      gE: "≧",
      ge: "≥",
      gEl: "⪌",
      gel: "⋛",
      geq: "≥",
      geqq: "≧",
      geqslant: "⩾",
      ges: "⩾",
      gescc: "⪩",
      gesdot: "⪀",
      gesdoto: "⪂",
      gesdotol: "⪄",
      gesl: "⋛︀",
      gesles: "⪔",
      Gfr: "𝔊",
      gfr: "𝔤",
      Gg: "⋙",
      gg: "≫",
      ggg: "⋙",
      gimel: "ℷ",
      GJcy: "Ѓ",
      gjcy: "ѓ",
      gl: "≷",
      gla: "⪥",
      glE: "⪒",
      glj: "⪤",
      gnap: "⪊",
      gnapprox: "⪊",
      gnE: "≩",
      gne: "⪈",
      gneq: "⪈",
      gneqq: "≩",
      gnsim: "⋧",
      Gopf: "𝔾",
      gopf: "𝕘",
      grave: "`",
      GreaterEqual: "≥",
      GreaterEqualLess: "⋛",
      GreaterFullEqual: "≧",
      GreaterGreater: "⪢",
      GreaterLess: "≷",
      GreaterSlantEqual: "⩾",
      GreaterTilde: "≳",
      Gscr: "𝒢",
      gscr: "ℊ",
      gsim: "≳",
      gsime: "⪎",
      gsiml: "⪐",
      Gt: "≫",
      GT: ">",
      gt: ">",
      gtcc: "⪧",
      gtcir: "⩺",
      gtdot: "⋗",
      gtlPar: "⦕",
      gtquest: "⩼",
      gtrapprox: "⪆",
      gtrarr: "⥸",
      gtrdot: "⋗",
      gtreqless: "⋛",
      gtreqqless: "⪌",
      gtrless: "≷",
      gtrsim: "≳",
      gvertneqq: "≩︀",
      gvnE: "≩︀",
      Hacek: "ˇ",
      hairsp: " ",
      half: "½",
      hamilt: "ℋ",
      HARDcy: "Ъ",
      hardcy: "ъ",
      hArr: "⇔",
      harr: "↔",
      harrcir: "⥈",
      harrw: "↭",
      Hat: "^",
      hbar: "ℏ",
      Hcirc: "Ĥ",
      hcirc: "ĥ",
      hearts: "♥",
      heartsuit: "♥",
      hellip: "…",
      hercon: "⊹",
      Hfr: "ℌ",
      hfr: "𝔥",
      HilbertSpace: "ℋ",
      hksearow: "⤥",
      hkswarow: "⤦",
      hoarr: "⇿",
      homtht: "∻",
      hookleftarrow: "↩",
      hookrightarrow: "↪",
      Hopf: "ℍ",
      hopf: "𝕙",
      horbar: "―",
      HorizontalLine: "─",
      Hscr: "ℋ",
      hscr: "𝒽",
      hslash: "ℏ",
      Hstrok: "Ħ",
      hstrok: "ħ",
      HumpDownHump: "≎",
      HumpEqual: "≏",
      hybull: "⁃",
      hyphen: "‐",
      Iacute: "Í",
      iacute: "í",
      ic: "⁣",
      Icirc: "Î",
      icirc: "î",
      Icy: "И",
      icy: "и",
      Idot: "İ",
      IEcy: "Е",
      iecy: "е",
      iexcl: "¡",
      iff: "⇔",
      Ifr: "ℑ",
      ifr: "𝔦",
      Igrave: "Ì",
      igrave: "ì",
      ii: "ⅈ",
      iiiint: "⨌",
      iiint: "∭",
      iinfin: "⧜",
      iiota: "℩",
      IJlig: "Ĳ",
      ijlig: "ĳ",
      Im: "ℑ",
      Imacr: "Ī",
      imacr: "ī",
      image: "ℑ",
      ImaginaryI: "ⅈ",
      imagline: "ℐ",
      imagpart: "ℑ",
      imath: "ı",
      imof: "⊷",
      imped: "Ƶ",
      Implies: "⇒",
      in: "∈",
      incare: "℅",
      infin: "∞",
      infintie: "⧝",
      inodot: "ı",
      Int: "∬",
      int: "∫",
      intcal: "⊺",
      integers: "ℤ",
      Integral: "∫",
      intercal: "⊺",
      Intersection: "⋂",
      intlarhk: "⨗",
      intprod: "⨼",
      InvisibleComma: "⁣",
      InvisibleTimes: "⁢",
      IOcy: "Ё",
      iocy: "ё",
      Iogon: "Į",
      iogon: "į",
      Iopf: "𝕀",
      iopf: "𝕚",
      Iota: "Ι",
      iota: "ι",
      iprod: "⨼",
      iquest: "¿",
      Iscr: "ℐ",
      iscr: "𝒾",
      isin: "∈",
      isindot: "⋵",
      isinE: "⋹",
      isins: "⋴",
      isinsv: "⋳",
      isinv: "∈",
      it: "⁢",
      Itilde: "Ĩ",
      itilde: "ĩ",
      Iukcy: "І",
      iukcy: "і",
      Iuml: "Ï",
      iuml: "ï",
      Jcirc: "Ĵ",
      jcirc: "ĵ",
      Jcy: "Й",
      jcy: "й",
      Jfr: "𝔍",
      jfr: "𝔧",
      jmath: "ȷ",
      Jopf: "𝕁",
      jopf: "𝕛",
      Jscr: "𝒥",
      jscr: "𝒿",
      Jsercy: "Ј",
      jsercy: "ј",
      Jukcy: "Є",
      jukcy: "є",
      Kappa: "Κ",
      kappa: "κ",
      kappav: "ϰ",
      Kcedil: "Ķ",
      kcedil: "ķ",
      Kcy: "К",
      kcy: "к",
      Kfr: "𝔎",
      kfr: "𝔨",
      kgreen: "ĸ",
      KHcy: "Х",
      khcy: "х",
      KJcy: "Ќ",
      kjcy: "ќ",
      Kopf: "𝕂",
      kopf: "𝕜",
      Kscr: "𝒦",
      kscr: "𝓀",
      lAarr: "⇚",
      Lacute: "Ĺ",
      lacute: "ĺ",
      laemptyv: "⦴",
      lagran: "ℒ",
      Lambda: "Λ",
      lambda: "λ",
      Lang: "⟪",
      lang: "⟨",
      langd: "⦑",
      langle: "⟨",
      lap: "⪅",
      Laplacetrf: "ℒ",
      laquo: "«",
      Larr: "↞",
      lArr: "⇐",
      larr: "←",
      larrb: "⇤",
      larrbfs: "⤟",
      larrfs: "⤝",
      larrhk: "↩",
      larrlp: "↫",
      larrpl: "⤹",
      larrsim: "⥳",
      larrtl: "↢",
      lat: "⪫",
      lAtail: "⤛",
      latail: "⤙",
      late: "⪭",
      lates: "⪭︀",
      lBarr: "⤎",
      lbarr: "⤌",
      lbbrk: "❲",
      lbrace: "{",
      lbrack: "[",
      lbrke: "⦋",
      lbrksld: "⦏",
      lbrkslu: "⦍",
      Lcaron: "Ľ",
      lcaron: "ľ",
      Lcedil: "Ļ",
      lcedil: "ļ",
      lceil: "⌈",
      lcub: "{",
      Lcy: "Л",
      lcy: "л",
      ldca: "⤶",
      ldquo: "“",
      ldquor: "„",
      ldrdhar: "⥧",
      ldrushar: "⥋",
      ldsh: "↲",
      lE: "≦",
      le: "≤",
      LeftAngleBracket: "⟨",
      LeftArrow: "←",
      Leftarrow: "⇐",
      leftarrow: "←",
      LeftArrowBar: "⇤",
      LeftArrowRightArrow: "⇆",
      leftarrowtail: "↢",
      LeftCeiling: "⌈",
      LeftDoubleBracket: "⟦",
      LeftDownTeeVector: "⥡",
      LeftDownVector: "⇃",
      LeftDownVectorBar: "⥙",
      LeftFloor: "⌊",
      leftharpoondown: "↽",
      leftharpoonup: "↼",
      leftleftarrows: "⇇",
      LeftRightArrow: "↔",
      Leftrightarrow: "⇔",
      leftrightarrow: "↔",
      leftrightarrows: "⇆",
      leftrightharpoons: "⇋",
      leftrightsquigarrow: "↭",
      LeftRightVector: "⥎",
      LeftTee: "⊣",
      LeftTeeArrow: "↤",
      LeftTeeVector: "⥚",
      leftthreetimes: "⋋",
      LeftTriangle: "⊲",
      LeftTriangleBar: "⧏",
      LeftTriangleEqual: "⊴",
      LeftUpDownVector: "⥑",
      LeftUpTeeVector: "⥠",
      LeftUpVector: "↿",
      LeftUpVectorBar: "⥘",
      LeftVector: "↼",
      LeftVectorBar: "⥒",
      lEg: "⪋",
      leg: "⋚",
      leq: "≤",
      leqq: "≦",
      leqslant: "⩽",
      les: "⩽",
      lescc: "⪨",
      lesdot: "⩿",
      lesdoto: "⪁",
      lesdotor: "⪃",
      lesg: "⋚︀",
      lesges: "⪓",
      lessapprox: "⪅",
      lessdot: "⋖",
      lesseqgtr: "⋚",
      lesseqqgtr: "⪋",
      LessEqualGreater: "⋚",
      LessFullEqual: "≦",
      LessGreater: "≶",
      lessgtr: "≶",
      LessLess: "⪡",
      lesssim: "≲",
      LessSlantEqual: "⩽",
      LessTilde: "≲",
      lfisht: "⥼",
      lfloor: "⌊",
      Lfr: "𝔏",
      lfr: "𝔩",
      lg: "≶",
      lgE: "⪑",
      lHar: "⥢",
      lhard: "↽",
      lharu: "↼",
      lharul: "⥪",
      lhblk: "▄",
      LJcy: "Љ",
      ljcy: "љ",
      Ll: "⋘",
      ll: "≪",
      llarr: "⇇",
      llcorner: "⌞",
      Lleftarrow: "⇚",
      llhard: "⥫",
      lltri: "◺",
      Lmidot: "Ŀ",
      lmidot: "ŀ",
      lmoust: "⎰",
      lmoustache: "⎰",
      lnap: "⪉",
      lnapprox: "⪉",
      lnE: "≨",
      lne: "⪇",
      lneq: "⪇",
      lneqq: "≨",
      lnsim: "⋦",
      loang: "⟬",
      loarr: "⇽",
      lobrk: "⟦",
      LongLeftArrow: "⟵",
      Longleftarrow: "⟸",
      longleftarrow: "⟵",
      LongLeftRightArrow: "⟷",
      Longleftrightarrow: "⟺",
      longleftrightarrow: "⟷",
      longmapsto: "⟼",
      LongRightArrow: "⟶",
      Longrightarrow: "⟹",
      longrightarrow: "⟶",
      looparrowleft: "↫",
      looparrowright: "↬",
      lopar: "⦅",
      Lopf: "𝕃",
      lopf: "𝕝",
      loplus: "⨭",
      lotimes: "⨴",
      lowast: "∗",
      lowbar: "_",
      LowerLeftArrow: "↙",
      LowerRightArrow: "↘",
      loz: "◊",
      lozenge: "◊",
      lozf: "⧫",
      lpar: "(",
      lparlt: "⦓",
      lrarr: "⇆",
      lrcorner: "⌟",
      lrhar: "⇋",
      lrhard: "⥭",
      lrm: "‎",
      lrtri: "⊿",
      lsaquo: "‹",
      Lscr: "ℒ",
      lscr: "𝓁",
      Lsh: "↰",
      lsh: "↰",
      lsim: "≲",
      lsime: "⪍",
      lsimg: "⪏",
      lsqb: "[",
      lsquo: "‘",
      lsquor: "‚",
      Lstrok: "Ł",
      lstrok: "ł",
      Lt: "≪",
      LT: "<",
      lt: "<",
      ltcc: "⪦",
      ltcir: "⩹",
      ltdot: "⋖",
      lthree: "⋋",
      ltimes: "⋉",
      ltlarr: "⥶",
      ltquest: "⩻",
      ltri: "◃",
      ltrie: "⊴",
      ltrif: "◂",
      ltrPar: "⦖",
      lurdshar: "⥊",
      luruhar: "⥦",
      lvertneqq: "≨︀",
      lvnE: "≨︀",
      macr: "¯",
      male: "♂",
      malt: "✠",
      maltese: "✠",
      Map: "⤅",
      map: "↦",
      mapsto: "↦",
      mapstodown: "↧",
      mapstoleft: "↤",
      mapstoup: "↥",
      marker: "▮",
      mcomma: "⨩",
      Mcy: "М",
      mcy: "м",
      mdash: "—",
      mDDot: "∺",
      measuredangle: "∡",
      MediumSpace: " ",
      Mellintrf: "ℳ",
      Mfr: "𝔐",
      mfr: "𝔪",
      mho: "℧",
      micro: "µ",
      mid: "∣",
      midast: "*",
      midcir: "⫰",
      middot: "·",
      minus: "−",
      minusb: "⊟",
      minusd: "∸",
      minusdu: "⨪",
      MinusPlus: "∓",
      mlcp: "⫛",
      mldr: "…",
      mnplus: "∓",
      models: "⊧",
      Mopf: "𝕄",
      mopf: "𝕞",
      mp: "∓",
      Mscr: "ℳ",
      mscr: "𝓂",
      mstpos: "∾",
      Mu: "Μ",
      mu: "μ",
      multimap: "⊸",
      mumap: "⊸",
      nabla: "∇",
      Nacute: "Ń",
      nacute: "ń",
      nang: "∠⃒",
      nap: "≉",
      napE: "⩰̸",
      napid: "≋̸",
      napos: "ŉ",
      napprox: "≉",
      natur: "♮",
      natural: "♮",
      naturals: "ℕ",
      nbsp: " ",
      nbump: "≎̸",
      nbumpe: "≏̸",
      ncap: "⩃",
      Ncaron: "Ň",
      ncaron: "ň",
      Ncedil: "Ņ",
      ncedil: "ņ",
      ncong: "≇",
      ncongdot: "⩭̸",
      ncup: "⩂",
      Ncy: "Н",
      ncy: "н",
      ndash: "–",
      ne: "≠",
      nearhk: "⤤",
      neArr: "⇗",
      nearr: "↗",
      nearrow: "↗",
      nedot: "≐̸",
      NegativeMediumSpace: "​",
      NegativeThickSpace: "​",
      NegativeThinSpace: "​",
      NegativeVeryThinSpace: "​",
      nequiv: "≢",
      nesear: "⤨",
      nesim: "≂̸",
      NestedGreaterGreater: "≫",
      NestedLessLess: "≪",
      NewLine: `
`,
      nexist: "∄",
      nexists: "∄",
      Nfr: "𝔑",
      nfr: "𝔫",
      ngE: "≧̸",
      nge: "≱",
      ngeq: "≱",
      ngeqq: "≧̸",
      ngeqslant: "⩾̸",
      nges: "⩾̸",
      nGg: "⋙̸",
      ngsim: "≵",
      nGt: "≫⃒",
      ngt: "≯",
      ngtr: "≯",
      nGtv: "≫̸",
      nhArr: "⇎",
      nharr: "↮",
      nhpar: "⫲",
      ni: "∋",
      nis: "⋼",
      nisd: "⋺",
      niv: "∋",
      NJcy: "Њ",
      njcy: "њ",
      nlArr: "⇍",
      nlarr: "↚",
      nldr: "‥",
      nlE: "≦̸",
      nle: "≰",
      nLeftarrow: "⇍",
      nleftarrow: "↚",
      nLeftrightarrow: "⇎",
      nleftrightarrow: "↮",
      nleq: "≰",
      nleqq: "≦̸",
      nleqslant: "⩽̸",
      nles: "⩽̸",
      nless: "≮",
      nLl: "⋘̸",
      nlsim: "≴",
      nLt: "≪⃒",
      nlt: "≮",
      nltri: "⋪",
      nltrie: "⋬",
      nLtv: "≪̸",
      nmid: "∤",
      NoBreak: "⁠",
      NonBreakingSpace: " ",
      Nopf: "ℕ",
      nopf: "𝕟",
      Not: "⫬",
      not: "¬",
      NotCongruent: "≢",
      NotCupCap: "≭",
      NotDoubleVerticalBar: "∦",
      NotElement: "∉",
      NotEqual: "≠",
      NotEqualTilde: "≂̸",
      NotExists: "∄",
      NotGreater: "≯",
      NotGreaterEqual: "≱",
      NotGreaterFullEqual: "≧̸",
      NotGreaterGreater: "≫̸",
      NotGreaterLess: "≹",
      NotGreaterSlantEqual: "⩾̸",
      NotGreaterTilde: "≵",
      NotHumpDownHump: "≎̸",
      NotHumpEqual: "≏̸",
      notin: "∉",
      notindot: "⋵̸",
      notinE: "⋹̸",
      notinva: "∉",
      notinvb: "⋷",
      notinvc: "⋶",
      NotLeftTriangle: "⋪",
      NotLeftTriangleBar: "⧏̸",
      NotLeftTriangleEqual: "⋬",
      NotLess: "≮",
      NotLessEqual: "≰",
      NotLessGreater: "≸",
      NotLessLess: "≪̸",
      NotLessSlantEqual: "⩽̸",
      NotLessTilde: "≴",
      NotNestedGreaterGreater: "⪢̸",
      NotNestedLessLess: "⪡̸",
      notni: "∌",
      notniva: "∌",
      notnivb: "⋾",
      notnivc: "⋽",
      NotPrecedes: "⊀",
      NotPrecedesEqual: "⪯̸",
      NotPrecedesSlantEqual: "⋠",
      NotReverseElement: "∌",
      NotRightTriangle: "⋫",
      NotRightTriangleBar: "⧐̸",
      NotRightTriangleEqual: "⋭",
      NotSquareSubset: "⊏̸",
      NotSquareSubsetEqual: "⋢",
      NotSquareSuperset: "⊐̸",
      NotSquareSupersetEqual: "⋣",
      NotSubset: "⊂⃒",
      NotSubsetEqual: "⊈",
      NotSucceeds: "⊁",
      NotSucceedsEqual: "⪰̸",
      NotSucceedsSlantEqual: "⋡",
      NotSucceedsTilde: "≿̸",
      NotSuperset: "⊃⃒",
      NotSupersetEqual: "⊉",
      NotTilde: "≁",
      NotTildeEqual: "≄",
      NotTildeFullEqual: "≇",
      NotTildeTilde: "≉",
      NotVerticalBar: "∤",
      npar: "∦",
      nparallel: "∦",
      nparsl: "⫽⃥",
      npart: "∂̸",
      npolint: "⨔",
      npr: "⊀",
      nprcue: "⋠",
      npre: "⪯̸",
      nprec: "⊀",
      npreceq: "⪯̸",
      nrArr: "⇏",
      nrarr: "↛",
      nrarrc: "⤳̸",
      nrarrw: "↝̸",
      nRightarrow: "⇏",
      nrightarrow: "↛",
      nrtri: "⋫",
      nrtrie: "⋭",
      nsc: "⊁",
      nsccue: "⋡",
      nsce: "⪰̸",
      Nscr: "𝒩",
      nscr: "𝓃",
      nshortmid: "∤",
      nshortparallel: "∦",
      nsim: "≁",
      nsime: "≄",
      nsimeq: "≄",
      nsmid: "∤",
      nspar: "∦",
      nsqsube: "⋢",
      nsqsupe: "⋣",
      nsub: "⊄",
      nsubE: "⫅̸",
      nsube: "⊈",
      nsubset: "⊂⃒",
      nsubseteq: "⊈",
      nsubseteqq: "⫅̸",
      nsucc: "⊁",
      nsucceq: "⪰̸",
      nsup: "⊅",
      nsupE: "⫆̸",
      nsupe: "⊉",
      nsupset: "⊃⃒",
      nsupseteq: "⊉",
      nsupseteqq: "⫆̸",
      ntgl: "≹",
      Ntilde: "Ñ",
      ntilde: "ñ",
      ntlg: "≸",
      ntriangleleft: "⋪",
      ntrianglelefteq: "⋬",
      ntriangleright: "⋫",
      ntrianglerighteq: "⋭",
      Nu: "Ν",
      nu: "ν",
      num: "#",
      numero: "№",
      numsp: " ",
      nvap: "≍⃒",
      nVDash: "⊯",
      nVdash: "⊮",
      nvDash: "⊭",
      nvdash: "⊬",
      nvge: "≥⃒",
      nvgt: ">⃒",
      nvHarr: "⤄",
      nvinfin: "⧞",
      nvlArr: "⤂",
      nvle: "≤⃒",
      nvlt: "<⃒",
      nvltrie: "⊴⃒",
      nvrArr: "⤃",
      nvrtrie: "⊵⃒",
      nvsim: "∼⃒",
      nwarhk: "⤣",
      nwArr: "⇖",
      nwarr: "↖",
      nwarrow: "↖",
      nwnear: "⤧",
      Oacute: "Ó",
      oacute: "ó",
      oast: "⊛",
      ocir: "⊚",
      Ocirc: "Ô",
      ocirc: "ô",
      Ocy: "О",
      ocy: "о",
      odash: "⊝",
      Odblac: "Ő",
      odblac: "ő",
      odiv: "⨸",
      odot: "⊙",
      odsold: "⦼",
      OElig: "Œ",
      oelig: "œ",
      ofcir: "⦿",
      Ofr: "𝔒",
      ofr: "𝔬",
      ogon: "˛",
      Ograve: "Ò",
      ograve: "ò",
      ogt: "⧁",
      ohbar: "⦵",
      ohm: "Ω",
      oint: "∮",
      olarr: "↺",
      olcir: "⦾",
      olcross: "⦻",
      oline: "‾",
      olt: "⧀",
      Omacr: "Ō",
      omacr: "ō",
      Omega: "Ω",
      omega: "ω",
      Omicron: "Ο",
      omicron: "ο",
      omid: "⦶",
      ominus: "⊖",
      Oopf: "𝕆",
      oopf: "𝕠",
      opar: "⦷",
      OpenCurlyDoubleQuote: "“",
      OpenCurlyQuote: "‘",
      operp: "⦹",
      oplus: "⊕",
      Or: "⩔",
      or: "∨",
      orarr: "↻",
      ord: "⩝",
      order: "ℴ",
      orderof: "ℴ",
      ordf: "ª",
      ordm: "º",
      origof: "⊶",
      oror: "⩖",
      orslope: "⩗",
      orv: "⩛",
      oS: "Ⓢ",
      Oscr: "𝒪",
      oscr: "ℴ",
      Oslash: "Ø",
      oslash: "ø",
      osol: "⊘",
      Otilde: "Õ",
      otilde: "õ",
      Otimes: "⨷",
      otimes: "⊗",
      otimesas: "⨶",
      Ouml: "Ö",
      ouml: "ö",
      ovbar: "⌽",
      OverBar: "‾",
      OverBrace: "⏞",
      OverBracket: "⎴",
      OverParenthesis: "⏜",
      par: "∥",
      para: "¶",
      parallel: "∥",
      parsim: "⫳",
      parsl: "⫽",
      part: "∂",
      PartialD: "∂",
      Pcy: "П",
      pcy: "п",
      percnt: "%",
      period: ".",
      permil: "‰",
      perp: "⊥",
      pertenk: "‱",
      Pfr: "𝔓",
      pfr: "𝔭",
      Phi: "Φ",
      phi: "φ",
      phiv: "ϕ",
      phmmat: "ℳ",
      phone: "☎",
      Pi: "Π",
      pi: "π",
      pitchfork: "⋔",
      piv: "ϖ",
      planck: "ℏ",
      planckh: "ℎ",
      plankv: "ℏ",
      plus: "+",
      plusacir: "⨣",
      plusb: "⊞",
      pluscir: "⨢",
      plusdo: "∔",
      plusdu: "⨥",
      pluse: "⩲",
      PlusMinus: "±",
      plusmn: "±",
      plussim: "⨦",
      plustwo: "⨧",
      pm: "±",
      Poincareplane: "ℌ",
      pointint: "⨕",
      Popf: "ℙ",
      popf: "𝕡",
      pound: "£",
      Pr: "⪻",
      pr: "≺",
      prap: "⪷",
      prcue: "≼",
      prE: "⪳",
      pre: "⪯",
      prec: "≺",
      precapprox: "⪷",
      preccurlyeq: "≼",
      Precedes: "≺",
      PrecedesEqual: "⪯",
      PrecedesSlantEqual: "≼",
      PrecedesTilde: "≾",
      preceq: "⪯",
      precnapprox: "⪹",
      precneqq: "⪵",
      precnsim: "⋨",
      precsim: "≾",
      Prime: "″",
      prime: "′",
      primes: "ℙ",
      prnap: "⪹",
      prnE: "⪵",
      prnsim: "⋨",
      prod: "∏",
      Product: "∏",
      profalar: "⌮",
      profline: "⌒",
      profsurf: "⌓",
      prop: "∝",
      Proportion: "∷",
      Proportional: "∝",
      propto: "∝",
      prsim: "≾",
      prurel: "⊰",
      Pscr: "𝒫",
      pscr: "𝓅",
      Psi: "Ψ",
      psi: "ψ",
      puncsp: " ",
      Qfr: "𝔔",
      qfr: "𝔮",
      qint: "⨌",
      Qopf: "ℚ",
      qopf: "𝕢",
      qprime: "⁗",
      Qscr: "𝒬",
      qscr: "𝓆",
      quaternions: "ℍ",
      quatint: "⨖",
      quest: "?",
      questeq: "≟",
      QUOT: '"',
      quot: '"',
      rAarr: "⇛",
      race: "∽̱",
      Racute: "Ŕ",
      racute: "ŕ",
      radic: "√",
      raemptyv: "⦳",
      Rang: "⟫",
      rang: "⟩",
      rangd: "⦒",
      range: "⦥",
      rangle: "⟩",
      raquo: "»",
      Rarr: "↠",
      rArr: "⇒",
      rarr: "→",
      rarrap: "⥵",
      rarrb: "⇥",
      rarrbfs: "⤠",
      rarrc: "⤳",
      rarrfs: "⤞",
      rarrhk: "↪",
      rarrlp: "↬",
      rarrpl: "⥅",
      rarrsim: "⥴",
      Rarrtl: "⤖",
      rarrtl: "↣",
      rarrw: "↝",
      rAtail: "⤜",
      ratail: "⤚",
      ratio: "∶",
      rationals: "ℚ",
      RBarr: "⤐",
      rBarr: "⤏",
      rbarr: "⤍",
      rbbrk: "❳",
      rbrace: "}",
      rbrack: "]",
      rbrke: "⦌",
      rbrksld: "⦎",
      rbrkslu: "⦐",
      Rcaron: "Ř",
      rcaron: "ř",
      Rcedil: "Ŗ",
      rcedil: "ŗ",
      rceil: "⌉",
      rcub: "}",
      Rcy: "Р",
      rcy: "р",
      rdca: "⤷",
      rdldhar: "⥩",
      rdquo: "”",
      rdquor: "”",
      rdsh: "↳",
      Re: "ℜ",
      real: "ℜ",
      realine: "ℛ",
      realpart: "ℜ",
      reals: "ℝ",
      rect: "▭",
      REG: "®",
      reg: "®",
      ReverseElement: "∋",
      ReverseEquilibrium: "⇋",
      ReverseUpEquilibrium: "⥯",
      rfisht: "⥽",
      rfloor: "⌋",
      Rfr: "ℜ",
      rfr: "𝔯",
      rHar: "⥤",
      rhard: "⇁",
      rharu: "⇀",
      rharul: "⥬",
      Rho: "Ρ",
      rho: "ρ",
      rhov: "ϱ",
      RightAngleBracket: "⟩",
      RightArrow: "→",
      Rightarrow: "⇒",
      rightarrow: "→",
      RightArrowBar: "⇥",
      RightArrowLeftArrow: "⇄",
      rightarrowtail: "↣",
      RightCeiling: "⌉",
      RightDoubleBracket: "⟧",
      RightDownTeeVector: "⥝",
      RightDownVector: "⇂",
      RightDownVectorBar: "⥕",
      RightFloor: "⌋",
      rightharpoondown: "⇁",
      rightharpoonup: "⇀",
      rightleftarrows: "⇄",
      rightleftharpoons: "⇌",
      rightrightarrows: "⇉",
      rightsquigarrow: "↝",
      RightTee: "⊢",
      RightTeeArrow: "↦",
      RightTeeVector: "⥛",
      rightthreetimes: "⋌",
      RightTriangle: "⊳",
      RightTriangleBar: "⧐",
      RightTriangleEqual: "⊵",
      RightUpDownVector: "⥏",
      RightUpTeeVector: "⥜",
      RightUpVector: "↾",
      RightUpVectorBar: "⥔",
      RightVector: "⇀",
      RightVectorBar: "⥓",
      ring: "˚",
      risingdotseq: "≓",
      rlarr: "⇄",
      rlhar: "⇌",
      rlm: "‏",
      rmoust: "⎱",
      rmoustache: "⎱",
      rnmid: "⫮",
      roang: "⟭",
      roarr: "⇾",
      robrk: "⟧",
      ropar: "⦆",
      Ropf: "ℝ",
      ropf: "𝕣",
      roplus: "⨮",
      rotimes: "⨵",
      RoundImplies: "⥰",
      rpar: ")",
      rpargt: "⦔",
      rppolint: "⨒",
      rrarr: "⇉",
      Rrightarrow: "⇛",
      rsaquo: "›",
      Rscr: "ℛ",
      rscr: "𝓇",
      Rsh: "↱",
      rsh: "↱",
      rsqb: "]",
      rsquo: "’",
      rsquor: "’",
      rthree: "⋌",
      rtimes: "⋊",
      rtri: "▹",
      rtrie: "⊵",
      rtrif: "▸",
      rtriltri: "⧎",
      RuleDelayed: "⧴",
      ruluhar: "⥨",
      rx: "℞",
      Sacute: "Ś",
      sacute: "ś",
      sbquo: "‚",
      Sc: "⪼",
      sc: "≻",
      scap: "⪸",
      Scaron: "Š",
      scaron: "š",
      sccue: "≽",
      scE: "⪴",
      sce: "⪰",
      Scedil: "Ş",
      scedil: "ş",
      Scirc: "Ŝ",
      scirc: "ŝ",
      scnap: "⪺",
      scnE: "⪶",
      scnsim: "⋩",
      scpolint: "⨓",
      scsim: "≿",
      Scy: "С",
      scy: "с",
      sdot: "⋅",
      sdotb: "⊡",
      sdote: "⩦",
      searhk: "⤥",
      seArr: "⇘",
      searr: "↘",
      searrow: "↘",
      sect: "§",
      semi: ";",
      seswar: "⤩",
      setminus: "∖",
      setmn: "∖",
      sext: "✶",
      Sfr: "𝔖",
      sfr: "𝔰",
      sfrown: "⌢",
      sharp: "♯",
      SHCHcy: "Щ",
      shchcy: "щ",
      SHcy: "Ш",
      shcy: "ш",
      ShortDownArrow: "↓",
      ShortLeftArrow: "←",
      shortmid: "∣",
      shortparallel: "∥",
      ShortRightArrow: "→",
      ShortUpArrow: "↑",
      shy: "­",
      Sigma: "Σ",
      sigma: "σ",
      sigmaf: "ς",
      sigmav: "ς",
      sim: "∼",
      simdot: "⩪",
      sime: "≃",
      simeq: "≃",
      simg: "⪞",
      simgE: "⪠",
      siml: "⪝",
      simlE: "⪟",
      simne: "≆",
      simplus: "⨤",
      simrarr: "⥲",
      slarr: "←",
      SmallCircle: "∘",
      smallsetminus: "∖",
      smashp: "⨳",
      smeparsl: "⧤",
      smid: "∣",
      smile: "⌣",
      smt: "⪪",
      smte: "⪬",
      smtes: "⪬︀",
      SOFTcy: "Ь",
      softcy: "ь",
      sol: "/",
      solb: "⧄",
      solbar: "⌿",
      Sopf: "𝕊",
      sopf: "𝕤",
      spades: "♠",
      spadesuit: "♠",
      spar: "∥",
      sqcap: "⊓",
      sqcaps: "⊓︀",
      sqcup: "⊔",
      sqcups: "⊔︀",
      Sqrt: "√",
      sqsub: "⊏",
      sqsube: "⊑",
      sqsubset: "⊏",
      sqsubseteq: "⊑",
      sqsup: "⊐",
      sqsupe: "⊒",
      sqsupset: "⊐",
      sqsupseteq: "⊒",
      squ: "□",
      Square: "□",
      square: "□",
      SquareIntersection: "⊓",
      SquareSubset: "⊏",
      SquareSubsetEqual: "⊑",
      SquareSuperset: "⊐",
      SquareSupersetEqual: "⊒",
      SquareUnion: "⊔",
      squarf: "▪",
      squf: "▪",
      srarr: "→",
      Sscr: "𝒮",
      sscr: "𝓈",
      ssetmn: "∖",
      ssmile: "⌣",
      sstarf: "⋆",
      Star: "⋆",
      star: "☆",
      starf: "★",
      straightepsilon: "ϵ",
      straightphi: "ϕ",
      strns: "¯",
      Sub: "⋐",
      sub: "⊂",
      subdot: "⪽",
      subE: "⫅",
      sube: "⊆",
      subedot: "⫃",
      submult: "⫁",
      subnE: "⫋",
      subne: "⊊",
      subplus: "⪿",
      subrarr: "⥹",
      Subset: "⋐",
      subset: "⊂",
      subseteq: "⊆",
      subseteqq: "⫅",
      SubsetEqual: "⊆",
      subsetneq: "⊊",
      subsetneqq: "⫋",
      subsim: "⫇",
      subsub: "⫕",
      subsup: "⫓",
      succ: "≻",
      succapprox: "⪸",
      succcurlyeq: "≽",
      Succeeds: "≻",
      SucceedsEqual: "⪰",
      SucceedsSlantEqual: "≽",
      SucceedsTilde: "≿",
      succeq: "⪰",
      succnapprox: "⪺",
      succneqq: "⪶",
      succnsim: "⋩",
      succsim: "≿",
      SuchThat: "∋",
      Sum: "∑",
      sum: "∑",
      sung: "♪",
      Sup: "⋑",
      sup: "⊃",
      sup1: "¹",
      sup2: "²",
      sup3: "³",
      supdot: "⪾",
      supdsub: "⫘",
      supE: "⫆",
      supe: "⊇",
      supedot: "⫄",
      Superset: "⊃",
      SupersetEqual: "⊇",
      suphsol: "⟉",
      suphsub: "⫗",
      suplarr: "⥻",
      supmult: "⫂",
      supnE: "⫌",
      supne: "⊋",
      supplus: "⫀",
      Supset: "⋑",
      supset: "⊃",
      supseteq: "⊇",
      supseteqq: "⫆",
      supsetneq: "⊋",
      supsetneqq: "⫌",
      supsim: "⫈",
      supsub: "⫔",
      supsup: "⫖",
      swarhk: "⤦",
      swArr: "⇙",
      swarr: "↙",
      swarrow: "↙",
      swnwar: "⤪",
      szlig: "ß",
      Tab: "	",
      target: "⌖",
      Tau: "Τ",
      tau: "τ",
      tbrk: "⎴",
      Tcaron: "Ť",
      tcaron: "ť",
      Tcedil: "Ţ",
      tcedil: "ţ",
      Tcy: "Т",
      tcy: "т",
      tdot: "⃛",
      telrec: "⌕",
      Tfr: "𝔗",
      tfr: "𝔱",
      there4: "∴",
      Therefore: "∴",
      therefore: "∴",
      Theta: "Θ",
      theta: "θ",
      thetasym: "ϑ",
      thetav: "ϑ",
      thickapprox: "≈",
      thicksim: "∼",
      ThickSpace: "  ",
      thinsp: " ",
      ThinSpace: " ",
      thkap: "≈",
      thksim: "∼",
      THORN: "Þ",
      thorn: "þ",
      Tilde: "∼",
      tilde: "˜",
      TildeEqual: "≃",
      TildeFullEqual: "≅",
      TildeTilde: "≈",
      times: "×",
      timesb: "⊠",
      timesbar: "⨱",
      timesd: "⨰",
      tint: "∭",
      toea: "⤨",
      top: "⊤",
      topbot: "⌶",
      topcir: "⫱",
      Topf: "𝕋",
      topf: "𝕥",
      topfork: "⫚",
      tosa: "⤩",
      tprime: "‴",
      TRADE: "™",
      trade: "™",
      triangle: "▵",
      triangledown: "▿",
      triangleleft: "◃",
      trianglelefteq: "⊴",
      triangleq: "≜",
      triangleright: "▹",
      trianglerighteq: "⊵",
      tridot: "◬",
      trie: "≜",
      triminus: "⨺",
      TripleDot: "⃛",
      triplus: "⨹",
      trisb: "⧍",
      tritime: "⨻",
      trpezium: "⏢",
      Tscr: "𝒯",
      tscr: "𝓉",
      TScy: "Ц",
      tscy: "ц",
      TSHcy: "Ћ",
      tshcy: "ћ",
      Tstrok: "Ŧ",
      tstrok: "ŧ",
      twixt: "≬",
      twoheadleftarrow: "↞",
      twoheadrightarrow: "↠",
      Uacute: "Ú",
      uacute: "ú",
      Uarr: "↟",
      uArr: "⇑",
      uarr: "↑",
      Uarrocir: "⥉",
      Ubrcy: "Ў",
      ubrcy: "ў",
      Ubreve: "Ŭ",
      ubreve: "ŭ",
      Ucirc: "Û",
      ucirc: "û",
      Ucy: "У",
      ucy: "у",
      udarr: "⇅",
      Udblac: "Ű",
      udblac: "ű",
      udhar: "⥮",
      ufisht: "⥾",
      Ufr: "𝔘",
      ufr: "𝔲",
      Ugrave: "Ù",
      ugrave: "ù",
      uHar: "⥣",
      uharl: "↿",
      uharr: "↾",
      uhblk: "▀",
      ulcorn: "⌜",
      ulcorner: "⌜",
      ulcrop: "⌏",
      ultri: "◸",
      Umacr: "Ū",
      umacr: "ū",
      uml: "¨",
      UnderBar: "_",
      UnderBrace: "⏟",
      UnderBracket: "⎵",
      UnderParenthesis: "⏝",
      Union: "⋃",
      UnionPlus: "⊎",
      Uogon: "Ų",
      uogon: "ų",
      Uopf: "𝕌",
      uopf: "𝕦",
      UpArrow: "↑",
      Uparrow: "⇑",
      uparrow: "↑",
      UpArrowBar: "⤒",
      UpArrowDownArrow: "⇅",
      UpDownArrow: "↕",
      Updownarrow: "⇕",
      updownarrow: "↕",
      UpEquilibrium: "⥮",
      upharpoonleft: "↿",
      upharpoonright: "↾",
      uplus: "⊎",
      UpperLeftArrow: "↖",
      UpperRightArrow: "↗",
      Upsi: "ϒ",
      upsi: "υ",
      upsih: "ϒ",
      Upsilon: "Υ",
      upsilon: "υ",
      UpTee: "⊥",
      UpTeeArrow: "↥",
      upuparrows: "⇈",
      urcorn: "⌝",
      urcorner: "⌝",
      urcrop: "⌎",
      Uring: "Ů",
      uring: "ů",
      urtri: "◹",
      Uscr: "𝒰",
      uscr: "𝓊",
      utdot: "⋰",
      Utilde: "Ũ",
      utilde: "ũ",
      utri: "▵",
      utrif: "▴",
      uuarr: "⇈",
      Uuml: "Ü",
      uuml: "ü",
      uwangle: "⦧",
      vangrt: "⦜",
      varepsilon: "ϵ",
      varkappa: "ϰ",
      varnothing: "∅",
      varphi: "ϕ",
      varpi: "ϖ",
      varpropto: "∝",
      vArr: "⇕",
      varr: "↕",
      varrho: "ϱ",
      varsigma: "ς",
      varsubsetneq: "⊊︀",
      varsubsetneqq: "⫋︀",
      varsupsetneq: "⊋︀",
      varsupsetneqq: "⫌︀",
      vartheta: "ϑ",
      vartriangleleft: "⊲",
      vartriangleright: "⊳",
      Vbar: "⫫",
      vBar: "⫨",
      vBarv: "⫩",
      Vcy: "В",
      vcy: "в",
      VDash: "⊫",
      Vdash: "⊩",
      vDash: "⊨",
      vdash: "⊢",
      Vdashl: "⫦",
      Vee: "⋁",
      vee: "∨",
      veebar: "⊻",
      veeeq: "≚",
      vellip: "⋮",
      Verbar: "‖",
      verbar: "|",
      Vert: "‖",
      vert: "|",
      VerticalBar: "∣",
      VerticalLine: "|",
      VerticalSeparator: "❘",
      VerticalTilde: "≀",
      VeryThinSpace: " ",
      Vfr: "𝔙",
      vfr: "𝔳",
      vltri: "⊲",
      vnsub: "⊂⃒",
      vnsup: "⊃⃒",
      Vopf: "𝕍",
      vopf: "𝕧",
      vprop: "∝",
      vrtri: "⊳",
      Vscr: "𝒱",
      vscr: "𝓋",
      vsubnE: "⫋︀",
      vsubne: "⊊︀",
      vsupnE: "⫌︀",
      vsupne: "⊋︀",
      Vvdash: "⊪",
      vzigzag: "⦚",
      Wcirc: "Ŵ",
      wcirc: "ŵ",
      wedbar: "⩟",
      Wedge: "⋀",
      wedge: "∧",
      wedgeq: "≙",
      weierp: "℘",
      Wfr: "𝔚",
      wfr: "𝔴",
      Wopf: "𝕎",
      wopf: "𝕨",
      wp: "℘",
      wr: "≀",
      wreath: "≀",
      Wscr: "𝒲",
      wscr: "𝓌",
      xcap: "⋂",
      xcirc: "◯",
      xcup: "⋃",
      xdtri: "▽",
      Xfr: "𝔛",
      xfr: "𝔵",
      xhArr: "⟺",
      xharr: "⟷",
      Xi: "Ξ",
      xi: "ξ",
      xlArr: "⟸",
      xlarr: "⟵",
      xmap: "⟼",
      xnis: "⋻",
      xodot: "⨀",
      Xopf: "𝕏",
      xopf: "𝕩",
      xoplus: "⨁",
      xotime: "⨂",
      xrArr: "⟹",
      xrarr: "⟶",
      Xscr: "𝒳",
      xscr: "𝓍",
      xsqcup: "⨆",
      xuplus: "⨄",
      xutri: "△",
      xvee: "⋁",
      xwedge: "⋀",
      Yacute: "Ý",
      yacute: "ý",
      YAcy: "Я",
      yacy: "я",
      Ycirc: "Ŷ",
      ycirc: "ŷ",
      Ycy: "Ы",
      ycy: "ы",
      yen: "¥",
      Yfr: "𝔜",
      yfr: "𝔶",
      YIcy: "Ї",
      yicy: "ї",
      Yopf: "𝕐",
      yopf: "𝕪",
      Yscr: "𝒴",
      yscr: "𝓎",
      YUcy: "Ю",
      yucy: "ю",
      Yuml: "Ÿ",
      yuml: "ÿ",
      Zacute: "Ź",
      zacute: "ź",
      Zcaron: "Ž",
      zcaron: "ž",
      Zcy: "З",
      zcy: "з",
      Zdot: "Ż",
      zdot: "ż",
      zeetrf: "ℨ",
      ZeroWidthSpace: "​",
      Zeta: "Ζ",
      zeta: "ζ",
      Zfr: "ℨ",
      zfr: "𝔷",
      ZHcy: "Ж",
      zhcy: "ж",
      zigrarr: "⇝",
      Zopf: "ℤ",
      zopf: "𝕫",
      Zscr: "𝒵",
      zscr: "𝓏",
      zwj: "‍",
      zwnj: "‌"
    }), o.entityMap = o.HTML_ENTITIES;
  }(At)), At;
}
var lt = {}, Pt;
function Tr() {
  if (Pt) return lt;
  Pt = 1;
  var o = nt(), t = tr(), u = dt(), s = o.isHTMLEscapableRawTextElement, a = o.isHTMLMimeType, n = o.isHTMLRawTextElement, l = o.hasOwn, g = o.NAMESPACE, b = u.ParseError, O = u.DOMException, A = 0, f = 1, M = 2, S = 3, k = 4, Y = 5, te = 6, x = 7;
  function H() {
  }
  H.prototype = {
    parse: function(h, p, d) {
      var T = this.domBuilder;
      T.startDocument(), G(p, p = /* @__PURE__ */ Object.create(null)), re(h, p, d, T, this.errorHandler), T.endDocument();
    }
  };
  var j = /&#?\w+;?/g;
  function re(h, p, d, T, I) {
    var D = a(T.mimeType);
    h.indexOf(t.UNICODE_REPLACEMENT_CHARACTER) >= 0 && I.warning("Unicode replacement character detected, source encoding issues?");
    function R(V) {
      if (V > 65535) {
        V -= 65536;
        var ee = 55296 + (V >> 10), Ae = 56320 + (V & 1023);
        return String.fromCharCode(ee, Ae);
      } else
        return String.fromCharCode(V);
    }
    function X(V) {
      var ee = V[V.length - 1] === ";" ? V : V + ";";
      if (!D && ee !== V)
        return I.error("EntityRef: expecting ;"), V;
      var Ae = t.Reference.exec(ee);
      if (!Ae || Ae[0].length !== ee.length)
        return I.error("entity not matching Reference production: " + V), V;
      var ge = ee.slice(1, -1);
      return l(d, ge) ? d[ge] : ge.charAt(0) === "#" ? R(parseInt(ge.substring(1).replace("x", "0x"))) : (I.error("entity not found:" + V), V);
    }
    function B(V) {
      if (V > ae) {
        var ee = h.substring(ae, V).replace(j, X);
        Q && he(ae), T.characters(ee, 0, V - ae), ae = V;
      }
    }
    var y = 0, w = 0, q = /\r\n?|\n|$/g, Q = T.locator;
    function he(V, ee) {
      for (; V >= w && (ee = q.exec(h)); )
        y = w, w = ee.index + ee[0].length, Q.lineNumber++;
      Q.columnNumber = V - y + 1;
    }
    for (var be = [{ currentNSMap: p }], ve = [], ae = 0; ; ) {
      try {
        var F = h.indexOf("<", ae);
        if (F < 0) {
          if (!D && ve.length > 0)
            return I.fatalError("unclosed xml tag(s): " + ve.join(", "));
          if (!h.substring(ae).match(/^\s*$/)) {
            var Le = T.doc, we = Le.createTextNode(h.substring(ae));
            if (Le.documentElement)
              return I.error("Extra content at the end of the document");
            Le.appendChild(we), T.currentElement = we;
          }
          return;
        }
        if (F > ae) {
          var fe = h.substring(ae, F);
          !D && ve.length === 0 && (fe = fe.replace(new RegExp(t.S_OPT.source, "g"), ""), fe && I.error("Unexpected content outside root element: '" + fe + "'")), B(F);
        }
        switch (h.charAt(F + 1)) {
          case "/":
            var oe = h.indexOf(">", F + 2), Fe = h.substring(F + 2, oe > 0 ? oe : void 0);
            if (!Fe)
              return I.fatalError("end tag name missing");
            var _e = oe > 0 && t.reg("^", t.QName_group, t.S_OPT, "$").exec(Fe);
            if (!_e)
              return I.fatalError('end tag name contains invalid characters: "' + Fe + '"');
            if (!T.currentElement && !T.doc.documentElement)
              return;
            var ye = ve[ve.length - 1] || T.currentElement.tagName || T.doc.documentElement.tagName || "";
            if (ye !== _e[1]) {
              var xe = _e[1].toLowerCase();
              if (!D || ye.toLowerCase() !== xe)
                return I.fatalError('Opening and ending tag mismatch: "' + ye + '" != "' + Fe + '"');
            }
            var ke = be.pop();
            ve.pop();
            var qe = ke.localNSMap;
            if (T.endElement(ke.uri, ke.localName, ye), qe)
              for (var Te in qe)
                l(qe, Te) && T.endPrefixMapping(Te);
            oe++;
            break;
          // end element
          case "?":
            Q && he(F), oe = C(h, F, T, I);
            break;
          case "!":
            Q && he(F), oe = U(h, F, T, I, D);
            break;
          default:
            Q && he(F);
            var K = new L(), Pe = be[be.length - 1].currentNSMap, oe = m(h, F, K, Pe, X, I, D), Ge = K.length;
            if (K.closed || (D && o.isHTMLVoidElement(K.tagName) ? K.closed = !0 : ve.push(K.tagName)), Q && Ge) {
              for (var st = ie(Q, {}), Ve = 0; Ve < Ge; Ve++) {
                var ze = K[Ve];
                he(ze.offset), ze.locator = ie(Q, {});
              }
              T.locator = st, N(K, T, Pe) && be.push(K), T.locator = Q;
            } else
              N(K, T, Pe) && be.push(K);
            D && !K.closed ? oe = P(h, oe, K.tagName, X, T) : oe++;
        }
      } catch (V) {
        if (V instanceof b)
          throw V;
        if (V instanceof O)
          throw new b(V.name + ": " + V.message, T.locator, V);
        I.error("element parse error: " + V), oe = -1;
      }
      oe > ae ? ae = oe : B(Math.max(F, ae) + 1);
    }
  }
  function ie(h, p) {
    return p.lineNumber = h.lineNumber, p.columnNumber = h.columnNumber, p;
  }
  function m(h, p, d, T, I, D, R) {
    function X(he, be, ve) {
      if (l(d.attributeNames, he))
        return D.fatalError("Attribute " + he + " redefined");
      if (!R && be.indexOf("<") >= 0)
        return D.fatalError("Unescaped '<' not allowed in attributes values");
      d.addValue(
        he,
        // @see https://www.w3.org/TR/xml/#AVNormalize
        // since the xmldom sax parser does not "interpret" DTD the following is not implemented:
        // - recursive replacement of (DTD) entity references
        // - trimming and collapsing multiple spaces into a single one for attributes that are not of type CDATA
        be.replace(/[\t\n\r]/g, " ").replace(j, I),
        ve
      );
    }
    for (var B, y, w = ++p, q = A; ; ) {
      var Q = h.charAt(w);
      switch (Q) {
        case "=":
          if (q === f)
            B = h.slice(p, w), q = S;
          else if (q === M)
            q = S;
          else
            throw new Error("attribute equal must after attrName");
          break;
        case "'":
        case '"':
          if (q === S || q === f)
            if (q === f && (D.warning('attribute value must after "="'), B = h.slice(p, w)), p = w + 1, w = h.indexOf(Q, p), w > 0)
              y = h.slice(p, w), X(B, y, p - 1), q = Y;
            else
              throw new Error("attribute value no end '" + Q + "' match");
          else if (q == k)
            y = h.slice(p, w), X(B, y, p), D.warning('attribute "' + B + '" missed start quot(' + Q + ")!!"), p = w + 1, q = Y;
          else
            throw new Error('attribute value must after "="');
          break;
        case "/":
          switch (q) {
            case A:
              d.setTagName(h.slice(p, w));
            case Y:
            case te:
            case x:
              q = x, d.closed = !0;
            case k:
            case f:
              break;
            case M:
              d.closed = !0;
              break;
            //case S_EQ:
            default:
              throw new Error("attribute invalid close char('/')");
          }
          break;
        case "":
          return D.error("unexpected end of input"), q == A && d.setTagName(h.slice(p, w)), w;
        case ">":
          switch (q) {
            case A:
              d.setTagName(h.slice(p, w));
            case Y:
            case te:
            case x:
              break;
            //normal
            case k:
            //Compatible state
            case f:
              y = h.slice(p, w), y.slice(-1) === "/" && (d.closed = !0, y = y.slice(0, -1));
            case M:
              q === M && (y = B), q == k ? (D.warning('attribute "' + y + '" missed quot(")!'), X(B, y, p)) : (R || D.warning('attribute "' + y + '" missed value!! "' + y + '" instead!!'), X(y, y, p));
              break;
            case S:
              if (!R)
                return D.fatalError(`AttValue: ' or " expected`);
          }
          return w;
        /*xml space '\x20' | #x9 | #xD | #xA; */
        case "":
          Q = " ";
        default:
          if (Q <= " ")
            switch (q) {
              case A:
                d.setTagName(h.slice(p, w)), q = te;
                break;
              case f:
                B = h.slice(p, w), q = M;
                break;
              case k:
                var y = h.slice(p, w);
                D.warning('attribute "' + y + '" missed quot(")!!'), X(B, y, p);
              case Y:
                q = te;
                break;
            }
          else
            switch (q) {
              //case S_TAG:void();break;
              //case S_ATTR:void();break;
              //case S_ATTR_NOQUOT_VALUE:void();break;
              case M:
                R || D.warning('attribute "' + B + '" missed value!! "' + B + '" instead2!!'), X(B, B, p), p = w, q = f;
                break;
              case Y:
                D.warning('attribute space is required"' + B + '"!!');
              case te:
                q = f, p = w;
                break;
              case S:
                q = k, p = w;
                break;
              case x:
                throw new Error("elements closed character '/' and '>' must be connected to");
            }
      }
      w++;
    }
  }
  function N(h, p, d) {
    for (var T = h.tagName, I = null, q = h.length; q--; ) {
      var D = h[q], R = D.qName, X = D.value, Q = R.indexOf(":");
      if (Q > 0)
        var B = D.prefix = R.slice(0, Q), y = R.slice(Q + 1), w = B === "xmlns" && y;
      else
        y = R, B = null, w = R === "xmlns" && "";
      D.localName = y, w !== !1 && (I == null && (I = /* @__PURE__ */ Object.create(null), G(d, d = /* @__PURE__ */ Object.create(null))), d[w] = I[w] = X, D.uri = g.XMLNS, p.startPrefixMapping(w, X));
    }
    for (var q = h.length; q--; )
      D = h[q], D.prefix && (D.prefix === "xml" && (D.uri = g.XML), D.prefix !== "xmlns" && (D.uri = d[D.prefix]));
    var Q = T.indexOf(":");
    Q > 0 ? (B = h.prefix = T.slice(0, Q), y = h.localName = T.slice(Q + 1)) : (B = null, y = h.localName = T);
    var he = h.uri = d[B || ""];
    if (p.startElement(he, y, T, h), h.closed) {
      if (p.endElement(he, y, T), I)
        for (B in I)
          l(I, B) && p.endPrefixMapping(B);
    } else
      return h.currentNSMap = d, h.localNSMap = I, !0;
  }
  function P(h, p, d, T, I) {
    var D = s(d);
    if (D || n(d)) {
      var R = h.indexOf("</" + d + ">", p), X = h.substring(p + 1, R);
      return D && (X = X.replace(j, T)), I.characters(X, 0, X.length), R;
    }
    return p + 1;
  }
  function G(h, p) {
    for (var d in h)
      l(h, d) && (p[d] = h[d]);
  }
  function W(h, p) {
    var d = p;
    function T(w) {
      return w = w || 0, h.charAt(d + w);
    }
    function I(w) {
      w = w || 1, d += w;
    }
    function D() {
      for (var w = 0; d < h.length; ) {
        var q = T();
        if (q !== " " && q !== `
` && q !== "	" && q !== "\r")
          return w;
        w++, I();
      }
      return -1;
    }
    function R() {
      return h.substring(d);
    }
    function X(w) {
      return h.substring(d, d + w.length) === w;
    }
    function B(w) {
      return h.substring(d, d + w.length).toUpperCase() === w.toUpperCase();
    }
    function y(w) {
      var q = t.reg("^", w), Q = q.exec(R());
      return Q ? (I(Q[0].length), Q[0]) : null;
    }
    return {
      char: T,
      getIndex: function() {
        return d;
      },
      getMatch: y,
      getSource: function() {
        return h;
      },
      skip: I,
      skipBlanks: D,
      substringFromIndex: R,
      substringStartsWith: X,
      substringStartsWithCaseInsensitive: B
    };
  }
  function v(h, p) {
    function d(X, B) {
      var y = t.PI.exec(X.substringFromIndex());
      return y ? y[1].toLowerCase() === "xml" ? B.fatalError(
        "xml declaration is only allowed at the start of the document, but found at position " + X.getIndex()
      ) : (X.skip(y[0].length), y[0]) : B.fatalError("processing instruction is not well-formed at position " + X.getIndex());
    }
    var T = h.getSource();
    if (h.char() === "[") {
      h.skip(1);
      for (var I = h.getIndex(); h.getIndex() < T.length; ) {
        if (h.skipBlanks(), h.char() === "]") {
          var D = T.substring(I, h.getIndex());
          return h.skip(1), D;
        }
        var R = null;
        if (h.char() === "<" && h.char(1) === "!")
          switch (h.char(2)) {
            case "E":
              h.char(3) === "L" ? R = h.getMatch(t.elementdecl) : h.char(3) === "N" && (R = h.getMatch(t.EntityDecl));
              break;
            case "A":
              R = h.getMatch(t.AttlistDecl);
              break;
            case "N":
              R = h.getMatch(t.NotationDecl);
              break;
            case "-":
              R = h.getMatch(t.Comment);
              break;
          }
        else if (h.char() === "<" && h.char(1) === "?")
          R = d(h, p);
        else if (h.char() === "%")
          R = h.getMatch(t.PEReference);
        else
          return p.fatalError("Error detected in Markup declaration");
        if (!R)
          return p.fatalError("Error in internal subset at position " + h.getIndex());
      }
      return p.fatalError("doctype internal subset is not well-formed, missing ]");
    }
  }
  function U(h, p, d, T, I) {
    var D = W(h, p);
    switch (I ? D.char(2).toUpperCase() : D.char(2)) {
      case "-":
        var R = D.getMatch(t.Comment);
        return R ? (d.comment(R, t.COMMENT_START.length, R.length - t.COMMENT_START.length - t.COMMENT_END.length), D.getIndex()) : T.fatalError("comment is not well-formed at position " + D.getIndex());
      case "[":
        var X = D.getMatch(t.CDSect);
        return X ? !I && !d.currentElement ? T.fatalError("CDATA outside of element") : (d.startCDATA(), d.characters(X, t.CDATA_START.length, X.length - t.CDATA_START.length - t.CDATA_END.length), d.endCDATA(), D.getIndex()) : T.fatalError("Invalid CDATA starting at position " + p);
      case "D": {
        if (d.doc && d.doc.documentElement)
          return T.fatalError("Doctype not allowed inside or after documentElement at position " + D.getIndex());
        if (I ? !D.substringStartsWithCaseInsensitive(t.DOCTYPE_DECL_START) : !D.substringStartsWith(t.DOCTYPE_DECL_START))
          return T.fatalError("Expected " + t.DOCTYPE_DECL_START + " at position " + D.getIndex());
        if (D.skip(t.DOCTYPE_DECL_START.length), D.skipBlanks() < 1)
          return T.fatalError("Expected whitespace after " + t.DOCTYPE_DECL_START + " at position " + D.getIndex());
        var B = {
          name: void 0,
          publicId: void 0,
          systemId: void 0,
          internalSubset: void 0
        };
        if (B.name = D.getMatch(t.Name), !B.name)
          return T.fatalError("doctype name missing or contains unexpected characters at position " + D.getIndex());
        if (I && B.name.toLowerCase() !== "html" && T.warning("Unexpected DOCTYPE in HTML document at position " + D.getIndex()), D.skipBlanks(), D.substringStartsWith(t.PUBLIC) || D.substringStartsWith(t.SYSTEM)) {
          var y = t.ExternalID_match.exec(D.substringFromIndex());
          if (!y)
            return T.fatalError("doctype external id is not well-formed at position " + D.getIndex());
          y.groups.SystemLiteralOnly !== void 0 ? B.systemId = y.groups.SystemLiteralOnly : (B.systemId = y.groups.SystemLiteral, B.publicId = y.groups.PubidLiteral), D.skip(y[0].length);
        } else if (I && D.substringStartsWithCaseInsensitive(t.SYSTEM)) {
          if (D.skip(t.SYSTEM.length), D.skipBlanks() < 1)
            return T.fatalError("Expected whitespace after " + t.SYSTEM + " at position " + D.getIndex());
          if (B.systemId = D.getMatch(t.ABOUT_LEGACY_COMPAT_SystemLiteral), !B.systemId)
            return T.fatalError(
              "Expected " + t.ABOUT_LEGACY_COMPAT + " in single or double quotes after " + t.SYSTEM + " at position " + D.getIndex()
            );
        }
        return I && B.systemId && !t.ABOUT_LEGACY_COMPAT_SystemLiteral.test(B.systemId) && T.warning("Unexpected doctype.systemId in HTML document at position " + D.getIndex()), I || (D.skipBlanks(), B.internalSubset = v(D, T)), D.skipBlanks(), D.char() !== ">" ? T.fatalError("doctype not terminated with > at position " + D.getIndex()) : (D.skip(1), d.startDTD(B.name, B.publicId, B.systemId, B.internalSubset), d.endDTD(), D.getIndex());
      }
      default:
        return T.fatalError('Not well-formed XML starting with "<!" at position ' + p);
    }
  }
  function C(h, p, d, T) {
    var I = h.substring(p).match(t.PI);
    if (!I)
      return T.fatalError("Invalid processing instruction starting at position " + p);
    if (I[1].toLowerCase() === "xml") {
      if (p > 0)
        return T.fatalError(
          "processing instruction at position " + p + " is an xml declaration which is only at the start of the document"
        );
      if (!t.XMLDecl.test(h.substring(p)))
        return T.fatalError("xml declaration is not well-formed");
    }
    return d.processingInstruction(I[1], I[2]), p + I[0].length;
  }
  function L() {
    this.attributeNames = /* @__PURE__ */ Object.create(null);
  }
  return L.prototype = {
    setTagName: function(h) {
      if (!t.QName_exact.test(h))
        throw new Error("invalid tagName:" + h);
      this.tagName = h;
    },
    addValue: function(h, p, d) {
      if (!t.QName_exact.test(h))
        throw new Error("invalid attribute:" + h);
      this.attributeNames[h] = this.length, this[this.length++] = { qName: h, value: p, offset: d };
    },
    length: 0,
    getLocalName: function(h) {
      return this[h].localName;
    },
    getLocator: function(h) {
      return this[h].locator;
    },
    getQName: function(h) {
      return this[h].qName;
    },
    getURI: function(h) {
      return this[h].uri;
    },
    getValue: function(h) {
      return this[h].value;
    }
    //	,getIndex:function(uri, localName)){
    //		if(localName){
    //
    //		}else{
    //			var qName = uri
    //		}
    //	},
    //	getValue:function(){return this.getValue(this.getIndex.apply(this,arguments))},
    //	getType:function(uri,localName){}
    //	getType:function(i){},
  }, lt.XMLReader = H, lt.parseUtils = W, lt.parseDoctypeCommentOrCData = U, lt;
}
var Ut;
function Cr() {
  if (Ut) return We;
  Ut = 1;
  var o = nt(), t = rr(), u = dt(), s = gr(), a = Tr(), n = t.DOMImplementation, l = o.hasDefaultHTMLNamespace, g = o.isHTMLMimeType, b = o.isValidMimeType, O = o.MIME_TYPE, A = o.NAMESPACE, f = u.ParseError, M = a.XMLReader;
  function S(m) {
    return m.replace(/\r[\n\u0085]/g, `
`).replace(/[\r\u0085\u2028\u2029]/g, `
`);
  }
  function k(m) {
    if (m = m || {}, m.locator === void 0 && (m.locator = !0), this.assign = m.assign || o.assign, this.domHandler = m.domHandler || Y, this.onError = m.onError || m.errorHandler, m.errorHandler && typeof m.errorHandler != "function")
      throw new TypeError("errorHandler object is no longer supported, switch to onError!");
    m.errorHandler && m.errorHandler("warning", "The `errorHandler` option has been deprecated, use `onError` instead!", this), this.normalizeLineEndings = m.normalizeLineEndings || S, this.locator = !!m.locator, this.xmlns = this.assign(/* @__PURE__ */ Object.create(null), m.xmlns);
  }
  k.prototype.parseFromString = function(m, N) {
    if (!b(N))
      throw new TypeError('DOMParser.parseFromString: the provided mimeType "' + N + '" is not valid.');
    var P = this.assign(/* @__PURE__ */ Object.create(null), this.xmlns), G = s.XML_ENTITIES, W = P[""] || null;
    l(N) ? (G = s.HTML_ENTITIES, W = A.HTML) : N === O.XML_SVG_IMAGE && (W = A.SVG), P[""] = W, P.xml = P.xml || A.XML;
    var v = new this.domHandler({
      mimeType: N,
      defaultNamespace: W,
      onError: this.onError
    }), U = this.locator ? {} : void 0;
    this.locator && v.setDocumentLocator(U);
    var C = new M();
    C.errorHandler = v, C.domBuilder = v;
    var L = !o.isHTMLMimeType(N);
    return L && typeof m != "string" && C.errorHandler.fatalError("source is not a string"), C.parse(this.normalizeLineEndings(String(m)), P, G), v.doc.documentElement || C.errorHandler.fatalError("missing root element"), v.doc;
  };
  function Y(m) {
    var N = m || {};
    this.mimeType = N.mimeType || O.XML_APPLICATION, this.defaultNamespace = N.defaultNamespace || null, this.cdata = !1, this.currentElement = void 0, this.doc = void 0, this.locator = void 0, this.onError = N.onError;
  }
  function te(m, N) {
    N.lineNumber = m.lineNumber, N.columnNumber = m.columnNumber;
  }
  Y.prototype = {
    /**
     * Either creates an XML or an HTML document and stores it under `this.doc`.
     * If it is an XML document, `this.defaultNamespace` is used to create it,
     * and it will not contain any `childNodes`.
     * If it is an HTML document, it will be created without any `childNodes`.
     *
     * @see http://www.saxproject.org/apidoc/org/xml/sax/ContentHandler.html
     */
    startDocument: function() {
      var m = new n();
      this.doc = g(this.mimeType) ? m.createHTMLDocument(!1) : m.createDocument(this.defaultNamespace, "");
    },
    startElement: function(m, N, P, G) {
      var W = this.doc, v = W.createElementNS(m, P || N), U = G.length;
      j(this, v), this.currentElement = v, this.locator && te(this.locator, v);
      for (var C = 0; C < U; C++) {
        var m = G.getURI(C), L = G.getValue(C), P = G.getQName(C), h = W.createAttributeNS(m, P);
        this.locator && te(G.getLocator(C), h), h.value = h.nodeValue = L, v.setAttributeNode(h);
      }
    },
    endElement: function(m, N, P) {
      this.currentElement = this.currentElement.parentNode;
    },
    startPrefixMapping: function(m, N) {
    },
    endPrefixMapping: function(m) {
    },
    processingInstruction: function(m, N) {
      var P = this.doc.createProcessingInstruction(m, N);
      this.locator && te(this.locator, P), j(this, P);
    },
    ignorableWhitespace: function(m, N, P) {
    },
    characters: function(m, N, P) {
      if (m = H.apply(this, arguments), m) {
        if (this.cdata)
          var G = this.doc.createCDATASection(m);
        else
          var G = this.doc.createTextNode(m);
        this.currentElement ? this.currentElement.appendChild(G) : /^\s*$/.test(m) && this.doc.appendChild(G), this.locator && te(this.locator, G);
      }
    },
    skippedEntity: function(m) {
    },
    endDocument: function() {
      this.doc.normalize();
    },
    /**
     * Stores the locator to be able to set the `columnNumber` and `lineNumber`
     * on the created DOM nodes.
     *
     * @param {Locator} locator
     */
    setDocumentLocator: function(m) {
      m && (m.lineNumber = 0), this.locator = m;
    },
    //LexicalHandler
    comment: function(m, N, P) {
      m = H.apply(this, arguments);
      var G = this.doc.createComment(m);
      this.locator && te(this.locator, G), j(this, G);
    },
    startCDATA: function() {
      this.cdata = !0;
    },
    endCDATA: function() {
      this.cdata = !1;
    },
    startDTD: function(m, N, P, G) {
      var W = this.doc.implementation;
      if (W && W.createDocumentType) {
        var v = W.createDocumentType(m, N, P, G);
        this.locator && te(this.locator, v), j(this, v), this.doc.doctype = v;
      }
    },
    reportError: function(m, N) {
      if (typeof this.onError == "function")
        try {
          this.onError(m, N, this);
        } catch (P) {
          throw new f("Reporting " + m + ' "' + N + '" caused ' + P, this.locator);
        }
      else
        console.error("[xmldom " + m + "]	" + N, x(this.locator));
    },
    /**
     * @see http://www.saxproject.org/apidoc/org/xml/sax/ErrorHandler.html
     */
    warning: function(m) {
      this.reportError("warning", m);
    },
    error: function(m) {
      this.reportError("error", m);
    },
    /**
     * This function reports a fatal error and throws a ParseError.
     *
     * @param {string} message
     * - The message to be used for reporting and throwing the error.
     * @returns {never}
     * This function always throws an error and never returns a value.
     * @throws {ParseError}
     * Always throws a ParseError with the provided message.
     */
    fatalError: function(m) {
      throw this.reportError("fatalError", m), new f(m, this.locator);
    }
  };
  function x(m) {
    if (m)
      return `
@#[line:` + m.lineNumber + ",col:" + m.columnNumber + "]";
  }
  function H(m, N, P) {
    return typeof m == "string" ? m.substr(N, P) : m.length >= N + P || N ? new java.lang.String(m, N, P) + "" : m;
  }
  "endDTD,startEntity,endEntity,attributeDecl,elementDecl,externalEntityDecl,internalEntityDecl,resolveEntity,getExternalSubset,notationDecl,unparsedEntityDecl".replace(
    /\w+/g,
    function(m) {
      Y.prototype[m] = function() {
        return null;
      };
    }
  );
  function j(m, N) {
    m.currentElement ? m.currentElement.appendChild(N) : m.doc.appendChild(N);
  }
  function re(m) {
    if (m === "error") throw "onErrorStopParsing";
  }
  function ie() {
    throw "onWarningStopParsing";
  }
  return We.__DOMHandler = Y, We.DOMParser = k, We.normalizeLineEndings = S, We.onErrorStopParsing = re, We.onWarningStopParsing = ie, We;
}
var kt;
function br() {
  if (kt) return J;
  kt = 1;
  var o = nt();
  J.assign = o.assign, J.hasDefaultHTMLNamespace = o.hasDefaultHTMLNamespace, J.isHTMLMimeType = o.isHTMLMimeType, J.isValidMimeType = o.isValidMimeType, J.MIME_TYPE = o.MIME_TYPE, J.NAMESPACE = o.NAMESPACE;
  var t = dt();
  J.DOMException = t.DOMException, J.DOMExceptionName = t.DOMExceptionName, J.ExceptionCode = t.ExceptionCode, J.ParseError = t.ParseError;
  var u = rr();
  J.Attr = u.Attr, J.CDATASection = u.CDATASection, J.CharacterData = u.CharacterData, J.Comment = u.Comment, J.Document = u.Document, J.DocumentFragment = u.DocumentFragment, J.DocumentType = u.DocumentType, J.DOMImplementation = u.DOMImplementation, J.Element = u.Element, J.Entity = u.Entity, J.EntityReference = u.EntityReference, J.LiveNodeList = u.LiveNodeList, J.NamedNodeMap = u.NamedNodeMap, J.Node = u.Node, J.NodeList = u.NodeList, J.Notation = u.Notation, J.ProcessingInstruction = u.ProcessingInstruction, J.Text = u.Text, J.XMLSerializer = u.XMLSerializer;
  var s = Cr();
  return J.DOMParser = s.DOMParser, J.normalizeLineEndings = s.normalizeLineEndings, J.onErrorStopParsing = s.onErrorStopParsing, J.onWarningStopParsing = s.onWarningStopParsing, J;
}
var qt = br();
class yr {
  constructor({ xml: t }) {
    this.parent = null, this.child = null, this.minval = NaN, this.maxval = NaN, this.origin = new ut(), this.name = t.getAttribute($.Name) ?? "unknown_name", this.type = t.getAttribute($.Type);
    const u = t.getElementsByTagName($.Parent);
    u.length > 0 && (this.parent = u[0].getAttribute($.Link));
    const s = t.getElementsByTagName($.Child);
    s.length > 0 && (this.child = s[0].getAttribute($.Link));
    const a = t.getElementsByTagName($.Limit);
    a.length > 0 && (this.minval = parseFloat(a[0].getAttribute($.Lower) ?? "NaN"), this.maxval = parseFloat(a[0].getAttribute($.Upper) ?? "NaN"));
    const n = t.getElementsByTagName($.Origin);
    n.length > 0 && (this.origin = bt(n[0]));
  }
}
class Nr {
  constructor({ xml: t, string: u }) {
    this.materials = {}, this.links = {}, this.joints = {};
    let s = t;
    if (u && (s = new qt.DOMParser().parseFromString(u, qt.MIME_TYPE.XML_TEXT).documentElement), !s)
      throw new Error("No URDF document parsed!");
    this.name = s.getAttribute($.Name);
    const a = s.childNodes;
    for (const n of a)
      if (yt(n))
        switch (n.tagName) {
          case "material": {
            const l = new Ct({ xml: n });
            if (!Object.hasOwn(this.materials, l.name)) {
              this.materials[l.name] = l;
              break;
            }
            this.materials[l.name].isLink() ? this.materials[l.name].assign(l) : console.warn(`Material ${l.name} is not unique.`);
            break;
          }
          case "link": {
            const l = new er({ xml: n });
            if (Object.hasOwn(this.links, l.name)) {
              console.warn(`Link ${l.name} is not unique.`);
              break;
            }
            for (const g of l.visuals) {
              const b = g.material;
              b?.name && (Object.hasOwn(this.materials, b.name) ? g.material = this.materials[b.name] : this.materials[b.name] = b);
            }
            this.links[l.name] = l;
            break;
          }
          case "joint": {
            const l = new yr({ xml: n });
            this.joints[l.name] = l;
            break;
          }
        }
  }
}
const wr = /* @__PURE__ */ Object.freeze(/* @__PURE__ */ Object.defineProperty({
  __proto__: null,
  UrdfAttrs: $,
  UrdfBox: Qt,
  UrdfColor: Wt,
  UrdfCylinder: Jt,
  UrdfLink: er,
  UrdfMaterial: Ct,
  UrdfMesh: Zt,
  UrdfModel: Nr,
  UrdfSphere: $t,
  UrdfType: it,
  UrdfVisual: Kt,
  isElement: yt,
  parseUrdfOrigin: bt
}, Symbol.toStringTag, { value: "Module" })), _r = "1.4.1", Sr = {
  REVISION: _r,
  ...pr,
  ...dr,
  ...Er,
  ...vr,
  ...wr
};
globalThis.ROSLIB = Sr;
export {
  Xt as A,
  Ht as G,
  zt as P,
  Ue as Q,
  _r as R,
  se as S,
  me as T,
  Qt as U,
  Me as V,
  ur as a,
  fr as b,
  Tt as c,
  mr as d,
  jt as e,
  ut as f,
  xr as g,
  ht as h,
  Yt as i,
  Dr as j,
  Wt as k,
  Jt as l,
  er as m,
  Ct as n,
  Zt as o,
  Nr as p,
  $t as q,
  Kt as r,
  $ as s,
  it as t,
  yt as u,
  bt as v
};
