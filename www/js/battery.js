// Battery indicator: a small badge in the top right corner of the page showing the device's state of
// charge, from the /battery topic (sensor_msgs/BatteryState, published by ROS2/cyclops/cyclops/battery_status.py)
// via rosbridge. Include it on any page with <script src="/js/battery.js"></script>; it loads roslib itself
// if the page hasn't.
(() => {
  const STALE_MS = 10000; // grey the badge out if nothing has arrived for this long
  const CHARGING_AMPS = 0.02; // current above this counts as charging (positive is into the battery)

  const badge = document.createElement("span");
  badge.id = "battery-indicator";
  badge.className = "badge text-bg-secondary";
  badge.textContent = "🔋 —";
  badge.title = "Battery: no data yet";
  Object.assign(badge.style, {
    position: "fixed", top: "0.5rem", right: "0.5rem", zIndex: 1080,
    fontSize: "0.85rem", pointerEvents: "auto",
  });
  const addBadge = () => document.body.appendChild(badge);
  if (document.body) addBadge(); else document.addEventListener("DOMContentLoaded", addBadge);

  let lastUpdate = 0;

  function show(msg) {
    lastUpdate = Date.now();
    // BatteryState.percentage is 0-1, but allow for a publisher that sends 0-100
    let pct = msg.percentage;
    if (pct == null || Number.isNaN(pct)) return;
    if (pct <= 1.0) pct *= 100;
    pct = Math.max(0, Math.min(100, Math.round(pct)));
    const charging = msg.current > CHARGING_AMPS;
    badge.textContent = `${charging ? "⚡" : "🔋"} ${pct}%`;
    badge.className = "badge " +
      (charging ? "text-bg-info" : pct <= 15 ? "text-bg-danger" : pct <= 30 ? "text-bg-warning" : "text-bg-success");
    const details = [`${pct}%`];
    if (msg.voltage) details.push(`${msg.voltage.toFixed(2)} V`);
    if (msg.current != null) details.push(`${msg.current.toFixed(2)} A`);
    if (msg.temperature) details.push(`${msg.temperature.toFixed(1)} °C`);
    badge.title = `Battery${charging ? " (charging)" : ""}: ${details.join(", ")}`;
  }

  function markStale(reason) {
    badge.className = "badge text-bg-secondary";
    badge.textContent = badge.textContent.replace(/^⚡/, "🔋");
    badge.title = `Battery: ${reason}`;
  }

  setInterval(() => {
    if (lastUpdate && Date.now() - lastUpdate > STALE_MS) markStale("no recent data");
  }, 2000);

  function loadScript(src) {
    return new Promise((resolve, reject) => {
      const s = document.createElement("script");
      s.src = src;
      s.onload = resolve;
      s.onerror = () => reject(new Error(`couldn't load ${src}`));
      document.head.appendChild(s);
    });
  }

  function connect() {
    const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });
    ros.on("error", () => markStale("can't reach ROS (rosbridge)"));
    ros.on("close", () => {
      markStale("disconnected from ROS");
      setTimeout(connect, 5000); // rosbridge may not be up yet, or restarted
    });
    ros.on("connection", () => {
      new ROSLIB.Topic({
        ros, name: "/battery", messageType: "sensor_msgs/msg/BatteryState", throttle_rate: 2000,
      }).subscribe(show);
    });
  }

  (window.ROSLIB ? Promise.resolve() : loadScript("/js/roslib.js"))
    .then(connect)
    .catch((err) => markStale(err.message));
})();
