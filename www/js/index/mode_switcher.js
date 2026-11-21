// Tracks the switcher node's mode (published on /mode) and keeps the UI in step.
//
// Only the Calibrate panes need a particular mode, so entering one switches to
// "calibrate" and leaving it resets. Capture is started and stopped explicitly
// with the buttons on the Capture tab (or the hardware button), and keeps
// running while other non-calibrate tabs are viewed.
var current_mode = "";      // mode as last reported by the switcher
var pane_mode = "";         // data-mode of the pane currently shown
var mode_call_pending = false;

const set_mode_service = new ROSLIB.Service({
    ros: ros,
    name: '/set_mode',
    type: "tactical_frog/SetMode",
});

function request_mode(mode, on_done) {
    mode_call_pending = true;
    update_capture_controls();
    set_mode_service.callService({'mode': mode},
        function(response) {
            mode_call_pending = false;
            if (!response.success) {
                alert("Call to switch mode to " + mode + " failed");
            }
            update_capture_controls();
            if (on_done) on_done(response.success);
        },
        function(err) {
            mode_call_pending = false;
            alert("Error during call to switch to " + mode + ": " + err);
            update_capture_controls();
            if (on_done) on_done(false);
        });
}

function sync_mode_to_pane() {
    if (pane_mode === "calibrate") {
        if (current_mode !== "calibrate") {
            if (current_mode === "capture" &&
                !confirm("A capture is in progress. Stop it and switch to calibration?")) {
                return;
            }
            request_mode("calibrate");
        }
    } else if (current_mode === "calibrate") {
        request_mode("reset");
    }
}

new ROSLIB.Topic({
    ros: ros,
    name: '/mode',
    messageType: 'std_msgs/msg/String',
}).subscribe(function(msg) {
    if (msg.data === current_mode) return;
    current_mode = msg.data;
    update_capture_controls();
});

const mode_observer = new IntersectionObserver((entries) => {
  entries.forEach(entry => {
    if (entry.isIntersecting) {
      console.debug('Pane is now visible: ' + entry.target.id);
      let mode = entry.target.dataset.mode;
      if (pane_mode != mode) {
        pane_mode = mode;
        sync_mode_to_pane();
      }
    }
  });
});

const panes = document.querySelectorAll('div.tab-pane');
panes.forEach(function(element) {
    mode_observer.observe(element);
});

// Capture tab controls
const capture_start_btn = document.getElementById('capture-start');
const capture_stop_btn = document.getElementById('capture-stop');
const capture_status = document.getElementById('capture-status');

const CAPTURE_STATUS = {
    "capture":   ["Recording", "text-bg-danger"],
    "stopping":  ["Stopping and saving…", "text-bg-warning"],
    "calibrate": ["Calibration mode", "text-bg-info"],
    "reset":     ["Idle", "text-bg-secondary"],
    "":          ["Waiting for status…", "text-bg-dark"],
};

// ORB-SLAM3 tracking state (orb_slam3/msg/State), shown while recording.
const ORB_STATUS = {
    1: ["Recording · tracking", "text-bg-danger"],       // OK
    2: ["Recording · trying to connect", "text-bg-warning"], // RECENTLY_LOST
    3: ["Recording · map lost", "text-bg-dark"],    // LOST
    4: ["Recording · not tracking", "text-bg-dark"],     // OTHER (e.g. initialising)
};
const ORB_STATE_TIMEOUT_MS = 2000;
var orb_state = null;
var orb_state_timer = null;

new ROSLIB.Topic({
    ros: ros,
    name: '/orb/ORB/state',
    messageType: 'orb_slam3/msg/State',
    throttle_rate: 200,
}).subscribe(function(msg) {
    clearTimeout(orb_state_timer);
    orb_state_timer = setTimeout(function() {
        orb_state = null;
        update_capture_controls();
    }, ORB_STATE_TIMEOUT_MS);
    if (msg.state === orb_state) return;
    orb_state = msg.state;
    update_capture_controls();
});

function update_capture_controls() {
    if (!capture_status) return;
    let [text, cls] = CAPTURE_STATUS[current_mode] || [current_mode, "text-bg-secondary"];
    if (current_mode === "capture" && orb_state !== null && ORB_STATUS[orb_state]) {
        [text, cls] = ORB_STATUS[orb_state];
    }
    capture_status.textContent = text;
    capture_status.className = "badge fs-6 " + cls;
    const busy = mode_call_pending || current_mode === "stopping" || current_mode === "";
    capture_start_btn.disabled = busy || current_mode === "capture";
    capture_stop_btn.disabled = busy || current_mode !== "capture";
}

if (capture_start_btn) {
    capture_start_btn.onclick = function() { request_mode("capture"); };
    capture_stop_btn.onclick = function() { request_mode("reset"); };
    update_capture_controls();
}
