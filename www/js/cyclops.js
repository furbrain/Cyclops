// create ros object
const ros = new ROSLIB.Ros({
      url : 'ws://' + window.location.hostname +':9090'
    });

// setup intersector jobbies
const observer = new IntersectionObserver((entries) => {
  entries.forEach(entry => {
    if (entry.isIntersecting) {
      console.debug('Img is now visible: ' + entry.target.dataset.rosTopic);
      entry.target.src = "/vid/stream?topic=" + entry.target.dataset.rosTopic + "&qos_profile=sensor_data&quality=50"
      // Run your function here
    } else {
      console.debug('Img is no longer visible: ' + entry.target.dataset.rosTopic);
      entry.target.src = "#"
    }
  });
});

// make video feeds dynamic
const imgs = document.querySelectorAll('.ros-img');
imgs.forEach(function(element) {
    observer.observe(element);
});

//flash a button to a particular btn-"cls", then revert after timeout ms
function flash_button(btn, cls, msg="", timeout=5000) {
    if (msg) {
        btn.innerText = msg;
    }
    btn.classList.remove.apply(btn.classList, Array.from(btn.classList).filter(v=>v.startsWith("btn-")));
    btn.classList.add("btn-" + cls);
    if (timeout>0) {
        setTimeout(function() {
            reset_button(btn);
        }, timeout);
    }
}

function reset_button(btn) {
    btn.className = btn.old_classes;
    btn.innerText = btn.old_text;
}


//connect all .ros-service buttons to Trigger services
const btns_service = document.querySelectorAll('button.ros_service');
btns_service.forEach(function(btn) {
    btn.old_text = btn.innerText;
    btn.old_classes = btn.className;
    btn.service = new ROSLIB.Service({
        ros: ros,
        name: btn.dataset.rosService,
        type: "std_srvs/Trigger",
        });
    btn.onclick = function() {
        btn.service.callService({},
            function(response) {
                if (response.success) {
                    flash_button(btn,"success", response.message);
                } else {
                    flash_button(btn,"warning");
                    alert("Call to " + btn.dataset.rosService + " failed with message: " + response.message);
                }
            },
            function(err) {
                flash_button(btn,"danger");
                alert("Error during call to " + btn.dataset.rosService + ": " + err);                
            }
        );
    };
});

//connect all .ros-action buttons to Trigger services
const btns_action = document.querySelectorAll('button.ros-action');
btns_action.forEach(function(btn) {
    btn.old_text = btn.innerText;
    btn.old_classes = btn.className;
    btn.action = new ROSLIB.Action({
        ros: ros,
        name: btn.dataset.rosAction,
        actionType: "cyclops_interfaces/Calibrate",
        });
    btn.onclick = function() {
        btn.action.sendGoal({},
            function(response) {
                console.debug("action result received");
                if (response.success) {
                    reset_button(btn);
                    flash_button(btn,"success", response.message);
                } else {
                    flash_button(btn,"danger");
                    alert("Call to " + btn.dataset.rosAction + " failed with message: " + response.message);
                }
            },
            function(feedback) {
                console.debug("action feedback received");
                flash_button(btn, "warning", feedback.interim_message, timeout=0);
            },
            function(err) {
                console.log("action error received");
                reset_button(btn);
                flash_button(btn,"danger");
                alert("Error during call to " + btn.dataset.rosAction + ": " + err);
        });
    };
    btn.end_func = function() {
    }
});


//connect all .ros-action buttons to Trigger services
const cal_btns = document.querySelectorAll('button.calibrate');
cal_btns.forEach(function(btn) {
    btn.old_text = btn.innerText;
    btn.old_classes = btn.className;
    btn.action = new ROSLIB.Action({
        ros: ros,
        name: btn.dataset.namespace + '/do_calibration',
        actionType: "cyclops_interfaces/Calibrate",
        });
    btn.service = new ROSLIB.Service({
        ros: ros,
        name: btn.dataset.namespace + '/finalise_calibration',
        type: "std_srvs/Trigger",
        });
    btn.mode = "start";
    console.log("setting up button");
    btn.onclick = function() {
        if (btn.mode=="start") {
            btn.mode = "finish";
            flash_button(btn, "warning", "Starting calibration", timeout=0)
            btn.action.sendGoal({},
                function(response) {
                    console.log("result received");
                    if (response.success) {
                        reset_button(btn);
                        flash_button(btn,"success", response.message);
                    } else {
                        reset_button(btn);
                        flash_button(btn,"danger");
                        alert("Call to " + btn.dataset.rosAction + " failed with message: " + response.message);
                    }
                    btn.mode = "start";
                },
                function(feedback) {
                    console.log("feedback received");
                    btn.innerText = "Progress: " + feedback.count;
                },
                function(err) {
                    btn.mode = "start";
                    console.log("error received");
                    reset_button(btn);
                    flash_button(btn,"danger");
                    alert("Error during call to " + btn.dataset.rosAction + ": " + err);
            })
        } else if (btn.mode=="finish") {
            btn.mode = "start";
            btn.service.callService({},
                function(response) {
                    if (response.success) {
                        flash_button(btn, "warning", "Running Calibration");
                    } else {
                        flash_button(btn,"warning");
                        alert("Call to " + btn.dataset.rosService + " failed with message: " + response.message);
                    }
                },
                function(err) {
                    flash_button(btn,"danger");
                    alert("Error during call to " + btn.dataset.rosService + ": " + err);
                }
            );
        };
    };
});


// setup 3d viewer
// Create the main viewer.
var viewer = new ROS3D.Viewer({
  divID : 'viewer',
  width : 800,
  height : 600,
  antialias : true
});

// Setup a client to listen to TFs.
var tfClient = new ROSLIB.ROS2TFClient({
  ros : ros,
  angularThres : 0.01,
  transThres : 0.01,
  rate : 10.0,
  fixedFrame : '/map'
});

var cloudClient = new ROS3D.PointCloud2({
      ros: ros,
      tfClient: tfClient,
      rootObject: viewer.scene,
      topic: '/imu/cal/all_scaled_mag_points',
      material: { size: 1.0, color: 0xff00ff }
    });

var markerClient = new ROS3D.MarkerClient({
    ros: ros,
    tfClient: tfClient,
    rootObject: viewer.scene,
    topic: "/imu/cal/marker",
});