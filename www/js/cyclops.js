// create ros object
const ros = new ROSLIB.Ros({
      url : 'ws://' + window.location.hostname +':9090'
    });

// setup intersector jobbies
const observer = new IntersectionObserver((entries) => {
  entries.forEach(entry => {
    if (entry.isIntersecting) {
      console.log('Img is now visible: ' + entry.target.dataset.rosTopic);
      entry.target.src = "/vid/stream?topic=" + entry.target.dataset.rosTopic + "&qos_profile=sensor_data&quality=50"
      // Run your function here
    } else {
      console.log('Img is no longer visible: ' + entry.target.dataset.rosTopic);
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
                console.log("result received");
                if (response.success) {
                    reset_button(btn);
                    flash_button(btn,"success", response.message);
                } else {
                    flash_button(btn,"danger");
                    alert("Call to " + btn.dataset.rosAction + " failed with message: " + response.message);
                }
            },
            function(feedback) {
                console.log("feedback received");
                flash_button(btn, "warning", feedback.interim_message, timeout=0);
            },
            function(err) {
                console.log("error received");
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
