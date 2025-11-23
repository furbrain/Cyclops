//connect all .calibrate buttons to Calibrate Actions
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
