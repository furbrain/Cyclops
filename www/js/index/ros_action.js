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
