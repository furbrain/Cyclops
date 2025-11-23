//connect all .ros-service buttons to Trigger services
const btns_service = document.querySelectorAll('button.ros-service');
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
