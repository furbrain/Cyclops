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
    var classes = btn.className;
    var old_text = btn.innerText;
    if (msg) {
        btn.innerText = msg;
    }
    btn.classList.remove.apply(btn.classList, Array.from(btn.classList).filter(v=>v.startsWith("btn-")));
    btn.classList.add("btn-" + cls);
    setTimeout(function() {
        btn.className = classes;
        btn.innerText  = old_text;
    }, timeout);
}


//connect all .ros-service buttons to Trigger services
const btns = document.querySelectorAll('button.ros-service');
btns.forEach(function(btn) {
    btn.old_text = btn.innerText;
    btn.service = new ROSLIB.Service({
        ros: ros,
        name: btn.dataset.rosService,
        type: "std_srvs/Trigger",
        });
    btn.onclick = function() {
        btn.service.callService({}, function(response) {
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
        });
    };
});
