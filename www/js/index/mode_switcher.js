var current_mode = "";
const mode_observer = new IntersectionObserver((entries) => {
  entries.forEach(entry => {
    if (entry.isIntersecting) {
      console.debug('Pane is now visible: ' + entry.target.id);
      let mode = entry.target.dataset.mode;
      if (current_mode != mode) {
        serv = new ROSLIB.Service({
            ros: ros,
            name: '/set_mode',
            type: "cyclops_interfaces/SetMode",
        });
        serv.callService({'mode': mode},
                function(response) {
                    if (!response.success) {
                        alert("Call to switch mode " + mode + " failed");
                    }
                },
                function(err) {
                    alert("Error during call to switch to " + mode + ": " + err);
                });
        current_mode = mode;
      }
    }
  });
});

const panes = document.querySelectorAll('div.tab-pane');
panes.forEach(function(element) {
    mode_observer.observe(element);
});
