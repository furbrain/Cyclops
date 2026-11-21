//Needs to be sourced first
// create ros object
const ros = new ROSLIB.Ros({
      // via Caddy's /rosbridge proxy, so it's wss:// when the page is served over https
      url : (window.location.protocol === 'https:' ? 'wss://' : 'ws://') + window.location.host + '/rosbridge'
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

