// Refresh table
function reloadRecordings() {
    console.log("starting fetch");
    fetch('/api/getRecordings')
      .then(response => response.text())
      .then(data => document.getElementById("recordings_holder").innerHTML=data);

}

// Open modal and set fields
function openRenameModal(name) {
    document.getElementById("currentFileName").value = name;
    document.getElementById("newFileName").value = name;

    const modal = new bootstrap.Modal(document.getElementById("renameModal"));
    modal.show();
}

// Handle rename (hook into backend here)
function handleRename() {
    const currentName = document.getElementById("currentFileName").value;
    const newName = document.getElementById("newFileName").value;
    console.log("Renaming " + currentName + " to " + newName);

    // Close modal
    bootstrap.Modal.getInstance(document.getElementById("renameModal")).hide();
    fetch("/api/rename/"+ currentName + "/" + newName)
        .then(response => response.json())
        .then(data => {
            if (data.success == false) {
                alert("Rename failed")
            }
        });
    reloadRecordings();
}

function openDeleteModal(name) {
    document.getElementById("deleteFileName").innerHTML = name;
    const modal = new bootstrap.Modal(document.getElementById("deleteModal"));
    modal.show();
}

// Handle rename (hook into backend here)
function handleDelete() {
    const name = document.getElementById("deleteFileName").innerHTML;
    console.log("Deleting " + name);

    // Close modal
    bootstrap.Modal.getInstance(document.getElementById("deleteModal")).hide();
    fetch("/api/delete/" + name)
        .then(response => response.json())
        .then(data => {
            if (data.success == false) {
                alert("Delete failed")
            }
        });
    reloadRecordings();
}

const logger = document.getElementById("recording_log");

function clearLog() {
    logger.innerHTML = "";
}

// Helper: check whether the user is "near" the bottom
function isNearBottom(el, threshold = 50) {
    return (el.scrollHeight - el.scrollTop - el.clientHeight) <= threshold;
}


// Append a message and optionally auto-scroll
function addMessage(text) {
    const wasAtBottom = isNearBottom(logger);
    const wrapper = document.createElement('div');
    wrapper.className = 'log-item';
    const textSpan = document.createElement('span');
    textSpan.textContent = text;
    wrapper.appendChild(textSpan);
    logger.appendChild(wrapper);

    // If Auto-scroll is enabled and user was at bottom, scroll to bottom smoothly
    if (wasAtBottom) {
        // smooth scroll whooshes to bottom; change to behavior: 'auto' to disable animation
        logger.scrollTo({ top: logger.scrollHeight, behavior: 'smooth' });
    }
}

const btns_make_model = document.querySelectorAll('button.ros-make-model');
btns_make_model.forEach(function(btn) {
    btn.old_text = btn.innerText;
    btn.old_classes = btn.className;
    btn.action = new ROSLIB.Action({
        ros: ros,
        name: '/make_model',
        actionType: "cyclops_interfaces/MakeModel",
        });
    btn.onclick = function() {
        goal = {name: btn.dataset.name, style: parseInt(btn.dataset.modelStyle)};
        console.log(goal);
        btn.action.sendGoal(goal,
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
                addMessage(feedback.output);
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
