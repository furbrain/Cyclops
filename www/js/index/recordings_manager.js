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

// --- Combined models ---------------------------------------------------------

function openCombineModal() {
    document.getElementById("combineName").value = "";
    document.getElementById("combineError").textContent = "";
    new bootstrap.Modal(document.getElementById("combineModal")).show();
}

// Creates an empty combined model: sub-surveys, links and combining are all done from its model page
function handleCombine() {
    const errorEl = document.getElementById("combineError");
    errorEl.textContent = "";
    fetch("/model/api/combine", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({name: document.getElementById("combineName").value.trim()}),
    })
        .then(response => response.json().then(data => {
            if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
            window.location.href = "/model/" + encodeURIComponent(data.model_id);
        }))
        .catch(err => {
            errorEl.textContent = err.message;
        });
}
