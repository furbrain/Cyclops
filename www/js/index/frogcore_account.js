// FrogCore account: credentials are stored on the device; the password is never sent back
function openFrogcoreModal() {
    document.getElementById("frogcoreError").innerText = "";
    document.getElementById("frogcorePassword").value = "";
    fetch("/api/frogcore/account")
        .then(response => response.json())
        .then(data => {
            document.getElementById("frogcoreUsername").value = data.username;
            document.getElementById("frogcorePassword").placeholder =
                data.has_password ? "(unchanged)" : "";
        });
    bootstrap.Modal.getOrCreateInstance(document.getElementById("frogcoreModal")).show();
}

function handleFrogcoreSave() {
    const account = {
        username: document.getElementById("frogcoreUsername").value,
        password: document.getElementById("frogcorePassword").value,
    };
    document.getElementById("frogcoreError").innerText = "Checking login...";
    fetch("/api/frogcore/account", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(account),
    })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                bootstrap.Modal.getInstance(document.getElementById("frogcoreModal")).hide();
            } else {
                document.getElementById("frogcoreError").innerText = data.message;
            }
        });
}
