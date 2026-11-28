// Software update: the "Software update" section of the Settings tab, and an alert at the top of the page when
// a new release can be installed. Updates are only offered with internet access and the battery charging or
// over 50%; /api/update/status (www/py/update.py) says whether that's so, and the updater checks it again.
const UPDATE_CHECK_MS = 5 * 60 * 1000;
const UPDATE_PROGRESS_MS = 3000;
let updateInfo = null;
let updateAlertDismissed = false;

function updateEl(tag, text, className) {
    const el = document.createElement(tag);
    if (text != null) el.textContent = text;
    if (className) el.className = className;
    return el;
}

function describeInstalled(item) {
    if (!item || !item.version) return "not installed";
    return `${item.version} (${item.commit.slice(0, 10)})`;
}

function showUpdateInfo(info) {
    updateInfo = info;
    const rows = document.getElementById("update-versions");
    const message = document.getElementById("update-message");
    const button = document.getElementById("update-button");
    if (!rows) return;
    rows.replaceChildren();
    for (const [name, item] of Object.entries(info.installed || {})) {
        const row = updateEl("tr");
        row.append(updateEl("td", name), updateEl("td", describeInstalled(item)));
        if (info.repair && info.repair.includes(name)) {
            row.lastChild.append(updateEl("span", " not the one this TacticalFrog release needs", "text-warning"));
        }
        rows.append(row);
    }
    const latest = updateEl("tr");
    latest.append(updateEl("td", "Latest release"),
        updateEl("td", info.latest || (info.online ? "none published" : "unknown (offline)")));
    rows.append(latest);
    if (info.battery) {
        const battery = updateEl("tr");
        battery.append(updateEl("td", "Battery"),
            updateEl("td", `${info.battery.percent}%${info.battery.charging ? ", charging" : ""}`));
        rows.append(battery);
    }

    const wanted = info.update_available || (info.repair && info.repair.length > 0);
    button.textContent = info.update_available ? `Update to ${info.latest}` :
        wanted ? `Reinstall ${info.latest}` : "Update";
    button.disabled = !(wanted && info.allowed);
    if (info.running) {
        message.className = "small mb-2 text-info";
        message.textContent = (info.status && info.status.message) || "Updating...";
    } else if (info.status && info.status.state === "failed") {
        message.className = "small mb-2 text-danger";
        message.textContent = `${info.status.message}. ${wanted && !info.allowed ? `Can't update now: ${info.reason}` : ""}`;
    } else if (wanted && !info.allowed) {
        message.className = "small mb-2 text-warning";
        message.textContent = `An update is available, but can't be installed now: ${info.reason}.`;
    } else {
        message.className = "small mb-2 text-secondary";
        message.textContent = info.reason ? `${info.reason[0].toUpperCase()}${info.reason.slice(1)}.` : "";
    }
    showUpdateAlert(info, wanted);
}

function showUpdateAlert(info, wanted) {
    const holder = document.getElementById("update-alert");
    if (!holder) return;
    holder.replaceChildren();
    // only when it can be installed now: online, and the battery charging or over 50%
    if (!wanted || !info.allowed || info.running || updateAlertDismissed) return;
    const alert = updateEl("div", null, "alert alert-info alert-dismissible d-flex align-items-center gap-2 my-2 py-2");
    alert.append(updateEl("span", info.update_available ?
        `TacticalFrog ${info.latest} is available.` : "orb_slam3 or OpenMVS doesn't match this TacticalFrog release."));
    const open = updateEl("button", "Update in Settings", "btn btn-sm btn-primary");
    open.onclick = () => {
        const tab = document.getElementById("settings-tab");
        if (tab) bootstrap.Tab.getOrCreateInstance(tab).show();
    };
    const close = updateEl("button", null, "btn-close");
    close.type = "button";
    close.setAttribute("aria-label", "Close");
    close.onclick = () => { updateAlertDismissed = true; holder.replaceChildren(); };
    alert.append(open, close);
    holder.append(alert);
}

function showUpdateError(text) {
    const message = document.getElementById("update-message");
    if (message) {
        message.className = "small mb-2 text-danger";
        message.textContent = text;
    }
}

function checkForUpdate(fresh = false) {
    return fetch(`/api/update/status${fresh ? "?refresh=1" : ""}`)
        .then(response => response.json())
        .then(info => {
            if (info.error) throw new Error(info.error);
            showUpdateInfo(info);
            if (info.running) watchUpdate();
        })
        .catch(err => showUpdateError(`Couldn't check for updates: ${err.message}`));
}

function startUpdate() {
    if (!updateInfo) return;
    const target = updateInfo.latest;
    if (!confirm(`Update to TacticalFrog ${target}? Recording and the web page stop for a minute while it restarts.`)) {
        return;
    }
    document.getElementById("update-button").disabled = true;
    fetch("/api/update/start", { method: "POST" })
        .then(response => response.json())
        .then(data => {
            if (!data.success) throw new Error(data.message);
            showUpdateAlert({}, false);
            watchUpdate();
        })
        .catch(err => { showUpdateError(err.message); checkForUpdate(); });
}

let updateWatcher = null;

function watchUpdate() {
    if (updateWatcher) return;
    const message = document.getElementById("update-message");
    let sawRunning = false;
    updateWatcher = setInterval(() => {
        fetch("/api/update/status?progress=1")
            .then(response => response.json())
            .then(progress => {
                if (progress.running) sawRunning = true;
                if (progress.status && message) {
                    message.className = "small mb-2 " +
                        (progress.status.state === "failed" ? "text-danger" : "text-info");
                    message.textContent = progress.status.message;
                }
                if (!progress.running && (sawRunning || progress.status && progress.status.state !== "running")) {
                    clearInterval(updateWatcher);
                    updateWatcher = null;
                    if (progress.status && progress.status.state === "done") {
                        // the services restart, so load the new release's page once they're back
                        message.textContent = `${progress.status.message}. Reloading...`;
                        setTimeout(() => location.reload(), 10000);
                    } else {
                        checkForUpdate(true);
                    }
                }
            })
            .catch(() => { /* the web server restarts during the update */ });
    }, UPDATE_PROGRESS_MS);
}

document.addEventListener("DOMContentLoaded", () => {
    checkForUpdate();
    setInterval(() => { if (!updateWatcher) checkForUpdate(); }, UPDATE_CHECK_MS);
});
