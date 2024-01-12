POLL_INTERVAL = 1 * 1000;

async function updateDrones() {
    let drone_states;
    try {
        const res = await fetch("/api/info");
        drone_states = await res.json();
    } catch(e) {
        console.error("updateDrones: Poll error:", e);
        setTimeout(updateDrones, POLL_INTERVAL);
        return;
    }

    // Update based on states
    let table = document.createElement("table");
    // 1. Title row
    title_row = table.insertRow(0);
    title_row.insertCell(0).innerHTML = "Drone";
    title_row.insertCell(1).innerHTML = "Mode";
    title_row.insertCell(2).innerHTML = "Battery Percentage";
    title_row.insertCell(3).innerHTML = "Estimated RTT";
    title_row.insertCell(4).innerHTML = "Position";
    title_row.insertCell(5).innerHTML = "Last Command";

    let curRow = 1;

    // 2. Create rows
    for (const [drone_id, drone_str] of Object.entries(drone_states)) {
        let drone_state = JSON.parse(drone_str)
        let row = table.insertRow(curRow);
        row.insertCell(0).innerHTML = drone_id;
        row.insertCell(1).innerHTML = drone_state["mode"];
        row.insertCell(2).innerHTML = drone_state["battery_percentage"] + "%";
        row.insertCell(3).innerHTML = drone_state["estimated_rtt"];
        row.insertCell(4).innerHTML = drone_state["lat"] + "," + drone_state["lon"];
        row.insertCell(5).innerHTML = drone_state["last_command"];
    }

    document.getElementById("drone_listing").innerHTML = "";
    document.getElementById("drone_listing").appendChild(table);
    setTimeout(updateDrones, POLL_INTERVAL);
}

window.onload = function(e) {
    updateDrones();
};