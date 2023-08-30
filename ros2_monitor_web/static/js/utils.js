function setupSidebarToggle() {
    const sidebarToggle = document.getElementById('sidebarToggle');
    if (sidebarToggle) {
        sidebarToggle.addEventListener('click', function() {
            const sidebar = document.querySelector('.sidebar');
            sidebar.classList.toggle('collapsed');
        });
    }
}

function calculateAndDisplayLatency(data) {
    const serverTime = new Date(data.timestamp);
    const clientTime = new Date();
    const latency = clientTime - serverTime;

    console.log(`Latency is: ${latency}ms`);
}

function displayReceivedData(data) {
    console.log("Received data:", data);
    document.getElementById('total-topics').innerText = data.total_topics;
    document.getElementById('total-nodes').innerText = data.total_nodes;
    document.getElementById('total-memory').innerText = data.total_memory_usage + " MB";
}
