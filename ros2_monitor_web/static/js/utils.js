function setupSidebarToggle() {
    const sidebarToggle = document.getElementById('sidebarToggle');
    if (sidebarToggle) {
        sidebarToggle.addEventListener('click', function() {
            const sidebar = document.querySelector('.sidebar');
            sidebar.classList.toggle('collapsed');
        });
    }
}
document.addEventListener("DOMContentLoaded", function() {
    // 连接到SocketIO服务器
    var socket = io.connect('http://localhost:5000');

    // 定义处理数据的函数
    function processData(data) {
        // 获取当前时间戳
        var currentTime = new Date().getTime();

        // 使用接收到的current_time字段计算延迟
        var delay = currentTime - data.current_time;

        // 打印到console
        console.log("Delay: " + delay + " ms");

        // 如果需要，你还可以在这里处理data中的其他字段，比如:
        console.log("Total Nodes: " + data.total_nodes);
        console.log("Total topics: "+ data.total_topics);
        document.getElementById('total-topics').innerText = data.total_topics;
        document.getElementById('total-nodes').innerText = data.total_nodes;
        document.getElementById('total-memory').innerText = data.total_memory_usage + " MB";
        document.getElementById('average-latency').innerText = delay + "ms";
    }

    // 监听 'test_response' 事件
    socket.on('test_response', processData);
});
