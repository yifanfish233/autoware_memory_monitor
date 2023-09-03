// Variables
let memoryInterval;
let memoryHistoryChart;
let top10Chart;

// Update current memory usage
const updateMemoryUsage = () => {
    const node = $('#node-selector').val();
    if (node) {
        $.ajax({
            url: `/api/memory_usage/${node}`,
            type: 'GET',
            cache: false,
            success: data => {
                const $table = $('#memory-usage-table tbody');
                $table.empty();
                $table.append(`<tr><td>${data.node}</td><td>${data.memory}</td></tr>`);
                refreshMemoryUsageHistory(node);
                refreshMemoryUsageStats(node);
            },
            error: (jqXHR, textStatus) => {
                console.error(`Memory usage request error: ${textStatus}`);
            }
        });
    }
}


// Refresh memory usage history
const refreshMemoryUsageHistory = node => {
    $.ajax({
        url: '/api/memory_usage_history/' + node,
        type: 'GET',
        cache: false,
        success: function (data) {
            var labels = data.map(function (item) {
                return item.time;
            });
            var data = data.map(function (item) {
                return item.memory;
            });
            if (memoryHistoryChart) {
                memoryHistoryChart.destroy();
            }
            var ctx = document.getElementById('memory-history-chart').getContext('2d');
            memoryHistoryChart = new Chart(ctx, {
                type: 'line',
                data: {
                    labels: labels,
                    datasets: [{
                        label: 'Memory Usage History',
                        data: data,
                        backgroundColor: 'rgba(75, 192, 192, 0.2)',
                        borderColor: 'rgba(75, 192, 192, 1)',
                        borderWidth: 1
                    }]
                }
            });
        },
        error: function (jqXHR, textStatus, errorThrown) {
            console.error('Memory usage history request error: ' + textStatus);
        }
    });
}

// Refresh memory usage stats
const refreshMemoryUsageStats = node => {
    $.ajax({
        url: '/api/memory_usage_stats/' + node,
        type: 'GET',
        cache: false,
        success: function (data) {
            $('#memory-usage-stats').text('Average: ' + data.average + ', Std dev: ' + data.stddev);
        },
        error: function (jqXHR, textStatus, errorThrown) {
            console.error('Memory usage stats request error: ' + textStatus);
        }
    });
}

// Refresh top 10 memory usage
const refreshTop10 = () => {
    $.ajax({
        url: '/api/memory_usage_top10',
        type: 'GET',
        cache: false,
        success: function (data) {
            var labels = data.map(function (item) {
                return item.node;
            });
            var data = data.map(function (item) {
                return item.memory;
            });
            if (top10Chart) {
                top10Chart.destroy();
            }
            var ctx = document.getElementById('top10-chart').getContext('2d');
            top10Chart = new Chart(ctx, {
                type: 'bar',
                data: {
                    labels: labels,
                    datasets: [{
                        label: 'Top 10 Memory Usage',
                        data: data,
                        backgroundColor: 'rgba(75, 192, 192, 0.2)',
                        borderColor: 'rgba(75, 192, 192, 1)',
                        borderWidth: 1
                    }]
                }
            });
        },
        error: function (jqXHR, textStatus, errorThrown) {
            console.error('Top 10 memory usage request error: ' + textStatus);
        }
    });
}

// $(document).ready(() => {
//     const checkROS2Status = setInterval(() => {
//         $.ajax({
//             url: '/api/ros2_status',
//             type: 'GET',
//             cache: false,
//             success: function (data) {
//                 if (data.status === 'running') {
//                     clearInterval(checkROS2Status);
//                     $('#status').text('ROS2 Autoware is running');
//                     $('#refresh-button').hide();
//
//                     $.ajax({
//                         url: '/api/nodes',
//                         type: 'GET',
//                         cache: false,
//                         success: function (nodes) {
//                             var $nodeSelector = $('#node-selector');
//                             $nodeSelector.empty();
//                             nodes.forEach(function (node) {
//                                 $nodeSelector.append('<option value="' + node + '">' + node + '</option>');
//                             });
//                         },
//                         error: function (jqXHR, textStatus, errorThrown) {
//                             console.error('Nodes request error: ' + textStatus);
//                         }
//                     });
//
//                     if (memoryInterval) {
//                         clearInterval(memoryInterval);
//                     }
//                     updateMemoryUsage();
//                     memoryInterval = setInterval(updateMemoryUsage, 1000);
//                 } else {
//                     $('#status').text('No ROS2 launched');
//                     $('#refresh-button').show();
//                 }
//             },
//             error: function (jqXHR, textStatus, errorThrown) {
//                 console.error('ROS2 status request error: ' + textStatus);
//             }
//         });
//     }, 1000);
//
//     $('#node-selector').change(() => {
//         updateMemoryUsage();
//         if (memoryInterval) {
//             clearInterval(memoryInterval);
//         }
//         memoryInterval = setInterval(updateMemoryUsage, 1000);
//     });
//
//     $('#refresh-button').click(() => {
//         $.ajax({
//             url: '/api/refresh',
//             type: 'POST',
//             success: function () {
//                 checkROS2Status();
//             },
//             error: function (jqXHR, textStatus, errorThrown) {
//                 console.error('Refresh request error: ' + textStatus);
//             }
//         });
//     });
// });

$(document).ready(function(){
    // 当用户点击下拉菜单时
    $('#node-selector').focus(function() {
        // 发送AJAX请求到Flask获取模块列表
        $.ajax({
            url: "/get_modules", // Flask 端点
            method: "GET",
            success: function(response) {
                updateDropdown(response.modules); // 当数据返回时，更新下拉菜单
            }
        });
    });
});

function updateDropdown(modules) {
    var dropdown = $("#node-selector");
    dropdown.empty(); // 清空现有数据

    $.each(modules, function(index, module) {
        dropdown.append(new Option(module, module)); // 添加新选项
    });
}
