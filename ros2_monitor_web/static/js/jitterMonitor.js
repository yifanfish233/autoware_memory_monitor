$(document).ready(function() {
    // Fetch available topics and populate the dropdown
    fetchTopics();

    // Start monitoring the selected topic when the button is pressed
    $('#startJitterMonitorBtn').click(startMonitoring);
});

const fetchTopics = () => {
    $.getJSON('/api/topics', function(data) {
        data.forEach(topic => {
            $('#topicSelector').append($('<option>', {
                value: topic,
                text: topic
            }));
        });
    });
};

const startMonitoring = () => {
    const selectedTopic = $('#topicSelector').val();
    $.get(`/api/start_jitter_monitor/${selectedTopic}`, response => {
        alert(response.status); // Notify the user of the result
    });
};