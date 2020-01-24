var idleTime = 0;
$(document).ready(function () {
    //Increment the idle time counter every minute.
    var idleInterval = setInterval(timerIncrement, 1000); // 1 second

    //Zero the idle timer on mouse movement.
    $(this).mousemove(function (e) {
        idleTime = 0;
    });
    $(this).keypress(function (e) {
        idleTime = 0;
    });
});

function timerIncrement() {
    idleTime = idleTime + 1;
    idle_time_publisher.publish({data: idleTime})
}

function rosInit(ros_master_uri='') {

    if(ros_master_uri == ''){
	    ros_master_uri = 'ws://' + location.hostname + ':9090'
    }

    console.log('ROS master URI: ' + ros_master_uri)
    ros = new ROSLIB.Ros({
	    url : ros_master_uri
    });

    // Once connected, setup the ROS network
    ros.on('connection', function() {
        console.log('Connected to websocket server!');
        setupRosNetwork()
    });

    // If unable to connect or the connection closes, refresh the page
    // to try to reconnect
    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        reload_page_to_retry_connecting();
    });
    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        reload_page_to_retry_connecting();
    });
}

function reload_page_to_retry_connecting(wait_seconds=2) {
    sleep(wait_seconds).then( function () {
        document.location.reload(true);
    });
}

function sleep(seconds) {
  return new Promise(resolve => setTimeout(resolve, seconds*1000));
}

function setupRosNetwork() {

    display_listener = new ROSLIB.Topic({
        ros : ros,
        name : 'cordial/gui/display',
        messageType : 'cordial_gui/Display'
    });
    display_listener.subscribe(updateText);

    user_response_publisher = new ROSLIB.Topic({
        ros : ros,
        name : 'cordial/gui/user_response',
        queue_size: 1,
        messageType: 'std_msgs/String'
    });

    idle_time_publisher = new ROSLIB.Topic({
        ros : ros,
        name : '/cordial/gui/idle_time',
        queue_size: 1,
        messageType: 'std_msgs/UInt32'
    });

    document.getElementById("submit").onclick = submitResponse

    setText("ROS network setup!")
}

function updateText(msg) {
    var my_str = msg.content
    setText(my_str)
}

function setText(str) {
    var text = document.getElementById("text")
    text.innerHTML = str
    console.log("Set value to '" + str + "'");
}

function submitResponse() {
    var value = document.getElementById("input").value;
    console.log("User entered '" + value + "'");

    user_response_publisher.publish({data: value});
    document.getElementById("input").value = '';
}
