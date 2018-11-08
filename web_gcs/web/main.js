// Global variables
var ip = "localhost:5000";
var ual_ns = "";
var connected = false;
var state = 'NOT CONNECTED';
var timer;

// Config IP form
$(document).ready(function(){
    $(document).on('click', '#submit_ip', function() { // catch the form's submit event
        ip = $('#input_ip').val();
        $('#current_ip').text(ip);
        return false; // cancel original event to prevent form submitting
    });
});

// UAL ns form
$(document).ready(function(){
    $(document).on('click', '#submit_ns', function() { // catch the form's submit event
        ual_ns = $('#input_ns').val();
        $('#current_ns').text(ual_ns);
        return false; // cancel original event to prevent form submitting
    });
});

// Disconnect function
function disconnect(){
    $('#ual_status').text('NOT CONNECTED');
    $('#startstop').text('Connect');
    clearInterval(timer);
    connected = false;
    $('#submit_takeoff').attr("disabled", "disabled");
    $('#land').attr("disabled", "disabled");
    $('#submit_gtw').attr("disabled", "disabled");
    $('#submit_ip').removeAttr("disabled");
    $('#submit_ns').removeAttr("disabled");
}

// UAL status
function getUALStatus(){
    // Get data from Rostful backend through the Ajax call
    $.getJSON('http://'+ip+'/ros/'+ual_ns+'/ual/state', function(result){
        switch (result['state']){
            case 0:
                state = 'UNINITIALIZED';
                break;
            case 1:
                state = 'LANDED_DISARMED';
                break;
            case 2:
                state = 'LANDED_ARMED';
                break;
            case 3:
                state = 'TAKING_OFF';
                break;
            case 4:
                state = 'FLYING_AUTO';
                break;
            case 5:
                state = 'FLYING_MANUAL';
                break;
            case 6:
                state = 'LANDING';
                break;
        }
        $('#ual_status').text(state);
    })
    .error(function(){
        disconnect();
        alert('Not able to connect :(');
    });

    getPose();
}

// Get pose
function getPose(){
    // Get data from Rostful backend through the Ajax call
    $.getJSON('http://'+ip+'/ros/'+ual_ns+'/ual/pose', function(result){

        $('#ual_pose_x').text(result['pose']['position']['x'].toFixed(2));
        $('#ual_pose_y').text(result['pose']['position']['y'].toFixed(2));
        $('#ual_pose_z').text(result['pose']['position']['z'].toFixed(2));
        $('#ual_pose_frame_id').text(result['header']['frame_id']);
    })
}

// Connect and disconnect
$(document).ready(function(){
    $(document).on('click', '#startstop', function(){
        if (!connected) {
            timer = setInterval(getUALStatus,1000);
            $('#startstop').text('Disconnect');
            connected = true;
            $('#submit_takeoff').removeAttr("disabled");
            $('#land').removeAttr("disabled");
            $('#submit_gtw').removeAttr("disabled");
            $('#submit_ip').attr("disabled", "disabled");
            $('#submit_ns').attr("disabled", "disabled");
        }
        else {
            disconnect();
        }
    });
});


// Take off
$(document).ready(function(){
    $(document).on('click', '#submit_takeoff', function() { // catch the form's submit event

        msgdata = {};
        msgdata['blocking'] = false;
        msgdata['height'] = parseFloat($('#input_height').val());
    
        // Send data to Rostful backend through the Ajax call
        $.ajax({
            url: 'http://'+ip+'/ros/'+ual_ns+'/ual/take_off',
            data: JSON.stringify(msgdata),
            method: 'POST',
            dataType: 'json',
            beforeSend: function(jqXHR, settings) {
                // This callback function will trigger before data is sent
                //$("#post_request_chatter").text(JSON.stringify(msgdata,null,2));
                // $.mobile.loading('show');// This will show ajax spinner
            },
            complete: function() {
                // This callback function will trigger on data sent/received complete
                // $.mobile.loading('hide');// This will show ajax spinner
            },
            success: function (result) {
                //$("#post_result_chatter").text(JSON.stringify(result,null,2));
            },
            error: function (request,error) {
                // This callback function will trigger on unsuccessful action
                alert('Network error has occurred please try again!\nhttp://'+ip+'/ros/'+ual_ns+'/ual/take_off');
            }
        });
        return false; // cancel original event to prevent form submitting
    });
});

// Land
$(document).ready(function(){
    $(document).on('click', '#land', function() { // catch the form's submit event

        msgdata = {};
        msgdata['blocking'] = false;
    
        // Send data to Rostful backend through the Ajax call
        $.ajax({
            url: 'http://'+ip+'/ros/'+ual_ns+'/ual/land',
            data: JSON.stringify(msgdata),
            method: 'POST',
            dataType: 'json',
            beforeSend: function(jqXHR, settings) {
                // This callback function will trigger before data is sent
                //$("#post_request_chatter").text(JSON.stringify(msgdata,null,2));
                // $.mobile.loading('show');// This will show ajax spinner
            },
            complete: function() {
                // This callback function will trigger on data sent/received complete
                // $.mobile.loading('hide');// This will show ajax spinner
            },
            success: function (result) {
                //$("#post_result_chatter").text(JSON.stringify(result,null,2));
            },
            error: function (request,error) {
                // This callback function will trigger on unsuccessful action
                alert('Network error has occurred please try again!');
            }
        });
    });
});

// Go to waypoint
$(document).ready(function(){
    $(document).on('click', '#submit_gtw', function() { // catch the form's submit event

        msgdata = {};
        msgdata['waypoint'] = {};
        // msgdata['waypoint']['header'] = {};
        // msgdata['waypoint']['header']['seq'] = 0;
        // msgdata['waypoint']['header']['stamp'] = {};
        // msgdata['waypoint']['header']['stamp']['secs'] = 0;
        // msgdata['waypoint']['header']['stamp']['nsecs'] = 0;
        // msgdata['waypoint']['header']['frame_id'] = $('input[name="frame_id"]').val();
        msgdata['waypoint']['pose'] = {};
        msgdata['waypoint']['pose']['position'] = {};
        msgdata['waypoint']['pose']['position']['x'] = parseFloat($('input[name="x"]').val());
        msgdata['waypoint']['pose']['position']['y'] = parseFloat($('input[name="y"]').val());
        msgdata['waypoint']['pose']['position']['z'] = parseFloat($('input[name="z"]').val());
        msgdata['waypoint']['pose']['orientation'] = {};
        msgdata['waypoint']['pose']['orientation']['x'] = parseFloat($('input[name="q1"]').val());
        msgdata['waypoint']['pose']['orientation']['y'] = parseFloat($('input[name="q2"]').val());
        msgdata['waypoint']['pose']['orientation']['z'] = parseFloat($('input[name="q3"]').val());
        msgdata['waypoint']['pose']['orientation']['w'] = parseFloat($('input[name="q4"]').val());
        msgdata['blocking'] = false;
        console.log(msgdata);
    
        // Send data to Rostful backend through the Ajax call
        $.ajax({
            url: 'http://'+ip+'/ros/'+ual_ns+'/ual/go_to_waypoint',
            data: JSON.stringify(msgdata),
            method: 'POST',
            dataType: 'json',
            beforeSend: function(jqXHR, settings) {
                // This callback function will trigger before data is sent
                //$("#post_request_chatter").text(JSON.stringify(msgdata,null,2));
                // $.mobile.loading('show');// This will show ajax spinner
            },
            complete: function() {
                // This callback function will trigger on data sent/received complete
                // $.mobile.loading('hide');// This will show ajax spinner
            },
            success: function (result) {
                //$("#post_result_chatter").text(JSON.stringify(result,null,2));
            },
            error: function (request,error) {
                // This callback function will trigger on unsuccessful action
                alert('Network error has occurred please try again!');
            }
        });
    });
});