// =========================================================================================
// HEADER //
// =========================================================================================

// TODOS


// =========================================================================================
// Initialize global Variables and objects
// 
// init chart variables
var mylabels = [1, 2, 3, 4, 5, 6];
var mydata = [0, 0, 10, 10, 20, 10];
var mydata2 = [3, 3, 10, 11, 25, 10];
// chart variables
var labelsSensors = [1, 2, 3, 4, 5, 6];
var dataSensor1 = [1, 1, 1, 1, 1, 1];
var dataSensor2 = [20, 20, 20, 20, 20, 20];
var globalYLimit = 50;
// GET/POST variables
var myresponse;
var myresult
var myArrayResponse;
//
var currentValidSsid = "";      // current ssid from esp, used for fallback reasons
//
var ConfigMode = "none";        // "block" = display, "none" = hide configuration at bottom of webpage
var monitoringMode = "none";    // none=only Monitoring, block = Monitoring with brakes
var calibration = 0;            // dectivate calibration @ start
//
// Init array to log sensor values
var arrayToLog = [['Date & Time', 'Sensor 1', 'Sensor 2']];
var maxArrayLength = 2 * 3600;    // max amount of logged values
var LogInterval = 1000;         // [ms] logging interval
//
// Connection Status
var globalConnectionStatus = false; // false=trying to connect, true=connected
var globalMachineStatus = false; // false=standstill, true=running

// =========================================================================================
// create chart with standard values
// =========================================================================================
//
var ctx = document.getElementById('myChart').getContext('2d');
// define size of chart (not in .css)
ctx.canvas.parentNode.style.width = "1000px";
ctx.canvas.parentNode.style.height = "350px";
// create chart and define options
var myChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: mylabels,
        datasets: [{
            label: 'Yarn Tension 1',
            fill: false,
            data: mydata,
            borderWidth: 5,
            borderColor: 'rgba(000, 139, 000, 1)'
        }, {
            label: 'Yarn Tension 2',
            fill: false,
            data: mydata2,
            borderWidth: 5,
            borderColor: 'rgba(000, 000, 250, 1)'
        }]
    },
    options: {
        elements: {
            point: {
                radius: 0
            }
        },
        responsive: true,
        maintainAspectRatio: false,
        animation: {
            duration: 0
        },
        scales: {
            yAxes: [{
                //type: "linear",
                ticks: {
                    min: 0,
                    max: globalYLimit,
                    stepSize: 2,
                    beginAtZero: true
                },
                scaleLabel: {
                    display: true,
                    labelString: 'tension [cN]',
                    fontSize: 15
                }
            }],
            xAxes: [{
                //type: "linear",
                ticks: {
                    min: 0,
                    max: 360,
                    stepSize: 10,
                    beginAtZero: true,
                    suggestedMax: 0,
                    suggestedMax: 360
                },
                scaleLabel: {
                    display: true,
                    labelString: 'machine angle [°]',
                    fontSize: 15
                }
            }]
        }
    }
});



// myChart.options.scales.yAxes[0].ticks.max = 30;
// myChart.update();

// =========================================================================================
// function init(), called in html
// =========================================================================================
function init() {
    console.log("Init called");

    // initialize Config Mode
    ConfigMode = "block"; // --> comment out to initialize with config buttons,
    // because we don't have a "set" config mode, only a "toggle" (yes, not that intelligent but it works)
    toggleConfigMode();

    // init Monitoring Mode (1=show, 0=hide)
    //setMonitoringMode(0, 0);
    setMonitoringMode(1, 0);

    // Add Events to Buttons and Input fields

    // deactivate blinking @ startup
    document.getElementById("calibrateButton").style.animation = ''
    // Force Sensor values
    document.getElementById("outputBTSR1").value = 15;
    document.getElementById("outputBTSR2").value = 14;
    // enable button
    document.getElementById("enablebutton").addEventListener("click", enableHdrives);
    document.getElementById("disablebutton").addEventListener("click", disableHdrives);
    // setpoint inputs
    document.getElementById("InputAutoSetpoint1").addEventListener("change", function () {
        document.getElementById("InputAutoSetpoint1").style.color = "grey";
        document.getElementById("setpointButton1").style.animation = 'glowingGrey 1500ms infinite'
    });
    document.getElementById("InputAutoSetpoint2").addEventListener("change", function () {
        document.getElementById("InputAutoSetpoint2").style.color = "grey";
        document.getElementById("setpointButton2").style.animation = 'glowingGrey 1500ms infinite'
    });
    // setpoint buttons
    document.getElementById("setpointButton1").addEventListener("click", function () {
        sendhdrivesetpoint(1);
        document.getElementById("InputAutoSetpoint1").style.color = "black";
        document.getElementById("setpointButton1").style.animation = ''
    });
    document.getElementById("setpointButton2").addEventListener("click", function () {
        sendhdrivesetpoint(2);
        document.getElementById("InputAutoSetpoint2").style.color = "black";
        document.getElementById("setpointButton2").style.animation = ''
    });
    // change sensor views
    document.getElementById("changeToSensor1button").addEventListener("click", function () { switchSensorView(1) });
    document.getElementById("changeToSensor2button").addEventListener("click", function () { switchSensorView(2) });
    document.getElementById("changeToSensorAllbutton").addEventListener("click", function () { switchSensorView(3) });
    // toggle control mode
    document.getElementById("manualButton").addEventListener("click", function () {
        sendhdrivesetpoint(1);
        setTimeout(sendhdrivesetpoint(2), 500);
        setTimeout(toggleControlMode(0), 500)
    });
    document.getElementById("autoButton").addEventListener("click", function () {
        sendhdrivesetpoint(1);
        setTimeout(sendhdrivesetpoint(2), 500);
        setTimeout(toggleControlMode(1), 500)
    });
    // toggle Config
    document.getElementById("configurationButton").addEventListener("click", checkPswd);
    // toggle Monitoring-Mode
    document.getElementById("onlyMonitoringButton").addEventListener("click", toggleMonitoringMode);
    // set offset
    document.getElementById("setMachineAngleOffsetButton").addEventListener("click", sendAngleOffset);
    // toggle calibration
    document.getElementById("calibrateButton").addEventListener("click", function () {
        if (calibration == 0) {
            var r = confirm("WARNING: Start calibration?");
            if (r == true) {
                toggleCalibration();
            }
        } else {
            toggleCalibration();
        }
    });
    // Send IP & SSID
    document.getElementById("ipAdressButton").addEventListener("click", function () {
        var r = confirm("WARNING: Sending new IP/SSID will restart ESP32. Continue?");
        if (r == true) {
            sendIp();
            setTimeout(sendSsid(), 500);
        }
    });
    // Timing Input fields
    document.getElementById("InputOpeningTime").addEventListener("change", function () {
        document.getElementById("InputOpeningTime").style.color = "grey";
        document.getElementById("openingClosingButton").style.animation = 'glowingGrey 1500ms infinite'
    });
    document.getElementById("InputClosingTime").addEventListener("change", function () {
        document.getElementById("InputClosingTime").style.color = "grey";
        document.getElementById("openingClosingButton").style.animation = 'glowingGrey 1500ms infinite'
    });
    // set Timing (open/close times)
    document.getElementById("openingClosingButton").addEventListener("click", function () {
        sendTiming();
        document.getElementById("InputOpeningTime").style.color = "black";
        document.getElementById("InputClosingTime").style.color = "black";
        document.getElementById("openingClosingButton").style.animation = ''
    });
    // export log
    document.getElementById("logButton").addEventListener("click", function () {
        // get timestamp
        var name = getTimeStampForLog();
        // console.log(name);
        // use as log name and log data array
        exportToCsv(name + '.csv', arrayToLog);
    });
    // document.getElementById("logButton").addEventListener("click", sendTimestamp);
    // send Chart Y Limit
    document.getElementById("chartYLimitButton").addEventListener("click", sendChartYLimit);
    // diagnostic function
    document.getElementById("diagnosticButton").addEventListener("click", sendDiagInfo);


}
// ==========================================================================================================================
// ============ FUNCTIONS ===================================================================================================
// ==========================================================================================================================
// Testfunction
var i = 0;
function testfunction() {
    document.getElementById("testoutput").value = i;
    i++;
    i++;
}
// =========================================================================================
// Function to check if password is correct, then switch to Admin Mode
function checkPswd() {
    var confirmPassword = "admin";
    var password = document.getElementById("pswd").value;
    // if we have already opened options, let them be closed without password
    if (ConfigMode == "block") {
        toggleConfigMode();
        // else, check password
    } else if (password == confirmPassword) {
        toggleConfigMode();
        // if incorrect, alert
    } else {
        alert("Password incorrect");
    }
}
// =========================================================================================
// Function to toggle config Mode (starts turned off @ page load)
function toggleConfigMode() {
    //
    // disable/enable relevenat elements
    // and change text on button accordingly
    if (ConfigMode == "block") {
        ConfigMode = "none"
        document.getElementById("configurationButton").innerHTML = "Open Config"
    } else {
        ConfigMode = "block"
        document.getElementById("configurationButton").innerHTML = "Close Config"
    }
    // buttons
    document.getElementById("onlyMonitoringButton").style.display = ConfigMode
    document.getElementById("setMachineAngleOffsetButton").style.display = ConfigMode
    document.getElementById("calibrateButton").style.display = ConfigMode
    document.getElementById("manualButton").style.display = ConfigMode
    document.getElementById("autoButton").style.display = ConfigMode
    document.getElementById("ipAdressButton").style.display = ConfigMode
    document.getElementById("enablebutton").style.display = ConfigMode
    document.getElementById("disablebutton").style.display = ConfigMode
    document.getElementById("chartYLimitButton").style.display = ConfigMode
    // inputs/Outputs
    document.getElementById("InputMachineAngleOffset").style.display = ConfigMode
    document.getElementById("outputMachineAngle").style.display = ConfigMode
    document.getElementById("ipInput1").style.display = ConfigMode
    document.getElementById("ipInput2").style.display = ConfigMode
    document.getElementById("ipInput3").style.display = ConfigMode
    document.getElementById("ipInput4").style.display = ConfigMode
    document.getElementById("ssidInput").style.display = ConfigMode
    document.getElementById("outputCalibHigh").style.display = ConfigMode
    document.getElementById("outputCalibLow").style.display = ConfigMode
    document.getElementById("chartYLimit").style.display = ConfigMode
    // texts
    document.getElementById("textPW").style.display = ConfigMode


}
// =========================================================================================
// Function to toggle Monitoring Mode (0 = only Monitoring, >=1 = Monitoring & brakes)
function toggleMonitoringMode() {
    // if current mode is "block", switch to "none"=0=hide
    if (monitoringMode == "block") {
        setMonitoringMode(0, 1);
    } else { // if current mode is "none", switch to "block"=1=show
        setMonitoringMode(1, 1)
    }
}
// =========================================================================================
// Function to set Monitoring Mode (0 = only Monitoring, >=1 = Monitoring & brakes)
function setMonitoringMode(viewingMode, sendToEsp) {
    // disable/enable relevenat elements ("block" = display, "none" = hide)
    // and change text in button accordingly
    if (viewingMode == 0) {
        monitoringMode = "none" // set global variable
        document.getElementById("onlyMonitoringButton").innerHTML = "Activate Weft-Brakes"
    } else {
        monitoringMode = "block"
        document.getElementById("onlyMonitoringButton").innerHTML = "Deactivate Weft-Brakes"
    }

    // buttons
    document.getElementById("setpointButton1").style.display = monitoringMode
    document.getElementById("setpointButton2").style.display = monitoringMode
    document.getElementById("openingClosingButton").style.display = monitoringMode

    // texts
    document.getElementById("controlModeStatus").style.display = monitoringMode
    document.getElementById("text01").style.display = monitoringMode
    document.getElementById("text11").style.display = monitoringMode
    document.getElementById("text00").style.display = monitoringMode
    document.getElementById("text10").style.display = monitoringMode
    document.getElementById("text20").style.display = monitoringMode
    document.getElementById("text21").style.display = monitoringMode
    // textfields
    document.getElementById("InputAutoSetpoint1").style.display = monitoringMode
    document.getElementById("InputAutoSetpoint2").style.display = monitoringMode
    document.getElementById("InputManualSetpoint1").style.display = monitoringMode
    document.getElementById("InputManualSetpoint2").style.display = monitoringMode

    document.getElementById("InputOpeningTime").style.display = monitoringMode
    document.getElementById("InputClosingTime").style.display = monitoringMode




    // lines
    //document.getElementById("svgLineTop").style.display = monitoringMode
    //document.getElementById("svgLineMiddle").style.display = monitoringMode

    // send monitoing Mode to esp32 if requested
    if (sendToEsp == 1) {
        sendMonitoringMode(viewingMode);
    }

}
// =========================================================================================
// function to update chart
function plotTensionData(sensors2plot) {
    // temp variables for data handling, copy from global sensor data (USING SLICE()!)
    var tempdataSensor1 = dataSensor1.slice(0);
    var tempdataSensor2 = dataSensor2.slice(0);
    var templabelsSensors = labelsSensors.slice(0);
    // push 360° at end of data, 0° at start of data for consistent chart plot
    // --> push last value of sensor data at end of array and first vaklue of sensor data at beginning of array
    // we do this so that we have data which always begins with 0° and ends with 360°
    tempdataSensor1.unshift(tempdataSensor1[0]);
    tempdataSensor1.push(tempdataSensor1[tempdataSensor1.length - 1]);
    tempdataSensor2.unshift(tempdataSensor2[0]);
    tempdataSensor2.push(tempdataSensor2[tempdataSensor2.length - 1]);
    templabelsSensors.unshift(0);
    templabelsSensors.push(360);

    // console.log(dataSensor1);
    // console.log(labelsSensors);

    // update chart with tempValues
    if (sensors2plot == 1) {
        myChart.data.datasets[0].data = tempdataSensor1;
        myChart.data.datasets[0].label = 'Yarn Tension 1';
        myChart.data.datasets[0].borderColor = 'green';

        myChart.data.datasets[1].data = [];
        myChart.data.datasets[1].label = [];
        myChart.data.datasets[1].borderColor = 'white';

        myChart.data.labels = templabelsSensors;

    } else if (sensors2plot == 2) {
        myChart.data.datasets[0].data = tempdataSensor2;
        myChart.data.datasets[0].label = 'Yarn Tension 2';
        myChart.data.datasets[0].borderColor = 'blue';

        myChart.data.datasets[1].data = [];
        myChart.data.datasets[1].label = [];
        myChart.data.datasets[1].borderColor = 'white';

        myChart.data.labels = templabelsSensors;

    } else if (sensors2plot == 3) {
        myChart.data.datasets[0].data = tempdataSensor1;
        myChart.data.datasets[0].label = 'Yarn Tension 1';
        myChart.data.datasets[0].borderColor = 'green';

        myChart.data.datasets[1].data = tempdataSensor2;
        myChart.data.datasets[1].label = 'Yarn Tension 2';
        myChart.data.datasets[1].borderColor = 'blue';

        myChart.data.labels = templabelsSensors;

    }
    // update Y limit
    myChart.options.scales.yAxes[0].ticks.max = globalYLimit;
    // replot chart
    myChart.update();
}
// =========================================================================================
// function to swicth sensor view
// global variable for which Sensors should be plotted (1=Sensor1, 2=Sensor2, 3=both)
var sensorChoice = 3;
function switchSensorView(sensorNr) {
    // change sensor choice to visualize
    sensorChoice = sensorNr;
    // update chart first time after manual change of view
    plotTensionData(sensorChoice);
}
// =========================================================================================
// function to toggle calibration ON/OFF
function toggleCalibration() {
    if (calibration == 0) {
        calibration = 1;
        document.getElementById("calibrateButton").innerHTML = "Deactivate Calibration"
    } else if (calibration >= 1) {
        calibration = 0;
        document.getElementById("calibrateButton").innerHTML = "Activate Calibration"
    }
    sendCalibration(calibration);
}
// =========================================================================================
// function to export log-array
function exportToCsv(filename, rows) {
    var processRow = function (row) {
        var finalVal = '';
        for (var j = 0; j < row.length; j++) {
            var innerValue = row[j] === null ? '' : row[j].toString();
            if (row[j] instanceof Date) {
                innerValue = row[j].toLocaleString();
            };
            var result = innerValue.replace(/"/g, '""');
            if (result.search(/("|,|\n)/g) >= 0)
                result = '"' + result + '"';
            if (j > 0)
                finalVal += ';';
            finalVal += result;
        }
        return finalVal + '\n';
    };

    var csvFile = '';
    for (var i = 0; i < rows.length; i++) {
        csvFile += processRow(rows[i]);
    }

    var blob = new Blob([csvFile], { type: 'text/csv;charset=utf-8;' });
    if (navigator.msSaveBlob) { // IE 10+
        navigator.msSaveBlob(blob, filename);
    } else {
        var link = document.createElement("a");
        if (link.download !== undefined) { // feature detection
            // Browsers that support HTML5 download attribute
            var url = URL.createObjectURL(blob);
            link.setAttribute("href", url);
            link.setAttribute("download", filename);
            link.style.visibility = 'hidden';
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
        }
    }
}
// =========================================================================================
// function to add values log-array continuously in defined interval
var updateIncrementLogArray = setInterval(incrementLogArray, LogInterval);
function incrementLogArray() {
    // get date & time
    var now = new Date().toLocaleString();
    // get tension means
    var logVal1 = document.getElementById("outputBTSR1").value;
    var logVal2 = document.getElementById("outputBTSR2").value;
    // check if array is full
    if (arrayToLog.length < maxArrayLength) {
        // if not, push new value
        arrayToLog.push([now, logVal1, logVal2]);
    } else {
        // else, delete oldest value...
        arrayToLog.shift();
        // and push new
        arrayToLog.push([now, logVal1, logVal2]);
    }
}
// =========================================================================================
// function to get timestamp as string
function getTimeStampForLog() {
    // get current timestamp
    var tempNow = new Date();
    // extract day, add "0" to definitly get 2 chars, convert to string
    var dayTemp = tempNow.getDate();
    if (dayTemp < 10) {
        dayTemp = "0" + dayTemp.toString();
    } else {
        dayTemp = dayTemp.toString();
    }
    // same with month, add 1 to extracted month because of 0-index ("0"= month 1)
    var monthTemp = (tempNow.getMonth() + 1);
    if (monthTemp < 10) {
        monthTemp = "0" + monthTemp.toString();
    } else {
        monthTemp = monthTemp.toString();
    }
    // get year
    var yearTemp = tempNow.getFullYear().toString();
    // get hour and extend if we have to
    var hourTemp = tempNow.getHours();
    if (hourTemp < 10) {
        hourTemp = "0" + hourTemp.toString();
    } else {
        hourTemp = hourTemp.toString();
    }
    // get minute and do as usual
    var minuteTemp = tempNow.getMinutes();
    if (minuteTemp < 10) {
        minuteTemp = "0" + minuteTemp.toString();
    } else {
        minuteTemp = minuteTemp.toString();
    }
    // build csv name string
    var timestamp = yearTemp + monthTemp + dayTemp + "_" + hourTemp + minuteTemp;
    return timestamp;
}
// =========================================================================================
// function to get timestamp as string to send to esp for time synch
function getTimeStampForESP() {
    // get current timestamp
    var tempNow = new Date();
    // extract day, add "0" to definitly get 2 chars, convert to string
    var dayTemp = tempNow.getDate();
    if (dayTemp < 10) {
        dayTemp = "0" + dayTemp.toString();
    } else {
        dayTemp = dayTemp.toString();
    }
    // same with month, add 1 to extracted month because of 0-index ("0"= month 1)
    var monthTemp = (tempNow.getMonth() + 1);
    if (monthTemp < 10) {
        monthTemp = "0" + monthTemp.toString();
    } else {
        monthTemp = monthTemp.toString();
    }
    // get year
    var yearTemp = tempNow.getFullYear().toString();
    // get hour and extend if we have to
    var hourTemp = tempNow.getHours();
    if (hourTemp < 10) {
        hourTemp = "0" + hourTemp.toString();
    } else {
        hourTemp = hourTemp.toString();
    }
    // get minute and do as usual
    var minuteTemp = tempNow.getMinutes();
    if (minuteTemp < 10) {
        minuteTemp = "0" + minuteTemp.toString();
    } else {
        minuteTemp = minuteTemp.toString();
    }
    // get seconds and do as usual
    var secondTemp = tempNow.getSeconds();
    if (secondTemp < 10) {
        secondTemp = "0" + secondTemp.toString();
    } else {
        secondTemp = secondTemp.toString();
    }

    // build csv name string
    var timestamp = dayTemp + monthTemp + yearTemp + hourTemp + minuteTemp + secondTemp;
    return timestamp;
}
// =========================================================================================
// function to handle Status Text in webpage --> connected, machine running etc.
var updateStatus = setInterval(updateGlobalStatus, 1000);
function updateGlobalStatus() {
    //console.log(globalConnectionStatus);
    //console.log(globalMachineStatus);
    if (globalConnectionStatus == false) {
        // case no connection for certain time
        document.getElementById("textMaschineStatus").style.color = "black";
        document.getElementById("textMaschineStatus").innerHTML = "Trying to connect...";
    } else if (globalMachineStatus == true) {
        document.getElementById("textMaschineStatus").style.color = "green";
        document.getElementById("textMaschineStatus").innerHTML = "Machine Running";
    } else {
        document.getElementById("textMaschineStatus").style.color = "red";
        document.getElementById("textMaschineStatus").innerHTML = "Machine Stopped";
    }
}
// =========================================================================================
// function get a binary representation of a number as string
function num2BinaryString_8bit(num) {
    // convert to binary form
    var tempString = num.toString(2);
    // add leading zeros if needed
    if (num < 2) {
        tempString = "0000000" + tempString;
    } else if (num < 4) {
        tempString = "000000" + tempString;
    } else if (num < 8) {
        tempString = "00000" + tempString;
    } else if (num < 16) {
        tempString = "0000" + tempString;
    } else if (num < 32) {
        tempString = "000" + tempString;
    } else if (num < 64) {
        tempString = "00" + tempString;
    } else if (num < 128) {
        tempString = "0" + tempString;
    }
    return tempString;
}



// ==========================================================================================================================
// ============ GET REQUESTS ================================================================================================
// ==========================================================================================================================
//
// =========================================================================================
// function to get two arrays with valid sensor data from esp through http
// This function works together with "getTensionMean" to get all relevant data from esp. Also handles
// connection status and blocks an overflow of GET-Requests
var updateArray = setInterval(getArray, 200);
var isBusyCycleCounter = 15;
var isBusy = false;
function getArray() {

    // Status of Connection. if busycounter is a certain value, connection has been blocked
    if (isBusyCycleCounter >= 15) {
        isBusyCycleCounter = 0;
        globalConnectionStatus = false;
        // TESTING: if busy for 3 seconds, release busy flag again and get try to get data
        // every cycle 
        isBusy = false;
    } else {
        //globalConnectionStatus = true;
    }
    // log to console
    //console.log(globalConnectionStatus);
    //console.log(isBusyCycleCounter);
    //
    // if last callback of this function has not completed, jump out of this instance (block overflow of requests)
    if (isBusy == true) {
        console.log("still busy while trying to get Array");
        isBusyCycleCounter++; // increment busy counter every time function is blocked
        return;
    }

    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            myArrayResponse = new Uint8Array(this.response);
            // clear label vector and data vectors
            mylabels = [];
            mydata = [];
            mydata2 = [];
            labelsSensors = [];
            // get length of current, valid array of data (1 measurement)
            // this is length of response -1 byte, as first byte is cannel select!
            var numberOfDataPoints = (myArrayResponse.length - 1) / 4;
            // set up temp arrays
            var tempData1 = [];
            var tempData2 = [];
            // fill these arrays with received data and sum up total ("integral")
            for (var i = 0; i < numberOfDataPoints; i++) {
                // create time axis in degrees
                mylabels[i] = i;
                labelsSensors[i] = Math.round(i * 360 / numberOfDataPoints);
                // extract sensor data. make sure we don't pull unwanted bits with us when we bitshift
                tempData1[i] = (((myArrayResponse[2 * i + 2]) << 8) & 0xFF00) | ((myArrayResponse[2 * i + 1]) & 0x00FF);
                tempData1[i] = tempData1[i] / 100;
                tempData2[i] = (((myArrayResponse[2 * i + 2 + 2 * numberOfDataPoints]) << 8) & 0xFF00) | ((myArrayResponse[2 * i + 1 + 2 * numberOfDataPoints]) & 0x00FF);
                tempData2[i] = tempData2[i] / 100;
            }
            // first datapoint of myArrayresponse is in fact valid channel. so:
            // if 1, then channel[1] = channel 2
            // if 0, then channel[0] = channel 1
            if (myArrayResponse[0] >= 1) {
                dataSensor2 = [];
                dataSensor2 = tempData2;
            } else {
                dataSensor1 = [];
                dataSensor1 = tempData1;
            }
            // replot data depending on global sensor choice
            plotTensionData(sensorChoice);
            //document.getElementById("InputManualSetpoint2").value = numberOfDataPoints;

            // signal successful callback
            console.log("got Array successfully, trying to get Tension Mean");
            // wait a few miliseconds and try to request other data
            setTimeout(function () {
                // console.log("waited 50 miliseconds, trying to get Tension Mean");
                getTensionMean();
            }, 50)
        }
    };

    // set busy flag so no more requests are sent before callback has completed
    isBusy = true;
    console.log("____ Asking ESP32 for data");

    xhttp.open("GET", "getArray", true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();

}
// =========================================================================================
// function to get tension mean values AND current setpoint values through http from esp (getData in wifi.c)
// 17.06.21: function is called in getArray function
//
//var updateGetTensionMean = setInterval(getTensionMean, 200);
// this function gets called in getArray() after successfull callback
var initCounter = 0;
function getTensionMean() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            myresponse = new Uint8Array(this.response);
            //
            // first two bytes = control modes:
            var controlModeFeedback = myresponse[0];
            //
            // next four bytes are tension means:
            var tensionMean0 = (((myresponse[3] & 0xFF) << 8) | (myresponse[2] & 0xFF));
            var tensionMean1 = (((myresponse[5] & 0xFF) << 8) | (myresponse[4] & 0xFF));
            // divide by 100 because sent a such:
            tensionMean0 = tensionMean0 / 100;
            tensionMean1 = tensionMean1 / 100;
            // then round to 1 decimal:
            tensionMean0 = Math.round(tensionMean0 * 10) / 10;
            tensionMean1 = Math.round(tensionMean1 * 10) / 10;
            // next two bytes are feedback %-setpoint (actual closed) positions
            var feedbackSetpoint0 = myresponse[6];
            var feedbackSetpoint1 = myresponse[7];
            // next two bytes are feedback auto setpoint in [cN]
            var feedbackAutoSetpoint0 = myresponse[8];
            var feedbackAutoSetpoint1 = myresponse[9];
            // next two bytes are feedback manual setpoint in [%]
            var feedbackManualSetpoint0 = myresponse[10];
            var feedbackManualSetpoint1 = myresponse[11];
            // next byte is feedback Monitoring Mode
            var feedbackMonitoringMode = myresponse[12];
            // next two bytes are machine angle offset
            var feedbackActualOffset = (((myresponse[14] & 0xFF) << 8) | (myresponse[13] & 0xFF));
            // next two bytes are machine angle
            var feedbackActualAngle = (((myresponse[16] & 0xFF) << 8) | (myresponse[15] & 0xFF));
            // next byte is calibration status (on/off)
            var feedbackCalibration = myresponse[17];
            // next two bytes are closing time
            var feedbackOpeningTime = (((myresponse[19] & 0xFF) << 8) | (myresponse[18] & 0xFF));
            // next two bytes are opening time
            var feedbackClosingTime = (((myresponse[21] & 0xFF) << 8) | (myresponse[20] & 0xFF));
            // next four bytes are current IP Adress
            var feedbackIP1 = myresponse[22];
            var feedbackIP2 = myresponse[23];
            var feedbackIP3 = myresponse[24];
            var feedbackIP4 = myresponse[25];
            // next 2+2 bytes are calibration values (high/low)
            var feedbackCalibHigh = (((myresponse[27] & 0xFF) << 8) | (myresponse[26] & 0xFF));
            var feedbackCalibLow = (((myresponse[29] & 0xFF) << 8) | (myresponse[28] & 0xFF));
            // next 15 bytes is ssid
            // find out length of string (break when char=0 or already 15 chars)
            var m = 0;
            for (m = 0; m < 15; m++) {
                if (myresponse[30 + m] == 0) {
                    break;
                }
            }
            //
            var feedbackSsid = new Uint8Array(m);
            //
            for (var l = 0; l < m; l++) {
                feedbackSsid[l] = myresponse[30 + l];
            }
            let feedbackSsidStr = new TextDecoder().decode(feedbackSsid);
            // log to console
            //console.log(feedbackSsid);
            //console.log(feedbackSsidStr);
            //
            // continue from byte 60. next byte is machine running status
            globalMachineStatus = myresponse[60];
            // next byte is Y-Axis Limit
            var feedbackYLimit = myresponse[61];
            // next byte is diagnostic bits (1 byte = 8 bits)
            var feedbackDiagnosticBits = myresponse[62];




            // write directly to hmi, all the time
            document.getElementById("outputBTSR1").value = tensionMean0;
            document.getElementById("outputBTSR2").value = tensionMean1;
            document.getElementById("outputMachineAngle").value = feedbackActualAngle;
            // store valid ssid string for fallback
            currentValidSsid = feedbackSsidStr;
            // catch calibration status
            if (feedbackCalibration >= 1) {
                document.getElementById("calibrateButton").style.animation = 'glowing 1500ms infinite'
                calibration = 1;
            } else {
                document.getElementById("calibrateButton").style.animation = ''
                calibration = 0;
            }
            // calibration values
            document.getElementById("outputCalibHigh").value = feedbackCalibHigh;
            document.getElementById("outputCalibLow").value = feedbackCalibLow;
            // diagnostic values
            // document.getElementById("diagnosticOutput").innerHTML = feedbackDiagnosticBits.toString(2);
            document.getElementById("diagnosticOutput").innerHTML = num2BinaryString_8bit(feedbackDiagnosticBits);
            //
            // if Manual Mode:
            if (controlModeFeedback == 0) {
                document.getElementById("controlModeStatus").innerHTML = "MANUAL MODE";
                // change colors according to mode
                // active texts:
                document.getElementById("InputManualSetpoint1").style.color = "green";
                document.getElementById("InputManualSetpoint2").style.color = "green";
                document.getElementById("text00").style.color = "green";
                document.getElementById("text10").style.color = "green";
                // inactive texts:
                document.getElementById("InputAutoSetpoint1").style.color = "black";
                document.getElementById("InputAutoSetpoint2").style.color = "black";
                document.getElementById("text01").style.color = "black";
                document.getElementById("text11").style.color = "black";
                //
                // if Auto Mode:
            } else if (controlModeFeedback == 1) {
                document.getElementById("controlModeStatus").innerHTML = "AUTO MODE";
                // overwrite actual setpoint position in Auto Mode
                document.getElementById("InputManualSetpoint1").value = feedbackSetpoint0;
                document.getElementById("InputManualSetpoint2").value = feedbackSetpoint1;
                // active texts:
                document.getElementById("InputAutoSetpoint1").style.color = "green";
                document.getElementById("InputAutoSetpoint2").style.color = "green";
                document.getElementById("text01").style.color = "green";
                document.getElementById("text11").style.color = "green";
                // inactive texts:
                document.getElementById("InputManualSetpoint1").style.color = "black";
                document.getElementById("InputManualSetpoint2").style.color = "black";
                document.getElementById("text00").style.color = "black";
                document.getElementById("text10").style.color = "black";
            }
            // for first two (or maybe just one?) cycles, just load init values, read Monitoring Mode and try to send Timestamp
            if (initCounter < 2) {
                document.getElementById("InputManualSetpoint1").value = feedbackManualSetpoint0;
                document.getElementById("InputManualSetpoint2").value = feedbackManualSetpoint1;
                document.getElementById("InputAutoSetpoint1").value = feedbackAutoSetpoint0;
                document.getElementById("InputAutoSetpoint2").value = feedbackAutoSetpoint1;
                document.getElementById("InputMachineAngleOffset").value = feedbackActualOffset;
                document.getElementById("InputOpeningTime").value = feedbackOpeningTime;
                document.getElementById("InputClosingTime").value = feedbackClosingTime;
                // ip and ssid
                document.getElementById("ipInput1").value = feedbackIP1;
                document.getElementById("ipInput2").value = feedbackIP2;
                document.getElementById("ipInput3").value = feedbackIP3;
                document.getElementById("ipInput4").value = feedbackIP4;
                document.getElementById("ssidInput").value = feedbackSsidStr;
                // Maschine name, same as ssid
                document.getElementById("textTitle").innerHTML = feedbackSsidStr;
                // set monitoring Mode from feedback esp, without sending mode back to esp
                setMonitoringMode(feedbackMonitoringMode, 0);
                // read chart Y limit and set in browser
                globalYLimit = feedbackYLimit;
                document.getElementById("chartYLimit").value = feedbackYLimit;
                // send time stamp
                sendTimestamp()
                //
                initCounter++;
            }
            // signal successful callback
            console.log("got Tension Mean, Free to get Data Again");
            // reset busy flag so that requests can continoue
            // also signal connected status
            isBusy = false;
            globalConnectionStatus = true;
            isBusyCycleCounter = 0;
        }
    };

    // make sure busy flag is set (for instance if getTensionMean() is not called inside getArray())
    isBusy = true;

    xhttp.open("GET", "getData", true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();

}
// ==========================================================================================================================
// ============ POST REQUESTS ===============================================================================================
// ==========================================================================================================================
function disableHdrives() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    xhttp.open("POST", "disableHdrives", true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
function enableHdrives() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    xhttp.open("POST", "enableHdrives", true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
function sendhdrivesetpoint(hdrive_ch_nr) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    // build string to send (POST)
    var postCommandString = "sendhdrive" + hdrive_ch_nr + "setpoint";
    //
    // ==== get values of manual setpoint =========================
    var elementIdString = "InputManualSetpoint" + hdrive_ch_nr;
    var firstString = '';
    var tempVar = document.getElementById(elementIdString).value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries
    if (tempVar >= 100) {
        tempVar = 100;
        firstString = tempVar;
    } else if ((tempVar < 100) && (tempVar >= 10)) {
        firstString = "0" + tempVar;
    } else if ((tempVar < 10) && (tempVar >= 0)) {
        firstString = "00" + tempVar;
    } else if (tempVar < 0) {
        tempVar = 0;
        firstString = "000";
    } else {
        firstString = tempVar;
    }
    // copy to hmi
    document.getElementById(elementIdString).value = tempVar;
    //
    // ==== get values of auto setpoint ========================
    var elementIdString = "InputAutoSetpoint" + hdrive_ch_nr;
    var secondString = '';
    var tempVar = document.getElementById(elementIdString).value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries
    if (tempVar >= 50) {
        tempVar = 50;
        secondString = tempVar;
    } else if ((tempVar < 10) && (tempVar >= 0)) {
        secondString = "0" + tempVar;
    } else if (tempVar < 0) {
        tempVar = 0;
        secondString = "00";
    } else {
        secondString = tempVar;
    }
    // copy to hmi
    document.getElementById(elementIdString).value = tempVar;
    //
    // send strings
    xhttp.open("POST", postCommandString + firstString + secondString, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
// function to send angle offset
function sendAngleOffset() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    // build string to send (POST)
    var postCommandString = "resetAngleOffset";
    //
    // ==== get values of offset input =========================
    var firstString = '';
    var tempVar = document.getElementById("InputMachineAngleOffset").value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries
    if (tempVar >= 360) {
        tempVar = 360;
        firstString = tempVar;
    } else if ((tempVar < 100) && (tempVar >= 10)) {
        firstString = "0" + tempVar;
    } else if ((tempVar < 10) && (tempVar >= 0)) {
        firstString = "00" + tempVar;
    } else if (tempVar < 0) {
        tempVar = 0;
        firstString = "000";
    } else {
        firstString = tempVar;
    }
    // copy to hmi
    document.getElementById("InputMachineAngleOffset").value = tempVar;
    //
    // send strings
    xhttp.open("POST", postCommandString + firstString, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
// function to send opening & closing times
function sendTiming() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    // build string to send (POST)
    var postCommandString = "setTiming";
    //
    // ==== get values of opening time =========================
    var firstString = '';
    var tempVar = document.getElementById("InputOpeningTime").value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries --> 06.08.21 set boundaries between 0° and 150° for opening time
    if (tempVar >= 150) {
        tempVar = 150;
        firstString = tempVar;
    } else if ((tempVar < 100) && (tempVar >= 10)) {
        firstString = "0" + tempVar;
    } else if ((tempVar < 10) && (tempVar >= 0)) {
        firstString = "00" + tempVar;
    } else if (tempVar < 0) {
        tempVar = 0;
        firstString = "000";
    } else {
        firstString = tempVar;
    }
    // copy to hmi
    document.getElementById("InputOpeningTime").value = tempVar;
    // ==== get values of closing time =========================
    var secondString = '';
    var tempVar = document.getElementById("InputClosingTime").value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries --> Set boundaries between 200° and 350°
    if (tempVar >= 350) {
        tempVar = 350;
        secondString = tempVar;
    } else if (tempVar < 200) {
        tempVar = 200;
        secondString = tempVar;
        // } else if ((tempVar < 100) && (tempVar >= 10)) {
        //     secondString = "0" + tempVar;
        // } else if ((tempVar < 10) && (tempVar >= 0)) {
        //     secondString = "00" + tempVar;
        // } else if (tempVar < 0) {
        //     tempVar = 0;
        //     secondString = "000";
    } else {
        secondString = tempVar;
    }
    // copy to hmi
    document.getElementById("InputClosingTime").value = tempVar;
    //
    // send strings
    xhttp.open("POST", postCommandString + firstString + secondString, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
// function to toggle control mode (send request to toggle to esp32)
function toggleControlMode(mode) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    //
    var stringToSend = "controlMode" + mode;
    //
    xhttp.open("POST", stringToSend, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
// function to send Monitoring mode
function sendMonitoringMode(mode) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    //
    var stringToSend = "monitoringMode" + mode;
    //
    xhttp.open("POST", stringToSend, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
// function to send calibration ON/OFF
// OFF: enable=0, ON: enable=1
function sendCalibration(enable) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    xhttp.open("POST", "calibrateEncoder" + enable, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
// function to send ip adress
function sendIp() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    // build string to send (POST)
    var postCommandString = "setIP";
    //
    // ==== get first IP String ===============================
    var firstString = '';
    var tempVar = document.getElementById("ipInput1").value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries
    if (tempVar >= 255) {
        tempVar = 255;
    } else if (tempVar < 0) {
        tempVar = 0;
    }
    firstString = tempVar;
    // copy to hmi
    document.getElementById("ipInput1").value = tempVar;
    //
    // ==== get second IP String ===============================
    var secondString = '';
    tempVar = document.getElementById("ipInput2").value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries
    if (tempVar >= 255) {
        tempVar = 255;
    } else if (tempVar < 0) {
        tempVar = 0;
    }
    secondString = tempVar;
    // copy to hmi
    document.getElementById("ipInput2").value = tempVar;
    //
    // ==== get third IP String ===============================
    var thirdString = '';
    tempVar = document.getElementById("ipInput3").value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries
    if (tempVar >= 255) {
        tempVar = 255;
    } else if (tempVar < 0) {
        tempVar = 0;
    }
    thirdString = tempVar;
    // copy to hmi
    document.getElementById("ipInput3").value = tempVar;
    //
    // ==== get fourth IP String ===============================
    var fourthString = '';
    tempVar = document.getElementById("ipInput4").value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries
    if (tempVar >= 255) {
        tempVar = 255;
    } else if (tempVar < 0) {
        tempVar = 0;
    }
    fourthString = tempVar;
    var ipString = firstString + "." + secondString + "." + thirdString + "." + fourthString;
    var ipLength = "";
    if (ipString.length < 10) {
        ipLength = "0" + ipString.length;
    } else {
        ipLength = ipString.length;
    }
    var dataString = ipString + ipLength;
    // copy to hmi
    document.getElementById("ipInput4").value = tempVar;
    // send strings
    xhttp.open("POST", postCommandString, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send(dataString);
}
// =========================================================================================
// function to send ip adress
function sendSsid() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    // build string to send (POST)
    var postCommandString = "setSsid";
    //
    // ==== get ssid String ===============================
    var firstString = '';
    var tempVar = document.getElementById("ssidInput").value;

    // check length of ssid
    if (tempVar.length > 15) {
        tempVar = tempVar.substring(0, 15);
    } else if (tempVar.length < 1) {
        tempVar = currentValidSsid;
    }
    firstString = tempVar;
    // copy to hmi
    document.getElementById("ssidInput").value = tempVar;
    //
    // build string to send (ssid string + length of string)
    var ssidString = firstString;
    var ssidLength = "";
    if (ssidString.length < 10) {
        ssidLength = "0" + ssidString.length;
    } else {
        ssidLength = ssidString.length;
    }
    var dataString = ssidString + ssidLength;
    // send strings
    xhttp.open("POST", postCommandString, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send(dataString);
}
// =========================================================================================
// function to send Chart Y Limit
function sendChartYLimit() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    // build string to send (POST)
    var postCommandString = "chartYLimit";
    //
    // ==== get values of offset input =========================
    var firstString = '';
    var tempVar = document.getElementById("chartYLimit").value;
    // round value to nearest integer
    tempVar = Math.round(tempVar);
    // check boundries
    if (tempVar >= 250) {
        tempVar = 250;
        firstString = tempVar;
    } else if ((tempVar < 100) && (tempVar >= 10)) {
        firstString = "0" + tempVar;
    } else if ((tempVar < 10) && (tempVar >= 1)) {
        firstString = "00" + tempVar;
    } else if (tempVar < 1) {
        tempVar = 1;
        firstString = tempVar;
    } else {
        firstString = tempVar;
    }
    // copy to hmi
    document.getElementById("chartYLimit").value = tempVar;
    // copy to global status
    globalYLimit = tempVar;
    // replot chart
    plotTensionData(sensorChoice);
    //
    // send strings
    xhttp.open("POST", postCommandString + firstString, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
// function to send Chart Y Limit
function sendTimestamp() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    //
    // send strings
    xhttp.open("POST", "synchTimeStamp" + getTimeStampForESP(), true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}
// =========================================================================================
// function to do som diagnostic stuff
function sendDiagInfo() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            // do nothing with response
            ;
        }
    };
    //
    var stringToSend = "sendDiagInfo";
    //
    xhttp.open("POST", stringToSend, true);
    xhttp.responseType = "arraybuffer";
    xhttp.send();
}







