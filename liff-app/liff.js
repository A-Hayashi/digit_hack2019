// User service UUID: Change this to your generated service UUID
const USER_SERVICE_UUID = '3fd1e37b-83a3-4691-8a70-dc42cd486ef7'; // LED, Button
// User service characteristics
const LED_CHARACTERISTIC_UUID = 'E9062E71-9E62-4BC6-B0D3-35CDCD9B027B';
const BTN_CHARACTERISTIC_UUID = '62FBD229-6EDD-4D1A-B554-5C4E1BB29169';

// PSDI Service UUID: Fixed value for Developer Trial
const PSDI_SERVICE_UUID = 'E625601E-9E55-4597-A598-76018A0D293D'; // Device ID
const PSDI_CHARACTERISTIC_UUID = '26E2B12B-85F0-4F3F-9FDD-91D114270E6E';

// UI settings
let ledState = false; // true: LED on, false: LED off
let clickCount = 0;

// -------------- //
// On window load //
// -------------- //

window.addEventListener('load', function () {
    eruda.init();
    initializeApp();
}, true)

// ----------------- //
// Handler functions //
// ----------------- //

function handlerToggleLed() {
    ledState = !ledState;

    uiToggleLedButton(ledState);
    liffToggleDeviceLedState(ledState);
}

// ------------ //
// UI functions //
// ------------ //

function uiToggleLedButton(state) {
    const el = document.getElementById("btn-led-toggle");
    el.innerText = state ? "Switch LED OFF" : "Switch LED ON";

    if (state) {
        el.classList.add("led-on");
    } else {
        el.classList.remove("led-on");
    }
}

function uiCountPressButton() {
    clickCount++;

    const el = document.getElementById("click-count");
    el.innerText = clickCount;
}

function uiToggleStateButton(pressed) {
    const el = document.getElementById("btn-state");

    if (pressed) {
        el.classList.add("pressed");
        el.innerText = "Pressed";
    } else {
        el.classList.remove("pressed");
        el.innerText = "Released";
    }
}

function uiToggleDeviceConnected(connected) {
    const elStatus = document.getElementById("status");
    const elControls = document.getElementById("controls");

    elStatus.classList.remove("error");

    if (connected) {
        // Hide loading animation
        uiToggleLoadingAnimation(false);
        // Show status connected
        elStatus.classList.remove("inactive");
        elStatus.classList.add("success");
        elStatus.innerText = "Device connected";
        // Show controls
        elControls.classList.remove("hidden");
    } else {
        // Show loading animation
        uiToggleLoadingAnimation(true);
        // Show status disconnected
        elStatus.classList.remove("success");
        elStatus.classList.add("inactive");
        elStatus.innerText = "Device disconnected";
        // Hide controls
        elControls.classList.add("hidden");
    }
}

function uiToggleLoadingAnimation(isLoading) {
    const elLoading = document.getElementById("loading-animation");

    if (isLoading) {
        // Show loading animation
        elLoading.classList.remove("hidden");
    } else {
        // Hide loading animation
        elLoading.classList.add("hidden");
    }
}

function uiStatusError(message, showLoadingAnimation) {
    uiToggleLoadingAnimation(showLoadingAnimation);

    const elStatus = document.getElementById("status");
    const elControls = document.getElementById("controls");

    // Show status error
    elStatus.classList.remove("success");
    elStatus.classList.remove("inactive");
    elStatus.classList.add("error");
    elStatus.innerText = message;

    // Hide controls
    elControls.classList.add("hidden");
}

function makeErrorMsg(errorObj) {
    return "Error\n" + errorObj.code + "\n" + errorObj.message;
}

// -------------- //
// LIFF functions //
// -------------- //

function initializeApp() {
    liff.init(() => initializeLiff(), error => uiStatusError(makeErrorMsg(error), false));
}

function initializeLiff() {
    liff.initPlugins(['bluetooth']).then(() => {
        liffCheckAvailablityAndDo(() => liffRequestDevice());
    }).catch(error => {
        uiStatusError(makeErrorMsg(error), false);
    });
}

function liffCheckAvailablityAndDo(callbackIfAvailable) {
    // Check Bluetooth availability
    liff.bluetooth.getAvailability().then(isAvailable => {
        if (isAvailable) {
            uiToggleDeviceConnected(false);
            callbackIfAvailable();
        } else {
            uiStatusError("Bluetooth not available", true);
            setTimeout(() => liffCheckAvailablityAndDo(callbackIfAvailable), 10000);
        }
    }).catch(error => {
        uiStatusError(makeErrorMsg(error), false);
    });;
}

function liffRequestDevice() {
    liff.bluetooth.requestDevice().then(device => {
        liffConnectToDevice(device);
    }).catch(error => {
        uiStatusError(makeErrorMsg(error), false);
    });
}

function liffConnectToDevice(device) {
    device.gatt.connect().then(() => {
        document.getElementById("device-name").innerText = device.name;
        document.getElementById("device-id").innerText = device.id;

        // Show status connected
        uiToggleDeviceConnected(true);

        // Get service
        device.gatt.getPrimaryService(USER_SERVICE_UUID).then(service => {
            liffGetUserService(service);
        }).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });
        device.gatt.getPrimaryService(PSDI_SERVICE_UUID).then(service => {
            liffGetPSDIService(service);
        }).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });

        // Device disconnect callback
        const disconnectCallback = () => {
            // Show status disconnected
            uiToggleDeviceConnected(false);

            // Remove disconnect callback
            device.removeEventListener('gattserverdisconnected', disconnectCallback);

            // Reset LED state
            ledState = false;
            // Reset UI elements
            uiToggleLedButton(false);
            uiToggleStateButton(false);

            // Try to reconnect
            initializeLiff();
        };

        device.addEventListener('gattserverdisconnected', disconnectCallback);
    }).catch(error => {
        uiStatusError(makeErrorMsg(error), false);
    });
}

function liffGetUserService(service) {
    // Button pressed state
    service.getCharacteristic(BTN_CHARACTERISTIC_UUID).then(characteristic => {
        liffGetButtonStateCharacteristic(characteristic);
    }).catch(error => {
        uiStatusError(makeErrorMsg(error), false);
    });

    // Toggle LED
    service.getCharacteristic(LED_CHARACTERISTIC_UUID).then(characteristic => {
        window.ledCharacteristic = characteristic;

        // Switch off by default
        liffToggleDeviceLedState(false);
    }).catch(error => {
        uiStatusError(makeErrorMsg(error), false);
    });
}

function liffGetPSDIService(service) {
    // Get PSDI value
    service.getCharacteristic(PSDI_CHARACTERISTIC_UUID).then(characteristic => {
        return characteristic.readValue();
    }).then(value => {
        // Byte array to hex string
        const psdi = new Uint8Array(value.buffer)
            .reduce((output, byte) => output + ("0" + byte.toString(16)).slice(-2), "");
        document.getElementById("device-psdi").innerText = psdi;
    }).catch(error => {
        uiStatusError(makeErrorMsg(error), false);
    });
}

function liffGetButtonStateCharacteristic(characteristic) {
    // Add notification hook for button state
    // (Get notified when button state changes)
    characteristic.startNotifications().then(() => {
        characteristic.addEventListener('characteristicvaluechanged', e => {
            const val = (new Uint8Array(e.target.value.buffer))[0];
            if (val > 0) {
                // press
                uiToggleStateButton(true);
            } else {
                // release
                uiToggleStateButton(false);
                uiCountPressButton();
            }
        });
    }).catch(error => {
        uiStatusError(makeErrorMsg(error), false);
    });
}

function liffToggleDeviceLedState(state) {
    window.ledCharacteristic.writeValue(

        //0 byte目：LED制御コマンド:[0]
        //1 byte目：LED点灯状態[0:OFF, 1:ON]
        state ? new Uint8Array([0x00, 0x01]) : new Uint8Array([0x00, 0x00])
    ).catch(error => {
        uiStatusError(makeErrorMsg(error), false);
    });
}

// 前進/後退モータ
window.addEventListener('load', function () {
    var elem = document.getElementById('forward_back_motor');
    var target = document.getElementById('forward_back_value');
    var rangeValue = function (elem, target) {
        return function (evt) {
            target.innerHTML = elem.value;

            //0 byte目：モータ制御コマンド [1]
            //1 byte目：モータNo [0, 1]
            //2 byte目：モータ回転速度 [-100～100]
            var cmd_r = new Uint8Array([0x01, 0x00, elem.value]);
            var cmd_l = new Uint8Array([0x01, 0x01, elem.value]);
            console.log(cmd_r);
            console.log(cmd_l);

            window.ledCharacteristic.writeValue(
                cmd_r
            ).catch(error => {
                uiStatusError(makeErrorMsg(error), false);
            });

            window.ledCharacteristic.writeValue(
                cmd_l
            ).catch(error => {
                uiStatusError(makeErrorMsg(error), false);
            });
        }
    }
    elem.addEventListener('input', function () {
        throttle(rangeValue(elem, target), 500)
    });
})


// 右回転
window.addEventListener('load', function () {
    var button = document.getElementById('right_rotate_motor');

    button.addEventListener('mousedown', function () {
        var cmd_r = new Uint8Array([0x01, 0x00, 100]);
        var cmd_l = new Uint8Array([0x01, 0x01, -100]);
        console.log(cmd_r);
        console.log(cmd_l);

        window.ledCharacteristic.writeValue(
            cmd_r
        ).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });

        window.ledCharacteristic.writeValue(
            cmd_l
        ).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });
    });

    button.addEventListener('mouseup', function () {
        var cmd_r = new Uint8Array([0x01, 0x00, 0]);
        var cmd_l = new Uint8Array([0x01, 0x01, 0]);
        console.log(cmd_r);
        console.log(cmd_l);

        window.ledCharacteristic.writeValue(
            cmd_r
        ).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });

        window.ledCharacteristic.writeValue(
            cmd_l
        ).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });
    });
})

window.addEventListener('load', function () {
    var button = document.getElementById('left_rotate_motor');

    button.addEventListener('mousedown', function () {
        var cmd_r = new Uint8Array([0x01, 0x00, 100]);
        var cmd_l = new Uint8Array([0x01, 0x01, -100]);
        console.log(cmd_r);
        console.log(cmd_l);

        window.ledCharacteristic.writeValue(
            cmd_r
        ).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });

        window.ledCharacteristic.writeValue(
            cmd_l
        ).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });
    });

    button.addEventListener('mouseup', function () {
        var cmd_r = new Uint8Array([0x01, 0x00, 0]);
        var cmd_l = new Uint8Array([0x01, 0x01, 0]);
        console.log(cmd_r);
        console.log(cmd_l);

        window.ledCharacteristic.writeValue(
            cmd_r
        ).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });

        window.ledCharacteristic.writeValue(
            cmd_l
        ).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });
    });
})


// モータ制御 手動/自動
window.addEventListener('load', function () {
    var checkbox = document.querySelector("input[name=manual]");

    checkbox.addEventListener('change', function () {
        if (this.checked) {
            // Checkbox is checked..

            //0 byte目：モータ制御モード変更コマンド:[3]
            //1 byte目：モータ制御モード[0:自動, 1:手動]
            var cmd = new Uint8Array([0x03, 0x01]);
            console.log(cmd);

        } else {
            // Checkbox is not checked..

            //0 byte目：モータ制御モード変更コマンド:[3]
            //1 byte目：モータ制御モード[0:自動, 1:手動]
            var cmd = new Uint8Array([0x03, 0x00]);
            console.log(cmd);
        }
        window.ledCharacteristic.writeValue(
            cmd
        ).catch(error => {
            uiStatusError(makeErrorMsg(error), false);
        });
    });
})


// サーボ0
window.addEventListener('load', function () {
    var elem = document.getElementById('servo0');
    var target = document.getElementById('servo0_angle');
    var rangeValue = function (elem, target) {
        return function (evt) {
            target.innerHTML = elem.value;

            //0 byte目：サーボ制御コマンド [2]
            //1 byte目：サーボNo [0～2]
            //2 byte目：サーボ角度 [0度～180度]
            //3 byte目：サーボ回転速度 [0～100]
            var cmd = new Uint8Array([0x02, 0x00, elem.value, 30]);
            console.log(cmd);

            window.ledCharacteristic.writeValue(
                cmd
            ).catch(error => {
                uiStatusError(makeErrorMsg(error), false);
            });
        }
    }
    elem.addEventListener('input', function () {
        throttle(rangeValue(elem, target), 500)
    });
})

// サーボ1
window.addEventListener('load', function () {
    var elem = document.getElementById('servo1');
    var target = document.getElementById('servo1_angle');
    var rangeValue = function (elem, target) {
        return function (evt) {
            target.innerHTML = elem.value;

            //0 byte目：サーボ制御コマンド [2]
            //1 byte目：サーボNo [0～2]
            //2 byte目：サーボ角度 [0度～180度]
            //3 byte目：サーボ回転速度 [0～100]
            var cmd = new Uint8Array([0x02, 0x01, elem.value, 30]);
            console.log(cmd);

            window.ledCharacteristic.writeValue(
                cmd
            ).catch(error => {
                uiStatusError(makeErrorMsg(error), false);
            });
        }
    }
    elem.addEventListener('input', function () {
        throttle(rangeValue(elem, target), 500)
    });
})

// サーボ2
window.addEventListener('load', function () {
    var elem = document.getElementById('servo2');
    var target = document.getElementById('servo2_angle');
    var rangeValue = function (elem, target) {
        return function (evt) {
            target.innerHTML = elem.value;

            //0 byte目：サーボ制御コマンド [2]
            //1 byte目：サーボNo [0～2]
            //2 byte目：サーボ角度 [0度～180度]
            //3 byte目：サーボ回転速度 [0～100]
            var cmd = new Uint8Array([0x02, 0x02, elem.value, 30]);
            console.log(cmd);

            window.ledCharacteristic.writeValue(
                cmd
            ).catch(error => {
                uiStatusError(makeErrorMsg(error), false);
            });
        }
    }
    elem.addEventListener('input', function () {
        throttle(rangeValue(elem, target), 500)
    });
})


// イベントを間引く
var throttle = (function (callback, interval = 256) {
    var time = Date.now(),
        lag,
        debounceTimer,
        debounceDelay = 16 * 2;

    return function (callback) {
        lag = time + interval - Date.now();
        if (lag < 0) {
            //console.log( time + "：throttle：" + lag);
            callback();
            time = Date.now();
        } else {
            clearTimeout(debounceTimer);
            debounceTimer = setTimeout(function () {
                //console.log( time + "：debounce：" + (interval - lag + debounceDelay));
                callback();
            }, (interval - lag + debounceDelay));
        }
    }
})();