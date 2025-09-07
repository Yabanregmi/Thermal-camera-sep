"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.backendEventHandlers = void 0;
function eventHandlerBackendToFrontend(ioFrontend) {
    return {
        "ACK_SET_CONFIG": (payload, socket) => {
            if (ioFrontend.sockets.sockets.size > 0) {
                ioFrontend.emit("ACK_SET_CONFIG", payload);
            }
            else {
                console.warn("Kein Frontend-Client verbunden!");
            }
        },
        // ... weitere Events
    };
}
const backendToAppEventHandlers = {
    "connect": (payload, socket) => {
        console.log("Connect-Event:", payload);
        // Spezifische Logik für connect
    },
    "disconnect": (payload, socket) => {
        console.log("Disconnect-Event:", payload);
        // Spezifische Logik für disconnect
    },
    "connect_error": (payload, socket) => {
        console.log("Connect-Error-Event:", payload);
        // Spezifische Logik für connect_error
    },
    "REQ_RESET_ALARM": (payload, socket) => {
        console.log("Reset Alarm:", payload);
        // Spezifische Logik für REQ_RESET_ALARM
    },
    "REQ_RESET_ERROR": (payload, socket) => {
        console.log("Reset Error:", payload);
        // Spezifische Logik für REQ_RESET_ERROR
    },
    "REQ_SET_CONFIG": (payload, socket) => {
        console.log("Config-Event:", payload);
        // Spezifische Logik für REQ_SET_CONFIG
    },
    "REQ_SET_TEMPRETURE": (payload, socket) => {
        console.log("Set Tempreture:", payload);
        // Spezifische Logik für REQ_SET_TEMPRETURE
    },
    "REQ_MANUAL_START_RECORD": (payload, socket) => {
        console.log("Manual Start Record:", payload);
        // Spezifische Logik für REQ_MANUAL_START_RECORD
    },
    "REQ_MANUAL_STOP_RECORD": (payload, socket) => {
        console.log("Manual Stop Record:", payload);
        // Spezifische Logik für REQ_MANUAL_STOP_RECORD
    },
    "REQ_MANUAL_CALL_RECORD": (payload, socket) => {
        console.log("Manual Call Record:", payload);
        // Spezifische Logik für REQ_MANUAL_CALL_RECORD
    },
    "REQ_CALL_LIVE_TEMPRETURE": (payload, socket) => {
        console.log("Call Live Tempreture:", payload);
        // Spezifische Logik für REQ_CALL_LIVE_TEMPRETURE
    },
    "REQ_CALL_HISTORY_TEMPRETURE": (payload, socket) => {
        console.log("Call History Tempreture:", payload);
        // Spezifische Logik für REQ_CALL_HISTORY_TEMPRETURE
    },
    "REQ_SET_EVENT": (payload, socket) => {
        console.log("Set Event:", payload);
        // Spezifische Logik für REQ_SET_EVENT
    },
    "MESSAGE": (payload, socket) => {
        console.log("Message:", payload);
        // Spezifische Logik für MESSAGE
    },
    // ACK Events (SocketEventsToBackend)
    "ACK_RESET_ALARM": (payload, socket) => {
        console.log("ACK Reset Alarm:", payload);
        // Spezifische Logik für ACK_RESET_ALARM
    },
    "ACK_RESET_ERROR": (payload, socket) => {
        console.log("ACK Reset Error:", payload);
        // Spezifische Logik für ACK_RESET_ERROR
    },
    "ACK_SET_CONFIG": (payload, socket) => {
        console.log("ACK Set Config:", payload);
        // Spezifische Logik für ACK_SET_CONFIG
    },
    "ACK_SET_TEMPRETURE": (payload, socket) => {
        console.log("ACK Set Tempreture:", payload);
        // Spezifische Logik für ACK_SET_TEMPRETURE
    },
    "ACK_MANUAL_START_RECORD": (payload, socket) => {
        console.log("ACK Manual Start Record:", payload);
        // Spezifische Logik für ACK_MANUAL_START_RECORD
    },
    "ACK_MANUAL_STOP_RECORD": (payload, socket) => {
        console.log("ACK Manual Stop Record:", payload);
        // Spezifische Logik für ACK_MANUAL_STOP_RECORD
    },
    "ACK_TIMEOUT_STOP_RECORD": (payload, socket) => {
        console.log("ACK Timeout Stop Record:", payload);
        // Spezifische Logik für ACK_TIMEOUT_STOP_RECORD
    },
    "ACK_MANUAL_CALL_RECORD": (payload, socket) => {
        console.log("ACK Manual Call Record:", payload);
        // Spezifische Logik für ACK_MANUAL_CALL_RECORD
    },
    "ACK_CALL_LIVE_TEMPRETURE": (payload, socket) => {
        console.log("ACK Call Live Tempreture:", payload);
        // Spezifische Logik für ACK_CALL_LIVE_TEMPRETURE
    },
    "ACK_CALL_HISTORY_TEMPRETURE": (payload, socket) => {
        console.log("ACK Call History Tempreture:", payload);
        // Spezifische Logik für ACK_CALL_HISTORY_TEMPRETURE
    },
    "ACK_SET_EVENT": (payload, socket) => {
        console.log("ACK Set Event:", payload);
        // Spezifische Logik für ACK_SET_EVENT
    },
    "ACK_MESSAGE": (payload, socket) => {
        console.log("ACK Message:", payload);
        // Spezifische Logik für ACK_MESSAGE
    },
};
exports.backendEventHandlers = backendToAppEventHandlers;
