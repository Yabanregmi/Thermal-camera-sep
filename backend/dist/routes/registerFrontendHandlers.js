"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.registerFrontendHandlers = registerFrontendHandlers;
function registerFrontendHandlers(ioApp) {
    return (socket) => {
        socket.on("connect", () => {
            console.log("Connect-Event");
            ioApp.emit('REQ_SET_CONFIG', { foo: 'bar' });
        });
        socket.on("disconnect", () => {
            console.log("Disconnect-Event");
            // Spezifische Logik für disconnect
        });
        socket.on("connect_error", (payload) => {
            console.log("Connect-Error-Event:", payload);
            // Spezifische Logik für connect_error
        });
        socket.on("REQ_RESET_ALARM", (payload) => {
            console.log("Reset Alarm:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_RESET_ALARM", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("REQ_RESET_ERROR", (payload) => {
            console.log("Reset Error:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_RESET_ERROR", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("REQ_SET_CONFIG", (payload) => {
            console.log("Config-Event:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_SET_CONFIG", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("REQ_SET_TEMPRETURE", (payload) => {
            console.log("Set Tempreture:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_SET_TEMPRETURE", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("REQ_MANUAL_START_RECORD", (payload) => {
            console.log("Manual Start Record:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_MANUAL_START_RECORD", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("REQ_MANUAL_STOP_RECORD", (payload) => {
            console.log("Manual Stop Record:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_MANUAL_STOP_RECORD", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("REQ_MANUAL_CALL_RECORD", (payload) => {
            console.log("Manual Call Record:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_MANUAL_CALL_RECORD", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("REQ_CALL_LIVE_TEMPRETURE", (payload) => {
            console.log("Call Live Tempreture:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_CALL_LIVE_TEMPRETURE", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("REQ_CALL_HISTORY_TEMPRETURE", (payload) => {
            console.log("Call History Tempreture:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_CALL_HISTORY_TEMPRETURE", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("REQ_SET_EVENT", (payload) => {
            console.log("Set Event:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("REQ_SET_EVENT", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
        socket.on("MESSAGE", (payload) => {
            console.log("Message:", payload);
            if (ioApp.sockets.sockets.size > 0) {
                ioApp.emit("MESSAGE", payload);
                console.log("Event an Backend-App weitergeleitet.");
            }
            else {
                console.warn("Kein Backend-App-Client verbunden! Event nicht gesendet.");
            }
        });
    };
}
