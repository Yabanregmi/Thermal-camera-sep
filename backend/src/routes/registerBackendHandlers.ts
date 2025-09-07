import { Server as SocketIOServer, Socket } from 'socket.io';

export function registerBackendHandlers(ioFrontend: SocketIOServer) {
    return (socket: Socket) => {
    
    socket.on("disconnect", (payload) => {
      console.log("Disconnect-Event:", payload);
      // Kein Weiterleiten nötig, nur Logging
    });

    socket.on("connect_error", (payload) => {
      console.log("Connect-Error-Event:", payload);
      // Kein Weiterleiten nötig, nur Logging
    });

    socket.on("ACK_RESET_ALARM", (payload) => {
      console.log("ACK Reset Alarm:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_RESET_ALARM", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_RESET_ERROR", (payload) => {
      console.log("ACK Reset Error:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_RESET_ERROR", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_SET_CONFIG", (payload) => {
      console.log("ACK Set Config:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_SET_CONFIG", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_SET_TEMPRETURE", (payload) => {
      console.log("ACK Set Tempreture:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_SET_TEMPRETURE", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_MANUAL_START_RECORD", (payload) => {
      console.log("ACK Manual Start Record:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_MANUAL_START_RECORD", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_MANUAL_STOP_RECORD", (payload) => {
      console.log("ACK Manual Stop Record:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_MANUAL_STOP_RECORD", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_TIMEOUT_STOP_RECORD", (payload) => {
      console.log("ACK Timeout Stop Record:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_TIMEOUT_STOP_RECORD", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_MANUAL_CALL_RECORD", (payload) => {
      console.log("ACK Manual Call Record:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_MANUAL_CALL_RECORD", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_CALL_LIVE_TEMPRETURE", (payload) => {
      console.log("ACK Call Live Tempreture:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_CALL_LIVE_TEMPRETURE", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_CALL_HISTORY_TEMPRETURE", (payload) => {
      console.log("ACK Call History Tempreture:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_CALL_HISTORY_TEMPRETURE", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_SET_EVENT", (payload) => {
      console.log("ACK Set Event:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_SET_EVENT", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("ACK_MESSAGE", (payload) => {
      console.log("ACK Message:", payload);
      if (ioFrontend.sockets.sockets.size > 0) {
        ioFrontend.emit("ACK_MESSAGE", payload);
        console.log("Event an Frontend weitergeleitet.");
      } else {
        console.warn("Kein Frontend-Client verbunden! Event nicht gesendet.");
      }
    });

    socket.on("REQ_TEST", (payload) => {
      console.log("REQ_TEST:", payload);
    });
  }
}