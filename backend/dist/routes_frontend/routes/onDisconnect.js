"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.onDisconnect = onDisconnect;
function onDisconnect(socket, configLock) {
    socket.on("disconnect", () => {
        console.log("Disconnet");
        if (configLock.id === socket.id) {
            configLock.id = "";
            socket.broadcast.emit("lockFreed");
        }
    });
}
