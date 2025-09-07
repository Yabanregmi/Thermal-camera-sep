"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.onRequestConfig = onRequestConfig;
const zod_1 = require("zod");
// Zod-Schema für das Event-Payload (kein Payload erwartet)
const requestConfigSchema = zod_1.z.undefined(); // oder z.void()
function onRequestConfig(socket, configLock) {
    socket.on("requestConfig", (data) => {
        // Input-Validierung
        const parseResult = requestConfigSchema.safeParse(data);
        if (!parseResult.success) {
            socket.emit("lockConfigDenied", { reason: "invalid_payload" });
            return;
        }
        if (configLock.id === "") {
            configLock.id = socket.id;
            socket.emit("lockConfigSuccess");
        }
        else {
            socket.emit('lockConfigDenied', { reason: "already_locked", id: configLock.id });
        }
    });
}
