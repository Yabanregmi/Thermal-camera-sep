"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.onReleaseConfig = onReleaseConfig;
const zod_1 = require("zod");
// Zod-Schema für das Event-Payload (hier: kein Payload)
const releaseConfigSchema = zod_1.z.undefined(); // oder z.void()
function onReleaseConfig(socket, configLock) {
    socket.on("releaseConfig", (data) => {
        // Input-Validierung
        const parseResult = releaseConfigSchema.safeParse(data);
        if (!parseResult.success) {
            socket.emit("lockReleasedDenied", { reason: "invalid_payload" });
            return;
        }
        if (configLock.id === socket.id) {
            configLock.id = "";
            socket.emit('lockReleased');
            socket.broadcast.emit('lockFreed');
        }
        else {
            socket.emit('lockReleasedDenied', { reason: "not_lock_owner" });
        }
    });
}
