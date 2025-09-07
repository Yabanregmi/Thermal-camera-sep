"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.onRefreshConfigLock = onRefreshConfigLock;
const zod_1 = require("zod");
// Zod-Schema für das Event-Payload (hier: kein Payload)
const refreshConfigLockSchema = zod_1.z.undefined(); // oder: z.void()
function onRefreshConfigLock(socket, configLock, AUTO_RELEASE_MS) {
    socket.on("refreshConfigLock", (data) => {
        // Input-Validierung
        const parseResult = refreshConfigLockSchema.safeParse(data);
        if (!parseResult.success) {
            socket.emit("lockReleasedDenied", { reason: "invalid_payload" });
            return;
        }
        if (configLock.id === socket.id) {
            if (configLock.timeout) {
                clearTimeout(configLock.timeout);
            }
            configLock.expiresAt = Date.now() + AUTO_RELEASE_MS;
            configLock.timeout = setTimeout(() => {
                configLock.id = "";
                configLock.timeout = undefined;
                configLock.expiresAt = undefined;
                socket.emit("lockReleased", { reason: "timeout" });
            }, AUTO_RELEASE_MS);
            const remaining = configLock.expiresAt - Date.now();
            socket.emit("lockTimeoutRefreshed", { remainingMs: remaining });
        }
        else {
            socket.emit("lockReleasedDenied", { reason: "not_lock_owner" });
        }
    });
}
