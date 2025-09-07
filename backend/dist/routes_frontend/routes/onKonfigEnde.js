"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.onKonfigEnde = onKonfigEnde;
const zod_1 = require("zod");
/**
 * Handler für das Ende des Konfigurationsmodus via Socket.io
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param io - Die Socket.io-Server-Instanz (für Broadcasts)
 * @param options - Objekt mit Timeout-Getter und Setter für den Konfigurationsstatus
 */
function onKonfigEnde(socket, io, { konfigAckTimeoutRef, setKonfigReady }) {
    let imKonfigModus = false; // Lokaler Status für den Konfigurationsmodus
    // Zod-Schema für das Event (es wird keine Payload erwartet)
    const payloadSchema = zod_1.z.undefined();
    // Event-Handler für das "konfigEnde"-Event
    socket.on("konfigEnde", (payload) => {
        // Input-Validierung mit zod
        const parseResult = payloadSchema.safeParse(payload);
        if (!parseResult.success) {
            socket.emit("error", { error: "Ungültige Daten für konfigEnde-Event." });
            return;
        }
        // Prüfe, ob der Modus überhaupt aktiv ist
        if (!imKonfigModus) {
            socket.emit("info", "Konfigurationsmodus ist nicht aktiv.");
            return;
        }
        imKonfigModus = false; // Modus beenden
        io.emit("konfigModusEnde"); // Broadcast an alle Clients
        const timeout = konfigAckTimeoutRef();
        if (timeout)
            clearTimeout(timeout); // Timeout ggf. löschen
        setKonfigReady(false); // Globalen Status zurücksetzen
        socket.emit("info", "Konfigurationsmodus verlassen.");
    });
}
