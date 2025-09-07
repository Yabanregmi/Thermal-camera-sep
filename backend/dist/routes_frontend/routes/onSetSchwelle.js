"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.onSetSchwelle = onSetSchwelle;
const zod_1 = require("zod");
/**
 * Handler für das Setzen des Schwellenwerts via Socket.io
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param io - Die Socket.io-Server-Instanz (für Broadcasts)
 * @param setSchwelle - Setter für den Schwellenwert
 */
function onSetSchwelle(socket, io, setSchwelle) {
    // Zod-Schema für Schwellenwert (nur Zahlen von 0 bis 200 erlaubt)
    const schwelleSchema = zod_1.z.number().min(0).max(200);
    // Registriere Event-Handler für "setSchwelle"
    socket.on("setSchwelle", (wert) => {
        // Input-Validierung mit zod
        // Versuche, Wert als Zahl zu parsen (sicherstellen, dass auch stringifizierte Zahlen akzeptiert werden)
        let parsedWert;
        if (typeof wert === 'number') {
            parsedWert = wert;
        }
        else if (typeof wert === 'string' && wert.trim() !== '') {
            parsedWert = Number(wert);
        }
        const parseResult = schwelleSchema.safeParse(parsedWert);
        if (!parseResult.success) {
            socket.emit("schwelleError", "Ungültiger Wert (muss Zahl zwischen 0 und 200 sein)");
            return;
        }
        const validWert = parseResult.data;
        setSchwelle(validWert); // Schwellenwert speichern/updaten
        io.emit("schwelle", validWert); // Broadcast an alle Clients
        socket.emit("info", `Neue Schwelle gesetzt: ${validWert}°C`);
        console.log("Neue Schwelle gesetzt:", validWert);
    });
}
