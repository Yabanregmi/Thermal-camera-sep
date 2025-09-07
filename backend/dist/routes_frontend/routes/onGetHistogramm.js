"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.onGetHistogramm = onGetHistogramm;
const zod_1 = require("zod");
/**
 * Registriert den Handler für das Event "getTemperaturHistogramm" auf dem Socket.
 * Wenn das Event ausgelöst wird, liest die Funktion das Temperatur-Histogramm aus der Datenbank
 * und sendet es an den Client zurück.
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param database - Die (async/await-fähige) Datenbankinstanz
 */
function onGetHistogramm(socket, database) {
    // Zod-Schema für das Event (es wird keine Payload erwartet)
    const payloadSchema = zod_1.z.undefined();
    // Registriere Listener für das Event "getTemperaturHistogramm"
    socket.on("getTemperaturHistogramm", async (payload) => {
        // Input-Validierung mit zod
        const parseResult = payloadSchema.safeParse(payload);
        if (!parseResult.success) {
            socket.emit("temperaturHistogrammError", "Ungültige Daten für getTemperaturHistogramm-Event.");
            return;
        }
        try {
            // Hole das Histogramm aus der Datenbank (z.B. als Array von "Bins")
            const bins = await database.getHistogramm();
            // Sende das Ergebnis an den Client zurück
            socket.emit("temperaturHistogramm", bins);
        }
        catch (err) {
            // Bei Fehler: Sende eine Fehlermeldung an den Client
            socket.emit("temperaturHistogrammError", "Fehler beim Lesen der Datenbank");
        }
    });
}
