"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.onLiveTemperatur = onLiveTemperatur;
const crc_1 = __importDefault(require("crc"));
const zod_1 = require("zod");
// Zod-Schema für LiveTemperaturData
const liveTemperaturSchema = zod_1.z.object({
    wert: zod_1.z.number(),
    zeit: zod_1.z.number(),
    crc: zod_1.z.union([zod_1.z.string(), zod_1.z.number()])
});
/**
 * Hilfsfunktion: Berechnet die CRC32-Prüfsumme aus Wert und Zeit
 */
function getCRC(wert, zeit) {
    return crc_1.default.crc32(`${wert}:${zeit}`).toString();
}
/**
 * Handler für Live-Temperaturdaten via Socket.io
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param io - Die Socket.io-Server-Instanz (für Broadcasts)
 * @param getLetzteLiveTemperatur - Getter für die letzte Temperatur
 * @param setLetzteLiveTemperatur - Setter für die letzte Temperatur
 */
function onLiveTemperatur(socket, io, getLetzteLiveTemperatur, setLetzteLiveTemperatur) {
    // Nach Verbindungsaufbau: Sende letzte bekannte Temperatur an den Client (falls vorhanden)
    const last = getLetzteLiveTemperatur();
    if (last) {
        socket.emit('temperatur', last);
    }
    // Event-Handler für neue Live-Temperaturdaten von der Python-App
    socket.on("liveTemperatur", (data) => {
        // Input-Validierung mit zod
        const parseResult = liveTemperaturSchema.safeParse(data);
        if (!parseResult.success) {
            console.log("Ungültige Datenstruktur von Python-Client!", data);
            socket.emit('temperaturError', 'Ungültige Datenstruktur!');
            return;
        }
        const { wert, zeit, crc: receivedCRC } = parseResult.data;
        // Prüfe CRC32-Prüfsumme
        const crcCheck = getCRC(wert, zeit);
        if (crcCheck === receivedCRC.toString()) {
            const msg = {
                id: Date.now(),
                wert,
                zeit
            };
            setLetzteLiveTemperatur(msg); // Temperatur speichern
            io.emit('temperatur', msg); // An alle Clients senden
        }
        else {
            console.log("Ungültige CRC32-Prüfsumme von Python-Client!", data);
            socket.emit('temperaturError', 'Ungültige CRC32-Prüfsumme!');
        }
    });
}
