import crc from 'crc';
import { Socket, Server as SocketIOServer } from 'socket.io';
import { z } from 'zod';

// Typ für eine Temperatur-Nachricht
export interface TemperaturMsg {
  id: number;
  wert: number;
  zeit: number;
}

// Typ für die empfangenen Daten vom Python-Client
export interface LiveTemperaturData {
  wert: number;
  zeit: number;
  crc: string | number;
}

// Zod-Schema für LiveTemperaturData
const liveTemperaturSchema = z.object({
  wert: z.number(),
  zeit: z.number(),
  crc: z.union([z.string(), z.number()])
});

/**
 * Hilfsfunktion: Berechnet die CRC32-Prüfsumme aus Wert und Zeit
 */
function getCRC(wert: number, zeit: number): string {
  return crc.crc32(`${wert}:${zeit}`).toString();
}

/**
 * Handler für Live-Temperaturdaten via Socket.io
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param io - Die Socket.io-Server-Instanz (für Broadcasts)
 * @param getLetzteLiveTemperatur - Getter für die letzte Temperatur
 * @param setLetzteLiveTemperatur - Setter für die letzte Temperatur
 */
export function onLiveTemperatur(
  socket: Socket,
  io: SocketIOServer,
  getLetzteLiveTemperatur: () => TemperaturMsg | null | undefined,
  setLetzteLiveTemperatur: (msg: TemperaturMsg) => void
) {
  // Nach Verbindungsaufbau: Sende letzte bekannte Temperatur an den Client (falls vorhanden)
  const last = getLetzteLiveTemperatur();
  if (last) {
    socket.emit('temperatur', last);
  }

  // Event-Handler für neue Live-Temperaturdaten von der Python-App
  socket.on("liveTemperatur", (data: unknown) => {
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
      const msg: TemperaturMsg = {
        id: Date.now(),
        wert,
        zeit
      };
      setLetzteLiveTemperatur(msg);   // Temperatur speichern
      io.emit('temperatur', msg);     // An alle Clients senden
    } else {
      console.log("Ungültige CRC32-Prüfsumme von Python-Client!", data);
      socket.emit('temperaturError', 'Ungültige CRC32-Prüfsumme!');
    }
  });
}
