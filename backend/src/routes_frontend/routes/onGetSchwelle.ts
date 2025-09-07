import { Socket, Server as SocketIOServer } from 'socket.io';
import { z } from 'zod';

/**
 * Handler für das Schwellenwert-Event via Socket.io
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param io - Die Socket.io-Server-Instanz (für Broadcasts)
 * @param getSchwelle - Getter für den aktuellen Schwellenwert
 * @param setSchwelle - Setter für den Schwellenwert
 */
export function onGetSchwelle(
  socket: Socket,
  io: SocketIOServer,
  getSchwelle: () => number,
  setSchwelle: (wert: number) => void
) {
  // Aktuellen Schwellenwert anfragenden Client senden
  socket.on("getSchwelle", () => {
    socket.emit("schwelle", getSchwelle());
  });

  // Zod-Schema für das setSchwelle-Event (erlaubt nur Zahlen von 0 bis 200)
  const schwelleSchema = z.number().min(0).max(200);

  // Schwellenwert setzen, validieren und an alle Clients broadcasten
  socket.on("setSchwelle", (wert) => {
    const parseResult = schwelleSchema.safeParse(wert);
    if (!parseResult.success) {
      socket.emit("schwelleError", "Ungültiger Wert (muss Zahl zwischen 0 und 200 sein)");
      return;
    }
    const parsed = parseResult.data;
    setSchwelle(parsed);
    io.emit("schwelle", parsed); // Broadcast an alle Clients
    socket.emit("info", `Neue Schwelle gesetzt: ${parsed}°C`);
    console.log("Neue Schwelle gesetzt:", parsed);
  });
}
