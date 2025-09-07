import { Socket, Server as SocketIOServer } from 'socket.io';
import { z } from 'zod';

/**
 * Handler für das "getKonfigStatus"-Event via Socket.io
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param io - Die Socket.io-Server-Instanz (für Broadcasts)
 */
export function onGetKonfigStatus(socket: Socket, io: SocketIOServer) {
  let imKonfigModus = false; // Lokaler Status für den Konfigurationsmodus

  // Zod-Schema für das Event (hier: keine Daten erwartet)
  const payloadSchema = z.undefined();

  socket.on("getKonfigStatus", (payload) => {
    // Input-Validierung mit zod
    const parseResult = payloadSchema.safeParse(payload);
    if (!parseResult.success) {
      // Optional: Logging oder Security-Response
      socket.emit("error", { error: "Invalid payload for getKonfigStatus" });
      return;
    }

    // Broadcast an alle Clients
    io.emit("konfigStatus", imKonfigModus);
  });
}
