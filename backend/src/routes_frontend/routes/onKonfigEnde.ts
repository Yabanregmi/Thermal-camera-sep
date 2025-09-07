import { Socket, Server as SocketIOServer } from 'socket.io';
import { z } from 'zod';

/**
 * Handler für das Ende des Konfigurationsmodus via Socket.io
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param io - Die Socket.io-Server-Instanz (für Broadcasts)
 * @param options - Objekt mit Timeout-Getter und Setter für den Konfigurationsstatus
 */
export function onKonfigEnde(
  socket: Socket,
  io: SocketIOServer,
  {
    konfigAckTimeoutRef,
    setKonfigReady
  }: {
    konfigAckTimeoutRef: () => NodeJS.Timeout | null | undefined,
    setKonfigReady: (status: boolean) => void
  }
) {
  let imKonfigModus = false; // Lokaler Status für den Konfigurationsmodus

  // Zod-Schema für das Event (es wird keine Payload erwartet)
  const payloadSchema = z.undefined();

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
    if (timeout) clearTimeout(timeout); // Timeout ggf. löschen
    setKonfigReady(false); // Globalen Status zurücksetzen
    socket.emit("info", "Konfigurationsmodus verlassen.");
  });
}
