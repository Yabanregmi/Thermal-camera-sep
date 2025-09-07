import { Socket, Server as SocketIOServer } from 'socket.io';
import { z } from 'zod';

/**
 * Handler für das "konfigReady"-Event via Socket.io
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param io - Die Socket.io-Server-Instanz (für Broadcasts)
 * @param options - Objekt mit Timeout-Getter und Setter für den Konfigurationsstatus
 */
export function onKonfigReady(
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
  // Zod-Schema für das Event (es wird keine Payload erwartet)
  const payloadSchema = z.undefined();

  // Registriere Event-Handler für "konfigReady"
  socket.on("konfigReady", (payload) => {
    // Input-Validierung mit zod
    const parseResult = payloadSchema.safeParse(payload);
    if (!parseResult.success) {
      socket.emit("error", { error: "Ungültige Daten für konfigReady-Event." });
      return;
    }

    setKonfigReady(true); // Setze globalen Status auf "bereit"

    // Lösche ggf. bestehenden Timeout
    const timeout = konfigAckTimeoutRef();
    if (timeout) clearTimeout(timeout);

    io.emit("info", "System bereit zur Konfiguration."); // Broadcast an alle Clients
    console.log("info", "System bereit zur Konfiguration.");
  });
}
