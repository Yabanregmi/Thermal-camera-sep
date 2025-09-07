import { Socket, Server as SocketIOServer } from 'socket.io';
import { z } from 'zod';

/**
 * Handler für das Starten des Konfigurationsmodus via Socket.io
 *
 * @param socket - Die Socket.io-Verbindung zum Client
 * @param io - Die Socket.io-Server-Instanz (für Broadcasts)
 * @param options - Objekt mit Timeout-Getter/Setter und Getter/Setter für den Konfigurationsstatus
 */
export function onKonfigStart(
  socket: Socket,
  io: SocketIOServer,
  {
    konfigAckTimeoutRef,
    setKonfigAckTimeout,
    konfigReadyRef,
    setKonfigReady
  }: {
    konfigAckTimeoutRef: () => NodeJS.Timeout | null | undefined,
    setKonfigAckTimeout: (timeout: NodeJS.Timeout) => void,
    konfigReadyRef: () => boolean,
    setKonfigReady: (status: boolean) => void
  }
) {
  let imKonfigModus = false; // Lokaler Status für diesen Socket

  // Zod-Schema für das Event (es wird keine Payload erwartet)
  const payloadSchema = z.undefined();

  socket.on("konfigStart", (payload) => {
    // Input-Validierung mit zod
    const parseResult = payloadSchema.safeParse(payload);
    if (!parseResult.success) {
      socket.emit("error", { error: "Ungültige Daten für konfigStart-Event." });
      return;
    }

    console.log("konfigStart");
    if (imKonfigModus) {
      socket.emit("info", "Konfigurationsmodus ist bereits aktiv.");
      return;
    }
    imKonfigModus = true;
    setKonfigReady(false); // Setze globalen Status auf "nicht bereit"
    
    io.emit("konfigModusStart"); // Broadcast an alle Clients (inkl. Python-App)

    // Vorherigen Timeout ggf. löschen
    const prevTimeout = konfigAckTimeoutRef();
    if (prevTimeout) clearTimeout(prevTimeout);

    // Timeout für die Bestätigung der Python-App starten (5 Sekunden)
    const timeout = setTimeout(() => {
      if (!konfigReadyRef()) {
        socket.emit("info", "Python app - No Response. Config not allowed.");
        imKonfigModus = false;
      }
    }, 5000);
    setKonfigAckTimeout(timeout); // Timeout speichern
  });
}
