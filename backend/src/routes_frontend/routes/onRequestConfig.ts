import { Socket } from 'socket.io';
import { z } from 'zod';
import { ConfigLock } from '../../getApi';

// Zod-Schema für das Event-Payload (kein Payload erwartet)
const requestConfigSchema = z.undefined(); // oder z.void()

export function onRequestConfig(
  socket: Socket,
  configLock: ConfigLock
) {
  socket.on("requestConfig", (data) => {
    // Input-Validierung
    const parseResult = requestConfigSchema.safeParse(data);
    if (!parseResult.success) {
      socket.emit("lockConfigDenied", { reason: "invalid_payload" });
      return;
    }

    if (configLock.id === "") {
      configLock.id = socket.id;
      socket.emit("lockConfigSuccess");
    } else {
      socket.emit('lockConfigDenied', { reason: "already_locked", id: configLock.id });
    }
  });
}
