import { Socket } from 'socket.io';
import { z } from 'zod';
import { ConfigLock } from '../../getApi';

// Zod-Schema für das Event-Payload (hier: kein Payload)
const releaseConfigSchema = z.undefined(); // oder z.void()

export function onReleaseConfig(
  socket: Socket,
  configLock: ConfigLock
) {
  socket.on("releaseConfig", (data) => {
    // Input-Validierung
    const parseResult = releaseConfigSchema.safeParse(data);
    if (!parseResult.success) {
      socket.emit("lockReleasedDenied", { reason: "invalid_payload" });
      return;
    }

    if (configLock.id === socket.id) {
      configLock.id = "";
      socket.emit('lockReleased');
      socket.broadcast.emit('lockFreed')
    } else {
      socket.emit('lockReleasedDenied', { reason: "not_lock_owner" });
    }
  });
}
