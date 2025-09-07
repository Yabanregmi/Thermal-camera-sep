import { Socket } from 'socket.io';
import { ConfigLock } from '../../getApi';

export function onDisconnect(
  socket: Socket,
  configLock: ConfigLock
) {
  socket.on("disconnect", () => {
    console.log("Disconnet");
    if (configLock.id === socket.id) {
      configLock.id = "";
      socket.broadcast.emit("lockFreed");
    }
  });
}
