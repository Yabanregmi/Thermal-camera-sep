const requestConfigSchema = z.undefined(); // oder z.void()

export function onRequestConfig(
  socket: Socket,
  configLock: ConfigLock
) {
  socket.on("REQ_RESET_ALARM", (payload) => {
    // Input-Validierung
    const parseResult = requestConfigSchema.safeParse(payload);
    if (!parseResult.success) {
      socket.emit("ERROR_RESET_ALARM", { reason: "invalid_payload" });
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
