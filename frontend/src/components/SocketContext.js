// SocketContext.js
import React, { createContext, useEffect, useState } from "react";
import { io } from "socket.io-client";

export const SocketContext = createContext(null);

export function SocketProvider({ children }) {
  const socket = React.useMemo(
    () => io("http://localhost:4000", { withCredentials: true }),
    []
  );

  const [isConnected, setIsConnected] = useState(socket.connected);
  console.log("isConnected", isConnected)

  useEffect(() => {
    function onConnect() {
      console.log("connected");
      setIsConnected(true);
    }

    function onDisconnect() {
      console.log("disconnected")
      setIsConnected(false);
    }
    socket.on('connect', onConnect);
    socket.on('disconnect', onDisconnect);

    return () => {
      socket.off('connect', onConnect);
      socket.off('disconnect', onDisconnect);
    };
  }, [socket]);

  return (
    <SocketContext.Provider value={socket}>{children}</SocketContext.Provider>
  );
}
