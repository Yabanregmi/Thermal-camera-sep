import { onKonfigStart } from './routes/onKonfigStart';
import { onKonfigEnde } from './routes/onKonfigEnde';
import { onGetKonfigStatus } from './routes/onGetKonfigStatus';
import { onKonfigReady } from './routes/onKonfigReady';
import { onSetSchwelle } from './routes/onSetSchwelle';
import { onGetSchwelle } from './routes/onGetSchwelle';
import { onLiveTemperatur, TemperaturMsg} from './routes/onLiveTemperatur';
import { onGetHistogramm } from './routes/onGetHistogramm';
import { onRequestConfig } from './routes/onRequestConfig';
import { onReleaseConfig } from './routes/onReleaseConfig';
import { onRefreshConfigLock } from './routes/onRefreshConfigLock';
import { ConfigLock } from '../getApi';
import { onDisconnect } from './routes/onDisconnect';
import { Server as SocketIOServer, Socket } from 'socket.io';
import { Database } from '../database/Sqlite3Database';

import { config } from 'process';

// Typ für die Factory-Props
export interface SocketHandlersProps {
  server_socket: SocketIOServer;
  database: Database;
  configLock : ConfigLock;
  konfigAckTimeout: NodeJS.Timeout | null;
  setKonfigAckTimeout: (v: NodeJS.Timeout | null) => void;
  konfigReady: boolean;
  setKonfigReady: (v: boolean) => void;
  temperSchwelle: number;
  setTemperSchwelle: (v: number) => void;
  letzteLiveTemperatur: TemperaturMsg | null;
  setLetzteLiveTemperatur: (v: TemperaturMsg | null) => void;
}

const AUTO_RELEASE_MS : number = 1000*60*3;
/**
 * Factory-Funktion, die bei jeder neuen Socket-Verbindung alle relevanten Event-Handler registriert.
 *
 * @param param0 - Enthält alle globalen States und Setter/Getter sowie die Server-Instanz und Datenbank.
 * @returns Handler, der beim 'connection'-Event von Socket.io aufgerufen wird.
 */
export function createSocketHandlers({
  server_socket,
  database,
  configLock,
  konfigAckTimeout,
  setKonfigAckTimeout,
  konfigReady,
  setKonfigReady,
  temperSchwelle,
  setTemperSchwelle,
  letzteLiveTemperatur,
  setLetzteLiveTemperatur
}: SocketHandlersProps) {
  return function (socket: Socket) {
    onDisconnect(socket, configLock);
    onRequestConfig(socket, configLock);
    onReleaseConfig(socket, configLock);
    onRefreshConfigLock(socket, configLock, AUTO_RELEASE_MS);
    // Handler für den Start der Konfiguration
    onKonfigStart(socket, server_socket, {
      konfigAckTimeoutRef: () => konfigAckTimeout,
      setKonfigAckTimeout,
      konfigReadyRef: () => konfigReady,
      setKonfigReady
    });
    // Handler für das Ende der Konfiguration
    onKonfigEnde(socket, server_socket, {
      konfigAckTimeoutRef: () => konfigAckTimeout,
      setKonfigReady
    });
    // Handler für Statusabfrage der Konfiguration
    onGetKonfigStatus(socket, server_socket);

    // Handler für das Event "Konfig bereit"
    onKonfigReady(socket, server_socket, {
      konfigAckTimeoutRef: () => konfigAckTimeout,
      setKonfigReady
    });

    // Handler für das Setzen und Abfragen der Temperaturschwelle
    onSetSchwelle(socket, server_socket, () => setTemperSchwelle);
    onGetSchwelle(socket, server_socket, () => temperSchwelle, setTemperSchwelle);

    // Handler für Live-Temperaturdaten
    onLiveTemperatur(socket, server_socket, () => letzteLiveTemperatur, setLetzteLiveTemperatur);

    // Handler für das Histogramm der Temperaturen
    onGetHistogramm(socket, database);
  };
}
