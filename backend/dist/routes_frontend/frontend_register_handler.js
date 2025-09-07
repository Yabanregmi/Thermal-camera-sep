"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.createSocketHandlers = createSocketHandlers;
const onKonfigStart_1 = require("./routes/onKonfigStart");
const onKonfigEnde_1 = require("./routes/onKonfigEnde");
const onGetKonfigStatus_1 = require("./routes/onGetKonfigStatus");
const onKonfigReady_1 = require("./routes/onKonfigReady");
const onSetSchwelle_1 = require("./routes/onSetSchwelle");
const onGetSchwelle_1 = require("./routes/onGetSchwelle");
const onLiveTemperatur_1 = require("./routes/onLiveTemperatur");
const onGetHistogramm_1 = require("./routes/onGetHistogramm");
const onRequestConfig_1 = require("./routes/onRequestConfig");
const onReleaseConfig_1 = require("./routes/onReleaseConfig");
const onRefreshConfigLock_1 = require("./routes/onRefreshConfigLock");
const onDisconnect_1 = require("./routes/onDisconnect");
const AUTO_RELEASE_MS = 1000 * 60 * 3;
/**
 * Factory-Funktion, die bei jeder neuen Socket-Verbindung alle relevanten Event-Handler registriert.
 *
 * @param param0 - Enthält alle globalen States und Setter/Getter sowie die Server-Instanz und Datenbank.
 * @returns Handler, der beim 'connection'-Event von Socket.io aufgerufen wird.
 */
function createSocketHandlers({ server_socket, database, configLock, konfigAckTimeout, setKonfigAckTimeout, konfigReady, setKonfigReady, temperSchwelle, setTemperSchwelle, letzteLiveTemperatur, setLetzteLiveTemperatur }) {
    return function (socket) {
        (0, onDisconnect_1.onDisconnect)(socket, configLock);
        (0, onRequestConfig_1.onRequestConfig)(socket, configLock);
        (0, onReleaseConfig_1.onReleaseConfig)(socket, configLock);
        (0, onRefreshConfigLock_1.onRefreshConfigLock)(socket, configLock, AUTO_RELEASE_MS);
        // Handler für den Start der Konfiguration
        (0, onKonfigStart_1.onKonfigStart)(socket, server_socket, {
            konfigAckTimeoutRef: () => konfigAckTimeout,
            setKonfigAckTimeout,
            konfigReadyRef: () => konfigReady,
            setKonfigReady
        });
        // Handler für das Ende der Konfiguration
        (0, onKonfigEnde_1.onKonfigEnde)(socket, server_socket, {
            konfigAckTimeoutRef: () => konfigAckTimeout,
            setKonfigReady
        });
        // Handler für Statusabfrage der Konfiguration
        (0, onGetKonfigStatus_1.onGetKonfigStatus)(socket, server_socket);
        // Handler für das Event "Konfig bereit"
        (0, onKonfigReady_1.onKonfigReady)(socket, server_socket, {
            konfigAckTimeoutRef: () => konfigAckTimeout,
            setKonfigReady
        });
        // Handler für das Setzen und Abfragen der Temperaturschwelle
        (0, onSetSchwelle_1.onSetSchwelle)(socket, server_socket, () => setTemperSchwelle);
        (0, onGetSchwelle_1.onGetSchwelle)(socket, server_socket, () => temperSchwelle, setTemperSchwelle);
        // Handler für Live-Temperaturdaten
        (0, onLiveTemperatur_1.onLiveTemperatur)(socket, server_socket, () => letzteLiveTemperatur, setLetzteLiveTemperatur);
        // Handler für das Histogramm der Temperaturen
        (0, onGetHistogramm_1.onGetHistogramm)(socket, database);
    };
}
