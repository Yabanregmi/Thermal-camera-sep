import React, { useContext, useEffect, useState, useRef } from "react";
import { SocketContext } from "./SocketContext";
import User1Features from "./User1TemperaturAuslesen";

export default function User2KonfigModus({ onLogout }) {
  const socket = useContext(SocketContext);

  const [schwelle, setSchwelle] = useState("");
  const [aktuelleSchwelle, setAktuelleSchwelle] = useState(null);
  const [konfigModus, setKonfigModus] = useState(false);
  const [loadingKonfig, setLoadingKonfig] = useState(false);
  const [nachricht, setNachricht] = useState("");
  const [lockStatus, setLockStatus] = useState("none"); // "none", "waiting", "granted", "denied"
  const [lockDeniedReason, setLockDeniedReason] = useState("");
  const timeoutRef = useRef(null);

  // === LOCK-LOGIK ===

  // Lock anfragen
  const handleLockRequest = () => {
    setLockStatus("waiting");
    setNachricht("Frage Lock für Konfigurationsmodus an...");
    socket.emit("requestConfig");
  };

  // Lock explizit freigeben
  const handleLockRelease = () => {
    socket.emit("releaseConfig");
    setNachricht("Lock wird freigegeben...");
    setLockStatus("none");
  };

  useEffect(() => {
    function handleLockSuccess() {
      setLockStatus("granted");
      setNachricht("Lock erhalten! Du kannst jetzt den Konfigurationsmodus starten.");
      setLoadingKonfig(false);
    }
    function handleLockDenied(data) {
      setLockStatus("denied");
      setLockDeniedReason(data?.reason || "");
      setNachricht(
        data?.reason === "already_locked"
          ? "Lock ist bereits von einem anderen Benutzer belegt. Warte auf Freigabe."
          : "Lock abgelehnt: " + (data?.reason || "")
      );
      setLoadingKonfig(false);
    }
    function handleLockReleased() {
      setLockStatus("none");
      setNachricht("Lock wurde freigegeben.");
    }
    function handleLockReleasedDenied(data) {
      setNachricht("Lock konnte nicht freigegeben werden: " + (data?.reason || ""));
    }
    function handleLockFreed() {
      setNachricht("Lock ist frei! Du kannst es erneut versuchen.");
      // Optional: Automatisch erneut anfragen:
      // handleLockRequest();
    }

    socket.on("lockConfigSuccess", handleLockSuccess);
    socket.on("lockConfigDenied", handleLockDenied);
    socket.on("lockReleased", handleLockReleased);
    socket.on("lockReleasedDenied", handleLockReleasedDenied);
    socket.on("lockFreed", handleLockFreed);

    return () => {
      socket.off("lockConfigSuccess", handleLockSuccess);
      socket.off("lockConfigDenied", handleLockDenied);
      socket.off("lockReleased", handleLockReleased);
      socket.off("lockReleasedDenied", handleLockReleasedDenied);
      socket.off("lockFreed", handleLockFreed);
    };
  }, [socket]);

  const handleKonfigButton = () => {
    setLoadingKonfig(true);
    setNachricht("Frage Lock für Konfigurationsmodus an...");
    socket.emit("requestConfig");
  };

  useEffect(() => {
    function handleKonfigStatus(status) {
      setLoadingKonfig(false);
      setKonfigModus(status);
      if (status) {
        setNachricht(
          "Konfigurationsmodus ist bereits aktiv. Du kannst ihn jetzt verlassen."
        );
      } else {
        handleKonfigStart();
      }
    }
    socket.on("konfigStatus", handleKonfigStatus);
    return () => socket.off("konfigStatus", handleKonfigStatus);
  }, [socket]);

  useEffect(() => {
    function handleSchwelle(wert) {
      setAktuelleSchwelle(wert);
      setSchwelle(wert);
    }

    function handleError(msg) {
      setNachricht(msg);
    }

    function handleInfo(msg) {
      setNachricht(msg);
      if (msg.includes("System bereit zur Konfiguration")) {
        setKonfigModus(true);
        if (timeoutRef.current) {
          clearTimeout(timeoutRef.current);
          timeoutRef.current = null;
        }
      }
      if (
        msg.includes("Konfiguration NICHT möglich") ||
        msg.includes("Konfigurationsmodus verlassen")
      ) {
        setKonfigModus(false);
        if (timeoutRef.current) {
          clearTimeout(timeoutRef.current);
          timeoutRef.current = null;
        }
      }
    }

    socket.on("schwelle", handleSchwelle);
    socket.on("schwelleError", handleError);
    socket.on("info", handleInfo);

    socket.emit("getSchwelle");

    return () => {
      socket.off("schwelle", handleSchwelle);
      socket.off("schwelleError", handleError);
      socket.off("info", handleInfo);
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
    };
  }, [socket]);

  useEffect(() => {
    function handleBeforeUnload(e) {
      if (konfigModus) {
        if (navigator.sendBeacon) {
          const url = "/konfigEnde";
          const data = JSON.stringify({ reason: "browser-leave" });
          navigator.sendBeacon(url, data);
        } else {
          socket.emit("konfigEnde");
        }
        e.preventDefault();
        e.returnValue =
          "Der Konfigurationsmodus wird beendet. Bitte stelle sicher, dass alle Änderungen gespeichert sind.";
        return e.returnValue;
      }
    }
    window.addEventListener("beforeunload", handleBeforeUnload);
    return () => window.removeEventListener("beforeunload", handleBeforeUnload);
  }, [konfigModus, socket]);

  useEffect(() => {
    function handleDisconnect() {
      if (konfigModus) {
        setKonfigModus(false);
        setNachricht(
          "Verbindung zum Server verloren. Konfigurationsmodus wurde beendet."
        );
      }
    }
    socket.on("disconnect", handleDisconnect);
    return () => socket.off("disconnect", handleDisconnect);
  }, [konfigModus, socket]);

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!socket) return;
    socket.emit("setSchwelle", schwelle);
  };

  const handleKonfigStart = () => {
    socket.emit("konfigStart");
    setNachricht("Warte auf Systemfreigabe...");
    if (timeoutRef.current) clearTimeout(timeoutRef.current);
    timeoutRef.current = setTimeout(() => {
      setNachricht(
        "Timeout: Keine Rückmeldung vom System. Konfiguration NICHT möglich."
      );
      setKonfigModus(false);
      timeoutRef.current = null;
    }, 7000);
  };

  const handleKonfigEnde = () => {
    socket.emit("konfigEnde");
    setNachricht("Konfigurationsmodus verlassen.");
    setKonfigModus(false);
    if (timeoutRef.current) clearTimeout(timeoutRef.current);
    timeoutRef.current = null;
  };

  const handleLogout = () => {
    if (konfigModus) {
      if (navigator.sendBeacon) {
        const url = "/konfigEnde";
        const data = JSON.stringify({ reason: "logout" });
        navigator.sendBeacon(url, data);
      } else {
        socket.emit("konfigEnde");
      }
      alert("Der Konfigurationsmodus wird jetzt beendet.");
      setTimeout(() => {
        onLogout && onLogout();
      }, 200);
      return;
    }
    onLogout && onLogout();
  };

  return (
    <div className="container py-4">
      <User1Features />
      <div className="mt-5">
        <h3 className="mb-4">Temperaturschwelle konfigurieren</h3>
        <div className="d-flex flex-wrap align-items-center gap-3 mb-3">
          {loadingKonfig ? (
            <button className="btn btn-secondary" disabled>
              <span className="spinner-border spinner-border-sm me-2" />
              Prüfe Status ...
            </button>
          ) : konfigModus ? (
            <button className="btn btn-danger" onClick={handleKonfigEnde}>
              Konfigurationsmodus verlassen
            </button>
          ) : (
            <button className="btn btn-primary" onClick={handleKonfigButton}>
              In den Konfigurationsmodus wechseln
            </button>
          )}
        </div>
        <form
          className="d-flex flex-wrap align-items-end gap-2 mb-3"
          onSubmit={handleSubmit}
        >
          <div>
            <label className="form-label mb-0 me-2">Schwelle (°C):</label>
            <input
              type="number"
              value={schwelle}
              onChange={(e) => setSchwelle(e.target.value)}
              step="1"
              min="0"
              max="250"
              className={`form-control d-inline-block ${
                konfigModus ? "border-success shadow" : ""
              }`}
              style={{
                width: 120,
                outline: konfigModus ? "2px solid #198754" : "",
                boxShadow: konfigModus ? "0 0 6px 2px #6f6" : "",
                transition: "box-shadow 0.2s, border 0.2s",
              }}
              disabled={!konfigModus}
            />
          </div>
          <button
            type="submit"
            className="btn btn-success"
            disabled={!konfigModus}
          >
            Schwelle setzen
          </button>
        </form>
        <div className="mb-2">
          <strong>Aktuelle Schwelle (vom Server):</strong>{" "}
          {aktuelleSchwelle !== null ? `${aktuelleSchwelle} °C` : "Lädt..."}
        </div>
        {nachricht && (
          <div className="alert alert-info py-2 mb-0" style={{ color: "blue" }}>
            {nachricht}
          </div>
        )}
      </div>
    </div>
  );
}
