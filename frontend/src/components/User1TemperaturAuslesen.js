import React, { useContext, useEffect, useState } from "react";
import { SocketContext } from "./SocketContext";
import { Line, Bar } from "react-chartjs-2";
import "chart.js/auto";

export default function User1Features() {
  const socket = useContext(SocketContext); // Holt sich den Socket aus dem Context.
  const intervalRef = useRef(null); // Speichert die ID des laufenden Intervalls (damit es später gestoppt werden kann)

  const [temperaturen, setTemperaturen] = useState([]);
  const [histogrammWerte, setHistogrammWerte] = useState([]);
  const [histogrammError, setHistogrammError] = useState("");
  const [schwelle, setSchwelle] = useState(null);

  // Wird einmal beim Mounten der Komponente ausgeführt (und wenn sich socket ändert).
  useEffect(() => {
    let ackReceived = false; // Lokale Variable, merkt sich, ob das ACK schon empfangen wurde.

    // Funktion, die das Event sendet – aber nur solange kein ACK empfangen wurde.
    function sendReq() {
      if (!ackReceived) {
        socket.emit("REQ_CALL_LIVE_TEMPRETURE");
      }
    }

    // Ruft sendReq jede Sekunde auf und speichert die Intervall-ID in intervalRef.current.
    intervalRef.current = setInterval(sendReq, 1000); // alle 1 Sekunde

    // Wird aufgerufen, wenn das ACK vom Backend kommt
    function handleAck(msg) {
      ackReceived = true; // Setzt ackReceived auf true, damit keine weiteren Requests mehr gesendet werde
      clearInterval(intervalRef.current); // Beendet das Intervall mit clearInterval().
      intervalRef.current = null; // Setzt die Referenz zurück.
      // Optional: Hier kannst du eine State-Variable setzen, um anzuzeigen, dass Live-Daten empfangen werden
      // z.B. setLiveMode(true);
    }

    socket.on("ACK_CALL_LIVE_TEMPRETURE", handleAck);

    // Stoppt das Intervall, falls es noch läuft, wenn die Komponente aus dem DOM entfernt wird.
    // Entfernt den Event-Listener für das ACK, damit es keine Speicherlecks gibt.
    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current);
      socket.off("ACK_CALL_LIVE_TEMPRETURE", handleAck);
    };
  }, [socket]);

  useEffect(() => {
    function handleTemp(msg) {
      setTemperaturen((alte) => [msg, ...alte].slice(0, 1000));
    }
    socket.on("temperatur", handleTemp);
    return () => {
      socket.off("temperatur", handleTemp);
    };
  }, [socket]);

  useEffect(() => {
    function handleSchwelle(wert) {
      setSchwelle(Number(wert));
    }
    socket.on("schwelle", handleSchwelle);
    socket.emit("getSchwelle");
    return () => {
      socket.off("schwelle", handleSchwelle);
    };
  }, [socket]);

  function ladeHistogramm() {
    setHistogrammError("");
    setHistogrammWerte([]);
    socket.emit("getTemperaturHistogramm");
  }

  useEffect(() => {
    const histHandler = (werte) => {
      setHistogrammWerte(werte);
    };
    const errorHandler = (msg) => {
      setHistogrammError(msg);
    };
    socket.on("temperaturHistogramm", histHandler);
    socket.on("temperaturHistogrammError", errorHandler);
    return () => {
      socket.off("temperaturHistogramm", histHandler);
      socket.off("temperaturHistogrammError", errorHandler);
    };
  }, [socket]);

  const temperaturenReversed = temperaturen.slice().reverse();
  const werte = temperaturenReversed.map((msg) => Number(msg.wert));
  const labels = temperaturenReversed.map((msg) =>
    new Date(msg.zeit).toLocaleTimeString()
  );
  const maxWert = werte.length > 0 ? Math.max(...werte) : null;

  const lineData = {
    labels,
    datasets: [
      {
        label: "Temperatur (°C)",
        data: werte,
        fill: false,
        borderColor: "rgb(75,192,192)",
        tension: 0.1,
        pointRadius: 0,
      },
      maxWert !== null && {
        label: `Maximalwert (${maxWert.toFixed(2)} °C)`,
        data: Array(werte.length).fill(maxWert),
        fill: false,
        borderColor: "red",
        borderWidth: 2,
        pointRadius: 0,
        pointHitRadius: 0,
        tension: 0,
      },
      schwelle !== null && {
        label: `Schwelle (${schwelle.toFixed(2)} °C)`,
        data: Array(werte.length).fill(schwelle),
        fill: false,
        borderColor: "orange",
        borderWidth: 2,
        borderDash: [6, 6],
        pointRadius: 0,
        pointHitRadius: 0,
        tension: 0,
      },
    ].filter(Boolean),
  };

  const lineOptions = {
    maintainAspectRatio: false,
    scales: {
      x: {
        ticks: {
          callback: function (value, index, ticks) {
            if (index === 0) return this.getLabelForValue(value);
            if (index === ticks.length - 1) return this.getLabelForValue(value);
            return "";
          },
          maxRotation: 0,
          autoSkip: false,
        },
      },
      y: {
        min: 0,
        max: 300,
      },
    },
  };

  const binStart = 0;
  const binStep = 5;
  let barData = null;
  if (histogrammWerte.length > 0) {
    const firstNonZero = histogrammWerte.findIndex((v) => v !== 0);
    const lastNonZero =
      histogrammWerte.length -
      1 -
      [...histogrammWerte].reverse().findIndex((v) => v !== 0);
    const relevantBins = histogrammWerte.slice(firstNonZero, lastNonZero + 1);
    const binLabels = Array.from(
      { length: relevantBins.length },
      (_, i) =>
        `${binStart + (firstNonZero + i) * binStep}–${
          binStart + (firstNonZero + i + 1) * binStep
        }°C`
    );
    barData = {
      labels: binLabels,
      datasets: [
        {
          label: "Häufigkeit",
          data: relevantBins,
          backgroundColor: "rgba(255,99,132,0.5)",
        },
      ],
    };
  }

  return (
    <div className="container py-4">
      <h3 className="mb-3">User1-Funktionen</h3>
      <ul className="mb-4">
        <li>Dashboard</li>
      </ul>
      <hr />

      <div className="row g-4">
        {/* Linien-Diagramm */}
        <div className="col-12 col-lg-6">
          <div className="card shadow-sm h-100">
            <div className="card-body">
              <h4 className="card-title mb-3">Linien-Chart</h4>
              <div style={{ height: 320 }}>
                <Line data={lineData} options={lineOptions} />
              </div>
            </div>
          </div>
        </div>
        {/* Histogramm */}
        <div className="col-12 col-lg-6">
          <div className="card shadow-sm h-100">
            <div className="card-body">
              <h4 className="card-title mb-3">Häufigkeits-Chart</h4>
              <button
                className="btn btn-outline-primary mb-3"
                onClick={ladeHistogramm}
              >
                Chart laden
              </button>
              {histogrammError && (
                <div className="alert alert-danger py-2 text-center">
                  {histogrammError}
                </div>
              )}
              {barData && (
                <div style={{ height: 320 }}>
                  <Bar
                    data={barData}
                    options={{
                      maintainAspectRatio: false,
                      plugins: { legend: { display: false } },
                    }}
                  />
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
