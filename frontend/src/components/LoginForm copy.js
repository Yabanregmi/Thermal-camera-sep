import React, { useState, useContext } from "react";
import { SocketContext } from "./SocketContext";

export default function LoginForm({ onLogin }) {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const socket = useContext(SocketContext);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError("");
    try {
      const res = await fetch("http://localhost:4000/api/login", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ username, password }),
        credentials: "omit", // <-- das ist wichtig!
      });
      const data = await res.json();
      if (!res.ok) {
        setError(data.error || "Login fehlgeschlagen");
        return;
      }
      if (socket) {
        socket.emit("auth", {
          username: data.username,
          role: data.role,
          name: data.name,
        });
      }
      onLogin && onLogin(data);
    } catch (err) {
      setError("Serverfehler");
    }
  };

  return (
    <div
      className="d-flex align-items-center justify-content-center min-vh-100"
      style={{
        background: "linear-gradient(120deg, #e0e7ff 0%, #f5f7fa 100%)",
      }}
    >
      <form
        className="bg-white p-4 rounded-4 shadow-sm"
        style={{ minWidth: 320, maxWidth: 400, width: "100%" }}
        onSubmit={handleSubmit}
      >
        <h2
          className="mb-4 text-center fw-semibold"
          style={{ letterSpacing: "0.02em", color: "#3b3b4f" }}
        >
          Login
        </h2>
        <div className="mb-3">
          <label className="form-label fw-medium text-secondary">
            Benutzername
          </label>
          <input
            className="form-control"
            value={username}
            onChange={(e) => setUsername(e.target.value)}
            autoFocus
          />
        </div>
        <div className="mb-3">
          <label className="form-label fw-medium text-secondary">
            Passwort
          </label>
          <input
            type="password"
            className="form-control"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
          />
        </div>
        {error && (
          <div
            className="alert alert-danger py-2 text-center"
            style={{ fontSize: "0.97rem" }}
          >
            {error}
          </div>
        )}
        <button
          className="btn w-100 text-white fw-semibold mt-2"
          style={{
            background: "linear-gradient(90deg, #6366f1 0%, #3b82f6 100%)",
            fontSize: "1.05rem",
            letterSpacing: "0.03em",
            boxShadow: "0 1px 6px 0 rgba(80, 80, 120, 0.09)",
          }}
          type="submit"
        >
          Login
        </button>
      </form>
    </div>
  );
}
