import React, { useState } from "react";
import LoginForm from "./components/LoginForm";
import Home from "./components/Home";
import BottomNavBar from "./components/BottomNavBar";
import { SocketProvider } from "./components/SocketContext";
import "bootstrap/dist/css/bootstrap.min.css";

function App() {
  const [user, setUser] = useState(null);

  const handleLogout = () => setUser(null);

  return (
    <SocketProvider>
      <div
        className="App text-center bg-light"
        style={{ minHeight: "100vh", paddingBottom: "70px" }}
      >
        {!user ? (
          <LoginForm onLogin={setUser} />
        ) : (
          <Home user={user} onLogout={handleLogout} />
        )}
      </div>
      <BottomNavBar user={user} onLogout={handleLogout} />
    </SocketProvider>
  );
}

export default App;
