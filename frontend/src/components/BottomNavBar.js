// components/BottomNavBar.jsx
import React from "react";

export default function BottomNavBar({ user, onLogout }) {
  return (
    <nav className="navbar navbar-expand navbar-light bg-light fixed-bottom border-top">
      <div className="container justify-content-around">
        {user && (
          <button className="btn btn-outline-danger" onClick={onLogout}>
            Logout
          </button>
        )}
      </div>
    </nav>
  );
}
