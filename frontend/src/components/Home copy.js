import React from "react";
import UserApp1 from "./User1App";
import UserApp2 from "./User2App";
import ServiceApp from "./ServiceApp";
import AdminApp from "./AdminApp";
import "./style/Home.css"; // oder "Home.css"

function RoleBasedApp({ user }) {
  if (user.role === "admin") return <AdminApp user={user} />;
  if (user.role === "service") return <ServiceApp user={user} />;
  if (user.role === "user2") return <UserApp2 user={user} />;
  if (user.role === "user1") return <UserApp1 user={user} />;
  return <div>Keine Anwendung für diese Rolle vorhanden.</div>;
}

export default function Home({ user, onLogout }) {
  return (
    <div className="app-wrapper">
      <div className="header-card">
        <h1>Willkommen, {user.name}!</h1>
        <p>
          Deine Rolle:
          <span className="role-label">{user.role}</span>
        </p>
      </div>
      <RoleBasedApp user={user} />
    </div>
  );
}
