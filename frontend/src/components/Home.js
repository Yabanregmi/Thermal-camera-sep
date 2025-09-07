import React from "react";
import UserApp1 from "./User1App";
import UserApp2 from "./User2App";
import ServiceApp from "./ServiceApp";
import AdminApp from "./AdminApp";

function RoleBasedApp({ user }) {
  if (user.role === "admin") return <AdminApp user={user} />;
  if (user.role === "service") return <ServiceApp user={user} />;
  if (user.role === "user2") return <UserApp2 user={user} />;
  if (user.role === "user1") return <UserApp1 user={user} />;
  return <div>Keine Anwendung für diese Rolle vorhanden.</div>;
}

export default function Home({ user }) {
  return (
    <div className="container-lg py-4 min-vh-100 bg-light">
      <div className="card shadow-sm rounded-4 text-center mb-4 px-3 pt-4 pb-3">
        <h1
          className="text-primary fw-bold mb-2"
          style={{ letterSpacing: "-1px", fontSize: "2.1em" }}
        >
          Willkommen, {user.name}!
        </h1>
        <p className="text-secondary mb-3" style={{ fontSize: "1.18em" }}>
          Deine Rolle:
          <span
            className="badge ms-2"
            style={{
              background: "linear-gradient(90deg, #3182ce 0%, #2b6cb0 100%)",
              fontSize: "1em",
              fontWeight: 500,
              borderRadius: "6px",
              padding: "0.16em 1em 0.18em 1em",
            }}
          >
            {user.role}
          </span>
        </p>
      </div>
      <RoleBasedApp user={user} />
    </div>
  );
}
