import React from "react";
import AdminFeatures from "./AdminFeatures";

export default function AdminApp({ user }) {
  return (
    <div>
      <h2>Admin-Dashboard</h2>
      <AdminFeatures />
    </div>
  );
}
