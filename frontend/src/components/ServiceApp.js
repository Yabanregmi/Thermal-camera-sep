import React from "react";
import ServiceFeatures from "./ServiceFeatures";

export default function ServiceApp({ user }) {
  return (
    <div>
      <h2>Service-Bereich</h2>
      <ServiceFeatures />
    </div>
  );
}
