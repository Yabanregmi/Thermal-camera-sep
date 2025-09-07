const { io } = require("socket.io-client");

const socket = io("http://localhost:4000"); // oder dein Port

socket.on("connect", () => {
    console.log("connected", socket.id);
    socket.emit("requestConfig");
});

socket.on("lockConfigSuccess", () => {
    console.log("Lock erfolgreich erhalten!");
    process.exit();
});

socket.on("lockConfigDenied", (data) => {
    console.log("Lock abgelehnt:", data);
    process.exit();
});
