const { io } = require("socket.io-client");

const NUM_CLIENTS = 3;
const clients = [];
let responsesReceived = 0;

function tryExit() {
    if (responsesReceived >= NUM_CLIENTS) {
        // Alle Sockets sauber schließen
        for (const socket of clients) {
            socket.disconnect();
        }
        // Prozess beenden
        process.exit();
    }
}

for (let i = 0; i < NUM_CLIENTS; i++) {
    const socket = io("http://localhost:4000");
    clients.push(socket);

    socket.on("connect", () => {
        console.log(`Client ${i} connected (${socket.id})`);
        socket.emit("requestConfig");
    });

    socket.on("lockConfigSuccess", () => {
        console.log(`Client ${i} hat den Lock bekommen!`);
        responsesReceived++;
        tryExit();
    });

    socket.on("lockConfigDenied", (data) => {
        console.log(`Client ${i} abgelehnt:`, data);
        responsesReceived++;
        tryExit();
    });

    socket.on("connect_error", (err) => {
        console.log(`Client ${i} konnte nicht verbinden:`, err.message);
        responsesReceived++;
        tryExit();
    });
}
