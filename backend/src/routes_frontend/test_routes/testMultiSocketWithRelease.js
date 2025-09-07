const { io } = require("socket.io-client");

const NUM_CLIENTS = 3;
const clients = [];
let locksAcquired = 0;
let waitingClients = [];

function tryExit() {
    if (locksAcquired >= NUM_CLIENTS) {
        for (const socket of clients) {
            socket.disconnect();
        }
        process.exit();
    }
}

for (let i = 0; i < NUM_CLIENTS; i++) {
    const socket = io("http://localhost:4000");
    clients.push(socket);

    socket.hasLock = false;

    socket.on("connect", () => {
        console.log(`Client ${i} connected (${socket.id})`);
        socket.emit("requestConfig");
    });

    socket.on("lockConfigSuccess", () => {
        if (!socket.hasLock) {
            socket.hasLock = true;
            locksAcquired++;
            console.log(`Client ${i} hat den Lock bekommen!`);
            // Nach 1 Sekunde Lock explizit freigeben
            setTimeout(() => {
                console.log(`Client ${i} gibt den Lock explizit frei (releaseConfig).`);
                socket.emit("releaseConfig");
            }, 1000);
        }
    });

    socket.on("lockReleased", () => {
        console.log(`Client ${i} hat den Lock erfolgreich freigegeben.`);
        // Nächster wartender Client versucht es erneut
        if (waitingClients.length > 0) {
            const nextIdx = waitingClients.shift();
            if (clients[nextIdx] && !clients[nextIdx].hasLock) {
                console.log(`Client ${nextIdx} versucht erneut, den Lock zu bekommen...`);
                clients[nextIdx].emit("requestConfig");
            }
        }
        tryExit();
    });

    socket.on("lockConfigDenied", (data) => {
        console.log(`Client ${i} abgelehnt:`, data);
        // Nur erneut versuchen, wenn noch nicht Lockbesitzer
        if (!socket.hasLock) {
            waitingClients.push(i);
        }
    });

    socket.on("lockReleasedDenied", (data) => {
        console.log(`Client ${i} konnte den Lock nicht freigeben:`, data);
        tryExit();
    });

    socket.on("connect_error", (err) => {
        console.log(`Client ${i} konnte nicht verbinden:`, err.message);
        tryExit();
    });

    socket.on("lockFreed", () => {
        console.log(`lockFreed empfangen`);
        tryExit();
    });
}
