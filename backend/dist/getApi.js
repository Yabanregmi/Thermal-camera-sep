"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.getApi = getApi;
const express_1 = __importDefault(require("express"));
const http_1 = __importDefault(require("http"));
const socket_io_1 = require("socket.io");
const body_parser_1 = __importDefault(require("body-parser"));
const cors_1 = __importDefault(require("cors"));
const helmet_1 = __importDefault(require("helmet"));
const express_rate_limit_1 = __importDefault(require("express-rate-limit"));
const postLogin_1 = require("./routes_express/postLogin");
const registerBackendHandlers_1 = require("./routes/registerBackendHandlers");
const registerFrontendHandlers_1 = require("./routes/registerFrontendHandlers");
/**
 * Initialisiert Express, HTTP-Server und Socket.io,
 * registriert alle HTTP- und Socket-Handler.
 * Gibt die HTTP-Server-Instanzen für den Serverstart zurück.
 */
function getApi({ database }) {
    // Globale Servervariablen (werden über Getter/Setter an die Handler gegeben)
    let temperSchwelle = 22;
    let letzteLiveTemperatur = null;
    let konfigAckTimeout = null;
    let konfigReady = false;
    let configLock = { id: "" };
    // === Frontend-Socket (für Browser) ===
    const appFrontend = (0, express_1.default)();
    // Security-Middleware
    appFrontend.use((0, helmet_1.default)());
    // CORS restriktiv konfigurieren!
    appFrontend.use((0, cors_1.default)({
        origin: ['http://localhost:3000'], // In Produktion: echte Domain(en) eintragen!
        credentials: true
    }));
    // Body-Parser mit Limit
    appFrontend.use(body_parser_1.default.json({ limit: '1mb' }));
    // Rate-Limiter für Login
    appFrontend.use('/api/login', (0, express_rate_limit_1.default)({
        windowMs: 60 * 1000, // 1 Minute
        max: 5, // Maximal 5 Versuche pro Minute
        standardHeaders: true,
        legacyHeaders: false,
        message: 'Zu viele Login-Versuche. Bitte warte kurz.'
    }));
    // HTTP-Server und Socket.io-Server initialisieren
    const server_frontend = http_1.default.createServer(appFrontend);
    const ioFrontend = new socket_io_1.Server(server_frontend, {
        cors: {
            origin: "http://localhost:3000",
            credentials: true
        }
    });
    appFrontend.post('/api/login', (0, postLogin_1.postLogin)({ database }));
    // // Getter/Setter-Objekt für Socket-Handler
    // const konfigState: KonfigState = {
    //   getTemperSchwelle: () => temperSchwelle,
    //   setTemperSchwelle: (v: number) => { temperSchwelle = v; },
    //   getLetzteLiveTemperatur: () => letzteLiveTemperatur,
    //   setLetzteLiveTemperatur: (v: TemperaturMsg | null) => { letzteLiveTemperatur = v; },
    //   getKonfigAckTimeout: () => konfigAckTimeout,
    //   setKonfigAckTimeout: (v: NodeJS.Timeout | null) => { konfigAckTimeout = v; },
    //   getKonfigReady: () => konfigReady,
    //   setKonfigReady: (v: boolean) => { konfigReady = v; }
    // };
    // Socket.io: Registriere alle Event-Handler bei jeder neuen Verbindung
    // === Interner Socket (z.B. für interne Dienste) ===
    const appIntern = (0, express_1.default)();
    // Security auch für interne API
    appIntern.use((0, helmet_1.default)());
    appIntern.use((0, cors_1.default)({
        origin: "http://localhost:4001", // In Produktion: echte Domain(en) eintragen!
        credentials: true
    }));
    appIntern.use(body_parser_1.default.json({ limit: '1mb' }));
    const server_intern = http_1.default.createServer(appIntern);
    const ioApp = new socket_io_1.Server(server_intern, {
        cors: {
            origin: "http://localhost:4001",
            credentials: true
        }
    });
    ioFrontend.on('connection', (0, registerFrontendHandlers_1.registerFrontendHandlers)(ioApp));
    ioApp.on('connection', (0, registerBackendHandlers_1.registerBackendHandlers)(ioFrontend));
    // Gebe HTTP-Server-Instanzen zurück (wird zum Starten des Servers verwendet)
    return { server_frontend, server_intern };
}
