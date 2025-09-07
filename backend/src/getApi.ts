import express, { Express } from 'express';
import http, { Server as HttpServer } from 'http';
import { Server as SocketIOServer } from 'socket.io';
import bodyParser from 'body-parser';
import cors from 'cors';
import helmet from 'helmet';
import rateLimit from 'express-rate-limit';
import { TemperaturMsg } from './routes_frontend/routes/onLiveTemperatur';
import { postLogin } from './routes_express/postLogin';
import { Database } from './database/Sqlite3Database';
import { registerBackendHandlers } from './routes/registerBackendHandlers';
import { registerFrontendHandlers } from './routes/registerFrontendHandlers';

// Typ für die Rückgabe
export interface ApiServers {
  server_frontend: HttpServer;
  server_intern: HttpServer;
}

export interface ConfigLock {
  id: string; // Socket-ID des Halters
  timeout?: NodeJS.Timeout;
  expiresAt?: number; // Zeitstempel, wann das Lock abläuft
 }

// Typisiere die Getter/Setter für den Socket-Handler
interface KonfigState {
  getTemperSchwelle: () => number;
  setTemperSchwelle: (v: number) => void;
  getLetzteLiveTemperatur: () => TemperaturMsg | null;
  setLetzteLiveTemperatur: (v: TemperaturMsg | null) => void;
  getKonfigAckTimeout: () => NodeJS.Timeout | null;
  setKonfigAckTimeout: (v: NodeJS.Timeout | null) => void;
  getKonfigReady: () => boolean;
  setKonfigReady: (v: boolean) => void;
}

/**
 * Initialisiert Express, HTTP-Server und Socket.io,
 * registriert alle HTTP- und Socket-Handler.
 * Gibt die HTTP-Server-Instanzen für den Serverstart zurück.
 */
export function getApi({ database }: { database: Database }): ApiServers {
  // Globale Servervariablen (werden über Getter/Setter an die Handler gegeben)
  let temperSchwelle = 22;
  let letzteLiveTemperatur: TemperaturMsg | null = null;
  let konfigAckTimeout: NodeJS.Timeout | null = null;
  let konfigReady = false;
  let configLock: ConfigLock = {id : ""};
  // === Frontend-Socket (für Browser) ===
  const appFrontend: Express = express();

  // Security-Middleware
  appFrontend.use(helmet());

  // CORS restriktiv konfigurieren!
  appFrontend.use(cors({
    origin: ['http://localhost:3000'], // In Produktion: echte Domain(en) eintragen!
    credentials: true
  }));

  // Body-Parser mit Limit
  appFrontend.use(bodyParser.json({ limit: '1mb' }));

  // Rate-Limiter für Login
  appFrontend.use('/api/login', rateLimit({
    windowMs: 60 * 1000, // 1 Minute
    max: 5, // Maximal 5 Versuche pro Minute
    standardHeaders: true,
    legacyHeaders: false,
    message: 'Zu viele Login-Versuche. Bitte warte kurz.'
  }));

  // HTTP-Server und Socket.io-Server initialisieren
  const server_frontend: HttpServer = http.createServer(appFrontend);

  const ioFrontend: SocketIOServer = new SocketIOServer(server_frontend, {
    cors: {
      origin: "http://localhost:3000",
      credentials: true
    }
  });

  appFrontend.post('/api/login', postLogin({ database }));

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
  const appIntern: Express = express();

  // Security auch für interne API
  appIntern.use(helmet());
  appIntern.use(cors({
    origin: "http://localhost:4001", // In Produktion: echte Domain(en) eintragen!
    credentials: true
  }));
  appIntern.use(bodyParser.json({ limit: '1mb' }));

  const server_intern: HttpServer = http.createServer(appIntern);

  const ioApp: SocketIOServer = new SocketIOServer(server_intern, {
    cors: {
      origin: "http://localhost:4001",
      credentials: true
    }
  });

  ioFrontend.on('connection', registerFrontendHandlers(ioApp));
  ioApp.on('connection', registerBackendHandlers(ioFrontend));


  // Gebe HTTP-Server-Instanzen zurück (wird zum Starten des Servers verwendet)
  return { server_frontend, server_intern };
}
