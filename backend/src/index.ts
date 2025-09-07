import { getApi, ApiServers } from './getApi';
import { Sqlite3Database, Database } from './database/Sqlite3Database';

(async () => {
  // Datenbankverbindung aufbauen und Tabellen anlegen
  const database = new Sqlite3Database();
  await database.initialize();
  await database.createUserTable();
  await database.writeUserTable();

  // Initialisiere Express, HTTP-Server und Socket.io, übergebe die Datenbank
  // Typisierung: ApiServers
  const { server_frontend, server_intern }: ApiServers = getApi({ database });

  // Starte den HTTP-/Socket.io-Server
  server_frontend.listen(4000, () => {
    console.log(`Frontend-Socket läuft auf Port 4000`);
  });

  server_intern.listen(4001, () => {
    console.log('Interner Socket läuft auf Port 4001');
  });
})();
