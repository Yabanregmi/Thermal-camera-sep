"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.Sqlite3Database = void 0;
const bcryptjs_1 = __importDefault(require("bcryptjs"));
const sqlite3_1 = __importDefault(require("sqlite3"));
const sqlite_1 = require("sqlite");
const promises_1 = __importDefault(require("fs/promises"));
const path_1 = __importDefault(require("path"));
class Sqlite3Database {
    constructor() {
        this.db = null;
    }
    async initialize() {
        this.db = await (0, sqlite_1.open)({
            filename: './users.db',
            driver: sqlite3_1.default.Database
        });
    }
    async createUserTable() {
        if (!this.db)
            throw new Error('Database not initialized');
        await this.db.run(`
      CREATE TABLE IF NOT EXISTS users (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        username TEXT UNIQUE,
        password TEXT,
        name TEXT,
        role TEXT
      )
    `);
    }
    async writeUserTable() {
        if (!this.db)
            throw new Error('Database not initialized');
        try {
            // Userdaten aus externer JSON-Datei laden
            const data = await promises_1.default.readFile(path_1.default.join(__dirname, 'seedUsers.json'), 'utf-8');
            const seedUsers = JSON.parse(data);
            for (const u of seedUsers) {
                try {
                    const row = await this.db.get("SELECT * FROM users WHERE username = ?", [u.username]);
                    if (!row) {
                        const hashedPw = await bcryptjs_1.default.hash(u.password, 10);
                        await this.db.run("INSERT INTO users (username, password, name, role) VALUES (?, ?, ?, ?)", [u.username, hashedPw, u.name, u.role]);
                        console.log(`User ${u.username} angelegt`);
                    }
                }
                catch (userErr) {
                    console.error(`Fehler beim Anlegen von User ${u.username}:`, userErr);
                }
            }
        }
        catch (err) {
            console.error('Fehler beim Laden oder Verarbeiten der seedUsers.json:', err);
        }
    }
    async getUser(username) {
        if (!this.db)
            throw new Error('Database not initialized');
        const user = await this.db.get("SELECT * FROM users WHERE username = ?", [username]);
        return user ?? null;
    }
    async checkPassword(username, password) {
        const user = await this.getUser(username);
        if (!user)
            return false;
        const valid = await bcryptjs_1.default.compare(password, user.password);
        return valid ? user : false;
    }
    async getHistogramm() {
        // Dummy: Gibt ein Array mit 10 Zufallszahlen zurück (z.B. für Tests)
        return [0, 2, 5, 3, 1, 0, 0, 4, 2, 1];
    }
    async createUser(user) {
        if (!this.db)
            throw new Error('Database not initialized');
        // Prüfen, ob User schon existiert
        const exists = await this.getUser(user.username);
        if (exists)
            throw new Error('Username already exists');
        // Passwort hashen
        const hashedPw = await bcryptjs_1.default.hash(user.password, 10);
        const result = await this.db.run("INSERT INTO users (username, password, name, role) VALUES (?, ?, ?, ?)", [user.username, hashedPw, user.name, user.role]);
        // Rückgabe des angelegten Users inkl. id
        return {
            id: result.lastID,
            username: user.username,
            password: hashedPw, // Achtung: Passwort-Hash!
            name: user.name,
            role: user.role,
        };
    }
    async updateUser(username, updates) {
        if (!this.db)
            throw new Error('Database not initialized');
        const user = await this.getUser(username);
        if (!user)
            return null;
        // Updates vorbereiten
        const fields = [];
        const values = [];
        if (updates.name !== undefined) {
            fields.push("name = ?");
            values.push(updates.name);
        }
        if (updates.role !== undefined) {
            fields.push("role = ?");
            values.push(updates.role);
        }
        if (updates.password !== undefined) {
            const hashedPw = await bcryptjs_1.default.hash(updates.password, 10);
            fields.push("password = ?");
            values.push(hashedPw);
        }
        if (fields.length === 0)
            return user; // nichts zu tun
        values.push(username);
        await this.db.run(`UPDATE users SET ${fields.join(", ")} WHERE username = ?`, values);
        // Aktuellen Stand zurückgeben
        return await this.getUser(username);
    }
    async deleteUser(username) {
        if (!this.db)
            throw new Error('Database not initialized');
        const result = await this.db.run("DELETE FROM users WHERE username = ?", [username]);
        if (typeof result.changes === "number") {
            return result.changes > 0;
        }
        return false;
    }
}
exports.Sqlite3Database = Sqlite3Database;
