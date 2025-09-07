import bcrypt from 'bcryptjs';
import sqlite3 from 'sqlite3';
import { open, Database as SqliteDatabase } from 'sqlite';
import fs from 'fs/promises';
import path from 'path';

// Typ für einen User
export interface User {
  id?: number;
  username: string;
  password: string;
  name: string;
  role: string;
}

export interface Database {
  initialize(): Promise<void>;
  createUserTable(): Promise<void>;
  writeUserTable(): Promise<void>;
  getUser(username: string): Promise<User | null>;
  checkPassword(username: string, password: string): Promise<User | false>;
  getHistogramm(): Promise<number[]>;
  createUser(user: Omit<User, 'id'>): Promise<User>; // Gibt den angelegten User zurück
  updateUser(username: string, updates: Partial<Omit<User, 'username' | 'id'>>): Promise<User | null>; // Gibt den aktualisierten User oder null zurück
  deleteUser(username: string): Promise<boolean>; // true bei Erfolg, false falls User nicht existiert
}

export class Sqlite3Database implements Database {
  private db: SqliteDatabase | null = null;

  async initialize(): Promise<void> {
    this.db = await open({
      filename: './users.db',
      driver: sqlite3.Database
    });
  }

  async createUserTable(): Promise<void> {
    if (!this.db) throw new Error('Database not initialized');
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

  async writeUserTable(): Promise<void> {
    if (!this.db) throw new Error('Database not initialized');
    try {
      // Userdaten aus externer JSON-Datei laden
      const data = await fs.readFile(path.join(__dirname, 'seedUsers.json'), 'utf-8');
      const seedUsers: User[] = JSON.parse(data);

      for (const u of seedUsers) {
        try {
          const row: User | undefined = await this.db.get<User>("SELECT * FROM users WHERE username = ?", [u.username]);
          if (!row) {
            const hashedPw = await bcrypt.hash(u.password, 10);
            await this.db.run(
              "INSERT INTO users (username, password, name, role) VALUES (?, ?, ?, ?)",
              [u.username, hashedPw, u.name, u.role]
            );
            console.log(`User ${u.username} angelegt`);
          }
        } catch (userErr) {
          console.error(`Fehler beim Anlegen von User ${u.username}:`, userErr);
        }
      }
    } catch (err) {
      console.error('Fehler beim Laden oder Verarbeiten der seedUsers.json:', err);
    }
  }

   async getUser(username: string): Promise<User | null> {
    if (!this.db) throw new Error('Database not initialized');
    const user = await this.db.get<User>("SELECT * FROM users WHERE username = ?", [username]);
    return user ?? null;
  }

  async checkPassword(username: string, password: string): Promise<User | false> {
    const user = await this.getUser(username);
    if (!user) return false;
    const valid = await bcrypt.compare(password, user.password);
    return valid ? user : false;
  }

  async getHistogramm(): Promise<number[]> {
    // Dummy: Gibt ein Array mit 10 Zufallszahlen zurück (z.B. für Tests)
    return [0, 2, 5, 3, 1, 0, 0, 4, 2, 1];
  }

  async createUser(user: Omit<User, 'id'>): Promise<User> {
    if (!this.db) throw new Error('Database not initialized');
    // Prüfen, ob User schon existiert
    const exists = await this.getUser(user.username);
    if (exists) throw new Error('Username already exists');
    // Passwort hashen
    const hashedPw = await bcrypt.hash(user.password, 10);
    const result = await this.db.run(
      "INSERT INTO users (username, password, name, role) VALUES (?, ?, ?, ?)",
      [user.username, hashedPw, user.name, user.role]
    );
    // Rückgabe des angelegten Users inkl. id
    return {
      id: result.lastID,
      username: user.username,
      password: hashedPw, // Achtung: Passwort-Hash!
      name: user.name,
      role: user.role,
    };
  }

  async updateUser(
    username: string,
    updates: Partial<Omit<User, 'username' | 'id'>>
  ): Promise<User | null> {
    if (!this.db) throw new Error('Database not initialized');
    const user = await this.getUser(username);
    if (!user) return null;

    // Updates vorbereiten
    const fields: string[] = [];
    const values: any[] = [];
    if (updates.name !== undefined) {
      fields.push("name = ?");
      values.push(updates.name);
    }
    if (updates.role !== undefined) {
      fields.push("role = ?");
      values.push(updates.role);
    }
    if (updates.password !== undefined) {
      const hashedPw = await bcrypt.hash(updates.password, 10);
      fields.push("password = ?");
      values.push(hashedPw);
    }
    if (fields.length === 0) return user; // nichts zu tun

    values.push(username);

    await this.db.run(
      `UPDATE users SET ${fields.join(", ")} WHERE username = ?`,
      values
    );
    // Aktuellen Stand zurückgeben
    return await this.getUser(username);
  }

  async deleteUser(username: string): Promise<boolean> {
    if (!this.db) throw new Error('Database not initialized');
    const result = await this.db.run(
      "DELETE FROM users WHERE username = ?",
      [username]
    );
      if (typeof result.changes === "number") {
      return result.changes > 0;
    }
    return false;
  }
}