import { Request, Response, RequestHandler } from 'express';
import { z } from 'zod';
import { Database } from "../database/Sqlite3Database";

// Zod-Schema für Login-Body
const loginSchema = z.object({
  username: z.string().min(1, "Username is required"),
  password: z.string().min(1, "Password is required"),
});

// Factory-Funktion für den Login-POST-Handler
export function postLogin({ database }: { database: Database }): RequestHandler {
  return async (req: Request, res: Response) => {
    console.log("Login");
    try {
      // Input-Validierung mit zod
      const parseResult = loginSchema.safeParse(req.body);
      if (!parseResult.success) {
        // Nur generische Fehlermeldung zurückgeben
        return res.status(400).json({ error: "Invalid input." });
      }
      const { username, password } = parseResult.data;

      // User-Authentifizierung über die Datenbank
      const user = await database.checkPassword(username, password);
      if (!user) {
        return res.status(401).json({ error: "Invalid username or password" });
      }

      // Login erfolgreich: Userdaten (ohne Passwort) zurückgeben
      return res.json({
        name: user.name,
        username: user.username,
        role: user.role,
      });
    } catch (error) {
      // Keine internen Fehler nach außen geben!
      return res.status(500).json({ error: "Internal server error" });
    }
  };
}
