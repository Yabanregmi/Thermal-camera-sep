  // Prüfsummen-Berechnung für die Datenbank (SHA256)
  function getChecksum(wert, zeit) {
    return crypto.createHash("sha256")
      .update(`${wert}:${zeit}`)
      .digest("hex");
  }

  // Prüfsummen-Berechnung für Live-Daten (CRC32)
  function getCRC(wert, zeit) {
    return crc.crc32(`${wert}:${zeit}`).toString();
  }
