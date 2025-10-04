import serial
import csv
import time

# --- Konfiguration ---
SERIAL_PORT = '/dev/ttyACM0'  # Linux/Mac. Für Windows z.B. 'COM3', 'COM4' etc.
BAUD_RATE = 115200
CSV_FILENAME = 'sensor_data.csv'

print(f"Versuche, mit {SERIAL_PORT} zu verbinden...")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    print("Verbindung hergestellt. Warte auf Pico...")
    time.sleep(2) # Gibt dem Pico Zeit, nach dem Verbindungsaufbau neu zu starten

    print("Sende Start-Signal ('S') an den Pico...")
    ser.write(b'S') # Sendet das 'Start'-Byte

    # CSV-Datei zum Schreiben öffnen
    with open(CSV_FILENAME, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        
        print("Warte auf CSV-Header vom Pico...")
        header_line = ser.readline().decode('utf-8').strip()
        
        if not header_line:
            print("\nFEHLER: Keine Antwort vom Pico erhalten. Läuft das richtige Programm?")
            exit()
        
        if "ERROR" in header_line:
            print(f"\nFEHLER vom Pico empfangen: {header_line}")
            print("Programm wird beendet. Bitte Sensor-Verkabelung prüfen.")
            exit()

        print(f"Header erfolgreich empfangen: {header_line}")
        csv_writer.writerow(header_line.split(',')) # Header in die CSV-Datei schreiben
        
        print(f"Logge Daten in {CSV_FILENAME}. Drücke Strg+C zum Beenden.")
        
        # Datenzeilen kontinuierlich lesen
        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    # Verbesserte Logik: Prüfen, ob die Zeile ein Komma enthält.
                    # Nur dann sind es CSV-Daten.
                    if ',' in line:
                        # Als CSV-Daten in die Datei schreiben
                        csv_writer.writerow(line.split(','))
                    else:
                        # Alles andere ist eine Status- oder Debug-Meldung und wird im Terminal ausgegeben.
                        print(f"STATUS: {line}")

            except KeyboardInterrupt:
                print("\nDaten-Logging wird durch Benutzer gestoppt.")
                break
            except UnicodeDecodeError:
                # Manchmal können beim Start ungültige Zeichen empfangen werden, diese ignorieren wir.
                pass
                
except serial.SerialException as e:
    print(f"\nFEHLER: Konnte den seriellen Port {SERIAL_PORT} nicht öffnen. Details: {e}")

finally:
    if 'ser' in locals() and ser.is_open: # type: ignore
        ser.close() # type: ignore
    print("Serieller Port geschlossen. Programm beendet.")

