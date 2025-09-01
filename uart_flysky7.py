import serial
import time

# Instellen van de seriële poort (pas aan indien nodig)
try:
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    print("[INFO] Seriële poort geopend: /dev/ttyAMA0 op 115200 baud.")
except serial.SerialException as e:
    print(f"[ERROR] Kan seriële poort niet openen: {e}")
    ser = None

# Parameters
HEARTBEAT_INTERVAL = 0.1  # 200 ms
COMMUNICATION_TIMEOUT = 1  # 500 ms

last_heartbeat_time = time.time()
last_communication_time = time.time()

# Flags voor eenmalige meldingen
heartbeat_message_displayed = False
timeout_message_displayed = False

# Functie om gegevens van de Arduino te lezen
def read_from_arduino():
    """Lees gegevens van de Arduino via de seriële poort."""
    global last_communication_time
    if ser.in_waiting > 0:
        try:
            data = ser.readline().decode('utf-8').strip()
            last_communication_time = time.time()  # Reset de communicatie-timer
            return data
        except UnicodeDecodeError as e:
            print(f"[ERROR] Decodeerfout: {e}")
        except Exception as e:
            print(f"[ERROR] Onverwachte fout bij het lezen van data: {e}")
    return None

# Functie om gegevens naar de Arduino te sturen
def send_to_arduino(data):
    """Stuur gegevens naar de Arduino via de seriële poort."""
    try:
        ser.write(data.encode())
    except Exception as e:
        print(f"[ERROR] Fout bij het verzenden van data: {e}")

# Functie om joystickwaarden om te zetten naar Twist
def calculate_twist(joystick_values):
    """
    Bereken Twist-waarden op basis van joystickwaarden.
    joystick_values: [ch1, ch2, ch3] (x, y, rotatie)
    """
    try:
        linear_x = joystick_values[1] / 500.0  # Normaliseer tussen -1 en 1
        linear_y = joystick_values[0] / 500.0
        angular_z = joystick_values[2] / 500.0

        # Debug: Toon berekende Twist
        print(f"[DATA] Twist berekend: linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular_z={angular_z:.2f}")

        return linear_x, linear_y, angular_z
    except Exception as e:
        print(f"[ERROR] Fout bij het berekenen van Twist: {e}")
        return 0.0, 0.0, 0.0

# Hoofdprogramma
def main():
    global last_heartbeat_time, last_communication_time
    global heartbeat_message_displayed, timeout_message_displayed

    if not ser:
        print("[ERROR] Seriële communicatie is niet beschikbaar. Programma wordt beëindigd.")
        return

    print("=== Raspberry Pi UART Communicatie met Arduino ===")
    time.sleep(2)  # Wacht even zodat de seriële poort stabiel is

    while True:
        try:
            # Lees gegevens van de Arduino
            arduino_data = read_from_arduino()

            if arduino_data:
                # Controleer of het een heartbeat ("OK") is
                if arduino_data == "OK":
                    if not heartbeat_message_displayed:
                        print("[HEARTBEAT] OK ontvangen van Arduino.")
                        heartbeat_message_displayed = True
                    timeout_message_displayed = False
                else:
                    # Verwerk joystickgegevens
                    try:
                        joystick_values = list(map(int, arduino_data.split(',')))

                        # Controleer of de waarden binnen het bereik [-500, 500] liggen
                        if all(-500 <= value <= 500 for value in joystick_values):
                            linear_x, linear_y, angular_z = calculate_twist(joystick_values)

                            # Stuur Twist-waarden terug naar de Arduino
                            twist_data = f"{linear_x:.2f},{linear_y:.2f},{angular_z:.2f}\n"
                            send_to_arduino(twist_data)
                            print(f"[PI -> ARDUINO] Verzonden Twist: {twist_data.strip()}")
                        else:
                            print(f"[ERROR] Ongeldige joystickwaarden ontvangen: {joystick_values}")
                    except ValueError as e:
                        print(f"[ERROR] Ongeldige data ontvangen: {arduino_data}. Fout: {e}")

            # Verstuur heartbeat naar de Arduino
            if time.time() - last_heartbeat_time > HEARTBEAT_INTERVAL:
                send_to_arduino("OK\n")
                last_heartbeat_time = time.time()
                #print("[DEBUG] Heartbeat verzonden naar Arduino.")

            # Controleer communicatie-timeout
            if time.time() - last_communication_time > COMMUNICATION_TIMEOUT:
                if not timeout_message_displayed:
                    print("[SAFETY] Communicatie-timeout! Controleer verbinding met Arduino.")
                    timeout_message_displayed = True
                heartbeat_message_displayed = False  # Reset heartbeat-status

            time.sleep(0.01)  # Kleine pauze voor stabiliteit

        except KeyboardInterrupt:
            print("\n[INFO] Programma beëindigd door gebruiker.")
            break
        except Exception as e:
            print(f"[ERROR] Onverwachte fout: {e}")
            break

    if ser:
        ser.close()
        print("[INFO] Seriële poort gesloten.")

if __name__ == "__main__":
    main()
