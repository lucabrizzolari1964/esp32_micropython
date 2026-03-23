from machine import I2C, Pin, time_pulse_us
import time
import urandom 
import network
import socket
import os

#server di log
SERVER_IP = "192.168.0.157" 
SERVER_PORT = 5005
indirizzo_ip=""

# --- CONFIGURAZIONE HARDWARE ---
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
addr = 0x20 

# Pin Echo (Tutti su ESP32)
echo_sx, echo_dx = Pin(18, Pin.IN), Pin(19, Pin.IN)
echo_sotto = Pin(5, Pin.IN)
echo_lat_sx, echo_lat_dx = Pin(32, Pin.IN), Pin(33, Pin.IN)

# Pin Trigger Laterali (Diretti su ESP32 per liberare l'expander)
trig_lat_sx = Pin(25, Pin.OUT)
trig_lat_dx = Pin(26, Pin.OUT)

button = Pin(23, Pin.IN, Pin.PULL_UP)
led = Pin(2, Pin.OUT)

# Sequenze Motori (8 fasi - Passo Passo)
seq_a = [0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09]
seq_b = [0x10, 0x30, 0x20, 0x60, 0x40, 0xC0, 0x80, 0x90]

# Configura il Wi-Fi
def connetti_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    if wlan.active():
        wlan.active(False)
        time.sleep(0.5)
    try:
        wlan.active(True) # Riattiva in modo pulito
    except OSError as e:
        print("Errore critico Wi-Fi, riavvio del robot...")
        machine.reset() # Se l'errore persiste, l'unica è il reboot hardware
    
    if not wlan.isconnected():
        print(f'Connessione a {ssid}...')
        wlan.connect(ssid, password)
        
        # Attesa della connessione (timeout di 10 secondi)
        tentativi = 0
        while not wlan.isconnected() and tentativi < 10:
            time.sleep(1)
            tentativi += 1
            print(".", end="")
            
    if wlan.isconnected():
        # Prende i dati della configurazione (IP, Subnet, Gateway, DNS)
        config = wlan.ifconfig()
        print('\n--- Connessione Riuscita! ---')
        print(f'Indirizzo IP Robot: {config[0]}') # Questo è l'IP da usare in VS Code
        print(f'Subnet Mask:      {config[1]}')
        print(f'Gateway:          {config[2]}')
        print('-----------------------------\n')
        print(f'Indirizzo IP Robot uscita : { indirizzo_ip} ')
        return config[0]
    else:
        print('\nErrore: Impossibile connettersi al Wi-Fi.')
        return None

def send_udp(message):
    # 1. Creazione del socket UDP (AF_INET per IPv4, SOCK_DGRAM per UDP)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # 2. Invio del messaggio (i dati devono essere convertiti in bytes)
        print(f"Invio messaggio {message} {SERVER_IP}:{SERVER_PORT}...")
        sock.sendto(message.encode(), (SERVER_IP, SERVER_PORT))
        print("Messaggio inviato correttamente.")
        
    except Exception as e:
        print("Errore durante l'invio:", e)
        
    finally:
        # 3. Chiusura del socket per liberare risorse
        sock.close()

# --- FUNZIONI DI LETTURA E MOVIMENTO ---

def leggi_distanza(sensore):
    """ 0:F_SX, 1:F_DX, 2:SOTTO, 3:L_SX, 4:L_DX """
    if sensore <= 2:
        trig_bit = [0x10, 0x20, 0x40][sensore]
        e_pin = [echo_sx, echo_dx, echo_sotto][sensore]
        i2c.writeto(addr, bytes([trig_bit, 0x00]))
        time.sleep_us(10)
        i2c.writeto(addr, bytes([0x00, 0x00]))
    else:
        t_pin, e_pin = (trig_lat_sx, echo_lat_sx) if sensore == 3 else (trig_lat_dx, echo_lat_dx)
        t_pin.value(1)
        time.sleep_us(10)
        t_pin.value(0)
    
    durata = time_pulse_us(e_pin, 1, 25000)
    return 400 if durata < 0 else (durata / 2) / 29.1

def rileva_terreno():
    """ Analizza la stabilità per distinguere erba/cemento """
    letture = []
    for _ in range(4):
        d = leggi_distanza(2)
        if d < 400: letture.append(d)
        time.sleep_ms(15) 
    if not letture: return "Errore"
    diff = max(letture) - min(letture)
    return "Cemento" if diff < 1 else "Erba"

#def muovi_fisico(passi, dir_a, dir_b, velocita=3):
#    idx_a, idx_b = 0, 0
#    for _ in range(passi):
#        b_low = seq_a[idx_a] if dir_a != 0 else 0
#        b_high = seq_b[idx_b] if dir_b != 0 else 0
#        i2c.writeto(addr, bytes([b_low, b_high]))
#        if dir_a != 0: idx_a = (idx_a + (1 if dir_a > 0 else -1)) % 8
#        if dir_b != 0: idx_b = (idx_b + (1 if dir_b > 0 else -1)) % 8
#        time.sleep_ms(velocita)
#    i2c.writeto(addr, b'\x00\x00')

def muovi_fisico(passi, dir_a, dir_b, velocita=3):
    idx_a, idx_b = 0, 0
    # Usiamo dei contatori decimali per decidere quando muovere il motore
    acc_a, acc_b = 0.0, 0.0
    
    # Portiamo le direzioni a valori assoluti per la logica di salto
    abs_a, abs_b = abs(dir_a), abs(dir_b)
    # Segno della direzione (1 o -1)
    sgn_a = 1 if dir_a > 0 else -1
    sgn_b = 1 if dir_b > 0 else -1

    for _ in range(passi):
        # 1. Accumuliamo la "velocità" richiesta
        acc_a += abs_a
        acc_b += abs_b
        
        # 2. Decidiamo quali bit inviare (default 0 = fermo)
        b_low, b_high = 0, 0
        
        # Se l'accumulatore supera 1, muoviamo il motore A
        if acc_a >= 1.0:
            b_low = seq_a[idx_a]
            idx_a = (idx_a + sgn_a) % 8
            acc_a -= 1.0 # Reset dell'accumulatore
            
        # Se l'accumulatore supera 1, muoviamo il motore B
        if acc_b >= 1.0:
            b_high = seq_b[idx_b]
            idx_b = (idx_b + sgn_b) % 8
            acc_b -= 1.0 # Reset dell'accumulatore

        # 3. Invio il comando combinato
        i2c.writeto(addr, bytes([b_low, b_high]))
        time.sleep_ms(velocita)
        
    # Spegnimento finale per non surriscaldare
    i2c.writeto(addr, b'\x00\x00')



# --- LOGICA AI AVANZATA ---

def manovra_spirale():
    print("[AI] Avvio taglio a SPIRALE...")
    send_udp("[AI] Avvio taglio a SPIRALE...")
    # Crea 5 cerchi concentrici sempre più larghi
    for raggio in range(2, 7):
        # Il motore esterno (SX) gira più del motore interno (DX)
        # Il coefficiente (0.15 * raggio) allarga la curva gradualmente
        muovi_fisico(600 * raggio, 1, 0.15 * raggio, 2)
        # Sicurezza: se vede un ostacolo durante la spirale, interrompe subito
        if leggi_distanza(0) < 20 or leggi_distanza(1) < 20:
            print("[AI] Ostacolo in spirale! Esco.")
            send_udp("[AI] Ostacolo in spirale! Esco.")
            break
    print("Esco dal taglio a spirale.")
    send_udp("Esco dal taglio a spirale.")


def esplora():
    print("\n" + "="*70)
    print("ROBOT TAGLIAERBA LUCA"+ "="*49)
    send_udp("ROBOT TAGLIAERBA LUCA AVVIATO!")
    print("="*70)
    
    urti_vicini = 0
    passi_totali = 0
    bilancio_sterzo = 0  # + = troppe DX, - = troppe SX
    modalita_turbo = False
    soglia_incastro = 4000 

    try:
        while True:
            # 1. GESTIONE TERRENO (Pulsante Pin 23)
            tipo_terreno = "Erba"
            dist_sotto = 0
            if button.value() == 0:
                led.value(1)
                tipo_terreno = rileva_terreno()
                dist_sotto = leggi_distanza(2)
            else:
                led.value(0)

            # 2. LETTURA RADAR COMPLETO
            fsx, fdx = leggi_distanza(0), leggi_distanza(1)
            time.sleep_ms(10)
            lsx, ldx = leggi_distanza(3), leggi_distanza(4)

            # SUPER DEBUG CON BUSSOLA E TURBO
            status = "TURBO" if modalita_turbo else "NORMAL"
            print(f"[{status}] LS:{lsx:2.0f}|FS:{fsx:2.0f}|FD{fdx:2.0f}|LD:{ldx:2.0f} | Bilancio sterzo:{bilancio_sterzo} Passi totali:{passi_totali} Urti Vicini:{urti_vicini}")
            send_udp(f"[{status}] LS:{lsx:2.0f}|FS:{fsx:2.0f}|FD{fdx:2.0f}|LD:{ldx:2.0f} | Bilancio sterzo:{bilancio_sterzo} Passi totali:{passi_totali} Urti Vicini:{urti_vicini}")
            # 3. PRIORITÀ: CEMENTO O VUOTO
            if tipo_terreno == "Cemento" or dist_sotto > 18:
                print(f"[AI] ALLERTA: {tipo_terreno.upper()}! Scappo...")
                send_udp(f"[AI] ALLERTA: {tipo_terreno.upper()}! Scappo...")
                modalita_turbo = False
                muovi_fisico(1200, -1, -1)
                muovi_fisico(10000, 1, -1)
                passi_totali = 0
                continue

            if passi_totali > 8000:
                manovra_spirale()
                passi_totali = 0
                continue

            # 4. LOGICA OSTACOLO / ANGOLO
            if fsx < 15 or fdx < 15 or (lsx < 15 and ldx < 15):
                modalita_turbo = False # Ferma il turbo se sbatte
                print(f"OSTACOLO! ... Vado più piano tolgo il turbo")
                send_udp("OSTACOLO! ... Vado più piano tolgo il turbo")
                # Bussola Virtuale: evita di girare sempre nello stesso verso
                if bilancio_sterzo > 3:
                    dir_fuga = -1 # Forza SX
                    print("[AI] BUSSOLA: Troppe DX, forzo rotazione a SX")
                    send_udp("[AI] BUSSOLA: Troppe DX, forzo rotazione a SX")
                elif bilancio_sterzo < -3:
                    dir_fuga = 1 # Forza DX
                    print("[AI] BUSSOLA: Troppe SX, forzo rotazione a DX")
                    send_udp("[AI] BUSSOLA: Troppe SX, forzo rotazione a DX")
                else:
                    dir_fuga = 1 if ldx > lsx else -1 # Scegli lato libero
                    print(f"[AI] BUSSOLA: dir_fuga :{dir_fuga} scelgo lato con più spazio")
                    send_udp
                # Gestione incastro (Angolo cieco)
                if passi_totali < soglia_incastro:
                    urti_vicini += 1
                    dir_fuga *= -1 # Se sbatte subito, cambia idea
                    print(f"[AI] Angolo rilevato (Urt:{urti_vicini}). Cambio rotazione.")
                    send_udp(f"[AI] Angolo rilevato (Urt:{urti_vicini}). Cambio rotazione.")
                else:
                    urti_vicini = 1

                bilancio_sterzo += dir_fuga
                passi_totali = 0

                # AZIONE DI FUGA
                if urti_vicini >= 3:
                    print("[AI] MODALITA DISPERAZIONE: Retromarcia curva + Turbo")
                    send_udp("[AI] MODALITA DISPERAZIONE: Retromarcia curva + Turbo")
                    muovi_fisico(1800, -1, -0.4) 
                    muovi_fisico(10000, 1, -1)
                    modalita_turbo = True # Prossima marcia sarà Turbo
                    urti_vicini = 0
                else:
                    muovi_fisico(700, -1, -1)
                    print(f"dir_fuga : {dir_fuga} mi muovo di 5000 con {dir_fuga} e -{dir_fuga}  ")
                    send_udp(f"dir_fuga : {dir_fuga} mi muovo di 5000 con {dir_fuga} e -{dir_fuga}  ")
                    muovi_fisico(5000, dir_fuga, -dir_fuga)
                continue

            # 5. CORREZIONI LATERALI (Wall Following leggero)
            elif lsx < 10:
                print("Ostacolo laterale SX")
                send_udp("Ostacolo laterale SX")
                muovi_fisico(250, 1, -1, 1)
                passi_totali += 250
            elif ldx < 10:
                print("Ostacolo laterale DS")
                send_udp("Ostacolo laterale DS")
                muovi_fisico(250, -1, 1, 1)
                passi_totali += 250

            # 6. MARCIA NORMALE O TURBO
            else:
                p_step = 2000 if modalita_turbo else 1000
                vel = 1 if modalita_turbo else 2 # vel 1 è più veloce per i motori stepper
                muovi_fisico(p_step, 1, 1, vel)
                passi_totali += p_step
                
                # Se viaggia libero, resetta i problemi
                if passi_totali > 20000:
                    urti_vicini = 0
                    bilancio_sterzo = 0
                    if modalita_turbo:
                        modalita_turbo = False
                        print("[AI] Turbo OFF: Zona libera raggiunta.")
                        send_udp("[AI] Turbo OFF: Zona libera raggiunta.")

    except KeyboardInterrupt:
        i2c.writeto(addr, b'\x00\x00')
        print("\n[STOP] Robot spento.")
        send_udp("[STOP] Robot spento.")
indirizzo_ip = connetti_wifi("briz", "Luca0001")
print(f'Indirizzo IP Robot: { indirizzo_ip }')
send_udp(f'Indirizzo ip del robot : {indirizzo_ip} ')
send_udp("Robot tagliaerba avviato e connesso al Wi-Fi!")
esplora()
