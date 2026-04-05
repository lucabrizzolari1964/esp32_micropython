from machine import I2C, Pin, time_pulse_us
import time
import urandom 
import network
import socket
import os
import gc
import math


#0: Avanti (Taglio)
#1: Retromarcia e Destra (Manovra)
#2: Destra (Rotazione)
#3: Retromarcia e Sinistra (Manovra)
#4: Sinistra (Rotazione)

# --- CONFIGURAZIONE RETE ---
SSID = "briz"
PASSWORD = "Luca0001"
SERVER_IP = "192.168.0.157" 
SERVER_PORT = 5005

# --- CONFIGURAZIONE Q-LEARNING ---
Q_TABLE_FILE = "qtable_mower.txt"
AZIONI = 5  
STATI = 64 
LR = 0.2    
GAMMA = 0.9 
EPSILON_BASE = 0.15 

# --- CONFIGURAZIONE HARDWARE ---
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
addr = 0x20 

echo_sx, echo_dx = Pin(18, Pin.IN), Pin(19, Pin.IN)
echo_sotto = Pin(5, Pin.IN)
echo_lat_sx, echo_lat_dx = Pin(32, Pin.IN), Pin(33, Pin.IN)
echo_dietro = Pin(35, Pin.IN) 
trig_lat_sx, trig_lat_dx = Pin(25, Pin.OUT), Pin(26, Pin.OUT) 
button, led = Pin(23, Pin.IN, Pin.PULL_UP), Pin(2, Pin.OUT)

seq_a = [0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09]
seq_b = [0x10, 0x30, 0x20, 0x60, 0x40, 0xC0, 0x80, 0x90]

lsx, ldx = 0, 0
fsx, fdx = 0, 0


# --- TUA FUNZIONE CONNETTI_WIFI ORIGINALE ---
def connetti_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    if wlan.active():
        wlan.active(False)
        time.sleep(0.5)
    try:
        wlan.active(True) # Riattiva in modo pulito
    except OSError as e:
        print("Errore critico Wi-Fi, riavvio del robot...")
        import machine
        machine.reset() 
    
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
        config = wlan.ifconfig()
        print('\n--- Connessione Riuscita! ---')
        print(f'Indirizzo IP Robot: {config[0]}') 
        print(f'Subnet Mask:      {config[1]}')
        print(f'Gateway:          {config[2]}')
        print('-----------------------------\n')
        return config[0]
    else:
        print('\nErrore: Impossibile connettersi al Wi-Fi.')
        return None

# --- FUNZIONI DI SUPPORTO ---
def send_debug(sock, tag, message):
    msg = f"[{time.ticks_ms()}] {tag}: {message}"
    print(msg)
    try: sock.sendto(msg.encode(), (SERVER_IP, SERVER_PORT))
    except: pass

def carica_q_table():
    if Q_TABLE_FILE in os.listdir():
        try:
            with open(Q_TABLE_FILE, "r") as f:
                return [[float(x) for x in line.strip().split(",")] for line in f]
        except: pass
    return [[0.0 for _ in range(AZIONI)] for _ in range(STATI)]

def salva_q_table(table):
    with open(Q_TABLE_FILE, "w") as f:
        for row in table:
            f.write(",".join([str(x) for x in row]) + "\n")


#--- SENSORI E MOVIMENTO ---
def leggi_distanza(sensore, campioni=3):
    letture = []
    
    for _ in range(campioni):
        # Logica originale per attivare il trigger
        if sensore <= 2 or sensore == 5:
            trig_bit = 0x80 if sensore == 5 else [0x10, 0x20, 0x40][sensore]
            e_pin = echo_dietro if sensore == 5 else [echo_sx, echo_dx, echo_sotto][sensore]
            i2c.writeto(addr, bytes([trig_bit, 0x00]))
            time.sleep_us(10)
            i2c.writeto(addr, bytes([0x00, 0x00]))
        else:
            t_pin, e_pin = (trig_lat_sx, echo_lat_sx) if sensore == 3 else (trig_lat_dx, echo_lat_dx)
            t_pin.value(1); time.sleep_us(10); t_pin.value(0)
        
        durata = time_pulse_us(e_pin, 1, 20000)
        
        # Invece di 400 subito, salviamo solo letture valide
        if durata > 0:
            dist = (durata / 2) / 29.1
            letture.append(dist)
        
        time.sleep_ms(5) # Breve pausa per evitare interferenze tra i segnali

    # Logica di filtraggio
    if not letture:
        return 400 # Se falliscono tutte, allora è fuori portata
    
    # Ritorna la media delle letture valide (più stabile)
    return sum(letture) / len(letture)


def muovi_sicuro(passi, dir_a, dir_b, sensore_stop=None, soglia=15, sock=None, freqSensori=40):
    idx_a, idx_b = 0, 0
    acc_a, acc_b = 0.0, 0.0
    for i in range(passi):
        if i % freqSensori == 0 and sensore_stop is not None:
            if leggi_distanza(sensore_stop) < soglia:
                i2c.writeto(addr, b'\x00\x00')
                if sock: send_debug(sock, "HALT", f"Stop d'emergenza sensore {sensore_stop}!")
                return False 
        acc_a += abs(dir_a); acc_b += abs(dir_b)
        bl, bh = 0, 0
        if acc_a >= 1.0: bl = seq_a[idx_a]; idx_a = (idx_a + (1 if dir_a>0 else -1)) % 8; acc_a -= 1.0
        if acc_b >= 1.0: bh = seq_b[idx_b]; idx_b = (idx_b + (1 if dir_b>0 else -1)) % 8; acc_b -= 1.0
        i2c.writeto(addr, bytes([bl, bh]))
        time.sleep_ms(2)
    return True


def analizza_terreno(sock,sensore_sotto, campioni=12):
    letture = []
    fuori_portata = 0
    send_debug(sock, "DEBUG", f"Analisi terreno: raccolgo {campioni} campioni dal sensore sotto...")
    for _ in range(campioni):
        d = leggi_distanza(sensore_sotto)
        if d < 400:
            letture.append(d)
        else:
            fuori_portata += 1
        time.sleep_ms(8) # Leggermente più veloce
    
    # Caso A: Troppi fuori portata (il robot è sollevato o su un buco)
    if fuori_portata > (campioni / 2):
        return "PERICOLO_VUOTO", 99.0

    # Caso B: Dati insufficienti
    if len(letture) < 3: 
        return "Ignoto", 0
    
    media = sum(letture) / len(letture)
    varianza = sum((x - media) ** 2 for x in letture) / len(letture)
    deviazione = math.sqrt(varianza)
    
    # LOGICA DI SOGLIA (Sperimentale)
    # Se la media è molto alta (> 15cm), forse non è erba ma il robot si è alzato
    if media > 15:
        tipo = "Sollevato"
    else:
        # L'erba assorbe/riflette male l'ultrasuono -> alta deviazione
        tipo = "Erba" if deviazione > 1.8 else "Cemento"
        
    return tipo, deviazione



def get_stato_completo(sock):
    global lsx, ldx, fsx, fdx
    lsx, ldx = leggi_distanza(3), leggi_distanza(4)
    fsx, fdx = leggi_distanza(0), leggi_distanza(1)
    dietro = leggi_distanza(5)
    
    
    #l_t = [leggi_distanza(2) for _ in range(3)]
    #leggo 3 volte il sensore sotto per avere una stima più stabile del tipo di terreno (Cemento vs Erba)
    #diff = max(l_t) - min(l_t)

    if button.value() == 0:
        led.value(1)
        tipo_t = "Erba"
        dev = 0.0
        send_debug(sock, "MODE", "Modalità Manuale: Forza ERBA (Pulsante attivo)")
    else:
        led.value(0)
        # Se il pulsante non è premuto, analizza il terreno normalmente
        tipo_t, dev = analizza_terreno(sock,2, campioni=10)

    #tipo_t = "Cemento" if (button.value()==0 and diff < 1.2) else "Erba"
   
    
    # Mappatura a 6 bit (0-63) - STATI deve essere 64
    idx = ((1 if tipo_t == "Cemento" else 0) << 5) | \
      ((1 if dietro < 25 else 0)       << 4) | \
      ((1 if lsx < 25 else 0)          << 3) | \
      ((1 if fsx < 25 else 0)          << 2) | \
      ((1 if fdx < 25 else 0)          << 1) | \
      ((1 if ldx < 25 else 0)          << 0)
    send_debug(sock, "DEBUG", f"letture : Tipo Terreno: {tipo_t} Deviazione: {dev:.2f}")
    send_debug(sock, "SCAN", f"B:{dietro:2.0f} T:{tipo_t} L:{lsx:2.0f} FS:{fsx:2.0f} FD:{fdx:2.0f} R:{ldx:2.0f} ")
    send_debug(sock, "DEBUG", f"idx: {idx} (bin: {bin(idx)})")

    return idx, tipo_t, dietro

# ... (Parti iniziali, import e setup hardware rimangono uguali) ...

def main():
    connetti_wifi(SSID, PASSWORD)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    q_table = carica_q_table()
    passi_in_stallo = 0
    ultimo_stato = -1
    passi_totali = 0
    
    send_debug(sock, "SYSTEM", f"--- ROBOT ONLINE: PASSI LUNGHI + CLIPPING --- Cerca Cemento {button.value()} ---")

    while True:
        send_debug(sock, "DEBUG", "-------------------- Nuovo Ciclo --------------------------")    
        s_att, terr_att, d_dietro_att = get_stato_completo(sock)
        send_debug(sock, "SCAN", f"SENSORI L:{lsx:2.0f} FS:{fsx:2.0f} FD:{fdx:2.0f} R:{ldx:2.0f} ")
        if s_att == ultimo_stato and s_att != 0:
            passi_in_stallo += 1
            send_debug(sock, "WARNING", f"Stallo! Contatore: {passi_in_stallo}")
        else:
            passi_in_stallo = 0
        ultimo_stato = s_att
        send_debug(sock, "DEBUG", f"q_table : {q_table}")
        send_debug(sock, "DEBUG", f"q_table[{s_att}] : {q_table[s_att]}")
        eps = 0.5 if passi_in_stallo > 3 else EPSILON_BASE
        #if urandom.random() < eps:
        #    azione = urandom.randint(0, 4)
        #    send_debug(sock, "BRAIN", f"Esploro (Eps:{eps:.2f})")
        #else:
        #    azione = q_table[s_att].index(max(q_table[s_att]))
        #    send_debug(sock, "BRAIN", f"Q-Best (Val:{max(q_table[s_att]):.2f}) Azione: {azione}")
        
        if fsx > 40 and fdx > 40 and lsx > 40 and ldx > 40 and d_dietro_att > 30:
            azione = 0  # Supponendo 0 = Avanti Veloce
            print("STRADA LIBERA: FS:{} FD:{} L:{} R:{} D:{} -> Vado dritto!".format(fsx, fdx, lsx, ldx, d_dietro_att))
            send_debug(sock, "DEBUG", "STRADA LIBERA: FS:{} FD:{} L:{} R:{} D:{} -> Vado dritto!".format(fsx, fdx, lsx, ldx, d_dietro_att))

        # 3. SE C'È UN OSTACOLO (< 40cm), SCEGLI COSA FARE
        else:
             if urandom.random() < eps: 
                 azione = urandom.randint(0, 4)
                 print("OSTACOLO! Provo azione casuale:", azione)
             else:
                 # Altrimenti usa la Q-Table (Memoria)
                 azione = q_table[s_att].index(max(q_table[s_att]))
                 send_debug(sock, "BRAIN", f"Q-Best (Val:{max(q_table[s_att]):.2f}) Azione: {azione}")

        if (s_att & 0b011110): 
            if urandom.random() < 0.3: # 30% di probabilità
                azione = urandom.choice([2, 4]) # Forza azione 2 (DX) o 4 (SX)
                send_debug(sock, "BRAIN", "FORZA ROTAZIONE ANTI-ANGOLO")

        # --- MODIFICA PASSI AZIONE 0 (AVANTI) ---
        if azione == 0: 
            # Aumentato a 1000 passi per tagliare molto più prato!
            # Controlla il sensore FS (0) ogni 40 passi.
            successo = muovi_sicuro(1000, 1, 1, 0, 18, sock,300) 
        elif azione == 1: 
            passi_random = urandom.randint(800, 1600)
            successo = muovi_sicuro(passi_random, -1, -1, 5, 15, sock) 
            passi_random = urandom.randint(800, 1600)
            if successo: muovi_sicuro(passi_random, 1, -1)
        elif azione == 2: 
            passi_random = urandom.randint(800, 1600)
            send_debug(sock, "ACT", f"Rotazione DX Random: {passi_random}")
            successo = muovi_sicuro(passi_random, 1, -1)
        elif azione == 3: 
            passi_random = urandom.randint(800, 1600)
            successo = muovi_sicuro(passi_random, -1, -1, 5, 15, sock)
            passi_random = urandom.randint(800, 1600)
            if successo: muovi_sicuro(passi_random, -1, 1)
        elif azione == 4: 
            passi_random = urandom.randint(800, 1600)
            send_debug(sock, "ACT", f"Rotazione SX Random: {passi_random}")
            successo = muovi_sicuro(passi_random, -1, 1)
        reward = 0
        s_nuovo, terr_nuovo, d_dietro_nuovo = get_stato_completo(sock)
        if s_att == s_nuovo and s_att != 0:
           reward = -50 # Punizione perché sei rimasto incastrato!
           send_debug(sock, "REWARD", "Punizione: Sei ancora incastrato!")
           
        # 3. Premio per il movimento (Incentivo)
        if successo:
          if azione == 0:
             reward += 20  # Alziamo a 20 per dare ancora più priorità all'avanti
          else:
             reward += 2   # Piccolo premio se si è mosso ma non dritto
        else:
          reward -= 100     # Punizione grave se ha sbattuto (successo = False)
        send_debug(sock, "DEBUG", f"Premio per il movimento:{reward} (Successo: {successo})")
        # 4. Aggiungo un piccolo bonus se la strada davanti è libera
        if fsx > 50 and fdx > 50 and azione == 0:
            reward += 10 
        
        if (s_att & 0b011111) and (s_nuovo & 0b011111) == 0:
            reward += 40
            send_debug(sock, "BRAIN", "BONUS: LIBERATO!")

        # --- BELLMAN UPDATE 
        q_table[s_att][azione] += LR * (reward + GAMMA * max(q_table[s_nuovo]) - q_table[s_att][azione])

        # normalizzo
        if q_table[s_att][azione] > 50: q_table[s_att][azione] = 50
        elif q_table[s_att][azione] < -50: q_table[s_att][azione] = -50

        send_debug(sock, "UPDATE", f"Az:{azione} Rew:{reward} -> Q:{q_table[s_att][azione]:.1f}")

        passi_totali += 1
        if passi_totali % 20 == 0:
            salva_q_table(q_table)
            send_debug(sock, "STORAGE", "Salvataggio Flash effettuato.")
            
        gc.collect()

if __name__ == "__main__":
    main()
