from machine import I2C, Pin, time_pulse_us
import time
import urandom 
import network
import socket
import os
import gc

#server di log
SERVER_IP = "192.168.0.157" 
SERVER_PORT = 5005
indirizzo_ip=""
log_msg_stati=""


# --- CONFIGURAZIONE HARDWARE ---
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
addr = 0x20 

# Pin Echo (Tutti su ESP32)
echo_sx, echo_dx = Pin(18, Pin.IN), Pin(19, Pin.IN)
echo_sotto = Pin(5, Pin.IN)
echo_lat_sx, echo_lat_dx = Pin(32, Pin.IN), Pin(33, Pin.IN)
#sensore centrale 
echo_centrale = Pin(35, Pin.IN)
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
        print(f'Indirizzo IP Robot : { indirizzo_ip} ')
        return config[0]
    else:
        print('\nErrore: Impossibile connettersi al Wi-Fi.')
        return None

def send_udp(sock,message):
    # 1. Creazione del socket UDP (AF_INET per IPv4, SOCK_DGRAM per UDP)
    #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        #print(f"Invio messaggio {message} {SERVER_IP}:{SERVER_PORT}...")
        sock.sendto(message.encode(), (SERVER_IP, SERVER_PORT))
        #print("Messaggio inviato correttamente.")
        
    except Exception as e:
        print("ERRORE ERRORE Errore durante l'invio:", e)    
    #finally:
        # 3. Chiusura del socket per liberare risorse
    #    sock.close()

# --- FUNZIONI DI LETTURA E MOVIMENTO ---

def leggi_distanza(sensore):
    """ 0:F_SX, 1:F_DX, 2:SOTTO, 3:L_SX, 4:L_DX, 5:F_CENTRALE """
    # Sensori su Expander I2C (0, 1, 2 e il nuovo 5 su P7)
    if sensore <= 2 or sensore == 5:
        # Se sensore è 5 usa 0x80 (P7), altrimenti usa la lista per 0,1,2
        trig_bit = 0x80 if sensore == 5 else [0x10, 0x20, 0x40][sensore]
        e_pin = echo_centrale if sensore == 5 else [echo_sx, echo_dx, echo_sotto][sensore]
        
        i2c.writeto(addr, bytes([trig_bit, 0x00]))
        time.sleep_us(10)
        i2c.writeto(addr, bytes([0x00, 0x00]))
    
    # Sensori con Trigger diretto su ESP32 (3 e 4)
    else:
        t_pin, e_pin = (trig_lat_sx, echo_lat_sx) if sensore == 3 else (trig_lat_dx, echo_lat_dx)
        t_pin.value(1)
        time.sleep_us(10)
        t_pin.value(0)
    
    durata = time_pulse_us(e_pin, 1, 25000)
    return 400 if durata < 0 else (durata / 2) / 29.1


def get_distanza_filtrata(sensore, campioni=5):
    letture = []
    
    for _ in range(campioni):
        # --- TUA LOGICA ORIGINALE (Leggermente ottimizzata) ---
        if sensore <= 2 or sensore == 5:
            trig_bit = 0x80 if sensore == 5 else [0x10, 0x20, 0x40][sensore]
            e_pin = echo_centrale if sensore == 5 else [echo_sx, echo_dx, echo_sotto][sensore]
            i2c.writeto(addr, bytes([trig_bit, 0x00]))
            time.sleep_us(10)
            i2c.writeto(addr, bytes([0x00, 0x00]))
        else:
            t_pin, e_pin = (trig_lat_sx, echo_lat_sx) if sensore == 3 else (trig_lat_dx, echo_lat_dx)
            t_pin.value(1)
            time.sleep_us(10)
            t_pin.value(0)
        
        durata = time_pulse_us(e_pin, 1, 25000)
        dist = 400 if durata < 0 else (durata / 2) / 29.1
        letture.append(dist)
        #print(f"Distanza letta dal sensore {sensore}: {dist} cm")   
        time.sleep_ms(5) # Breve pausa tra i campioni per evitare interferenze dell'eco precedente

    # --- FILTRO MEDIANO ---
    # Ordina le letture e prendi quella centrale (scarta i picchi da riflessione)
    letture.sort()
    distanza_finale = letture[len(letture) // 2]
    
    return distanza_finale




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
    #i2c.writeto(addr, b'\x00\x00')



# --- LOGICA AI AVANZATA ---

def manovra_spirale(sock,log_msg_stati):
    # Crea 5 cerchi concentrici sempre più larghi
    for raggio in range(2, 7):
        # Il motore esterno (SX) gira più del motore interno (DX)
        # Il coefficiente (0.15 * raggio) allarga la curva gradualmente
        print(log_msg_stati+" -> Spirale "+str(300 * raggio))
        send_udp(sock, log_msg_stati+" -> Spirale "+str(300 * raggio))
        muovi_fisico(300 * raggio, 1, 0.15 * raggio, 1)
        # Sicurezza: se vede un ostacolo durante la spirale, interrompe subito
        fsx, fdx = leggi_distanza(0), leggi_distanza(1)
        fcent = leggi_distanza(5)
        lsx, ldx = leggi_distanza(3), leggi_distanza(4)
        if fsx < 20 or fdx < 20 or (lsx < 20 or ldx < 20 or fcent < 20):
            print(log_msg_stati+" -> Spirale Esco ")
            send_udp(sock, log_msg_stati+" -> Spirale Esco ")
            modalita_turbo = False  
            break

def esplora(sock):
    print("\n" + "="*70)
    print("FUNC ESPLORA ROBOT TAGLIAERBA LUCA V 1.0- 5 SENSORI ATTIVI" + "="*30)
    send_udp(sock, "FUNC ESPLORA ROBOT TAGLIAERBA V 1.0 LUCA AVVIATO!")
    print("="*70)
    
    urti_vicini = 0
    passi_totali = 0
    bilancio_sterzo = 0  # + = troppe DX, - = troppe SX
    modalita_turbo = False
    
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

            # 2. LETTURA RADAR COMPLETO (Incluso il nuovo FC)
            #fsx, fdx = leggi_distanza(0), leggi_distanza(1)
            fsx, fdx = get_distanza_filtrata(0, 5), get_distanza_filtrata(1, 5)
            fcent = leggi_distanza(5) # <--- NUOVA LETTURA CENTRALE
            lsx, ldx = leggi_distanza(3), leggi_distanza(4)

            # LOG AGGIORNATO (Corretto con FC e i due punti su FD)
            status = "TURBO" if modalita_turbo else "NORMAL"
            log_msg_stati=""
            #log_msg_stati = f"[{status}] sinistro:{lsx:2.0f}|Fronte sinistro:{fsx:2.0f}|Davanti:{fcent:2.0f}|Fronte destro:{fdx:2.0f}|destro:{ldx:2.0f} | B:{bilancio_sterzo} | P:{passi_totali} | UV:{urti_vicini}"
            log_msg_stati = f"{lsx:2.0f}|{fsx:2.0f}|{fcent:2.0f}|{fdx:2.0f}|{ldx:2.0f}|{bilancio_sterzo}|{passi_totali}|{urti_vicini}"
            #print(log_msg_stati)
            #send_udp(sock, log_msg_stati)

            # 3. LOGICA DECISIONALE AGGIORNATA
            
            # PRIORITÀ 1: CEMENTO O VUOTO
            if tipo_terreno == "Cemento" or dist_sotto > 18:
                print(f"[AI] ALLERTA: {tipo_terreno}! Indietro.")
                muovi_fisico(300, -1, -1, 3)
                muovi_fisico(5000, 1, -1, 3)
                bilancio_sterzo += 1

            # PRIORITÀ 2: OSTACOLO CENTRALE (Nuova logica)
            elif fcent < 10: 
                modalita_turbo = False
                passi_totali = 0    
                muovi_fisico(1000, -1, -1, 3) # Retromarcia per staccarsi dal muro
                fsx, fdx = leggi_distanza(0), leggi_distanza(1)
                # Sceglie dove girare in base ai sensori diagonali e laterali
                if fsx > fdx:
                        passi_random = urandom.randint(2000, 8000)
                        print(log_msg_stati+" -> Retromarcia 1000 e Sinistra passi "+str(passi_random))
                        send_udp(sock, log_msg_stati+" -> Retromarcia 1000 e Sinistra passi "+str(passi_random))
                        muovi_fisico(passi_random, -1, 1, 3); bilancio_sterzo -= 1
                else:
                        passi_random = urandom.randint(2000, 8000)
                        print(log_msg_stati+" -> Retromarcia 1000 e Destra passi "+str(passi_random))
                        send_udp(sock, log_msg_stati+" -> Retromarcia 1000 e Destra passi "+str(passi_random))
                        muovi_fisico(passi_random, 1, -1, 3); bilancio_sterzo += 1
                urti_vicini += 2

            elif fsx < 20 and fdx < 20 and fcent < 25:
                modalita_turbo = False
                passi_totali = 0    
                passi_random = 1000
                muovi_fisico(passi_random, -1, -1, 3) # Retromarcia per staccarsi dal muro
                bilancio_sterzo += 1
                fsx, fdx = leggi_distanza(0), leggi_distanza(1)
                if lsx > ldx:
                        passi_random = urandom.randint(2000, 8000)
                        print(log_msg_stati+" -> Retromarcia 1000 e Sinistra passi "+str(passi_random))
                        send_udp(sock, log_msg_stati+" -> Retromarcia 1000 e Sinistra passi "+str(passi_random))
                        muovi_fisico(passi_random, -1, 1, 3); bilancio_sterzo -= 1
                else:
                        passi_random = urandom.randint(2000, 8000)
                        print(log_msg_stati+" -> Retromarcia 1000 e Destra passi "+str(passi_random))
                        send_udp(sock, log_msg_stati+" -> Retromarcia 1000 e Destra passi "+str(passi_random))
                        muovi_fisico(passi_random, 1, -1, 3); bilancio_sterzo += 1
                urti_vicini += 2
            
               # PRIORITÀ 3: OSTACOLI DIAGONALI (SX o DX)

            elif fsx < 20 or fdx < 20:
                if fsx < fdx: 
                        modalita_turbo = False  
                        passi_totali = 0
                        passi_random = urandom.randint(2000, 8000)
                        print(log_msg_stati+" -> Retromarcia 1000 e Destra "+str(passi_random))
                        send_udp(sock, log_msg_stati+" -> Retromarcia 1000 e Destra "+str(passi_random))
                        muovi_fisico(1000, -1, -1, 3) # Retromarcia per staccarsi dal muro
                        muovi_fisico(passi_random, 1, -1, 3)
                        bilancio_sterzo += 1
                        
                else:
                        modalita_turbo = False
                        passi_totali = 0
                        passi_random = urandom.randint(2000, 8000)
                        print(log_msg_stati+" -> Retromarcia 1000 e Sinistra "+str(passi_random))
                        send_udp(sock, log_msg_stati+" -> Retromarcia e Sinistra "+str(passi_random))
                        muovi_fisico(1000, -1, -1, 3) # Retromarcia per staccarsi dal muro
                        muovi_fisico(passi_random, -1, 1, 3)
                        bilancio_sterzo -= 1  
                urti_vicini += 1
            # PRIORITÀ 4: EVITARE INCASTRI LATERALI
            elif lsx < 15:
                modalita_turbo = False
                passi_totali = 0
                passi_random=600
                print(log_msg_stati+" -> Destra "+str(passi_random))
                send_udp(sock, log_msg_stati+" -> Destra "+str(passi_random))
                muovi_fisico(passi_random, 1, 0.6, 3) # Allontanati da SX
            
            elif ldx < 15:
                modalita_turbo = False
                passi_totali = 0
                passi_random=600
                print(log_msg_stati+" -> Sinistra "+str(passi_random))
                send_udp(sock, log_msg_stati+" -> Sinistra "+str(passi_random))
                muovi_fisico(passi_random, 0.6, 1, 3) # Allontanati da DX
        
            # PRIORITÀ 5: AVANTI TUTTA
            else:
                # Se non ci sono ostacoli da un po', ogni tanto fa una spirale
                if urti_vicini == 0 and passi_totali % 10 == 0 and passi_totali > 0:
                    print(log_msg_stati+" -> Spirale ")
                    send_udp(sock, log_msg_stati+" -> Spirale ")
                    manovra_spirale(sock,log_msg_stati)
                # Avanzamento rettilineo compensato
                corr_a = 1.0 if bilancio_sterzo <= 0 else 0.9
                corr_b = 1.0 if bilancio_sterzo > 0 else 0.9
                
                v_marcia = 1 if modalita_turbo else 3
                passi_da_fare = 1000 if not modalita_turbo else 2000
                passi_random=1000
                print(log_msg_stati+" -> Avanti  "+str(passi_random))
                send_udp(sock, log_msg_stati+" -> Avanti "+str(passi_random))
                muovi_fisico(passi_random, corr_a, corr_b, v_marcia)
                
                passi_totali += 1
                if urti_vicini > 0: urti_vicini -= 1

            # Gestione Turbo
            modalita_turbo = True if urti_vicini == 0 and passi_totali > 10 else False
            
            time.sleep_ms(10)
    except Exception as e:
        print("Errore loop:", e)
        send_udp(sock,"ERRORE LOOP: " + str(e))
        i2c.writeto(addr, b'\x00\x00') # Ferma tutto in caso di errore

try:
    indirizzo_ip = connetti_wifi("briz", "Luca0001")
    print(f'Indirizzo IP Robot: { indirizzo_ip }')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_udp(sock,f'Indirizzo ip del robot : {indirizzo_ip} ')
    send_udp(sock,f"Robot tagliaerba avviato e connesso al Wi-Fi!")
    send_udp(sock,f"[status] | [sinistro] | [Fronte sinistro] | [Davanti] | [Fronte destro] | [destro] | [bilancio_sterzo] | [passi_totali] | [urti_vicini] -> AZIONE ")
    esplora(sock)
except Exception as e:
    print("Errore critico all'avvio:", e)
    send_udp(sock,"ERRORE CRITICO ALL'AVVIO: " + str(e))    
