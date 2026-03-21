from machine import I2C, Pin, time_pulse_us
import time
import urandom 
from ir_rx.nec import NEC_8

# --- CONFIGURAZIONE ---
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
addr = 0x20 

# Pin Echo e Trigger diretti su ESP32 per i LATERALI
echo_sinistro = Pin(18, Pin.IN)
echo_destro = Pin(19, Pin.IN)
echo_sotto = Pin(5, Pin.IN)

# Nuovi sensori laterali (GPIO liberi su ESP32)
trig_lat_sx = Pin(25, Pin.OUT)
echo_lat_sx = Pin(32, Pin.IN)
trig_lat_dx = Pin(26, Pin.OUT)
echo_lat_dx = Pin(33, Pin.IN)

button = Pin(23, Pin.IN, Pin.PULL_UP)
led = Pin(2, Pin.OUT)
pin_ir = Pin(15, Pin.IN, Pin.PULL_UP)

# Sequenze Motori (8 fasi)
seq_a = [0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09]
seq_b = [0x10, 0x30, 0x20, 0x60, 0x40, 0xC0, 0x80, 0x90]

def leggi_distanza(sensore):
    nome = ["FRONT_SX", "FRONT_DX", "SOTTO", "LAT_SX", "LAT_DX"][sensore]
    if sensore <= 2:
        if sensore == 0: trig_bit, echo_pin = 0x10, echo_sinistro
        elif sensore == 1: trig_bit, echo_pin = 0x20, echo_destro
        else: trig_bit, echo_pin = 0x40, echo_sotto
        
        i2c.writeto(addr, bytes([trig_bit, 0x00]))
        time.sleep_us(10)
        i2c.writeto(addr, bytes([0x00, 0x00]))
        durata = time_pulse_us(echo_pin, 1, 25000)
    else:
        t_pin, e_pin = (trig_lat_sx, echo_lat_sx) if sensore == 3 else (trig_lat_dx, echo_lat_dx)
        t_pin.value(1)
        time.sleep_us(10)
        t_pin.value(0)
        durata = time_pulse_us(e_pin, 1, 25000)

    if durata < 0: 
        # print(f"  [!] Errore lettura su {nome}")
        return 400
    dist = (durata / 2) / 29.1
    return dist

def muovi_fisico(passi, dir_a, dir_b, velocita=3):
    idx_a, idx_b = 0, 0
    for _ in range(passi):
        b_low = seq_a[idx_a] if dir_a != 0 else 0
        b_high = seq_b[idx_b] if dir_b != 0 else 0
        i2c.writeto(addr, bytes([b_low, b_high]))
        if dir_a != 0: idx_a = (idx_a + dir_a) % 8
        if dir_b != 0: idx_b = (idx_b + dir_b) % 8
        time.sleep_ms(velocita)
    i2c.writeto(addr, b'\x00\x00')

def esplora():
    print("-" * 40)
    print("DEBUG MODE: AVVIO ROBOT")
    print("-" * 40)
    urti = 0
    ultimo_urto = time.time()

    try:
        while True:
            # 1. LETTURA SENSORI
            d_sx = leggi_distanza(0)
            time.sleep_ms(15)
            d_dx = leggi_distanza(1)
            time.sleep_ms(15)
            l_sx = leggi_distanza(3)
            time.sleep_ms(15)
            l_dx = leggi_distanza(4)
            
            # Stampa Stato Sensori
            print(f"DIST -> F_SX:{d_sx:>3.0f} F_DX:{d_dx:>3.0f} | L_SX:{l_sx:>3.0f} L_DX:{l_dx:>3.0f} | URTI:{urti}")

            if time.time() - ultimo_urto > 15 and urti > 0: 
                print("  [i] Reset contatore urti (tempo scaduto)")
                urti = 0

            # --- LOGICA DI FUGA ---
            if d_sx < 15 or d_dx < 15 or (l_sx < 12 and l_dx < 12):
                urti += 1
                ultimo_urto = time.time()
                
                if urti >= 3:
                    print("  [!!!] ANGOLO RILEVATO! Manovra 180 gradi")
                    muovi_fisico(1200, -1, -1)
                    muovi_fisico(10000, 1, -1)
                    urti = 0
                else:
                    print(f"  [W] Ostacolo trovato. Evitamento standard #{urti}")
                    muovi_fisico(500, -1, -1)
                    if l_sx < l_dx: 
                        print("  [>] Gira a DESTRA (più spazio)")
                        muovi_fisico(5000, 1, -1)
                    else: 
                        print("  [<] Gira a SINISTRA (più spazio)")
                        muovi_fisico(5000, -1, 1)
                continue # Ricomincia il loop per rileggere i sensori
            
            # --- CORREZIONI DI ROTTA ---
            elif l_sx < 10: 
                print("  [~] Correzione: muro a SX, accosto a DX")
                muovi_fisico(150, 1, 0) 
            elif l_dx < 10: 
                print("  [~] Correzione: muro a DX, accosto a SX")
                muovi_fisico(150, 0, 1) 
            
            # --- MARCIA NORMALE ---
            else:
                muovi_fisico(1000, 1, 1, 2)

    except KeyboardInterrupt:
        i2c.writeto(addr, b'\x00\x00')
        print("DEBUG: Robot fermato manualmente.")

# Avvio
esplora()
