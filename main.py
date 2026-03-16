from machine import I2C, Pin, time_pulse_us
import time
import urandom 

# --- CONFIGURAZIONE ---
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
addr = 0x20 

# Pin Echo diretti su ESP32
echo_sinistro = Pin(18, Pin.IN)
echo_destro = Pin(19, Pin.IN)

# Sequenze Motori (Mappatura corretta come testato)
seq_a = [0x01, 0x02, 0x04, 0x08] # Pin P0-P3
seq_b = [0x10, 0x20, 0x40, 0x80] # Pin P17-P14 (Byte Alto)

def leggi_distanza(sensore):
    """ Restituisce cm. sensore: 0 (Sinistro/P4), 1 (Destro/P5) """
    trig_bit = 0x10 if sensore == 0 else 0x20
    echo_pin = echo_sinistro if sensore == 0 else echo_destro
    
    i2c.writeto(addr, bytes([trig_bit, 0x00]))
    time.sleep_us(10)
    i2c.writeto(addr, bytes([0x00, 0x00]))
    
    durata = time_pulse_us(echo_pin, 1, 20000)
    if durata < 0: return 400
    return (durata / 2) / 29.1

def muovi_fisico(passi, dir_a, dir_b, velocita=3):
    """ Funzione di basso livello per muovere i motori tramite expander """
    idx_a = 0
    idx_b = 0
    for _ in range(passi):
        b_low = seq_a[idx_a] if dir_a != 0 else 0
        b_high = seq_b[idx_b] if dir_b != 0 else 0
        
        i2c.writeto(addr, bytes([b_low, b_high]))
        
        if dir_a != 0: 
            idx_a = (idx_a + dir_a) % 4
        if dir_b != 0: 
            idx_b = (idx_b + dir_b) % 4
        time.sleep_ms(velocita)
    i2c.writeto(addr, b'\x00\x00')

def esplora():
    print("Inizio esplorazione... Premi Ctrl+C per fermare.")
    try:
        while True:
            # 1. Controlla sensori
            print(f"Controllo sensori")
            dist_sx = leggi_distanza(0)
            dist_dx = leggi_distanza(1)
            print(f"distanza DX ({dist_dx:.1f}cm distanza SX ({dist_sx:.1f}cm")
            # 2. Logica decisionale
            if dist_sx < 10 and dist_dx < 10:
                print("Ostacolo frontale! Retromarcia...")
                muovi_fisico(1024, -1, -1) # Indietro
                print("Ostacolo frontale! giro ")
                muovi_fisico(1024, 1, -1)  # Gira sul posto
                
            elif dist_sx < 10:
                print(f"Ostacolo a SX ({dist_sx:.1f}cm). Gira a DESTRA")
                angolo_random = urandom.randint(300, 800)
                muovi_fisico(300, -1, -1) # Un po' indietro
                muovi_fisico(angolo_random, 1, 0)   # Gira solo motore A
                
            elif dist_dx < 10:
                print(f"Ostacolo a DX ({dist_dx:.1f}cm). Gira a SINISTRA")
                angolo_random = urandom.randint(300, 800)
                muovi_fisico(300, -1, -1) # Un po' indietro
                muovi_fisico(angolo_random, 0, 1)   # Gira solo motore B
                
            else:
                # Nessun ostacolo: avanti tutta per un piccolo tratto
                # Facciamo passi brevi per ricontrollare spesso i sensori
                print(f"Nessun ostacolo vado dritto")
                muovi_fisico(512, 1, 1) 
                
    except KeyboardInterrupt:
        i2c.writeto(addr, b'\x00\x00')
        print("Robot fermato.")

# Avvia il loop
esplora()
