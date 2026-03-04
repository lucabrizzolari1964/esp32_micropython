import machine
from machine import I2C, Pin
import time

i2c = I2C(0, scl=Pin(22), sda=Pin(21))
addr = 0x20 # Verifica con i2c.scan()


# Pin Echo diretti su ESP32
echo1 = machine.Pin(18, machine.Pin.IN)
echo2 = machine.Pin(19, machine.Pin.IN)

# Sequenze separate per i due motori
seq_a = [0x01, 0x02, 0x04, 0x08] # Pin 0, 1, 2, 3
# Invertiamo l'ordine della sequenza B direttamente qui per testare il cablaggio
seq_b = [0x80, 0x40, 0x20, 0x10] # Pin 17, 16, 15, 14 (ordine bit 7,6,5,4 del secondo byte)

def get_distanza(sensore_id):
    # sensore_id 0 = S1 (P4), 1 = S2 (P5)
    trig_bit = 0x10 if sensore_id == 0 else 0x20
    echo_pin = echo1 if sensore_id == 0 else echo2
    
    # Invia impulso Trig via Expander (10 microsecondi)
    i2c.writeto(addr, bytes([trig_bit, 0x00]))
    time.sleep_us(10)
    i2c.writeto(addr, bytes([0x00, 0x00]))
    
    # Misura durata Echo (timeout 30ms = ~5 metri)
    durata = machine.time_pulse_us(echo_pin, 1, 30000)
    
    if durata < 0: return 999 # Errore o fuori portata
    
    # Calcolo distanza in cm (velocità suono / 2)
    distanza = (durata / 2) / 29.1
    return distanza


def test_direzioni(passi, d_a=1, d_b=1):
    idx_a = 0
    idx_b = 0
    
    for _ in range(passi):
        # Calcolo Byte A
        val_a = seq_a[idx_a]
        idx_a = (idx_a + d_a) % 4
        
        # Calcolo Byte B (Già posizionato sui bit alti 14-17)
        val_b = seq_b[idx_b]
        idx_b = (idx_b + d_b) % 4
        
        # Invio all'expander
        i2c.writeto(addr, bytes([val_a, val_b]))
        time.sleep_ms(4)

    i2c.writeto(addr, b'\x00\x00')

# TEST: A avanti (1), B indietro (-1)

while True:
    print(f"avvio motori")
    test_direzioni(500, d_a=-1, d_b=-1)
    printf(f"fermo  i motori")
    d1 = get_distanza(0)
    d2 = get_distanza(1)
    print(f"Dist S1: {d1:.1f} cm | Dist S2: {d2:.1f} cm")
    
    # Esempio: se troppo vicino (< 10cm), potresti fermare i motori
    if d1 < 10 :
        print("!!! OSTACOLO Destro!!!")
        test_direzioni(500, d_a=-1, d_b=-1)

    if  d2 < 10:
        print("!!! OSTACOLO Sinistro!!!")
        test_direzioni(500, d_a=-1, d_b=-1)
        
    time.sleep(1)


