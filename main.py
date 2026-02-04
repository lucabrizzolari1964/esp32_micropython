import network, time, ujson, struct
from machine import Pin, UART, I2C  # Aggiungi UART qui
import ntptime
import dht
from pzem import PZEM 
from umqtt.simple import MQTTClient

# --- CONFIGURAZIONE ---
WIFI_SSID = "briz"
WIFI_PASS = "Luca0001"
BROKER_IP = "192.168.0.157"
USER = "luca"
PASS = "luca0001"

# --- HARDWARE ---
led = Pin(2, Pin.OUT)
sensor = dht.DHT22(Pin(14))

# --- TOPIC ---
DEV_ID = "luca01"
# Sensori
TOPIC_T_CONFIG  = "homeassistant/sensor/luca01t/config"
TOPIC_T_STATE   = "homeassistant/sensor/luca01t/state"
TOPIC_IP_CONFIG = "homeassistant/sensor/luca01ip/config"
TOPIC_IP_STATE  = "homeassistant/sensor/luca01ip/state"
# Switch
TOPIC_SW_CONFIG = "homeassistant/switch/luca01sw/config"
TOPIC_SW_STATE  = "homeassistant/switch/luca01sw/state"
TOPIC_SW_SET    = "homeassistant/switch/luca01sw/set"

uart = UART(2, baudrate=9600, tx=17, rx=16)
pzem = PZEM(uart)
# Topic e Discovery per la Potenza (W)
TOPIC_POW_CONFIG = "homeassistant/sensor/luca01p/config"
TOPIC_POW_STATE  = "homeassistant/sensor/luca01p/state"


# --- FUNZIONE LETTURA PZEM ---



def decode_pzem(res):
    try:
        # Tensione: Byte 3-4 (Uint16)
        v = struct.unpack(">H", res[3:5])[0] / 10.0
        
        # Corrente: Byte 5-9 (Uint32 con registri invertiti)
        # Leggiamo due Uint16 e li scambiamo
        i_l, i_h = struct.unpack(">HH", res[5:9])
        i = (i_h << 16 | i_l) / 1000.0
        
        # Potenza: Byte 9-13 (Uint32 con registri invertiti)
        p_l, p_h = struct.unpack(">HH", res[9:13])
        p = (p_h << 16 | p_l) / 10.0
        
        # Frequenza: Byte 19-20
        #f = struct.unpack(">H", res[19:21])[0] / 10.0
        f = struct.unpack(">H", res[17:19])[0] / 10.0
        return v, i, p, f
    except:
        return None, None, None, None


def read_and_publish():
    # 1. Fase di Lettura (READ)
    cmd = b"\x01\x04\x00\x00\x00\x0A\x70\x0D"
    uart.write(cmd)
    time.sleep(0.2)
    
    if uart.any():
        res = uart.read()
        if len(res) >= 25:
            # 2. Fase di Decodifica (DECODE)
            volts, amps, watts, freq = decode_pzem(res)
            
            if volts is not None:
                print(f"V: {volts}V | P: {watts}W | F: {freq}Hz")
                
                # 3. Fase di Pubblicazione (MQTT)
                #client.publish(TOPIC_V_STATE, str(volts))
                client.publish(TOPIC_POW_STATE, str(watts))
                #client.publish(TOPIC_F_STATE, str(freq))
        else:
            print("Pacchetto incompleto ricevuto")
    else:
        print("PZEM non risponde alla richiesta")


# --- CALLBACK PER RICEZIONE COMANDI ---
def sub_cb(topic, msg):
    print(f"Messaggio ricevuto: {topic} -> {msg}")
    if topic == TOPIC_SW_SET.encode():
        if msg == b"ON":
            led.value(1)
            client.publish(TOPIC_SW_STATE, "ON") # Conferma lo stato a HA
        elif msg == b"OFF":
            led.value(0)
            client.publish(TOPIC_SW_STATE, "OFF")

# --- CONNESSIONE WIFI ---
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASS)

print("Connessione Wi-Fi...", end="")
while not wlan.isconnected():
    time.sleep(1)
    print(".", end="")

mio_ip = wlan.ifconfig()[0]
print("\nIP:", mio_ip)

# --- PAYLOADS ---

dev_info = {"ids": [DEV_ID], "name": "Esp32_Luca"}

data_pow = {
    "name": "Potenza",
    "uniq_id": "luca01pow",
    "stat_t": TOPIC_POW_STATE,
    "unit_of_meas": "W",
    "dev_cla": "power",
    "dev": dev_info
}

data_t = {
    "name": "Temperatura",
    "uniq_id": "luca01t",
    "stat_t": TOPIC_T_STATE,
    "unit_of_meas": "C",
    "dev_cla": "temperature",
    "dev": dev_info
}

data_ip = {
    "name": "IP",
    "uniq_id": "luca01ip",
    "stat_t": TOPIC_IP_STATE,
    "ent_cat": "diagnostic",
    "dev": dev_info
}

data_sw = {
    "name": "Led Interno",
    "uniq_id": "luca01sw",
    "stat_t": TOPIC_SW_STATE,
    "cmd_t": TOPIC_SW_SET, # Topic dove riceve i comandi
    "payload_on": "ON",
    "payload_off": "OFF",
    "dev": dev_info
}

# --- MQTT SETUP ---
client = MQTTClient("esp32_luca", BROKER_IP, user=USER, password=PASS)
client.set_callback(sub_cb)

try:
    ntptime.settime()
    print("Sincronizzazione completata!")
except Exception as e:
    print("Errore durante la sincronizzazione:", e)

# 2. Visualizza l'ora attuale (UTC)
print("Ora attuale :", time.localtime())


try:
    client.connect()
    # Invia le 3 configurazioni
    client.publish(TOPIC_T_CONFIG, ujson.dumps(data_t).encode('utf-8'), retain=True)
    client.publish(TOPIC_IP_CONFIG, ujson.dumps(data_ip).encode('utf-8'), retain=True)
    client.publish(TOPIC_SW_CONFIG, ujson.dumps(data_sw).encode('utf-8'), retain=True)
    client.publish(TOPIC_POW_CONFIG, ujson.dumps(data_pow).encode('utf-8'), retain=True)

    # Sottoscrizione al topic del comando
    client.subscribe(TOPIC_SW_SET)
    
    # Stato iniziale
    client.publish(TOPIC_IP_STATE, mio_ip, retain=True)
    client.publish(TOPIC_SW_STATE, "OFF")
    print("MQTT Pronto e Sottoscritto!")
except Exception as e:
    print("Errore MQTT:", e)

# --- LOOP ---
while True:
    try:
        print("inizio loop principale")
        # Controlla se sono arrivati messaggi (fondamentale per lo switch)
        client.check_msg()
        #
        sensor.measure()
        t = sensor.temperature()
        client.publish(TOPIC_T_STATE, str(t))
        #
        read_and_publish()
        year, month, day, hour, minute, second, weekday, yearday = time.localtime()
        print(f"Date: {day}/{month}/{year}")
        print(f"Time: {hour}:{minute}:{second}")
        print("fine loop principale")
        print("....................................")
    except Exception as e:
        print("Errore loop:", e)
        try: client.connect(); client.subscribe(TOPIC_SW_SET)
        except: pass
    
    time.sleep(5) # Ridotto a 5 secondi per maggiore reattività
