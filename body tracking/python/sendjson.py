import paho.mqtt.client as mqtt
import json
import time

# Configurações do broker (IP da VM Linux)
BROKER = "10.1.1.113"   # Colocar o IP do Linux
PORT = 1883
TOPIC = "pingpong/ros"

# Callback de conexão
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Conectado ao broker MQTT")
    else:
        print("Falha na conexão, código:", rc)

# Cria cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.connect(BROKER, PORT, 60)

# Loop infinito com atraso de 10ms
while True:
    try:
        # Lê o JSON do arquivo
        with open("c:/TCC/body_tracking/body tracking/python/body_point.json", "r") as f:
            data = json.load(f)

        # Converte para string JSON
        payload = json.dumps(data)

        # Publica no tópico
        client.publish(TOPIC, payload)
        print(f"Enviado: {payload}")

        # Aguarda 10ms antes do próximo envio
        time.sleep(0.01)

    except Exception as e:
        print(f"Erro ao enviar JSON: {e}")
