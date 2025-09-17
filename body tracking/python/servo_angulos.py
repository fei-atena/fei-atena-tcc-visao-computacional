import json
import time

while True:
    try:
        with open('body_point.json', 'r') as f:
            body_points = json.load(f)
        print(body_points['0']['x'])
    except Exception as e:
        print(f"Erro ao abrir ou ler o arquivo: {e}")