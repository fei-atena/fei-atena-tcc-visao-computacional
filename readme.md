# Sistema de Comunicação MQTT para ROS

## Visão Geral
Este repositório faz parte do sistema de controle do robô Atena, sendo responsável pela comunicação entre o sistema de visão computacional e o ROS2. O sistema recebe dados de pose e estado das mãos via MQTT e os disponibiliza para o sistema ROS através de um arquivo JSON.

## Instalação

1. Instale as dependências Python:
```bash
pip install paho-mqtt
```

## Configuração

O sistema utiliza as seguintes configurações padrão:
- Broker MQTT: localhost (0.0.0.0)
- Porta: 1883
- Tópico: "pingpong/ros"
- Arquivo de saída: "recebido.json"

Para alterar o broker MQTT, edite a variável `BROKER` no código para o IP da sua máquina virtual ou servidor MQTT.

## Executando o Sistema

1. Inicie o broker MQTT (se estiver usando broker local):
```bash
mosquitto -v
```

2. Execute o script de comunicação:
```bash
python3 mqtt_subscriber.py
```

## Estrutura dos Dados

O sistema recebe dados da visão computacional no formato JSON contendo:
- Estado das mãos (aberta/fechada)
- Ângulos das juntas do corpo
- Informações de pose

Os dados são salvos no arquivo `recebido.json` e podem ser lidos pelo nó de processamento ROS.

## Integração com ROS

Este sistema trabalha em conjunto com os seguintes pacotes ROS:
- `communication_pkg`: Lê o arquivo JSON e publica os dados nos tópicos ROS
- `controller_pkg`: Controla os atuadores baseado nos dados recebidos

Para mais informações sobre os pacotes ROS, consulte a documentação específica de cada pacote.

## Depuração

Para verificar se os dados estão sendo recebidos corretamente:

1. Monitore as mensagens MQTT:
```bash
mosquitto_sub -t "pingpong/ros" -v
```

2. Verifique o arquivo JSON gerado:
```bash
cat recebido.json
```

## Observações

- Certifique-se de que o broker MQTT está acessível no endereço configurado
- O arquivo JSON é atualizado em tempo real conforme novas mensagens são recebidas
- Em caso de erro na decodificação JSON, o erro será exibido no console
