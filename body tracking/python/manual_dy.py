import os
import sys
import time
import json
import msvcrt  # Para Windows
from dynamixel_sdk import *  # Usa Dynamixel SDK

# Configurações do Dynamixel
DEVICENAME = 'COM12'  # Altere para sua porta COM (COM3, COM4, etc)
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DXL_ID = 9

# Endereços de controle
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Limites de posição (ajuste conforme necessário)
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095

# Configurações de velocidade (através de delays)
MIN_DELAY = 0.05    # Delay mínimo entre movimentos (mais rápido)
MAX_DELAY = 0.5     # Delay máximo entre movimentos (mais lento)
DEFAULT_DELAY = 0.01 # Delay padrão entre movimentos

# Configurações de movimento incremental
STEP_SIZE = 20       # Quantos ticks mover por passo
TOTAL_MOVEMENT = 1900 # Movimento total desejado

# Arquivo JSON
JSON_FILENAME = 'body_point.json'

# Inicialização do port handler e packet handler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Variável para controlar se a porta foi aberta com sucesso
port_opened = False

# Variáveis para controlar o estado anterior das mãos
previous_righthand = False
previous_lefthand = False

# Variável para controle de velocidade atual
current_delay = DEFAULT_DELAY

# Função para ler o arquivo JSON
def read_json_file():
    try:
        if not os.path.exists(JSON_FILENAME):
            # Se o arquivo não existe, cria com valores padrão
            default_data = {"righthand": False, "lefthand": False}
            with open(JSON_FILENAME, 'w') as f:
                json.dump(default_data, f, indent=4)
            print(f"Arquivo {JSON_FILENAME} criado com valores padrão")
            return default_data
        
        with open(JSON_FILENAME, 'r') as f:
            data = json.load(f)
            return data
            
    except Exception as e:
        print(f"Erro ao ler arquivo JSON: {e}")
        return {"righthand": False, "lefthand": False}

# Função para configurar a porta serial
def setup_serial():
    global port_opened
    try:
        # Abre a porta serial
        if portHandler.openPort():
            print("Porta serial aberta com sucesso")
            port_opened = True
        else:
            print("Falha ao abrir a porta serial")
            return False
        
        # Configura a velocidade de transmissão
        if portHandler.setBaudRate(BAUDRATE):
            print(f"Baudrate configurado para {BAUDRATE}")
            return True
        else:
            print("Falha ao configurar baudrate")
            portHandler.closePort()
            port_opened = False
            return False
            
    except Exception as e:
        print(f"Erro na configuração serial: {e}")
        if port_opened:
            portHandler.closePort()
            port_opened = False
        return False

# Função para habilitar/desabilitar torque
def set_torque(enabled):
    if not port_opened:
        print("Porta serial não está aberta")
        return False
    
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID, ADDR_TORQUE_ENABLE, enabled
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Erro de comunicação: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return False
    elif dxl_error != 0:
        print(f"Erro do Dynamixel: {packetHandler.getRxPacketError(dxl_error)}")
        return False
    else:
        status = "habilitado" if enabled else "desabilitado"
        print(f"Torque {status}")
        return True

# Função para ler posição atual
def read_position():
    if not port_opened:
        print("Porta serial não está aberta")
        return None
    
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, DXL_ID, ADDR_PRESENT_POSITION
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Erro de comunicação: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return None
    elif dxl_error != 0:
        print(f"Erro do Dynamixel: {packetHandler.getRxPacketError(dxl_error)}")
        return None
    return dxl_present_position

# Função para escrever posição desejada
def write_position(position):
    if not port_opened:
        print("Porta serial não está aberta")
        return False
    
    # Limita a posição dentro dos limites
    position = max(DXL_MINIMUM_POSITION_VALUE, min(position, DXL_MAXIMUM_POSITION_VALUE))
    
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, DXL_ID, ADDR_GOAL_POSITION, position
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Erro de comunicação: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return False
    elif dxl_error != 0:
        print(f"Erro do Dynamixel: {packetHandler.getRxPacketError(dxl_error)}")
        return False
    return True

# Função para movimento suave e lento
def smooth_move(target_position, current_position):
    global current_delay
    
    print(f"Movendo suavemente para: {target_position}")
    print(f"Velocidade: delay de {current_delay:.3f}s entre passos")
    
    direction = 1 if target_position > current_position else -1
    steps = abs(target_position - current_position) // STEP_SIZE
    
    if steps == 0:
        # Se a distância for menor que STEP_SIZE, move diretamente
        if write_position(target_position):
            time.sleep(current_delay * 2)
        return target_position
    
    for step in range(steps):
        intermediate_position = current_position + (direction * STEP_SIZE * (step + 1))
        
        # Garante que não ultrapasse o alvo
        if (direction > 0 and intermediate_position > target_position) or \
           (direction < 0 and intermediate_position < target_position):
            intermediate_position = target_position
        
        if write_position(intermediate_position):
            time.sleep(current_delay)
    
    # Movimento final para garantir posição exata
    if write_position(target_position):
        time.sleep(current_delay)
    
    return target_position

# Função para verificar se houve mudança de estado
def check_state_change(current_data):
    global previous_righthand, previous_lefthand
    
    current_righthand = current_data.get("righthand", False)
    current_lefthand = current_data.get("lefthand", False)
    
    # Verifica se houve mudança em qualquer uma das mãos
    righthand_changed = (current_righthand != previous_righthand)
    lefthand_changed = (current_lefthand != previous_lefthand)
    
    # Atualiza os estados anteriores
    previous_righthand = current_righthand
    previous_lefthand = current_lefthand
    
    return righthand_changed, lefthand_changed, current_righthand, current_lefthand

# Função para processar os dados do JSON e calcular a posição
def process_json_data(data, current_position, righthand_changed, lefthand_changed):
    movement_made = False
    target_position = current_position
    
    # Verifica se a mão direita mudou de estado
    if righthand_changed:
        if data.get("righthand", False):
            target_position -= TOTAL_MOVEMENT
            print("Mão direita ABERTA detectada: +30 ticks")
        else:
            target_position += TOTAL_MOVEMENT
            print("Mão direita FECHADA detectada: -30 ticks")
        movement_made = True
    
    # Verifica se a mão esquerda mudou de estado
    if lefthand_changed:
        if data.get("lefthand", False):
            target_position -= TOTAL_MOVEMENT
            print("Mão esquerda ABERTA detectada: +30 ticks")
        else:
            target_position += TOTAL_MOVEMENT
            print("Mão esquerda FECHADA detectada: -30 ticks")
        movement_made = True
    
    # Garante que a posição está dentro dos limites
    target_position = max(DXL_MINIMUM_POSITION_VALUE, 
                         min(target_position, DXL_MAXIMUM_POSITION_VALUE))
    
    return target_position, movement_made

# Função principal
def main():
    global port_opened, previous_righthand, previous_lefthand, current_delay
    
    try:
        # Configura a comunicação serial
        if not setup_serial():
            print("Não foi possível configurar a porta serial. Verifique:")
            print("1. Se a porta COM12 está correta")
            print("2. Se o cabo está conectado")
            print("3. Se outra aplicação não está usando a porta")
            print("4. Se você tem permissões para acessar a porta serial")
            return
        
        # Habilita o torque
        if not set_torque(1):
            print("Não foi possível habilitar o torque")
            return
        
        # Lê a posição inicial
        current_position = read_position()
        if current_position is None:
            print("Erro ao ler posição inicial")
            return
        
        # Lê o estado inicial do JSON para inicializar as variáveis anteriores
        initial_data = read_json_file()
        previous_righthand = initial_data.get("righthand", False)
        previous_lefthand = initial_data.get("lefthand", False)
        
        print(f"Posição inicial: {current_position}")
        print(f"Estado inicial - Mão direita: {previous_righthand}, Mão esquerda: {previous_lefthand}")
        print(f"Velocidade atual: delay de {current_delay:.3f}s entre passos")
        print("Monitorando arquivo body_point.json...")
        print("Pressione ESC para sair")
        print("Pressione '+' para aumentar velocidade (menos delay)")
        print("Pressione '-' para diminuir velocidade (mais delay)")
        print("Pressione 'r' para resetar velocidade")
        
        while True:
            # Verifica se o usuário quer sair ou ajustar velocidade
            if msvcrt.kbhit():
                key = msvcrt.getch()
                if key == b'\x1b':  # ESC
                    break
                elif key == b'+':  # Aumentar velocidade (menos delay)
                    current_delay = max(MIN_DELAY, current_delay - 0.05)
                    print(f"Velocidade aumentada: delay de {current_delay:.3f}s")
                elif key == b'-':  # Diminuir velocidade (mais delay)
                    current_delay = min(MAX_DELAY, current_delay + 0.05)
                    print(f"Velocidade diminuída: delay de {current_delay:.3f}s")
                elif key == b'r' or key == b'R':  # Resetar velocidade
                    current_delay = DEFAULT_DELAY
                    print(f"Velocidade resetada: delay de {current_delay:.3f}s")
            
            # Lê o arquivo JSON
            json_data = read_json_file()
            
            # Verifica se houve mudança de estado
            righthand_changed, lefthand_changed, current_righthand, current_lefthand = check_state_change(json_data)
            
            # Processa os dados e calcula nova posição (só move se houver mudança)
            target_position, movement_made = process_json_data(json_data, current_position, righthand_changed, lefthand_changed)
            
            # Se houve movimento e a posição mudou, move o motor suavemente
            if movement_made and target_position != current_position:
                print(f"Movimento iniciado de {current_position} para {target_position}")
                
                # Movimento suave e lento
                current_position = smooth_move(target_position, current_position)
                
                # Lê a posição atual para verificar
                actual_pos = read_position()
                if actual_pos is not None:
                    print(f"Posição final do motor: {actual_pos}")
                    current_position = actual_pos
            else:
                # Apenas exibe o estado atual sem mover o motor
                print(f"Estado atual - Mão direita: {current_righthand}, Mão esquerda: {current_lefthand}")
            
            # Pequena pausa para não sobrecarregar o sistema
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nPrograma interrompido pelo usuário")
    
    except Exception as e:
        print(f"Erro: {e}")
    
    finally:
        # Desabilita o torque antes de sair (apenas se a porta estiver aberta)
        if port_opened:
            print("Desabilitando torque...")
            set_torque(0)
            
            # Fecha a porta serial
            portHandler.closePort()
            print("Porta serial fechada")
        else:
            print("Porta serial não estava aberta, nenhuma ação necessária")

if __name__ == "__main__":
    main()