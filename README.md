# Projeto de Rastreamento Corporal - FEI Atena TCC

Este projeto implementa um sistema de rastreamento corporal em tempo real utilizando a câmera ZED e MediaPipe, capaz de detectar posições corporais e o estado das mãos (abertas/fechadas).


1. Instale as dependências Python:
```bash
python get_python_api.py  # Instala pyzed
pip install opencv-python numpy mediapipe PyOpenGL

## Estrutura do Projeto

- `tracking/`
  - `json_export.py`: Sistema principal de rastreamento corporal e exportação de dados
  - `bodies.json`: Arquivo de saída com dados do rastreamento
  - `ogl_viewer/`: Visualizador OpenGL para dados do rastreamento
- `get_python_api.py`: Script de instalação do pyzed

## Funcionalidades

- Rastreamento de 34 pontos do corpo usando ZED SDK
- Detecção do estado das mãos (abertas/fechadas) usando MediaPipe
- Exportação de dados em tempo real para JSON
- Visualização 3D do rastreamento
- Cálculo de orientação das articulações em ângulos de Euler

## Executando o Projeto

1. Certifique-se de que a câmera ZED está conectada e funcionando

2. Execute o sistema de rastreamento:
```bash
cd tracking
python json_export.py
```

## Dados de Saída

O arquivo `bodies.json` contém:
- Posições e orientações das articulações do corpo
- Estado das mãos (abertas/fechadas)
- Timestamps e níveis de confiança

## Observações

- Certifique-se de que a câmera ZED está corretamente conectada e calibrada
- Boa iluminação é essencial para detecção precisa
- O sistema rastreia especificamente:
  - Ombros (esquerdo e direito)
  - Cotovelos (esquerdo e direito)
  - Estado das mãos (abertas/fechadas)
- Aceleração GPU é recomendada para melhor desempenho
- O visualizador OpenGL mostra o rastreamento em tempo real

## Contribuindo

Sinta-se à vontade para abrir issues ou enviar pull requests. Por favor, teste suas alterações com a câmera ZED antes de submeter.
