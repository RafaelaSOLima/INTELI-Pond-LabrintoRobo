# Ponderada Robô Labirinto com ROS2

## Estrutura do Projeto
```
INTELI-Pond-LabrintoRobo/
├── src/
│   ├── cg/                          # Retirado do repo do Nicola
│   ├── cg_interfaces/               # Retirado do repo do Nicola
│   ├── cg_teleop/                   # Retirado do repo do Nicola
│   └── labirinteiro/                 # Minha Solução

├── build/                           # Arquivos de compilação
├── install/                         # Executáveis
├── log/                             # Logs
├── .gitignore
├── LICENSE
└── README.md                        # Este arquivo
``` 


### Estrutura da Minha Solução
```
src/labirinteiro/
├── include/labirinteiro/
│   ├── graph.hpp          # Estrutura de dados do grafo
│   ├── pathfinder.hpp     # A* (Parte 1)
│   └── mapper.hpp         # Trémaux e A* (Parte 2)
└── src/
    ├── graph.cpp
    ├── pathfinder.cpp
    ├── mapper.cpp
    ├── parte1_navegacao.cpp    # Navegação com mapa
    └── parte2_mapeamento.cpp   # Exploração + navegação
```

## Requisitos

Para rodar esse projeto, você precisará ter na sua maquina instalado:

- **ROS2** (Humble ou superior)
- **C++ Compiler** 
- **Python 3** 
- **Pygame** 

## Como Rodar:

Abra o terminal e:

1. Certifique-se de estar no repositório correto:

```bash
cd ~/Documents/GitHub/INTELI-Pond-LabrintoRobo
```
2. Build 

```bash
colcon build
source install/setup.bash
```

3. Configura o ambiente do terminal -- IMPORTANTE: SEMPRE RODE ESSE CÓDIGO AO ABRIR UM NOVO TERMINAL

```bash
source install/setup.bash
```

### Rodar parte 1:

**Terminal 1 - Simulador:**
```bash
ros2 run cg maze -- --generate
```

**Terminal 2 - Navegação:**
```bash
ros2 run labirinteiro parte1_navegacao
```
### Rodar parte 2:

**Terminal 1 - Simulador:**
```bash
ros2 run cg maze -- --generate
```

**Terminal 2 - Mapeamento:**
```bash
ros2 run labirinteiro parte2_mapeamento
```

### Outros comandos importantes:

* **Parar a execução/fechar simulação:**   Ctrl + C
* **ROdar novamente:** 

    * **Mesmo Labirinto:** execute os comandos novamente nos mesmos terminais
    * **Novo Labirinto Aleatório:** Feche o simulador atual (Ctrl+C) ou abra um novo terminal e rode os mesmos comandos.
* **Usar um labirinto específico:** 

No terminal 1 rode:
```bash
ros2 run cg maze -- --map <nome_do_arquivo.csv> #aqui o nome do arquivo escolhido
```
* **Apagar o Build para Buildar novamente:**

```bash
rm -rf build install log
```

## Como Adicionar Novo Labirinto

No caminho:
```
~/Documents/GitHub/INTELI-Pond-LabrintoRobo/src/cg/maps/
```
adicione o novo labirinto .csv que dever ter o seguinte formato:
   - Cada célula separada por vírgula
   - Valores: `f` (vazio), `b` (parede), `r` (robô), `t` (alvo)


