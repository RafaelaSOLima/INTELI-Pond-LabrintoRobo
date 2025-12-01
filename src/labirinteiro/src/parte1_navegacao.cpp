#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "labirinteiro/graph.hpp"
#include "labirinteiro/pathfinder.hpp"
#include <chrono>
#include <thread>
#include <set>
#include <map>
#include <fstream>

using namespace std::chrono_literals;

class Parte1Navegacao : public rclcpp::Node {
public:
    Parte1Navegacao() : Node("parte1_navegacao") {
        // Cliente para obter o mapa
        map_client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        
        // Cliente para enviar comandos de movimento
        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        
        RCLCPP_INFO(this->get_logger(), "=== PARTE 1: Navegação com Mapa ===");
        
        // Aguardar serviços
        while (!map_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Aguardando serviço /get_map...");
        }
        
        while (!move_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Aguardando serviço /move_command...");
        }
        
        // Executar navegação
        run();
    }

private:
    void save_map_visual(const labirinteiro::MazeGraph& graph, const std::string& filename) {
        std::ofstream file(filename);
        auto robot = graph.get_robot_position();
        auto target = graph.get_target_position();
        
        for (int y = 0; y < graph.get_height(); y++) {
            for (int x = 0; x < graph.get_width(); x++) {
                if (x == robot.x && y == robot.y) {
                    file << "R";
                } else if (x == target.x && y == target.y) {
                    file << "T";
                } else {
                    auto cell = graph.get_cell(x, y);
                    if (cell == labirinteiro::CellType::WALL) {
                        file << "#";
                    } else if (cell == labirinteiro::CellType::EMPTY) {
                        file << ".";
                    } else {
                        file << "?";
                    }
                }
            }
            file << "\n";
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "💾 Mapa visual salvo em: %s", filename.c_str());
    }

    void run() {
        // Passo 1: Obter o mapa completo
        RCLCPP_INFO(this->get_logger(), "📡 Obtendo mapa completo...");
        auto map_data = get_map();
        
        if (!map_data) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao obter mapa!");
            return;
        }
        
        // Verificar se recebemos dados válidos
        if (map_data->occupancy_grid_shape.size() != 2) {
            RCLCPP_ERROR(this->get_logger(), "Shape do mapa inválido!");
            return;
        }
        
        int height = map_data->occupancy_grid_shape[0];
        int width = map_data->occupancy_grid_shape[1];
        
        RCLCPP_INFO(this->get_logger(), "✓ Mapa recebido: %dx%d", width, height);
        RCLCPP_INFO(this->get_logger(), "   Total de células: %zu", map_data->occupancy_grid_flattened.size());
        
        // Debug: Valores únicos no mapa
        std::map<std::string, int> value_counts;
        for (const auto& cell : map_data->occupancy_grid_flattened) {
            value_counts[cell]++;
        }
        
        RCLCPP_INFO(this->get_logger(), "🔍 Análise do mapa:");
        for (const auto& [val, count] : value_counts) {
            std::string tipo = "";
            if (val == "b" || val == "blocked" || val == "wall") tipo = "(parede)";
            else if (val == "f" || val == "free" || val == "empty") tipo = "(vazio)";
            else if (val == "r" || val == "robot") tipo = "(robô)";
            else if (val == "t" || val == "target") tipo = "(alvo)";
            
            RCLCPP_INFO(this->get_logger(), "   '%s' %s: %d células", val.c_str(), tipo.c_str(), count);
        }
        
        // Passo 2: Criar grafo do labirinto
        RCLCPP_INFO(this->get_logger(), "🗺️  Criando grafo do labirinto...");
        labirinteiro::MazeGraph graph(width, height);
        graph.load_from_flat_data(map_data->occupancy_grid_flattened, width, height);
        
        auto start = graph.get_robot_position();
        auto goal = graph.get_target_position();
        
        RCLCPP_INFO(this->get_logger(), "   Posição inicial (robô): (%d, %d)", start.x, start.y);
        RCLCPP_INFO(this->get_logger(), "   Posição do alvo: (%d, %d)", goal.x, goal.y);
        
        // Salvar visualização do mapa
        save_map_visual(graph, "/tmp/maze_visual.txt");
        
        // Verificar vizinhos do robô
        auto neighbors = graph.get_neighbors(start);
        RCLCPP_INFO(this->get_logger(), "   Vizinhos válidos do robô: %zu", neighbors.size());
        for (size_t i = 0; i < neighbors.size() && i < 4; i++) {
            RCLCPP_INFO(this->get_logger(), "     → (%d, %d)", neighbors[i].x, neighbors[i].y);
        }
        
        // Verificar se as posições são válidas
        if (start.x == 0 && start.y == 0 && goal.x == 0 && goal.y == 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Posições inválidas! Robot e Target não foram encontrados no mapa.");
            RCLCPP_ERROR(this->get_logger(), "   Verifique /tmp/maze_visual.txt para ver o mapa parseado");
            return;
        }
        
        // Passo 3: Encontrar caminho ótimo usando A*
        RCLCPP_INFO(this->get_logger(), "🧮 Calculando caminho ótimo (A*)...");
        labirinteiro::PathFinder pathfinder(graph);
        auto path = pathfinder.find_path(start, goal);
        
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Nenhum caminho encontrado!");
            RCLCPP_ERROR(this->get_logger(), "   Possíveis causas:");
            RCLCPP_ERROR(this->get_logger(), "   - Labirinto sem solução");
            RCLCPP_ERROR(this->get_logger(), "   - Robô cercado por paredes");
            RCLCPP_ERROR(this->get_logger(), "   Verifique o mapa em /tmp/maze_visual.txt");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "✓ Caminho encontrado! Comprimento: %zu passos", path.size());
        
        // Mostrar primeiros passos do caminho
        RCLCPP_INFO(this->get_logger(), "   Primeiros passos do caminho:");
        for (size_t i = 0; i < std::min(size_t(5), path.size()); i++) {
            RCLCPP_INFO(this->get_logger(), "     [%zu] (%d, %d)", i, path[i].x, path[i].y);
        }
        
        // Passo 4: Converter caminho em comandos
        auto commands = pathfinder.path_to_commands(path);
        RCLCPP_INFO(this->get_logger(), "🎮 Comandos gerados: %zu", commands.size());
        
        // Mostrar primeiros comandos
        RCLCPP_INFO(this->get_logger(), "   Primeiros comandos:");
        for (size_t i = 0; i < std::min(size_t(5), commands.size()); i++) {
            RCLCPP_INFO(this->get_logger(), "     [%zu] %s", i, commands[i].c_str());
        }
        
        // Passo 5: Executar comandos
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "🚀 Iniciando navegação em 2 segundos...");
        RCLCPP_INFO(this->get_logger(), "   (Pressione Ctrl+C para cancelar)");
        std::this_thread::sleep_for(2s);
        
        execute_commands(commands);
        
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "✓ Navegação concluída com sucesso!");
    }
    
    std::shared_ptr<cg_interfaces::srv::GetMap::Response> get_map() {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto future = map_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            return future.get();
        }
        
        return nullptr;
    }
    
    bool send_move_command(const std::string& direction) {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;
        
        auto future = move_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 2s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            
            if (!response->success) {
                RCLCPP_WARN(this->get_logger(), "   ⚠️  Movimento não foi bem-sucedido");
            }
            
            return response->success;
        }
        
        RCLCPP_ERROR(this->get_logger(), "   ❌ Timeout ao enviar comando");
        return false;
    }
    
    void execute_commands(const std::vector<std::string>& commands) {
        RCLCPP_INFO(this->get_logger(), "");
        
        for (size_t i = 0; i < commands.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "📍 [%zu/%zu] Movendo: %s", 
                       i + 1, commands.size(), commands[i].c_str());
            
            if (!send_move_command(commands[i])) {
                RCLCPP_ERROR(this->get_logger(), "❌ Falha ao executar comando: %s", 
                            commands[i].c_str());
                RCLCPP_ERROR(this->get_logger(), "   Navegação interrompida no passo %zu de %zu", 
                            i + 1, commands.size());
                return;
            }
            
            // Pequeno delay para visualização
            std::this_thread::sleep_for(200ms);
        }
    }
    
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Parte1Navegacao>();
    rclcpp::shutdown();
    return 0;
}