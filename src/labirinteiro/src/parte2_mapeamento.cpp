#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "labirinteiro/graph.hpp"
#include "labirinteiro/pathfinder.hpp"
#include "labirinteiro/mapper.hpp"
#include <chrono>
#include <thread>
#include <fstream>

using namespace std::chrono_literals;

class Parte2Mapeamento : public rclcpp::Node {
public:
    Parte2Mapeamento() : Node("parte2_mapeamento"), 
                         mapper_(50, 50),  // Tamanho estimado menor
                         robot_pos_(1, 1),  // Posição inicial corrigida
                         target_found_(false),
                         exploration_moves_(0) {
        
        // Subscriber para sensores
        sensor_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 10,
            std::bind(&Parte2Mapeamento::sensor_callback, this, std::placeholders::_1));
        
        // Cliente para movimentação
        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        
        RCLCPP_INFO(this->get_logger(), "=== PARTE 2: Mapeamento e Navegação ===");
        
        // Aguardar serviços
        while (!move_client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Aguardando serviço /move_command...");
        }
        
        RCLCPP_INFO(this->get_logger(), "✓ Serviços prontos");
        RCLCPP_INFO(this->get_logger(), "📡 Aguardando dados dos sensores...");
        std::this_thread::sleep_for(1s);
        
        // Executar mapeamento e navegação
        run();
    }

private:
    void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        last_sensor_data_.up = msg->up;
        last_sensor_data_.down = msg->down;
        last_sensor_data_.left = msg->left;
        last_sensor_data_.right = msg->right;
        last_sensor_data_.up_left = msg->up_left;
        last_sensor_data_.up_right = msg->up_right;
        last_sensor_data_.down_left = msg->down_left;
        last_sensor_data_.down_right = msg->down_right;
        
        sensor_received_ = true;
    }
    
    void save_mapped_area(const std::string& filename) {
        std::ofstream file(filename);
        auto& graph = mapper_.get_graph();
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
                    } else if (cell == labirinteiro::CellType::UNKNOWN) {
                        file << "?";
                    } else {
                        file << "?";
                    }
                }
            }
            file << "\n";
        }
        file.close();
    }
    
    void run() {
        // Fase 1: Mapeamento por exploração
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "🗺️  === FASE 1: EXPLORAÇÃO E MAPEAMENTO ===");
        explore_maze();
        
        if (!target_found_) {
            RCLCPP_ERROR(this->get_logger(), "❌ Alvo não encontrado durante mapeamento!");
            return;
        }
        
        auto target = mapper_.get_graph().get_target_position();
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "✓ Mapeamento concluído!");
        RCLCPP_INFO(this->get_logger(), "   Total de movimentos de exploração: %d", exploration_moves_);
        RCLCPP_INFO(this->get_logger(), "   Alvo encontrado em: (%d, %d)", target.x, target.y);
        
        // Salvar mapa explorado
        save_mapped_area("/tmp/maze_mapped.txt");
        RCLCPP_INFO(this->get_logger(), "💾 Mapa explorado salvo em: /tmp/maze_mapped.txt");
        
        // Fase 2: Navegar usando o mapa criado
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "🧭 === FASE 2: NAVEGAÇÃO COM MAPA CRIADO ===");
        RCLCPP_INFO(this->get_logger(), "   Aguardando 2 segundos...");
        std::this_thread::sleep_for(2s);
        
        navigate_to_target();
        
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "🎉 Missão concluída com sucesso!");
    }
    
    void explore_maze() {
        int max_moves = 500;  // Reduzido para 500
        int moves_without_new_cells = 0;
        
        RCLCPP_INFO(this->get_logger(), "   🧭 Usando Algoritmo de Trémaux");
        RCLCPP_INFO(this->get_logger(), "   Iniciando exploração...");
        RCLCPP_INFO(this->get_logger(), "   Posição inicial: (%d, %d)", robot_pos_.x, robot_pos_.y);
        
        while (exploration_moves_ < max_moves && rclcpp::ok()) {
            // Aguardar dados do sensor
            wait_for_sensor();
            
            // Atualizar mapa com dados do sensor
            mapper_.update_from_sensor(robot_pos_, last_sensor_data_);
            mapper_.mark_visited(robot_pos_);
            
            // Log periódico
            if (exploration_moves_ % 20 == 0 && exploration_moves_ > 0) {
                int visits = mapper_.get_visit_count(robot_pos_);
                RCLCPP_INFO(this->get_logger(), "   📊 Mov: %d | Pos: (%d, %d) | Visitas: %d", 
                           exploration_moves_, robot_pos_.x, robot_pos_.y, visits);
            }
            
            // Verificar se encontrou o alvo nos sensores
            if (last_sensor_data_.up == "t" || last_sensor_data_.down == "t" ||
                last_sensor_data_.left == "t" || last_sensor_data_.right == "t") {
                
                if (!target_found_) {
                    target_found_ = true;
                    RCLCPP_INFO(this->get_logger(), "   🎯 Alvo detectado! Indo até ele...");
                    
                    // Ir direto para o alvo
                    std::string direction_to_target = "";
                    if (last_sensor_data_.up == "t") direction_to_target = "up";
                    else if (last_sensor_data_.down == "t") direction_to_target = "down";
                    else if (last_sensor_data_.left == "t") direction_to_target = "left";
                    else if (last_sensor_data_.right == "t") direction_to_target = "right";
                    
                    if (!direction_to_target.empty()) {
                        if (send_move_command(direction_to_target)) {
                            update_robot_position(direction_to_target);
                            exploration_moves_++;
                            RCLCPP_INFO(this->get_logger(), "   ✓ Alvo alcançado na exploração!");
                        } else {
                            RCLCPP_INFO(this->get_logger(), "   ✓ Já estamos no alvo!");
                        }
                    }
                    break;  // Parar exploração
                }
            }
            
            // Decidir próximo movimento usando Trémaux
            std::string next_move = mapper_.tremaux_next_move(robot_pos_, last_sensor_data_);
            
            if (next_move.empty()) {
                RCLCPP_WARN(this->get_logger(), "   ⚠️  Trémaux: Sem movimentos válidos");
                RCLCPP_INFO(this->get_logger(), "   Posição atual: (%d, %d)", robot_pos_.x, robot_pos_.y);
                RCLCPP_INFO(this->get_logger(), "   Sensores: up=%s down=%s left=%s right=%s",
                           last_sensor_data_.up.c_str(), last_sensor_data_.down.c_str(),
                           last_sensor_data_.left.c_str(), last_sensor_data_.right.c_str());
                break;
            }
            
            // Executar movimento
            if (send_move_command(next_move)) {
                update_robot_position(next_move);
                exploration_moves_++;
            } else {
                RCLCPP_WARN(this->get_logger(), "   ⚠️  Falha ao executar: %s", next_move.c_str());
            }
            
            std::this_thread::sleep_for(100ms);
        }
        
        if (exploration_moves_ >= max_moves) {
            RCLCPP_WARN(this->get_logger(), "   ⚠️  Limite de %d movimentos atingido", max_moves);
        }
    }
    
    void navigate_to_target() {
        auto& graph = mapper_.get_graph();
        auto start = robot_pos_;
        auto goal = graph.get_target_position();
        
        RCLCPP_INFO(this->get_logger(), "   Calculando rota otimizada...");
        RCLCPP_INFO(this->get_logger(), "   De: (%d, %d) → Até: (%d, %d)", 
                   start.x, start.y, goal.x, goal.y);
        
        // Verificar se já está no alvo
        if (start.x == goal.x && start.y == goal.y) {
            RCLCPP_INFO(this->get_logger(), "   ✓ Já estamos no alvo! Navegação desnecessária.");
            return;
        }
        
        // Verificar se está adjacente ao alvo (distância Manhattan = 1)
        int distance = abs(start.x - goal.x) + abs(start.y - goal.y);
        if (distance == 1) {
            RCLCPP_INFO(this->get_logger(), "   ✓ Alvo está ao lado! Um movimento apenas.");
            std::string final_move = "";
            if (goal.x > start.x) final_move = "right";
            else if (goal.x < start.x) final_move = "left";
            else if (goal.y > start.y) final_move = "down";
            else if (goal.y < start.y) final_move = "up";
            
            if (!final_move.empty() && send_move_command(final_move)) {
                RCLCPP_INFO(this->get_logger(), "   ✓ Alvo alcançado!");
                return;
            }
        }
        
        labirinteiro::PathFinder pathfinder(graph);
        auto path = pathfinder.find_path(start, goal);
        
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "   ❌ Não foi possível encontrar caminho!");
            RCLCPP_ERROR(this->get_logger(), "   O mapa pode estar incompleto");
            save_mapped_area("/tmp/maze_mapped_failed.txt");
            RCLCPP_ERROR(this->get_logger(), "   Veja /tmp/maze_mapped_failed.txt");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "   ✓ Caminho encontrado: %zu passos", path.size());
        
        // Converter e executar comandos
        auto commands = pathfinder.path_to_commands(path);
        RCLCPP_INFO(this->get_logger(), "   Comandos: %zu", commands.size());
        
        // Mostrar primeiros comandos
        RCLCPP_INFO(this->get_logger(), "   Primeiros comandos:");
        for (size_t i = 0; i < std::min(size_t(5), commands.size()); i++) {
            RCLCPP_INFO(this->get_logger(), "     [%zu] %s", i, commands[i].c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "");
        execute_commands(commands);
    }
    
    void execute_commands(const std::vector<std::string>& commands) {
        for (size_t i = 0; i < commands.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "   📍 [%zu/%zu] %s", 
                       i + 1, commands.size(), commands[i].c_str());
            
            if (!send_move_command(commands[i])) {
                RCLCPP_ERROR(this->get_logger(), "   ❌ Falha ao executar comando");
                return;
            }
            
            update_robot_position(commands[i]);
            std::this_thread::sleep_for(200ms);
        }
    }
    
    bool send_move_command(const std::string& direction) {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;
        
        auto future = move_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 2s) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            return future.get()->success;
        }
        
        return false;
    }
    
    void update_robot_position(const std::string& direction) {
        if (direction == "up") robot_pos_.y--;
        else if (direction == "down") robot_pos_.y++;
        else if (direction == "left") robot_pos_.x--;
        else if (direction == "right") robot_pos_.x++;
        
        // Garantir que a posição não fique negativa
        if (robot_pos_.x < 0) robot_pos_.x = 0;
        if (robot_pos_.y < 0) robot_pos_.y = 0;
        
        mapper_.get_graph().set_robot_position(robot_pos_);
    }
    
    void wait_for_sensor() {
        sensor_received_ = false;
        auto start_time = this->now();
        
        while (!sensor_received_ && (this->now() - start_time).seconds() < 2.0) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(10ms);
        }
    }
    
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    
    labirinteiro::Mapper mapper_;
    labirinteiro::SensorData last_sensor_data_;
    labirinteiro::Position robot_pos_;
    bool sensor_received_ = false;
    bool target_found_ = false;
    int exploration_moves_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Parte2Mapeamento>();
    rclcpp::shutdown();
    return 0;
}