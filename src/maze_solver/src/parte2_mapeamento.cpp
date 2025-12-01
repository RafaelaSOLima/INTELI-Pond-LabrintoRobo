#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "maze_solver/graph.hpp"
#include "maze_solver/pathfinder.hpp"
#include "maze_solver/mapper.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class Parte2Mapeamento : public rclcpp::Node {
public:
    Parte2Mapeamento() : Node("parte2_mapeamento"), 
                         mapper_(100, 100),  // Tamanho estimado do mapa
                         robot_pos_(0, 0),
                         target_found_(false) {
        
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
        
        RCLCPP_INFO(this->get_logger(), "Aguardando dados dos sensores...");
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
    
    void run() {
        // Fase 1: Mapeamento por exploração
        RCLCPP_INFO(this->get_logger(), "Iniciando fase de mapeamento...");
        explore_maze();
        
        if (!target_found_) {
            RCLCPP_ERROR(this->get_logger(), "Alvo não encontrado durante mapeamento!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "✓ Mapeamento concluído!");
        RCLCPP_INFO(this->get_logger(), "Alvo encontrado em: (%d, %d)", 
                   mapper_.get_graph().get_target_position().x,
                   mapper_.get_graph().get_target_position().y);
        
        // Fase 2: Navegar usando o mapa criado
        RCLCPP_INFO(this->get_logger(), "\nIniciando navegação com mapa criado...");
        navigate_to_target();
        
        RCLCPP_INFO(this->get_logger(), "✓ Missão concluída!");
    }
    
    void explore_maze() {
        int moves = 0;
        const int max_moves = 1000;
        
        while (moves < max_moves && rclcpp::ok()) {
            // Aguardar dados do sensor
            wait_for_sensor();
            
            // Atualizar mapa com dados do sensor
            mapper_.update_from_sensor(robot_pos_, last_sensor_data_);
            
            // Verificar se encontrou o alvo
            if (last_sensor_data_.up == "target" || last_sensor_data_.down == "target" ||
                last_sensor_data_.left == "target" || last_sensor_data_.right == "target") {
                target_found_ = true;
                RCLCPP_INFO(this->get_logger(), "🎯 Alvo encontrado!");
            }
            
            // Se alvo encontrado e já exploramos bastante, podemos parar
            if (target_found_ && moves > 50) {
                break;
            }
            
            // Decidir próximo movimento (exploração simples)
            std::string next_move = decide_exploration_move();
            
            if (next_move.empty()) {
                RCLCPP_INFO(this->get_logger(), "Exploração completa ou sem movimentos válidos");
                break;
            }
            
            // Executar movimento
            if (send_move_command(next_move)) {
                update_robot_position(next_move);
                moves++;
                
                if (moves % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Movimentos de exploração: %d", moves);
                }
            }
            
            std::this_thread::sleep_for(100ms);
        }
    }
    
    std::string decide_exploration_move() {
        // Estratégia simples: preferir células não visitadas
        std::vector<std::pair<std::string, std::string>> moves = {
            {"up", last_sensor_data_.up},
            {"right", last_sensor_data_.right},
            {"down", last_sensor_data_.down},
            {"left", last_sensor_data_.left}
        };
        
        // Primeiro, tentar ir para células vazias
        for (const auto& [direction, sensor] : moves) {
            if (sensor == "empty" || sensor == "0" || sensor == "target" || sensor == "3") {
                return direction;
            }
        }
        
        return "";
    }
    
    void navigate_to_target() {
        auto& graph = mapper_.get_graph();
        auto start = robot_pos_;
        auto goal = graph.get_target_position();
        
        RCLCPP_INFO(this->get_logger(), "Calculando rota otimizada...");
        RCLCPP_INFO(this->get_logger(), "De (%d, %d) até (%d, %d)", 
                   start.x, start.y, goal.x, goal.y);
        
        maze_solver::PathFinder pathfinder(graph);
        auto path = pathfinder.find_path(start, goal);
        
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Não foi possível encontrar caminho!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Caminho encontrado: %zu passos", path.size());
        
        // Converter e executar comandos
        auto commands = pathfinder.path_to_commands(path);
        execute_commands(commands);
    }
    
    void execute_commands(const std::vector<std::string>& commands) {
        for (size_t i = 0; i < commands.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "[%zu/%zu] Movendo: %s", 
                       i + 1, commands.size(), commands[i].c_str());
            
            if (!send_move_command(commands[i])) {
                RCLCPP_ERROR(this->get_logger(), "Falha ao executar comando");
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
    
    maze_solver::Mapper mapper_;
    maze_solver::SensorData last_sensor_data_;
    maze_solver::Position robot_pos_;
    bool sensor_received_ = false;
    bool target_found_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Parte2Mapeamento>();
    rclcpp::shutdown();
    return 0;
}