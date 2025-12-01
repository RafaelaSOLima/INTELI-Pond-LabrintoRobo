#include "labirinteiro/mapper.hpp"
#include "labirinteiro/pathfinder.hpp"
#include <algorithm>

namespace labirinteiro {

Mapper::Mapper(int estimated_width, int estimated_height) 
    : graph_(estimated_width, estimated_height) {}

CellType Mapper::sensor_string_to_cell_type(const std::string& sensor_value) const {
    if (sensor_value == "f" || sensor_value == "free" || sensor_value == "empty" || sensor_value == "0") {
        return CellType::EMPTY;
    } else if (sensor_value == "b" || sensor_value == "blocked" || sensor_value == "wall" || sensor_value == "1") {
        return CellType::WALL;
    } else if (sensor_value == "r" || sensor_value == "robot" || sensor_value == "2") {
        return CellType::ROBOT;
    } else if (sensor_value == "t" || sensor_value == "target" || sensor_value == "3") {
        return CellType::TARGET;
    }
    return CellType::UNKNOWN;
}

void Mapper::update_from_sensor(const Position& robot_pos, const SensorData& sensor) {
    // Atualizar posição do robô
    graph_.set_robot_position(robot_pos);
    graph_.set_cell(robot_pos.x, robot_pos.y, CellType::ROBOT);
    
    // Estrutura: posição relativa -> dado do sensor
    const std::vector<std::pair<std::pair<int, int>, std::string>> sensor_map = {
        {{0, -1}, sensor.up},
        {{0, 1}, sensor.down},
        {{-1, 0}, sensor.left},
        {{1, 0}, sensor.right},
        {{-1, -1}, sensor.up_left},
        {{1, -1}, sensor.up_right},
        {{-1, 1}, sensor.down_left},
        {{1, 1}, sensor.down_right}
    };
    
    // Atualizar células adjacentes
    for (const auto& [offset, value] : sensor_map) {
        Position neighbor(robot_pos.x + offset.first, robot_pos.y + offset.second);
        
        if (graph_.is_valid_position(neighbor)) {
            CellType cell_type = sensor_string_to_cell_type(value);
            graph_.set_cell(neighbor.x, neighbor.y, cell_type);
            
            // Se encontrou o alvo
            if (cell_type == CellType::TARGET) {
                graph_.set_target_position(neighbor);
            }
        }
    }
}

void Mapper::mark_visited(const Position& pos) {
    visit_count_[pos]++;
    path_history_.push_back(pos);
}

int Mapper::get_visit_count(const Position& pos) const {
    auto it = visit_count_.find(pos);
    return (it != visit_count_.end()) ? it->second : 0;
}

bool Mapper::is_direction_valid(const std::string& direction, const SensorData& sensor) const {
    std::string cell_value;
    
    if (direction == "up") cell_value = sensor.up;
    else if (direction == "down") cell_value = sensor.down;
    else if (direction == "left") cell_value = sensor.left;
    else if (direction == "right") cell_value = sensor.right;
    else return false;
    
    // Válido se for vazio ou alvo
    return (cell_value == "f" || cell_value == "free" || cell_value == "empty" || 
            cell_value == "t" || cell_value == "target");
}

std::string Mapper::tremaux_next_move(const Position& current_pos, const SensorData& sensor) {
    /*
     * Algoritmo de Trémaux:
     * 1. Ao chegar em uma junção, marque o caminho de entrada
     * 2. Escolha um caminho não marcado (prioridade)
     * 3. Se todos estão marcados 1x, escolha qualquer um
     * 4. Se todos estão marcados 2x, volte pelo caminho de entrada
     * 5. Nunca passe mais de 2x pelo mesmo caminho
     */
    
    // Calcular posições dos vizinhos
    Position up_pos(current_pos.x, current_pos.y - 1);
    Position down_pos(current_pos.x, current_pos.y + 1);
    Position left_pos(current_pos.x - 1, current_pos.y);
    Position right_pos(current_pos.x + 1, current_pos.y);
    
    // Lista de movimentos possíveis com contadores
    struct MoveOption {
        std::string direction;
        Position next_pos;
        int visit_count;
        bool is_valid;
    };
    
    std::vector<MoveOption> moves = {
        {"up", up_pos, get_visit_count(up_pos), is_direction_valid("up", sensor)},
        {"down", down_pos, get_visit_count(down_pos), is_direction_valid("down", sensor)},
        {"left", left_pos, get_visit_count(left_pos), is_direction_valid("left", sensor)},
        {"right", right_pos, get_visit_count(right_pos), is_direction_valid("right", sensor)}
    };
    
    // Verificar se há alvo em alguma direção (prioridade máxima)
    if (sensor.up == "t" || sensor.up == "target") return "up";
    if (sensor.down == "t" || sensor.down == "target") return "down";
    if (sensor.left == "t" || sensor.left == "target") return "left";
    if (sensor.right == "t" || sensor.right == "target") return "right";
    
    // Regra 1: Priorizar caminhos nunca visitados (visit_count == 0)
    for (const auto& move : moves) {
        if (move.is_valid && move.visit_count == 0) {
            return move.direction;
        }
    }
    
    // Regra 2: Se não há caminhos não visitados, escolher visitado apenas 1x
    for (const auto& move : moves) {
        if (move.is_valid && move.visit_count == 1) {
            return move.direction;
        }
    }
    
    // Regra 3: Se todos visitados 2x ou mais, voltar (backtrack)
    // Escolher qualquer direção válida
    for (const auto& move : moves) {
        if (move.is_valid) {
            return move.direction;
        }
    }
    
    // Sem movimentos válidos
    return "";
}

bool Mapper::is_exploration_complete() const {
    // Exploração completa quando encontramos o alvo
    Position target = graph_.get_target_position();
    return (target.x != 0 || target.y != 0);
}

} // namespace labirinteiro