#include "maze_solver/mapper.hpp"
#include "maze_solver/pathfinder.hpp"

namespace maze_solver {

Mapper::Mapper(int estimated_width, int estimated_height) 
    : graph_(estimated_width, estimated_height) {}

CellType Mapper::sensor_string_to_cell_type(const std::string& sensor_value) const {
    if (sensor_value == "empty" || sensor_value == "0") {
        return CellType::EMPTY;
    } else if (sensor_value == "wall" || sensor_value == "1") {
        return CellType::WALL;
    } else if (sensor_value == "robot" || sensor_value == "2") {
        return CellType::ROBOT;
    } else if (sensor_value == "target" || sensor_value == "3") {
        return CellType::TARGET;
    }
    return CellType::UNKNOWN;
}

void Mapper::update_from_sensor(const Position& robot_pos, const SensorData& sensor) {
    // Atualizar posição do robô
    graph_.set_robot_position(robot_pos);
    graph_.set_cell(robot_pos.x, robot_pos.y, CellType::ROBOT);
    mark_visited(robot_pos);
    
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
            
            // Adicionar à fronteira se for explorável
            if (cell_type == CellType::EMPTY || cell_type == CellType::TARGET) {
                if (!visited_.count(neighbor)) {
                    frontier_.insert(neighbor);
                }
            }
        }
    }
}

void Mapper::mark_visited(const Position& pos) {
    visited_.insert(pos);
    frontier_.erase(pos);
}

Position Mapper::find_next_exploration_target(const Position& current) {
    // Se não há fronteira, retornar posição inválida
    if (frontier_.empty()) {
        return Position(-1, -1);
    }
    
    // Encontrar a célula de fronteira mais próxima
    Position closest = *frontier_.begin();
    int min_distance = current.manhattan_distance(closest);
    
    for (const Position& pos : frontier_) {
        int distance = current.manhattan_distance(pos);
        if (distance < min_distance) {
            min_distance = distance;
            closest = pos;
        }
    }
    
    return closest;
}

bool Mapper::is_exploration_complete() const {
    // Exploração completa quando não há mais fronteira
    return frontier_.empty();
}

void Mapper::update_frontier(const Position& pos) {
    const std::vector<std::pair<int, int>> directions = {
        {0, -1}, {0, 1}, {-1, 0}, {1, 0}
    };
    
    for (const auto& [dx, dy] : directions) {
        Position neighbor(pos.x + dx, pos.y + dy);
        
        if (graph_.is_valid_position(neighbor) && 
            !visited_.count(neighbor) && 
            graph_.get_cell(neighbor.x, neighbor.y) == CellType::UNKNOWN) {
            frontier_.insert(neighbor);
        }
    }
}

} // namespace maze_solver