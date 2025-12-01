#include "maze_solver/graph.hpp"
#include <algorithm>
#include <cctype>

namespace maze_solver {

MazeGraph::MazeGraph(int width, int height) 
    : width_(width), height_(height), robot_pos_(0, 0), target_pos_(0, 0) {
    grid_.resize(height, std::vector<CellType>(width, CellType::UNKNOWN));
}

void MazeGraph::set_cell(int x, int y, CellType type) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        grid_[y][x] = type;
    }
}

CellType MazeGraph::get_cell(int x, int y) const {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        return grid_[y][x];
    }
    return CellType::WALL;
}

void MazeGraph::set_robot_position(const Position& pos) {
    robot_pos_ = pos;
}

void MazeGraph::set_target_position(const Position& pos) {
    target_pos_ = pos;
}

bool MazeGraph::is_valid_position(const Position& pos) const {
    return pos.x >= 0 && pos.x < width_ && pos.y >= 0 && pos.y < height_;
}

bool MazeGraph::is_walkable(const Position& pos) const {
    if (!is_valid_position(pos)) return false;
    CellType cell = get_cell(pos.x, pos.y);
    return cell == CellType::EMPTY || cell == CellType::TARGET || cell == CellType::ROBOT;
}

std::vector<Position> MazeGraph::get_neighbors(const Position& pos) const {
    std::vector<Position> neighbors;
    
    // 4 direções: cima, direita, baixo, esquerda
    const std::vector<std::pair<int, int>> directions = {
        {0, -1},  // cima
        {1, 0},   // direita
        {0, 1},   // baixo
        {-1, 0}   // esquerda
    };
    
    for (const auto& [dx, dy] : directions) {
        Position neighbor(pos.x + dx, pos.y + dy);
        if (is_walkable(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

void MazeGraph::load_from_flat_data(const std::vector<std::string>& data, int width, int height) {
    width_ = width;
    height_ = height;
    grid_.clear();
    grid_.resize(height, std::vector<CellType>(width, CellType::EMPTY));
    
    bool robot_found = false;
    bool target_found = false;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = y * width + x;
            if (index < static_cast<int>(data.size())) {
                std::string value = data[index];
                
                // Remover espaços em branco
                value.erase(std::remove_if(value.begin(), value.end(), 
                    [](unsigned char c) { return std::isspace(c); }), value.end());
                
                // Converter para lowercase
                std::transform(value.begin(), value.end(), value.begin(),
                    [](unsigned char c) { return std::tolower(c); });
                
                // Formato do simulador:
                // 'f' = free (vazio)
                // 'b' = blocked (parede)
                // 'r' = robot
                // 't' = target
                if (value == "f" || value == "free" || value == "empty" || value == "0") {
                    grid_[y][x] = CellType::EMPTY;
                } else if (value == "b" || value == "blocked" || value == "wall" || value == "1") {
                    grid_[y][x] = CellType::WALL;
                } else if (value == "r" || value == "robot" || value == "2") {
                    grid_[y][x] = CellType::ROBOT;
                    robot_pos_ = Position(x, y);
                    robot_found = true;
                } else if (value == "t" || value == "target" || value == "3") {
                    grid_[y][x] = CellType::TARGET;
                    target_pos_ = Position(x, y);
                    target_found = true;
                } else {
                    // Valor desconhecido, tratar como vazio
                    grid_[y][x] = CellType::EMPTY;
                }
            }
        }
    }
    
    // Se não encontrou, procurar manualmente
    if (!robot_found) {
        for (size_t i = 0; i < data.size(); i++) {
            std::string val = data[i];
            std::transform(val.begin(), val.end(), val.begin(),
                [](unsigned char c) { return std::tolower(c); });
            
            if (val == "r" || val == "robot" || val == "2") {
                int y = i / width;
                int x = i % width;
                robot_pos_ = Position(x, y);
                if (y < height && x < width) {
                    grid_[y][x] = CellType::ROBOT;
                }
                break;
            }
        }
    }
    
    if (!target_found) {
        for (size_t i = 0; i < data.size(); i++) {
            std::string val = data[i];
            std::transform(val.begin(), val.end(), val.begin(),
                [](unsigned char c) { return std::tolower(c); });
            
            if (val == "t" || val == "target" || val == "3") {
                int y = i / width;
                int x = i % width;
                target_pos_ = Position(x, y);
                if (y < height && x < width) {
                    grid_[y][x] = CellType::TARGET;
                }
                break;
            }
        }
    }
}

} // namespace maze_solver