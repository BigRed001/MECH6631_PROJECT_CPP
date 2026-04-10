#pragma once
#include "Types.h"
#include "Fuzzy.h"
#include "AStar.h"
#include "Waypoint.h"
#include <optional>
#include <vector>

class DefenseStrategy {
public:
    struct Result {
        Command cmd;
        std::optional<std::vector<std::pair<int, int>>> path;
        FuzzyDecision decision;
        std::optional<std::pair<double, double>> hiding_point;
    };

    DefenseStrategy(AStarPlanner* planner,
        WaypointFollower* follower,
        FuzzyLogic* fuzzy);

    Result compute(
        const std::vector<RobotTrack>& tracks,
        int my_id,
        const Grid& grid,
        int cell_px,
        int lookahead_cells,
        int samples,
        double v_max,
        const std::vector<Obstacle>& obstacles);

private:
    AStarPlanner* planner_;
    WaypointFollower* follower_;
    FuzzyLogic* fuzzy_;
};