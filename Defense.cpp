#include "Defense.h"
#include <cmath>
#include <algorithm>
#include <random>

static const RobotTrack* find_me(const std::vector<RobotTrack>& tracks, int my_id) {
    for (auto& t : tracks)
        if (t.id == my_id)
            return &t;
    return nullptr;
}

static const RobotTrack* pick_nearest_enemy(const std::vector<RobotTrack>& tracks, int my_id) {
    const RobotTrack* me = find_me(tracks, my_id);
    if (!me) return nullptr;

    const RobotTrack* best = nullptr;
    double best_d = 1e9;

    for (auto& t : tracks) {
        if (t.id == my_id) continue;
        double d = std::hypot(t.x - me->x, t.y - me->y);
        if (d < best_d) {
            best_d = d;
            best = &t;
        }
    }
    return best;
}

static const Obstacle* best_hiding_obstacle(
    const RobotTrack& me,
    const RobotTrack& enemy,
    const std::vector<Obstacle>& obs)
{
    const Obstacle* best = nullptr;
    double best_score = -1e9;

    auto line_dist = [&](double px, double py) {
        double ax = me.x, ay = me.y;
        double bx = enemy.x, by = enemy.y;

        double abx = bx - ax;
        double aby = by - ay;
        double apx = px - ax;
        double apy = py - ay;

        double ab2 = abx * abx + aby * aby;
        if (ab2 < 1e-9) return std::hypot(px - ax, py - ay);

        double t = (apx * abx + apy * aby) / ab2;
        t = clamp(t, 0.0, 1.0);

        double qx = ax + t * abx;
        double qy = ay + t * aby;

        return std::hypot(px - qx, py - qy);
        };

    for (auto& o : obs) {
        double cx = o.cx;
        double cy = o.cy;

        double r = 0.5 * std::hypot(o.w, o.h) + 12.0;
        double dline = line_dist(cx, cy);

        bool blocks = (dline <= r);

        double d_me = std::hypot(me.x - cx, me.y - cy);
        double d_en = std::hypot(enemy.x - cx, enemy.y - cy);

        double score = 0.0;
        if (blocks) score += 200.0;
        score += 0.15 * d_en;
        score -= 0.35 * d_me;

        if (score > best_score) {
            best_score = score;
            best = &o;
        }
    }

    return best;
}

static std::pair<double, double> hiding_point_behind_obstacle(
    const RobotTrack& enemy,
    const Obstacle& o,
    double stand_off_px)
{
    double cx = o.cx;
    double cy = o.cy;

    double vx = cx - enemy.x;
    double vy = cy - enemy.y;

    double n = std::hypot(vx, vy);
    if (n < 1e-9) return { cx, cy };

    double ux = vx / n;
    double uy = vy / n;

    double radius = 0.5 * std::hypot(o.w, o.h);

    double tx = cx + (radius + stand_off_px) * ux;
    double ty = cy + (radius + stand_off_px) * uy;

    return { tx, ty };
}

DefenseStrategy::DefenseStrategy(AStarPlanner* p,
    WaypointFollower* f,
    FuzzyLogic* fl)
    : planner_(p), follower_(f), fuzzy_(fl) {
}

DefenseStrategy::Result DefenseStrategy::compute(
    const std::vector<RobotTrack>& tracks,
    int my_id,
    const Grid& grid,
    int cell_px,
    int lookahead_cells,
    int samples,
    double v_max,
    const std::vector<Obstacle>& obstacles)
{
    Result res;
    res.cmd = { 0.0, 0.0, false };
    res.path = std::nullopt;
    res.hiding_point = std::nullopt;

    const RobotTrack* me = find_me(tracks, my_id);
    const RobotTrack* enemy = pick_nearest_enemy(tracks, my_id);

    if (!me || !enemy)
        return res;

    TacticalFeatures feat = fuzzy_->extractFeatures(*me, *enemy, obstacles);
    FuzzyDecision dec = fuzzy_->defense(feat);
    res.decision = dec;

    double v_scaled = v_max * dec.speed_scale;
    int dyn_look = std::max(1, (int)std::round(lookahead_cells * dec.lookahead_scale));

    std::optional<std::vector<std::pair<int, int>>> path;

    // Hiding logic
    if (dec.use_hiding_obstacle) {
        const Obstacle* o = best_hiding_obstacle(*me, *enemy, obstacles);
        if (o) {
            auto hp = hiding_point_behind_obstacle(*enemy, *o, 35.0);
            res.hiding_point = hp;

            int sy = (int)(me->y / cell_px);
            int sx = (int)(me->x / cell_px);
            int gy = (int)(hp.second / cell_px);
            int gx = (int)(hp.first / cell_px);

            path = planner_->plan(grid, { sy, sx }, { gy, gx });
        }
    }

    // Flee logic or fallback
    if (!path || path->size() < 2 || dec.tactic == "FLEE") {
        int gh = grid.size();
        int gw = grid[0].size();

        std::mt19937 rng(0);
        std::uniform_int_distribution<int> dy(0, gh - 1);
        std::uniform_int_distribution<int> dx(0, gw - 1);

        double best_score = -1e9;

        int sy = (int)(me->y / cell_px);
        int sx = (int)(me->x / cell_px);

        for (int i = 0; i < samples; i++) {
            int gy = dy(rng);
            int gx = dx(rng);

            if (grid[gy][gx] == 1) continue;

            double px = (gx + 0.5) * cell_px;
            double py = (gy + 0.5) * cell_px;

            double score = std::hypot(px - enemy->x, py - enemy->y)
                - 0.15 * std::hypot(px - me->x, py - me->y);

            if (dec.tactic == "HIDE" || dec.tactic == "ORBIT_HIDE") {
                int block = 0;
                for (auto& o : obstacles) {
                    double d_me = std::hypot(px - o.cx, py - o.cy);
                    double d_en = std::hypot(enemy->x - o.cx, enemy->y - o.cy);
                    if (d_me < d_en) block++;
                }
                score += block * 80.0;
            }

            auto cand = planner_->plan(grid, { sy, sx }, { gy, gx });
            if (cand && score > best_score) {
                best_score = score;
                path = cand;
            }
        }

        if (!path)
            return res;
    }

    // Follow path
    int idx = std::min((int)path->size() - 1, dyn_look);
    auto [wy, wx] = (*path)[idx];

    double wx_pix = (wx + 0.5) * cell_px;
    double wy_pix = (wy + 0.5) * cell_px;

    res.cmd = follower_->follow(
        me->x, me->y, me->theta,
        { wx_pix, wy_pix },
        20.0,
        2.0,
        0.02,
        v_scaled
    );

    res.cmd.laser = false;
    res.path = path;

    return res;
}