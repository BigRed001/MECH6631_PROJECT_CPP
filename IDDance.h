#pragma once
#include "image_transfer.h"
#include "vision.h"
#include "Types.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include <vector>

class IDDance {
public:
    IDDance();

    // Run ID dance for a few seconds until identity is determined.
    // Returns my_id (0 or 1 or 2...) or -1 if not found.
    int run(
        double now,
        image& rgb,
        MarkerDetector& detector,
        Tracker& tracker);

    // Whether ID dance is finished
    bool done() const { return done_; }

    // My assigned ID
    int my_id() const { return my_id_; }

    // NEW: Get the wheel + laser command for the current dance state
    Command currentCommand() const;

private:
    enum State {
        INIT,
        FLASH_ON,
        FLASH_OFF,
        OBSERVE,
        FINISHED
    };

    State state_;
    double state_start_time_;
    int my_id_;
    bool done_;

    // Flash pattern parameters
    double flash_on_time_;
    double flash_off_time_;
    double observe_time_;

    // Internal tracking
    std::vector<RobotTrack> tracks_;
};
