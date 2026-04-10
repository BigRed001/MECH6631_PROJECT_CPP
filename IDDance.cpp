#include "IDDance.h"
#include <cmath>
#include <iostream>

IDDance::IDDance()
{
    state_ = INIT;
    state_start_time_ = 0.0;
    my_id_ = -1;
    done_ = false;

    flash_on_time_ = 0.35;   // seconds
    flash_off_time_ = 0.35;
    observe_time_ = 2.0;     // observe others
}

Command IDDance::currentCommand() const
{
    Command cmd{ 0.0, 0.0, false };

    switch (state_)
    {
    case FLASH_ON:
        cmd.left = 0.0;
        cmd.right = 0.0;
        cmd.laser = true;
        break;

    case FLASH_OFF:
        cmd.left = 0.0;
        cmd.right = 0.0;
        cmd.laser = false;
        break;

    case OBSERVE:
        cmd.left = 0.0;
        cmd.right = 0.0;
        cmd.laser = false;
        break;

    default:
        break;
    }

    return cmd;
}

int IDDance::run(
    double now,
    image& rgb,
    MarkerDetector& detector,
    Tracker& tracker)
{
    if (done_) return my_id_;

    // Detect markers
    std::vector<Blob> front, rear;
    detector.detect_markers(rgb, front, rear);

    // Pair into robot detections
    std::optional<double> expected_sep;
    auto dets = tracker.pairMarkers(front, rear, expected_sep, 0.55, 1200.0);

    // Update tracks
    tracks_ = tracker.updateTracks(
        tracks_,
        dets,
        now,
        80.0,
        10
    );

    // -------------------------
    // State machine
    // -------------------------

    switch (state_) {

    case INIT:
        state_start_time_ = now;
        state_ = FLASH_ON;
        break;

    case FLASH_ON:
        if (now - state_start_time_ > flash_on_time_) {
            state_start_time_ = now;
            state_ = FLASH_OFF;
        }
        break;

    case FLASH_OFF:
        if (now - state_start_time_ > flash_off_time_) {
            state_start_time_ = now;
            state_ = OBSERVE;
        }
        break;

    case OBSERVE:
        if (now - state_start_time_ > observe_time_) {

            // Determine which robot flashed
            // The robot whose FRONT marker disappeared during FLASH_OFF
            // and reappeared during FLASH_ON is "me".

            // Count detections in each phase
            int count_on = 0;
            int count_off = 0;

            for (auto& t : tracks_) {
                if (t.last_seen > state_start_time_ - observe_time_) {
                    // Rough heuristic: if robot was missing during OFF phase
                    // but present during ON phase, it's me.
                    if (t.last_seen > state_start_time_ - flash_on_time_)
                        count_on++;
                    else
                        count_off++;
                }
            }

            // Simple rule: if only one robot "blinked", that's me.
            if (count_on == 1 && count_off == 0) {
                // Assign ID based on track index
                if (!tracks_.empty()) {
                    my_id_ = tracks_[0].id;
                }
            }
            else {
                // Fallback: choose robot closest to center
                double best = 1e9;
                for (auto& t : tracks_) {
                    double d = std::hypot(t.x - 320.0, t.y - 240.0);
                    if (d < best) {
                        best = d;
                        my_id_ = t.id;
                    }
                }
            }

            done_ = true;
            state_ = FINISHED;
        }
        break;

    case FINISHED:
        return my_id_;
    }

    return -1;
}