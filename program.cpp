#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <conio.h>
#include <windows.h>
#include <cstdio>

using namespace std;

#include "image_transfer.h"
#include "vision.h"
#include "timer.h"

#include "MarkerDetector.h"
#include "Tracking.h"
#include "Overlay.h"
#include "Obstacles.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

int main()
{
    double t_duration, t_sample, t_position;
    int width, height;
    image rgb1; // declare some image structures

    cout << "\npress space key to continue";
    pause();

    activate_vision();

    // ------------------------------------------------------------
    // Load video
    // ------------------------------------------------------------
    width = 720;
    height = 1280;

    //activate_camera(cam_number, width, height);	// activate camera
    open_video_input("robot_test2.mp4", t_duration, width, height);

    cout << "\nwidth = " << width;
    cout << "\nheight = " << height;
    cout << "\nduration (s) = " << t_duration;

    open_video_output("robot_output1.mp4");

    t_position = 0;
    position_video_input(t_position);

    rgb1.type = RGB_IMAGE;
    rgb1.width = width;
    rgb1.height = height;

    // allocate dynamic memory for the image
    // one big 1D dynamic array
    allocate_image(rgb1);

    // ------------------------------------------------------------
    // Vision modules
    // ------------------------------------------------------------
    MarkerDetector detector;
    Tracker tracker;
    Obstacles obst; // <- added

    vector<RobotTrack> tracks;
    double max_match_dist_px = 60.0;
    int max_misses = 8;

    double t0 = high_resolution_time();

    cout << "Press X to stop." << endl;

    // ------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------
    while (true)
    {
        // Stop if user presses X
        if (KEY('X'))
            break;

        // get rgb image from input video file
        read_video_input(rgb1, t_sample);
        
        // stop reading when there are no more samples in the video input file
        // -- in that case t_sample is set to zero
        if (t_sample < 1.0e-3)
        {
            cout << "End of video." << endl;
            break;
        }

        double now = high_resolution_time() - t0;

        // --------------------------------------------------------
        // Marker detection
        // --------------------------------------------------------
        vector<Blob> front_blobs, rear_blobs;
        detector.detect_markers(rgb1, front_blobs, rear_blobs);

        // Combine front and rear blobs for obstacle detection
        std::vector<Blob> robot_blobs = front_blobs;
        robot_blobs.insert(robot_blobs.end(), rear_blobs.begin(), rear_blobs.end());

        // --------------------------------------------------------
        // Pair markers + update tracks
        // --------------------------------------------------------
        auto dets = tracker.pairMarkers(front_blobs, rear_blobs, std::nullopt, 0.55, 1200.0);
        tracks = tracker.updateTracks(tracks, dets, now, max_match_dist_px, max_misses);

        // --------------------------------------------------------
        // Overlays: blobs, pairs, tracks
        // --------------------------------------------------------

        // Draw front blobs (blue)
        for (auto& b : front_blobs)
            draw_circle_rgb(rgb1, (int)b.x, (int)b.y, 8, 0, 0, 255);

        // Draw rear blobs (red)
        for (auto& b : rear_blobs)
            draw_circle_rgb(rgb1, (int)b.x, (int)b.y, 8, 255, 0, 0);

        // Draw paired markers (green line)
        for (auto& d : dets)
        {
            draw_line_rgb(rgb1,
                (int)d.front.x, (int)d.front.y,
                (int)d.rear.x, (int)d.rear.y,
                0, 255, 0);
        }

        // Draw tracks (ID + heading arrow)
        for (auto& tr : tracks)
        {
            // Robot center
            draw_circle_rgb(rgb1, (int)tr.x, (int)tr.y, 10, 255, 255, 0);

            // Heading arrow
            double hx = tr.x + 30 * cos(tr.theta);
            double hy = tr.y + 30 * sin(tr.theta);
            draw_line_rgb(rgb1, (int)tr.x, (int)tr.y, (int)hx, (int)hy, 255, 255, 0);

            // ID label (using your custom draw_text_rgb)
            char buf[16];
            sprintf(buf, "%d", tr.id);
            draw_text_rgb(rgb1, (int)tr.x + 12, (int)tr.y - 12, buf, 255, 255, 0);
        }

        // --------------------------------------------------------
        // Obstacle detection (ignore marker HSV ranges so markers are not labeled as obstacles)
        // --------------------------------------------------------
        auto obstacles = obst.detect(rgb1, robot_blobs, detector.blue_range, detector.red_range, 500);

        // Draw obstacles
        for (auto &o : obstacles) {
            // use draw_obstacle_overlay if available; fallback to draw_rect_rgb if not
            draw_obstacle_overlay(rgb1, o, 255, 0, 0);
        }

        // --------------------------------------------------------
        // Debug print
        // --------------------------------------------------------
        static int frame = 0;
        if (frame++ % 10 == 0)
        {
            cout << "Tracks: " << tracks.size() << "  Obstacles: " << obstacles.size() << endl;
            for (auto& t : tracks)
            {
                cout << "  ID " << t.id
                    << "  x=" << t.x
                    << "  y=" << t.y
                    << "  th=" << t.theta << endl;
            }
            for (size_t i = 0; i < obstacles.size(); ++i) {
                cout << "  Obs " << i << " cx=" << obstacles[i].cx << " cy=" << obstacles[i].cy << " area=" << obstacles[i].area << endl;
            }
        }

        // --------------------------------------------------------
        // Display frame
        // --------------------------------------------------------
        view_rgb_image(rgb1);

        // --------------------------------------------------------
        // Save to output video
        // --------------------------------------------------------
        write_video_output(rgb1, now);

    }

    // save the last image processed to a bmp file
    save_rgb_image("output1.bmp", rgb1);

    // ------------------------------------------------------------
    // Cleanup free the image memory before the program completes
    // ------------------------------------------------------------
    free_image(rgb1);
    close_video_input();
    close_video_output();
    deactivate_vision();

    cout << "Done." << endl;
    return 0;
}