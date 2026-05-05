import csv
import math
import sys
from pathlib import Path
import matplotlib.pyplot as plt


# ============================================================
# ---------------------- CSV UTILITIES -----------------------
# ============================================================

def to_float(value):
    """Convert a CSV cell to float. Invalid or empty values become NaN."""
    try:
        if value is None:
            return math.nan
        value = str(value).strip()
        if value == "" or value.lower() in ("nan", "none", "null"):
            return math.nan
        return float(value)
    except ValueError:
        return math.nan


def read_csv_log(csv_path):
    """Read the diagnostics CSV into a dictionary: column_name -> list of floats."""
    data = {}

    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key, value in row.items():
                if key not in data:
                    data[key] = []
                data[key].append(to_float(value))

    return data


def get_col(data, name):
    """Return a column from the data dictionary, or an empty list if absent."""
    return data.get(name, [])


def relative_time(data):
    """Return time vector starting at zero."""
    t = get_col(data, "t")
    if not t:
        return []

    # Find first valid time value
    t0 = None
    for value in t:
        if not math.isnan(value):
            t0 = value
            break

    if t0 is None:
        return [math.nan for _ in t]

    return [(value - t0) if not math.isnan(value) else math.nan for value in t]


def has_any_valid(values):
    """Return True if at least one value is valid."""
    return any(not math.isnan(v) for v in values)


def safe_plot(x, y, label=None, linestyle=None):
    """Plot only if the vector exists and contains at least one valid value."""
    if not y or not has_any_valid(y):
        return False
    if len(x) != len(y):
        n = min(len(x), len(y))
        x = x[:n]
        y = y[:n]
    plt.plot(x, y, label=label, linestyle=linestyle)
    return True


def finish_plot(title, xlabel, ylabel, out_path, legend=True):
    """Apply common formatting and save the current figure."""
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True, alpha=0.3)
    if legend:
        plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=180)
    plt.close()


# ============================================================
# --------------------------- PLOTS ---------------------------
# ============================================================

def plot_pwm(data, out_prefix):
    """Plot left/right motor commands and laser command."""
    t = relative_time(data)
    out_path = f"{out_prefix}_pwm.png"

    plt.figure(figsize=(10, 5))
    any_plot = False
    any_plot |= safe_plot(t, get_col(data, "pwm_left"), label="PWM Left")
    any_plot |= safe_plot(t, get_col(data, "pwm_right"), label="PWM Right")
    any_plot |= safe_plot(t, get_col(data, "laser"), label="Laser", linestyle="--")

    if any_plot:
        finish_plot("PWM / Command History", "Time [s]", "Normalized command", out_path)
        return out_path

    plt.close()
    return None


def plot_geometry(data, out_prefix):
    """Plot combat geometry: distances, bearing, and obstacle counts."""
    t = relative_time(data)
    out_path = f"{out_prefix}_geometry.png"

    fig = plt.figure(figsize=(10, 8))

    ax1 = plt.subplot(3, 1, 1)
    plotted = False
    if get_col(data, "enemy_dist_px"):
        ax1.plot(t, get_col(data, "enemy_dist_px"), label="Enemy distance")
        plotted = True
    if get_col(data, "nearest_obs_dist_px"):
        ax1.plot(t, get_col(data, "nearest_obs_dist_px"), label="Nearest obstacle distance")
        plotted = True
    ax1.set_ylabel("Distance [px]")
    ax1.set_title("Combat Geometry vs Time")
    ax1.grid(True, alpha=0.3)
    if plotted:
        ax1.legend()

    ax2 = plt.subplot(3, 1, 2, sharex=ax1)
    if get_col(data, "enemy_bearing_deg"):
        ax2.plot(t, get_col(data, "enemy_bearing_deg"), label="Enemy bearing")
        ax2.legend()
    ax2.set_ylabel("Bearing [deg]")
    ax2.grid(True, alpha=0.3)

    ax3 = plt.subplot(3, 1, 3, sharex=ax1)
    plotted = False
    if get_col(data, "blocking_obs_count"):
        ax3.plot(t, get_col(data, "blocking_obs_count"), label="Blocking obstacles")
        plotted = True
    if get_col(data, "num_obstacles"):
        ax3.plot(t, get_col(data, "num_obstacles"), label="Total obstacles")
        plotted = True
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Count")
    ax3.grid(True, alpha=0.3)
    if plotted:
        ax3.legend()

    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    return out_path


def plot_strategy(data, out_prefix):
    """Plot tactical decisions and controller scaling values when available."""
    t = relative_time(data)
    out_path = f"{out_prefix}_strategy.png"

    fig = plt.figure(figsize=(10, 8))

    ax1 = plt.subplot(3, 1, 1)
    if get_col(data, "tactic_id"):
        ax1.step(t, get_col(data, "tactic_id"), where="post", label="Tactic ID")
        ax1.legend()
    elif get_col(data, "offense_mode"):
        ax1.step(t, get_col(data, "offense_mode"), where="post", label="Offense mode")
        ax1.legend()
    ax1.set_ylabel("Decision ID")
    ax1.set_title("Strategy / Tactical Decisions")
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot(3, 1, 2, sharex=ax1)
    plotted = False
    if get_col(data, "speed_scale"):
        ax2.plot(t, get_col(data, "speed_scale"), label="Speed scale")
        plotted = True
    if get_col(data, "lookahead_scale"):
        ax2.plot(t, get_col(data, "lookahead_scale"), label="Lookahead scale")
        plotted = True
    ax2.set_ylabel("Scale")
    ax2.grid(True, alpha=0.3)
    if plotted:
        ax2.legend()

    ax3 = plt.subplot(3, 1, 3, sharex=ax1)
    if get_col(data, "phase_id"):
        ax3.step(t, get_col(data, "phase_id"), where="post", label="Phase")
        ax3.legend()
    elif get_col(data, "my_id"):
        ax3.step(t, get_col(data, "my_id"), where="post", label="My robot ID")
        ax3.legend()
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("State")
    ax3.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    return out_path


def plot_performance(data, out_prefix):
    """Plot FPS and loop timing metrics."""
    t = relative_time(data)
    out_path = f"{out_prefix}_performance.png"

    fig = plt.figure(figsize=(10, 8))

    ax1 = plt.subplot(3, 1, 1)
    if get_col(data, "fps"):
        ax1.plot(t, get_col(data, "fps"), label="FPS")
        ax1.legend()
    ax1.set_ylabel("FPS")
    ax1.set_title("Runtime Performance")
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot(3, 1, 2, sharex=ax1)
    plotted = False
    for col, label in [
        ("detect_ms", "Detect ms"),
        ("obstacles_ms", "Obstacles ms"),
        ("astar_ms", "A* ms"),
        ("control_ms", "Control ms"),
        ("total_ms", "Total loop ms"),
    ]:
        if get_col(data, col):
            ax2.plot(t, get_col(data, col), label=label)
            plotted = True
    ax2.set_ylabel("Time [ms]")
    ax2.grid(True, alpha=0.3)
    if plotted:
        ax2.legend()

    ax3 = plt.subplot(3, 1, 3, sharex=ax1)
    plotted = False
    for col, label in [
        ("reaction_time_ms", "Reaction time ms"),
        ("loop_dt_ms", "Loop dt ms"),
    ]:
        if get_col(data, col):
            ax3.plot(t, get_col(data, col), label=label)
            plotted = True
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Time [ms]")
    ax3.grid(True, alpha=0.3)
    if plotted:
        ax3.legend()

    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    return out_path


def plot_xy(data, out_prefix):
    """Plot XY trajectories of our robot and the enemy when available."""
    out_path = f"{out_prefix}_xy.png"

    plt.figure(figsize=(8, 8))
    any_plot = False

    my_x = get_col(data, "my_x")
    my_y = get_col(data, "my_y")
    enemy_x = get_col(data, "enemy_x")
    enemy_y = get_col(data, "enemy_y")

    if my_x and my_y and has_any_valid(my_x) and has_any_valid(my_y):
        n = min(len(my_x), len(my_y))
        plt.plot(my_x[:n], my_y[:n], label="My robot trajectory")
        any_plot = True

    if enemy_x and enemy_y and has_any_valid(enemy_x) and has_any_valid(enemy_y):
        n = min(len(enemy_x), len(enemy_y))
        plt.plot(enemy_x[:n], enemy_y[:n], label="Enemy trajectory")
        any_plot = True

    if any_plot:
        plt.xlabel("X [px]")
        plt.ylabel("Y [px]")
        plt.title("XY Trajectories")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.gca().invert_yaxis()
        plt.tight_layout()
        plt.savefig(out_path, dpi=180)
        plt.close()
        return out_path

    plt.close()
    return None


# ============================================================
# ---------------------------- MAIN ---------------------------
# ============================================================

def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_diagnostics.py run_diagnostics.csv")
        return 1

    csv_path = Path(sys.argv[1])
    if not csv_path.exists():
        print(f"Error: file not found: {csv_path}")
        return 1

    out_prefix = csv_path.with_suffix("").name
    data = read_csv_log(csv_path)

    generated = []
    for func in [plot_pwm, plot_geometry, plot_strategy, plot_performance, plot_xy]:
        out = func(data, out_prefix)
        if out is not None:
            generated.append(out)

    print("Generated figures:")
    for path in generated:
        print(" -", path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
