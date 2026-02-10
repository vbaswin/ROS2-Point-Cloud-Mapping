# ============================================================
# Project Configuration
# ============================================================
# These are justfile variables — like Makefile macros but cleaner.
# `just` evaluates them at parse time, not runtime.

preset := "debug"
executable := "build/minimal_mapping_tool"
world_file := "depth_camera_world.sdf"
topic := "/camera/depth/points"

# Status indicators (emoji for readable terminal output)
SUCCESS := "✅"
NOTICE  := "ℹ️"
CAUTION := "⚠️"
FAILURE := "❌"

# Lifecycle icons
INIT    := "⚙️"
BUILD   := "🔨"
LINK    := "🔗"
RUN     := "▶️"
CLEAN   := "🧹"
TEST    := "🧪"
SIM     := "🌍"
ROBOT   := "🤖"
DEBUG   := "🔍"


# ============================================================
# DEFAULT: what happens when you just type `just`
# ============================================================
default: run


# ============================================================
# BUILD LIFECYCLE
# ============================================================
# These recipes handle the cmake configure → build → run cycle.
# `init` is a one-time setup; `build` and `run` are your daily drivers.

# First-time setup: configure CMake + symlink compile_commands.json
init: configure setup

# Configure CMake using presets (defined in CMakePresets.json)
# Presets standardize build configs across machines — no more
# "it works on my machine" because everyone uses the same flags.
configure:
    @if [ -z "$$ROS_DISTRO" ]; then echo "{{FAILURE}} Error: ROS2 is not sourced. Run: source /opt/ros/humble/setup.bash"; exit 1; fi
    @echo "{{INIT}} Configuring CMake with preset '{{preset}}'..."
    @cmake --preset {{preset}}

# Incremental build (only recompiles changed files)
# Ninja (our generator) tracks file dependencies automatically.
build:
    @echo "{{BUILD}} Building project..."
    @cmake --build --preset {{preset}}

# Build + Run the mapping app
# This is YOUR APP — it subscribes to the point cloud topic
# and opens the Qt/VTK window. It does NOT produce data.
# You need a data source running in another terminal (see below).
run: build
    @echo "{{RUN}} Running Application..."
    @echo "{{NOTICE}} Make sure a data source is running: just gazebo  OR  just test-pub"
    @./{{executable}}

# Symlink compile_commands.json to project root for IDE integration
# (clangd, ccls, etc. need this file to understand your includes)
setup:
    @echo "{{LINK}} Linking compile_commands.json..."
    @ln -sf build/compile_commands.json .

# Nuke build artifacts and start fresh
clean:
    @rm -rf build compile_commands.json
    @echo "{{SUCCESS}} Cleaned."


# ============================================================
# DATA SOURCES (run in a SEPARATE terminal)
# ============================================================
# Your app CONSUMES point clouds — it doesn't create them.
# You need one of these running before `just run` will show anything.
#
# Concept: ROS2 Pub/Sub (Observer Pattern)
#   Publisher (Gazebo or test node) → Topic → Subscriber (your app)
#   They're decoupled. Neither knows the other exists.

# Option A: Launch Gazebo with the depth camera robot world
# This gives you a full 3D simulation with a movable robot.
# The depth camera sensor publishes PointCloud2 to /camera/depth/points.
#
# After this starts, use `just teleop` in ANOTHER terminal to drive the robot.
gazebo:
    @if [ -z "$$ROS_DISTRO" ]; then echo "{{FAILURE}} Error: ROS2 not sourced."; exit 1; fi
    @echo "{{SIM}} Launching Gazebo with depth camera world..."
    @echo "{{NOTICE}} Drive the robot with: just teleop  (in another terminal)"
    @gazebo --verbose {{world_file}}

# Option B: Lightweight test publisher (no Gazebo needed)
# Publishes a rotating cube of colored points at 10 Hz.
# Great for testing your app's pipeline without GPU-heavy Gazebo.
#
# Concept: PointCloud2 message format
#   Each point is 16 bytes: [x(f32), y(f32), z(f32), rgb(f32)]
#   The rgb is packed as a single float using bit-shifting:
#   (r << 16) | (g << 8) | b  → reinterpret as float
test-pub:
    @if [ -z "$$ROS_DISTRO" ]; then echo "{{FAILURE}} Error: ROS2 not sourced."; exit 1; fi
    @echo "{{ROBOT}} Publishing test point cloud to {{topic}}..."
    @python3 test_pointcloud_publisher.py


# ============================================================
# ROBOT CONTROL (run in a THIRD terminal, only with Gazebo)
# ============================================================
# The diff_drive plugin in depth_camera_world.sdf listens on
# /camera_robot/cmd_vel for Twist messages (linear + angular velocity).
#
# teleop_twist_keyboard translates your WASD/arrow keys into
# Twist messages. This is how you "drive" the robot.
#
# Concept: Why teleop matters for mapping
#   ICP aligns DIFFERENT viewpoints. If the robot doesn't move,
#   every frame is identical — you're not mapping, just re-registering.
#   You need the camera to see the scene from different angles.

# Drive the Gazebo robot with keyboard (WASD keys)
# Requires: ros-humble-teleop-twist-keyboard package
teleop:
    @if [ -z "$$ROS_DISTRO" ]; then echo "{{FAILURE}} Error: ROS2 not sourced."; exit 1; fi
    @echo "{{ROBOT}} Launching keyboard teleop..."
    @echo "{{NOTICE}} Use WASD keys to drive. Press 'q' to quit."
    @ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/camera_robot/cmd_vel


# ============================================================
# DEBUGGING & INTROSPECTION
# ============================================================
# These recipes help you verify data is flowing through the
# ROS2 graph. Use them when "nothing is showing up."
#
# Concept: ROS2 CLI introspection tools
#   ros2 topic list    → What topics exist?
#   ros2 topic hz      → How fast is data arriving?
#   ros2 topic echo    → What does the actual data look like?
#   ros2 node list     → What nodes are running?

# List all active ROS2 topics (verify your data source is publishing)
topics:
    @echo "{{DEBUG}} Active ROS2 topics:"
    @ros2 topic list

# Check the publish rate of the point cloud topic
# Expected: ~10 Hz from Gazebo (update_rate=10 in SDF)
# Expected: ~10 Hz from test publisher (timer=0.1s)
hz:
    @echo "{{DEBUG}} Measuring publish rate on {{topic}}..."
    @ros2 topic hz {{topic}}

# Print raw point cloud message info (header, dimensions, point count)
# Ctrl+C to stop. Useful to verify data format.
echo-info:
    @echo "{{DEBUG}} Echoing topic info (Ctrl+C to stop)..."
    @ros2 topic info {{topic}} --verbose

# List all running ROS2 nodes
nodes:
    @echo "{{DEBUG}} Active ROS2 nodes:"
    @ros2 node list


# ============================================================
# TESTING
# ============================================================
# Unit tests for CloudMapper. These link ONLY against PCL + GTest,
# proving CloudMapper has zero Qt/ROS2 dependencies (SRP proof).
#
# Concept: Test isolation
#   The test executable is a completely separate binary.
#   It doesn't need ROS2 running, Qt display, or Gazebo.
#   This is what proper Dependency Inversion buys you.

# Run all tests via CTest
test: build
    @echo "{{TEST}} Running tests..."
    @cd build && ctest --output-on-failure

# Run a specific test by name (e.g., just test-one CloudMapperTests)
test-one name: build
    @echo "{{TEST}} Running test: {{name}}..."
    @cd build && ctest -R {{name}} --output-on-failure


# ============================================================
# QUICK REFERENCE (printed help)
# ============================================================
# Concept: Self-documenting build systems
#   A good justfile/Makefile should tell you how to use it.

help:
    @echo ""
    @echo "╔══════════════════════════════════════════════════════════════╗"
    @echo "║           ROS2 Point Cloud Mapping Tool                    ║"
    @echo "╠══════════════════════════════════════════════════════════════╣"
    @echo "║                                                            ║"
    @echo "║  FULL SIMULATION (3 terminals):                            ║"
    @echo "║    Terminal 1:  just gazebo     ← 3D world + depth camera  ║"
    @echo "║    Terminal 2:  just run        ← your mapping app         ║"
    @echo "║    Terminal 3:  just teleop     ← drive robot with WASD    ║"
    @echo "║                                                            ║"
    @echo "║  QUICK TEST (2 terminals):                                 ║"
    @echo "║    Terminal 1:  just test-pub   ← rotating cube publisher  ║"
    @echo "║    Terminal 2:  just run        ← your mapping app         ║"
    @echo "║                                                            ║"
    @echo "║  DEBUGGING:                                                ║"
    @echo "║    just topics     ← list active topics                    ║"
    @echo "║    just hz         ← check publish rate                    ║"
    @echo "║    just nodes      ← list active nodes                    ║"
    @echo "║                                                            ║"
    @echo "║  BUILD:                                                    ║"
    @echo "║    just init       ← first-time cmake configure           ║"
    @echo "║    just build      ← compile only                         ║"
    @echo "║    just clean      ← nuke build/                          ║"
    @echo "║    just test       ← run unit tests                       ║"
    @echo "║                                                            ║"
    @echo "╚══════════════════════════════════════════════════════════════╝"
    @echo ""
