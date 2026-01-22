preset := "debug"
# hello
executable := "build/minimal_mapping_tool"

SUCCESS := "✅"
NOTICE  := "ℹ️"
FAILURE := "❌"
BUILD   := "🔨"
RUN     := "▶️"
LINK    := "🔗"


default: run
init: configure setup

configure: 
    @if [ -z "$$ROS_DISTRO" ]; then echo {{FAILURE}} "Error: ROS2 is not sourced."; fi
    @echo {{NOTICE}} " Configuring CMake with preset {{preset}}..."
    @cmake --build --preset {{preset}}

build: 
    @echo {{BUILD}} " Building project..."
    @cmake --build --preset {{preset}}

run: build 
    @echo {{RUN}} " Running Application..."
    @./{{executable}}

setup:
    @echo {{LINK}} " Linking compile_commands.json..."
    @ln -sf build/compile_commands.json .

clean:
    @rm -rf build compile_commands.json
    @echo {{SUCCESS}} " Cleaned."
        

