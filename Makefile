# Day-to-day commands, all runnable from the repo root. `make` lists them.
#
# The sim is MuJoCo (sim/spar_sim), headless in the sim container; the robot
# containers run nothing by default, see docker/entrypoint.sh. `make
# ros2_container` only builds the image and starts the containers; it does
# not build the code, even on a fresh clone. Building and running the code
# are both on you:
#
#   make ros2_container    # build the image, start the container, drop you into a shell
#   (in that shell) colcon build --symlink-install   # first build
#   (in that shell) cd build/spar_ground && make   # NOT cmake .. && make;
#                    colcon's build dir caches its own source path, plain make is correct
#   (in that shell) ros2 launch spar_ground autonomy.launch.py
#                    logs go to logs/runNNN automatically (ROS_LOG_DIR is set
#                    once per container start, see docker/entrypoint.sh)
#
# Ctrl-C the launch and rerun it after every rebuild. `make shell` gets you
# back into a running container later without restarting anything. `make
# smoke` checks a stack that is already running.
#
# If you restart the sim, restart the containers after it: the sim owns
# /clock, and a fresh sim rewinds time, which clears TF buffers, puts Nav2's
# lifecycle servers to sleep ("Action server is inactive"), and rewinds the
# clock under PX4, whose lockstep only moves forward. Same order for the air
# track.
COMPOSE   := docker compose -f docker/compose.yaml
CONTAINER := spar
LAUNCH    := spar_ground autonomy.launch.py
WORLD     ?= utility_depot_40_v2
SEED      ?=
BRIEF     ?=
WORLDGEN_ARGS ?=
ROS_ENV   := source /ws/scripts/env.sh

# The *_air targets below are two-line aliases onto this TRACK switch,
# which points the same recipes at the air container (PX4 + spar_air).
# The compose profile means only the air track ever builds the PX4 image;
# a bare `docker compose up` can't start it by accident. shut_down/clean
# always include the profile so they act on the whole project.
TRACK ?= ground
WORKDIRS := ground/build ground/install logs
SMOKE    := /ws/scripts/smoke_test.sh
ifeq ($(TRACK),air)
  COMPOSE   := $(COMPOSE) --profile air
  CONTAINER := spar-air
  LAUNCH    := spar_air air.launch.py
  WORKDIRS  += air/build air/install logs/air
  SMOKE     := /ws/scripts/smoke_test_air.sh
endif
COMPOSE_ALL := docker compose -f docker/compose.yaml --profile air

.PHONY: help worldgen ros2_container ros2_container_air clean shut_down start_sim stop_sim view inspect shell shell_air tail tail_air smoke smoke_air rviz

help:           ## list commands
	@grep -E '^[a-z0-9_-]+: .*##' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  make %-10s %s\n", $$1, $$2}'

worldgen: export SPAR_WORLDGEN_SEED := $(SEED)
worldgen: export SPAR_WORLDGEN_BRIEF := $(BRIEF)
worldgen:       ## generate, export, and validate one world (set WORLD; optionally SEED and BRIEF)
	uv run python -m worldgen $(WORLD) $(WORLDGEN_ARGS)

ros2_container: ## build + start the container (nothing built or launched yet) and drop you into a shell
	@mkdir -p $(WORKDIRS)
	$(COMPOSE) up --build -d
	docker exec -it $(CONTAINER) bash -lc '$(ROS_ENV) && exec bash'

ros2_container_air: ## same, for the air track (first build compiles PX4, takes a while)
	$(MAKE) ros2_container TRACK=air

shut_down:      ## stop and remove the containers
	$(COMPOSE_ALL) down

clean: shut_down ## shut down, then remove both tracks' build trees (forces a full rebuild on the next `make ros2_container`)
	rm -rf ground/build ground/install air/build air/install

start_sim:      ## start the headless sim (its container comes up if needed; make stop_sim ends it)
	@mkdir -p logs
	@$(COMPOSE) up --build -d sim
	@docker exec spar-sim pkill -f spar_sim.sim 2>/dev/null; true
	docker exec -d spar-sim bash -lc '$(ROS_ENV) && cd /ws/sim && \
	  MUJOCO_GL=egl WORLD=$(WORLD) python3 -m spar_sim.sim > /ws/logs/sim.log 2>&1'
	@echo "sim starting (logs/sim.log)"

stop_sim:       ## stop the sim
	@docker exec spar-sim pkill -f spar_sim.sim && echo "stopped" || echo "not running"

view:           ## native viewer window attached to the running sim
	@command -v uv >/dev/null || { \
	  echo "uv is required; follow docs/install.md"; exit 1; \
	}
	@# macOS's launch_passive needs mjpython; elsewhere plain python works.
	@if [ "$$(uname -s)" = "Darwin" ]; then \
	  uv run mjpython sim/viewer.py --world $(WORLD); \
	else \
	  uv run python sim/viewer.py --world $(WORLD); \
	fi

inspect:        ## open one world directly in MuJoCo, without a running sim
	@command -v uv >/dev/null || { \
	  echo "uv is required; follow docs/install.md"; exit 1; \
	}
	@test -f sim/worlds/$(WORLD).xml || { \
	  echo "world '$(WORLD)' does not exist; generate it first"; exit 1; \
	}
	@if [ "$$(uname -s)" = "Darwin" ]; then \
	  uv run mjpython sim/inspect_world.py --world $(WORLD); \
	else \
	  uv run python sim/inspect_world.py --world $(WORLD); \
	fi

shell:          ## a shell inside the container, ROS already sourced
	docker exec -it $(CONTAINER) bash -lc '$(ROS_ENV) && exec bash'

shell_air:      ## a shell inside the air container
	$(MAKE) shell TRACK=air

tail:           ## echo /rosout live, every node's log messages merged (fails if the track's launch isn't running)
	@docker exec $(CONTAINER) pgrep -f 'ros2 launch $(LAUNCH)' >/dev/null \
	  || { echo "$(LAUNCH) isn't running"; exit 1; }
	docker exec -it $(CONTAINER) bash -lc '$(ROS_ENV) && ros2 topic echo /rosout'

tail_air:       ## echo the air track's /rosout live
	$(MAKE) tail TRACK=air

rviz:           ## rviz2 in a browser (macOS can't pop an X window out of the container); Ctrl-C to stop
	@open "http://localhost:6080/vnc.html?autoconnect=true&resize=remote"
	docker exec -it $(CONTAINER) /ws/scripts/rviz.sh

smoke:          ## end-to-end test of the whole behavior arc (~4 min, ends in PASS)
	docker exec $(CONTAINER) bash -lc '$(ROS_ENV) && $(SMOKE)'

smoke_air:      ## end-to-end test of the air track (~4-6 min, ends in PASS)
	$(MAKE) smoke TRACK=air
