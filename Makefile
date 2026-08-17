# Ground-only SPAR development commands. Run `make` to list them.
COMPOSE := docker compose -f docker/compose.yaml
CONTAINER := spar
WORLD ?= utility_depot_40_v2
SEED ?=
BRIEF ?=
WORLDGEN_ARGS ?=
ROS_ENV := source /ws/scripts/env.sh

.PHONY: help worldgen dev down clean sim stop_sim view inspect shell tail rviz smoke

help: ## list commands
	@grep -E '^[a-z0-9_-]+: .*##' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  make %-10s %s\n", $$1, $$2}'

worldgen: export SPAR_WORLDGEN_SEED := $(SEED)
worldgen: export SPAR_WORLDGEN_BRIEF := $(BRIEF)
worldgen: ## generate, export, and validate one world
	uv run python -m worldgen $(WORLD) $(WORLDGEN_ARGS)

dev: ## build/start the ground development containers and open a ROS shell
	@mkdir -p ros/build ros/install logs
	$(COMPOSE) up --build -d
	docker exec -it $(CONTAINER) bash -lc '$(ROS_ENV) && exec bash'

down: ## stop and remove all SPAR containers
	$(COMPOSE) down

clean: down ## remove the consolidated ROS build and install trees
	rm -rf ros/build ros/install

sim: ## start the headless MuJoCo simulator for WORLD
	@mkdir -p logs
	@$(COMPOSE) up --build -d sim
	@docker exec spar-sim pkill -f spar_sim.sim 2>/dev/null; true
	docker exec -d spar-sim bash -lc '$(ROS_ENV) && cd /ws/sim && \
	  MUJOCO_GL=egl WORLD=$(WORLD) python3 -m spar_sim.sim > /ws/logs/sim.log 2>&1'
	@echo "sim starting (logs/sim.log)"

stop_sim: ## stop the simulator process
	@docker exec spar-sim pkill -f spar_sim.sim && echo "stopped" || echo "not running"

view: ## open the host viewer attached to the running simulator
	@command -v uv >/dev/null || { echo "uv is required; see docs/install.md"; exit 1; }
	@if [ "$$(uname -s)" = "Darwin" ]; then \
	  uv run mjpython sim/viewer.py --world $(WORLD); \
	else \
	  uv run python sim/viewer.py --world $(WORLD); \
	fi

inspect: ## open WORLD directly in MuJoCo
	@command -v uv >/dev/null || { echo "uv is required; see docs/install.md"; exit 1; }
	@test -f sim/worlds/$(WORLD).xml || { echo "world '$(WORLD)' does not exist"; exit 1; }
	@if [ "$$(uname -s)" = "Darwin" ]; then \
	  uv run mjpython sim/inspect_world.py --world $(WORLD); \
	else \
	  uv run python sim/inspect_world.py --world $(WORLD); \
	fi

shell: ## open a shell in the ground ROS container
	docker exec -it $(CONTAINER) bash -lc '$(ROS_ENV) && exec bash'

tail: ## echo the ground stack's /rosout stream
	@docker exec $(CONTAINER) pgrep -f 'ros2 launch spar navigation.launch.py' >/dev/null \
	  || { echo "navigation.launch.py is not running"; exit 1; }
	docker exec -it $(CONTAINER) bash -lc '$(ROS_ENV) && ros2 topic echo /rosout'

rviz: ## run RViz in the browser-backed container display
	@open "http://localhost:6080/vnc.html?autoconnect=true&resize=remote"
	docker exec -it $(CONTAINER) /ws/scripts/rviz.sh

smoke: ## verify the complete ground autonomy mission lifecycle
	docker exec $(CONTAINER) bash -lc '$(ROS_ENV) && /ws/scripts/smoke_test.sh $(WORLD)'
