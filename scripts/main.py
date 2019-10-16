import sys
from agent.Swarm import Swarm


if __name__ == '__main__':
    if len(sys.argv) == 0:
        print("Wrong usage! Give number of drones to be used.")
        exit(0)

    drone_ids = [int(id) for id in sys.argv[1:]]
    swarm = Swarm(drone_ids)
    swarm.run_controller()
