import cflib.crtp
import sys

MAX_DRONES = 10
HEADER_NUMBER = "0xE7E7E7E7"

if __name__ == "__main__":
    drone_numbers = []
    # Parsing entry
    if len(sys.argv) == 1:
        drone_numbers = list(range(1, MAX_DRONES + 1))
    else:
        drone_numbers = [str("%02d" % int(sys.argv[i])) for i in range(1, len(sys.argv))]

    # Transforming to hexadecimal and getting real addressees
    address_dict = {}
    cflib.crtp.init_drivers()
    for drone_number in drone_numbers:
        number = int(HEADER_NUMBER+drone_number, 0)
        address = cflib.crtp.scan_interfaces(number)[0][0]
        address_dict[int(drone_number)] = address
        print("Drone %s at address: %s" % (drone_number, address))

    print(address_dict)
