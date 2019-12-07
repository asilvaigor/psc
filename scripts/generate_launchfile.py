#!/usr/bin/env python3

import cflib.crtp
import sys, os

MAX_DRONES = 10
HEADER_NUMBER = "0xE7E7E7E7"
RETRIES = 10
LOCATION = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))

if __name__ == "__main__":
    drone_numbers = []
    retry = True
    # Parsing entry
    if len(sys.argv) == 1:
        print("No drone specified, trying all, please wait...")
        drone_numbers = [str("%02d" % i) for i in range(1, MAX_DRONES + 1)]
        retry = False
    else:
        drone_numbers = [str("%02d" % int(sys.argv[i])) for i in range(1, len(sys.argv))]

    # Transforming to hexadecimal and getting real addressees
    address_dict = {}
    cflib.crtp.init_drivers()
    for drone_number in drone_numbers:
        number = int(HEADER_NUMBER + drone_number, 0)
        address = cflib.crtp.scan_interfaces(number)
        for i in range(RETRIES):
            if not retry:
                break
            if len(address) <= 0:
                print("Unable to connect to drone %s retrying %02d/%d..." % (drone_number, i+1, RETRIES), flush=True)
            else:
                continue
            address = cflib.crtp.scan_interfaces(number)
        if len(address) <= 0:
            print("Unable to connect to drone %s" % drone_number, flush=True)
            continue
        address_dict[int(drone_number)] = address[0][0]
        # address_dict[int(drone_number)] = drone_number
        print("Drone %s at address: %s" % (drone_number, address[0][0]), flush=True)
        # print("Drone %s at address: %s" % (drone_number, drone_number), flush=True)

    print("Connecting to drones: ", end="")
    print(list(address_dict.keys()))
    print("Addressees: ", end="")
    print("{"+str(list(address_dict.values()))[1:-1]+"}")

    # Editing connected_drones.txt
    f = open(LOCATION+"/../launch/connected_drones.txt", "w")
    f.write(' '.join(str(e) for e  in address_dict.keys()))
    f.close()

    # Adding file header
    generated_file = open(LOCATION+"/templates_launch/header.xml", "r").read()

    # Adding drones header file
    for drone in address_dict:
        generated_file += "\n"+open(LOCATION + "/templates_launch/launch_drone_header.xml", "r").\
            read().replace("($i)", str(drone)).replace("($n)", address_dict[drone])

    # Adding first part of the body
    generated_file += "\n"+open(LOCATION + "/templates_launch/body_1.xml", "r").read()

    # Adding drones body file
    for drone in address_dict:
        generated_file += "\n" + open(LOCATION + "/templates_launch/launch_drone_body.xml", "r"). \
            read().replace("($i)", str(drone))

    generated_file += open(LOCATION + "/templates_launch/tail.xml", "r"). \
        read()

    # TODO check the connection with the swarm controller plugin
    # f = open(LOCATION+"/../launch/run_real.launch", "w")
    # f.write(generated_file)
    # f.close()
