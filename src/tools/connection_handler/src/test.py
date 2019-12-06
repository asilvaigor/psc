from subprocess import Popen, PIPE, STDOUT
import psutil
import ast

drones_ids = [2, 3]

if __name__ == "__main__":
    # process = Popen(["rosrun", "psc", "generate_launchfile.py", str(drone_id)], stdout=PIPE)
    # read = ""
    # while process.poll() is None:
    #     (output, err) = process.communicate()
    #     read += output
    # print(read)
    # radio_id = read[read.find('{')+2:read.find('}')-1]
    # if radio_id == "":
    #     print("No crazyflie found")
    # else : print(radio_id)
    #
    #
    # process = Popen(["roslaunch", "psc", "run_real.launch", "cf:="+str(drone_id), "radio_id:="+radio_id],
    #                     stdin=PIPE, stdout=PIPE)
    # process = Popen("roslaunch psc run_real.launch cf:=" + str(drone_id),
    #                 shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    # while process.poll() is None:
    #     # (output, err) = process.communicate()
    #     output = process.stdout.read()
    #     print(output)
    #     # print(err)
    # parent = psutil.Process(process.pid)
    #
    # s = ""
    # children = parent.children(recursive=True)
    # for child in children:
    #     s += str(child.pid) + " "
    # print(s+"ola")
    # while True:
    #     output = process.stdout.readline()
    #     if output == '' and process.poll() is not None:
    #         break
    #     if output:
    #         print output.strip()
    # rc = process.poll()
    # exit_code = process.wait()
    # Adding widget and text
    # for drone_id in drones_ids:
    #     item = QListWidgetItem("", type=drone_id)
    #     self.__widget.connected_list.addItem(item)
    #     # TODO: change connect to add and disconnect to remove
    #     # TODO: change to update in separate thread
    #     start_message = "Connecting..."
    #     label = QLabel(formatted_string(drone_id, start_message, "blue"))
    #     self.__widget.connected_list.setItemWidget(item, label)

    # Getting drones ids

    process = Popen(["rosrun", "psc", "generate_launchfile.py"] + list(map(str, drones_ids)), stdout=PIPE)
    read = ""
    while process.poll() is None:
        (output, err) = process.communicate()
        read += output
        print(read)
    connected_drones = ast.literal_eval(read[read.find('['):read.find(']') + 1])
    radio_ids = ast.literal_eval("[" + read[read.find('{') + 1:read.find('}')] + "]")
    # print(radio_ids)
    if len(connected_drones) == 0:
        message = "Unable to connect"
        # label.setText(formatted_string(drone_id, message, "red"))
    else: print(radio_ids)