import threading
import time

from PyQt5 import QtCore


class ServerThread(QtCore.QThread):
    data_downloaded = QtCore.pyqtSignal(object)

    def __init__(self, threadID, name):
        QtCore.QThread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.cont = 1


    def run(self):
        print "Starting " + self.name
        self.print_time(self.name, 5, self.cont)
        print "Exiting " + self.name

    def print_time(self, threadName, counter, delay):
        while counter:
            # if exitFlag:
            #     threadName.exit()
            time.sleep(delay)
            print "%s: %s" % (threadName, time.ctime(time.time()))
            # self.widget.status_label.setText("<font color=\"%s\">%s</font>" % ("red", "%s: %s" % (threadName, time.ctime(time.time()))))
            self.data_downloaded.emit(str(counter))
            counter -= 1


if __name__=="__main__":
    t = ServerThread(1 ,"la")
    t.start()