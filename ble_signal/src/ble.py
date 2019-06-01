
from __future__ import print_function

from time import gmtime, strftime, sleep
from bluepy.btle import Scanner, DefaultDelegate, BTLEException
import sys
import numpy as np
all_rssi = []
class ScanDelegate(DefaultDelegate):
    def handleDiscovery(self, dev, isNewDev, isNewData):
        if dev.addr == "40:06:a0:94:f5:90":
		all_rssi.append(dev.rssi)
		#print("HM-10!!!!!!!!!!!!")
		#print(strftime("%Y-%m-%d %H:%M:%S", gmtime()), dev.addr, dev.rssi)
        sys.stdout.flush()

    def scan(self):
	scanner = Scanner().withDelegate(ScanDelegate())
	scanner.scan(2.0, passive=True)
	print(np.mean(all_rssi))

if __name__ == "__main__":
    ScanDelegate().scan()
