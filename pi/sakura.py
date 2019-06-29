# coding:utf-8

import schedule
import time
import random

from sakuraio.hardware.rpi import SakuraIOSMBus

def job():
	print(time.time())
	sakuraio = SakuraIOSMBus()
	ch = random.randint(0,3)
	data = random.randint(100,1000)
	
	sakuraio.enqueue_tx(ch, data)
	sakuraio.send()


schedule.every(5).seconds.do(job)

while True:
	schedule.run_pending()
	time.sleep(1)
