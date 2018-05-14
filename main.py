# entrance main function
def make_special_char(pac):
    rtn_packet = bytes([])
    for i in range(0,len(pac)):
        if bytes([pac[i]]) == b'\x7D':
            rtn_packet = rtn_packet + b'\x7D\x5D'
        elif bytes([pac[i]]) == b'\x7E':
            rtn_packet = rtn_packet + b'\x7D\x5E'
        elif bytes([pac[i]]) == b'\x21':
            rtn_packet = rtn_packet + b'\x7D\x51'
        else :
            rtn_packet = rtn_packet + bytes([pac[i]])
    return rtn_packet

def make_data_bytes(rdata,type):
    #type 0 => float
    #type 1 => integer
    if type == 0:
        rtn_bytes = bytes([rdata[3]]) + bytes([rdata[2]]) + bytes([rdata[1]]) + bytes([rdata[0]])
    elif type == 1:
        rtn_bytes = bytes([rdata[1]]) + bytes([rdata[0]]) + bytes([rdata[3]]) + bytes([rdata[2]])
    return rtn_bytes

from machine import UART
from machine import Pin
from machine import SPI
from machine import Timer
import machine
import time
import sys
import struct
import pycom
import peripheral_query
import os
import gc
import uio
import ujson

class Clock:
    def __init__(self):
        self.last_packet_number = 0
        self.__alarm = Timer.Alarm(self._seconds_handler,45,periodic=True)
    def _seconds_handler(self,alarm):
        global state
        global packet_number
        if state == 1:
            print("[3]Timer timeout when state 1")
            if packet_number != self.last_packet_number:
                state = 1
                print("[3]Timer remains state 1")
            else:
                state = 0
                print("[3]Timer resets state to 0")
            self.last_packet_number = packet_number
        gc.collect()
        #print(gc.mem_free())

with uio.open('/flash/configure.json', 'r', encoding = "utf-8") as handle:
    psd_json = ujson.load(handle)

p_in_mod = Pin('P12', mode=Pin.IN , pull = Pin.PULL_UP)

if p_in_mod() == 0:
    print('[1]MODE_1 selected. Entering sensor zero-point calibration mode...')
    calibration(1, psd_json["calibration"]["vref"])
    sys.exit()

time.sleep(0.5)

p_in_mod = Pin('P12', mode=Pin.IN , pull = Pin.PULL_UP)
p_out_mod = Pin('P11', mode=Pin.OUT)

p_out_mod.value(0)
time.sleep(0.5)

if p_in_mod() == 0:
    print('[1]MODE_0 selected. Entering ADC bias calibration mode...')
    calibration(0,psd_json["calibration"]["vref"])
    sys.exit()

print('[1]MODE_2 selected. Entering normal operation...')

uart_as62 = UART(1, baudrate=9600,pins=('P3','P4'))
time.sleep(0.05)
p_out_md1 = Pin('P22', mode=Pin.OUT)
p_out_md1.value(1)
p_out_md0 = Pin('P23', mode=Pin.OUT)
p_out_md0.value(1)
#p_in = Pin('P23', mode=Pin.IN, pull=Pin.PULL_UP)
p_out_ctrla = Pin('P21', mode=Pin.OUT)
p_out_ctrlb = Pin('P20', mode=Pin.OUT)
p_out_ctrla.value(0)
p_out_ctrlb.value(0)
time.sleep(0.5)

p_out_led = Pin('P2', mode=Pin.OUT)
p_out_led.value(1)

dev_addr = bytes([0x00,0x00,0x0F]) + bytes([psd_json["firmware"]["devaddr"]])
lora_addr = dev_addr[2:4]

cfg_r_str = 'NO'
while cfg_r_str[0:2]!='OK':
    uart_as62.readall()
    uart_as62.write(bytes([0xC2]) + lora_addr + bytes([0x1D,0x17,0xC0])) #write cfg bytes
    time.sleep(0.5)
    cfg_r_str = uart_as62.readall().decode('utf-8')
    print(cfg_r_str)
    time.sleep(0.5)

time.sleep(0.5)
p_out_md1.value(0)
p_out_md0.value(0)
time.sleep(0.5)
pdata_bytes = peripheral_query.peripheral_query(0,p_out_ctrla,p_out_ctrlb)
state = 0
# state = 0 unregistered
# state = 1 registered
gateway_addr = bytes([0x00,0x00,0x00,0x00])
header = bytes([0x7E]) + struct.pack(">H",psd_json["firmware"]["protocol"])
packet_number = 0
time_slot = 30
uart_as62 = UART(1, baudrate=9600,pins=('P3','P4'))
p_out_ctrla.value(0)
p_out_ctrlb.value(0)
print('Initialization Finished...')

clock = Clock()

while (1):
    rcvd_lines = None
    rcvd_lines = uart_as62.readall()
    time.sleep(0.01)
    if rcvd_lines == None:
        time.sleep(0.01)
        #print('rcvd nothing!')
    else :
        rcvd_split = rcvd_lines.split(b'\x21')
        for i in range(0,len(rcvd_split)-1):
            rcvd = rcvd_split[i]
            #print(rcvd)
            if state == 0 and len(rcvd)>=12:
                if (rcvd[12]==0x0D):
                    if (rcvd[11]==0x08): # broadcast
                        print ('[0]Received gateway broadcast...')
                        gateway_addr = rcvd[3:7] #fetch gateway addr
                        #header = rcvd[0:3] #fetch header
                        register_raw = header[1:3] + dev_addr + b'\x01'+rcvd[8:10]+b'\x00\x02\x0D\x00\x01'
                        crc = 0x00
                        for bit in register_raw:
                            crc = crc + bit
                            crc = crc & 0xFF # calculate checksum
                        register_pac = gateway_addr[2:4] + b'\x17\x7E' + make_special_char(register_raw) + bytes([crc]) + b'\x21'
                        duration = struct.unpack('>H',rcvd[13:15])[0]
                        slot = struct.unpack('>H',rcvd[15:17])[0]
                        rand_sleep = round(machine.rng()/int(0xFFFFFF)*(duration/slot))*slot/100+0.01
                        print('[0]CA Wait for ' + ('%.2f' % rand_sleep) + ' secs...')
                        time.sleep(rand_sleep)
                        uart_as62.write(register_pac)
                        print('[0]Responded gateway broadcast...')
                        time.sleep(0.01)
                    elif (rcvd[11]==0x02): #gateway register_success response
                        if (rcvd[13:15]==b'\x4F\x4B'):
                            print('[0]Received gateway response... Device registered!')
                            state = 1
                            p_out_led.value(0)
                            packet_number = 0
                    else :
                        time.sleep(0.01)
            elif state == 1 and len(rcvd)>=12:
                if (rcvd[12]==0x0E): #gateway polling
                    crc = 0x00
                    pdata_bytes = peripheral_query.peripheral_query(1,p_out_ctrla,p_out_ctrlb)
                    print('[1]Peripheral data collected...')
                    upload_raw = (header[1:3] + dev_addr + b'\x00' + struct.pack('>h',packet_number)
                                + b'\x00\x76\x09' + b'\x0D'
                                + b'\x00\x01\x00\x01\x00' + make_data_bytes(pdata_bytes.temp,1)
                                + b'\x00\x02\x00\x02\x00' + make_data_bytes(pdata_bytes.humd,1)
                                + b'\x00\x03\x00\x06\x01' + make_data_bytes(pdata_bytes.CO2,0)
                                + b'\x00\x04\x00\x15\x00' + make_data_bytes(pdata_bytes.NH3,1)
                                + b'\x00\x05\x00\x23\x00' + make_data_bytes(pdata_bytes.SO2,1)
                                + b'\x00\x06\x00\x14\x00' + make_data_bytes(pdata_bytes.H2S,1)
                                + b'\x00\x07\x00\xFA\x00' + make_data_bytes(pdata_bytes.lat,1)
                                + b'\x00\x08\x00\xFB\x00' + make_data_bytes(pdata_bytes.lon,1)
                                + b'\x00\x09\x00\xFC\x00' + make_data_bytes(pdata_bytes.alt,1)
                                + b'\x00\x0A\x00\xFD\x01' + make_data_bytes(pdata_bytes.date,0)
                                + b'\x00\x0B\x00\xFE\x00' + make_data_bytes(pdata_bytes.time,1)
                                + b'\x00\x0C\x00\x05\x00' + make_data_bytes(pdata_bytes.lux,1)
                                + b'\x00\x0D\x00\xF9\x00' + make_data_bytes(pdata_bytes.current,1))
                    for bit in upload_raw:
                        crc = crc + bit
                        crc = crc & 0xFF # calculate checksum
                    upload_pac = gateway_addr[2:4] + b'\x17\x7E' + make_special_char(upload_raw) + bytes([crc]) + b'\x21'
                    uart_as62 = UART(1, baudrate=9600,pins=('P3','P4'))
                    p_out_ctrla.value(0)
                    p_out_ctrlb.value(0)
                    uart_as62.write(upload_pac)
                    print('[1]Packet Sent')
                    p_out_led.value(1)
                    time.sleep(0.1)
                    p_out_led.value(0)
                    packet_number += 1
                    time.sleep(0.01)
                    gc.collect()
                    print(gc.mem_free())
