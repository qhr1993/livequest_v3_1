
class PeripheralData:
    CO2 = 0
    NH3 = 0.0
    SO2 = 0.0
    H2S = 0.0
    time = 999999.99
    lat = 99.99
    lon = 99.99
    alt = 0.0
    hdop = 99.99
    numSat = 0
    fix = 0
    temp = 0.0
    humd = 0.0
    date = 0
    lux = 0.0
    current = 0.0
class PeripheralBytes:
    def __init__(self, p_data_in):
        import struct
        self.CO2 = struct.pack('<i',p_data_in.CO2)
        self.NH3 = struct.pack('<f',p_data_in.NH3)
        self.SO2 = struct.pack('<f',p_data_in.SO2)
        self.H2S = struct.pack('<f',p_data_in.H2S)
        self.time = struct.pack('<f',p_data_in.time)
        self.lat = struct.pack('<f',p_data_in.lat)
        self.lon = struct.pack('<f',p_data_in.lon)
        self.alt = struct.pack('<f',p_data_in.alt)
        self.temp = struct.pack('<f',p_data_in.temp)
        self.humd = struct.pack('<f',p_data_in.humd)
        self.hdop = struct.pack('<f',p_data_in.hdop)
        self.numSat = struct.pack('<i',p_data_in.numSat)
        self.fix = struct.pack('<i',p_data_in.fix)
        self.date = struct.pack('<i',p_data_in.date)
        self.lux = struct.pack('<f',p_data_in.lux)
        self.current = struct.pack('<f',p_data_in.current)
def _get_temperature_from_buffer(data):
    """This function reads the first two bytes of data and
    returns the temperature in C by using the following function:
    T = -46.85 + (175.72 * (ST/2^16))
    where ST is the value from the sensor
    """
    unadjusted = data[0]*256 + data[1]
    unadjusted &= 0xFFFC  # zero the status bits
    unadjusted /= 65536.0 # divide by 2^16
    unadjusted *= 175.72
    unadjusted -= 46.85
    return unadjusted

def _get_humidity_from_buffer(data):
    """This function reads the first two bytes of data and returns
    the relative humidity in percent by using the following function:
    RH = -6 + (125 * (SRH / 2 ^16))
    where SRH is the value read from the sensor
    """
    unadjusted = data[0]*256 + data[1]
    unadjusted &= 0xFFFC  # zero the status bits
    unadjusted /= 65536.0  # divide by 2^16
    unadjusted *= 125
    unadjusted -= 6.0
    return unadjusted

def _get_sht31_humidity_from_buffer(data):
    """This function reads the first two bytes of data and returns
    the relative humidity in percent by using the following function:
    RH = 100 * (SRH / (2 ^16 - 1))
    where SRH is the value read from the sensor
    """
    unadjusted = data[3]*256 + data[4]
    unadjusted /= 65535  # divide by 2^16
    unadjusted *= 100
    return unadjusted

def _get_sht31_temp_from_buffer(data):
    """This function reads the first two bytes of data and returns
    the relative humidity in percent by using the following function:
    RH = 100 * (SRH / (2 ^16 - 1))
    where SRH is the value read from the sensor
    """
    unadjusted = data[0]*256 + data[1]
    unadjusted /= 65535  # divide by 2^16
    unadjusted *= 175
    unadjusted -= 45
    return unadjusted


def peripheral_query(is_init,p_out_ctrla,p_out_ctrlb):
    import pycom
    import time
    import socket
    import binascii
    import struct
    import gc
    import sys
    import os

    from machine import UART
    from machine import ADC
    from machine import I2C
    from machine import SPI
    from machine import Pin
    from tsl2591 import TSL2591

    sht31 = 1

    bias_nh3 = 595
    bias_so2 = 596
    bias_h2s = 597

    di_nh3 = 12.24
    di_so2 = 12.60
    di_h2s = 38.00

    i0_nh3 = -0.36
    i0_so2 = 0.00
    i0_h2s = -0.08

    vref = 1094

    p_data = PeripheralData()
    print('[2]===================')
    iter = 0
    buff_h2s = 0

    buff_nh3 = 0

    buff_so2 = 0

    adc0 = ADC(id=0)

    while iter<20:
        #adc0 = ADC(id=0)
        apin0 = adc0.channel(pin='P13')
        adc0.vref(vref)
        buff_nh3 += apin0.voltage()
        #adc0.deinit()
        time.sleep(0.001)
        #adc1 = ADC(id=0)
        apin1 = adc0.channel(pin='P14')
        adc0.vref(vref)
        buff_so2 += apin1.voltage()
        #adc1.deinit()
        time.sleep(0.001)
        #adc2 = ADC(id=0)
        apin2 = adc0.channel(pin='P15')
        adc0.vref(vref)
        buff_h2s += apin2.voltage()
        #adc2.deinit()
        time.sleep(0.001)
        iter += 1

    buff_nh3 /= 20
    buff_so2 /= 20
    buff_h2s /= 20

    #adc0_str = '%.2f' % (round((apin0()*1100/4096-577)/3.83,1))
    buff_nh3 = round((buff_nh3 - bias_nh3 - i0_nh3*624) * 50/(di_nh3*624),1)
    adc0_str = '%.1f' % ((buff_nh3))
    p_data.NH3 = (buff_nh3)
    #p_data.NH3 = (round((apin0()*1100/4096-577)/3.83,1))
    print('[2]NH3: '+adc0_str)

    buff_so2 = round((buff_so2 - bias_so2)/((di_so2 / 20 - i0_so2)*47*0.624),1)
    adc1_str = '%.1f' % ((buff_so2))
    #p_data.SO2 = apin1()*1100/4096
    p_data.SO2 = ((buff_so2))
    print('[2]SO2: '+adc1_str)

    buff_h2s = round((buff_h2s - bias_h2s)/((di_h2s / 50 - i0_h2s)*47*0.624),1)
    adc2_str = '%.1f' % ((buff_h2s))
    #p_data.H2S = apin2()*1100/4096
    p_data.H2S = ((buff_h2s))
    print('[2]H2S: '+adc2_str)
    adc0.deinit()
    time.sleep(0.01)


    adc3 = ADC(id=0)             # create an ADC object
    apin3 = adc3.channel(pin='P16')   # create an analog pin on P16
    adc3.vref(vref)
    adc3_str = '%.2f' % (apin3()*220/4096)
    p_data.current = apin3()*220/4096
    print('[2]Current@5V: '+adc3_str+'mA')
    adc3.deinit()

    p_out_ctrla.value(0)
    p_out_ctrlb.value(1)

    uart_mul = UART(1, baudrate=9600,pins=('P3','P4'))
    time.sleep(0.1)

    co2_set = is_init
    while co2_set==0:
        uart_mul.write('K 2\r\n');
        time.sleep(0.05)
        dumm = uart_mul.readline()
        if dumm == bytes([]):
            print('[2]CO2 sensor no respose!')
            break
        else:
            dumm_str = dumm.decode('utf-8')
            if dumm_str == ' K 00002\r\n':
                print('[2]CO2 sensor polling set successfully...')
                time.sleep(0.05)
                co2_set = 1;
            else:
                print('[2]CO2 sensor polling resetting...')
                time.sleep(0.05)
            uart_mul.write('M 00006\r\n')
            time.sleep(0.05)
            dumm_str = uart_mul.readall().decode('utf-8')
            if dumm_str == ' M 00006\r\n':
                print('[2]CO2 sensor key set successfully...')
            time.sleep(0.05)
    time.sleep(0.05);

    number = 0
    while number!=18:
        dummy = uart_mul.readall()
        uart_mul.write('Q\r\n')
        time.sleep(0.1)
        number = uart_mul.any()

    data_str = uart_mul.readall().decode("utf-8")
    print('[2]CO2_Filtered: '+data_str[4:8])
    print('[2]CO2_Instant: '+data_str[12:16])
    p_data.CO2 = int(data_str[4:8])*100-1600
    print('[2]CO2: '+('%d' % (p_data.CO2)))

    i2c = I2C(0,I2C.MASTER)
    i2c.init(I2C.MASTER, baudrate=100000,pins = ('P9','P10'))
    time.sleep(0.05)
    #print(i2c.scan())

    if sht31==0:
        i2c.writeto(0x40, bytes([0xF3]))
        time.sleep(0.1)
        temperature_data = i2c.readfrom(0x40,3)
        #print(temperature_data)
        time.sleep(0.1)
        temperature_value = _get_temperature_from_buffer(temperature_data)
        p_data.temp = temperature_value
        time.sleep(0.1)
        i2c.writeto(0x40, bytes([0xF5]))
        time.sleep(0.05)
        humidity_data = i2c.readfrom(0x40,3)
        humidity_value = _get_humidity_from_buffer(humidity_data)
        p_data.humd = humidity_value

        humidity_value_str = '%.2f' % humidity_value
        temperature_value_str = '%.2f' % temperature_value
        print('[2]Humidity: '+humidity_value_str)
        print('[2]Temperature: '+temperature_value_str)
    else:
        i2c.writeto(0x44, bytes([0x24,0x00]))
        time.sleep(0.02)
        combined_data = i2c.readfrom(0x44,6)

        temperature_value = _get_sht31_temp_from_buffer(combined_data)
        p_data.temp = temperature_value

        humidity_value = _get_sht31_humidity_from_buffer(combined_data)
        p_data.humd = humidity_value

        humidity_value_str = '%.2f' % humidity_value
        temperature_value_str = '%.2f' % temperature_value
        print('[2]Humidity (SHT31): '+humidity_value_str)
        print('[2]Temperature (SHT31): '+temperature_value_str)

    lux_sensor = TSL2591(i2c)
    lux_flt = lux_sensor.lux
    p_data.lux = lux_flt
    print('[2]Light: ' + '%.2f' % lux_flt + 'lux')

    uart_mul = UART(1, baudrate=9600,pins=('P3','P4'))
    p_out_ctrla.value(1)
    p_out_ctrlb.value(0)
    time.sleep(0.1)
    ggflag = 0
    ddflag = 0
    while ggflag==0 or ddflag==0:
        while uart_mul.any()==0:
            time.sleep(0.1)
        nmealine_bytes = uart_mul.readall()
        #print(nmealine_bytes)
        time.sleep(0.1)
        nmealine_all = nmealine_bytes.decode('utf-8')
        nmealine_all_split = nmealine_all.split('\r\n')
        for nmealine in nmealine_all_split:
            if (nmealine[0:6]=='$GNGGA') and (len(nmealine.split(','))>=15) and (ggflag==0):
                nmea_fields = nmealine.split(',')
                #print(nmea_fields)
                print('[2]Time: ' + nmea_fields[1])
                if nmea_fields[1]!='':
                    p_data.time = float(nmea_fields[1])
                print('[2]Lat: ' + nmea_fields[2] + nmea_fields[3])
                if nmea_fields[2]!='':
                    p_data.lat = float(nmea_fields[2])
                if nmea_fields[3]=='S':
                    p_data.lat *= -1
                print('[2]Lon: ' + nmea_fields[4] + nmea_fields[5])
                if nmea_fields[4]!='':
                    p_data.Lon = float(nmea_fields[4])
                if nmea_fields[5]=='W':
                    p_data.lon *= -1
                print('[2]Fix: ' + nmea_fields[6])
                print('[2]#Sat: ' + nmea_fields[7])
                print('[2]HDOP: ' + nmea_fields[8])
                print('[2]Alt: ' + nmea_fields[9])
                if nmea_fields[9]!='':
                    p_data.alt = float(nmea_fields[9])
                ggflag = 1
            elif (nmealine[0:6]=='$GNRMC') and (len(nmealine.split(','))>=13) and (ddflag==0):
                nmea_fields = nmealine.split(',')
                print('[2]Date: ' + nmea_fields[9])
                if nmea_fields[9]!='':
                    p_data.date = int(nmea_fields[9])
                ddflag = 1
    dummy = uart_mul.readall()
    p_data_bytes = PeripheralBytes(p_data)
    print('[2]===================')
    time.sleep(0.01)
    gc.collect()
    print(gc.mem_free())
    return p_data_bytes
if __name__ == "__main__":
    import os
    p_out_ctrla = Pin('P21', mode=Pin.OUT)
    p_out_ctrlb = Pin('P20', mode=Pin.OUT)
    peripheral_query(0,p_out_ctrla,p_out_ctrlb)
