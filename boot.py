import uos
from machine import UART
os.dupterm(None)
uart = UART(0, baudrate=115200,pins=('P1','P0'))
uart.init(115200,bits=8,parity=None,stop=1)
uos.dupterm(uart)
print('**REPL Enabled on UART0**')
