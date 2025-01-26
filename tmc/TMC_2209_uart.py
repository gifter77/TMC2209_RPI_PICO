#from gpiozero import LED
#from bitstring import BitArray
import time
import sys
import binascii
import struct
import machine
from machine import UART, Pin
from rp2 import PIO, StateMachine, asm_pio

@asm_pio(
    autopush=True,
    push_thresh=8,
    in_shiftdir=PIO.SHIFT_RIGHT,
    fifo_join=PIO.JOIN_RX,
    set_init=PIO.IN_HIGH,
)
def uart_rx_mini():
    # fmt: off
    # Wait for start bit
    wait(0, pin, 0)
    # Preload bit counter, delay until eye of first data bit
    set(x, 7)                 [10]
    # Loop 8 times
    label("bitloop")
    # Sample data
    in_(pins, 1)
    # Each iteration is 8 cycles
    jmp(x_dec, "bitloop")     [6]
    # fmt: on

@asm_pio(
    in_shiftdir=PIO.SHIFT_RIGHT,
)
def uart_rx():
    # fmt: off
    label("start")
    # Stall until start bit is asserted
    wait(0, pin, 0)
    # Preload bit counter, then delay until halfway through
    # the first data bit (12 cycles incl wait, set).
    set(x, 7)                 [10]
    label("bitloop")
    # Shift data bit into ISR
    in_(pins, 1)
    # Loop 8 times, each loop iteration is 8 cycles
    jmp(x_dec, "bitloop")     [6]
    # Check stop bit (should be high)
    jmp(pin, "good_stop")
    # Either a framing error or a break. Set a sticky flag
    # and wait for line to return to idle state.
    #irq(block, 4)
    wait(1, pin, 0)
    # Don't push data if we didn't see good framing.
    jmp("start")
    # No delay before returning to start; a little slack is
    # important in case the TX clock is slightly too fast.
    label("good_stop")
    push(block)
    # fmt: on
    
@asm_pio(set_init=PIO.IN_HIGH, sideset_init=PIO.IN_HIGH, out_init=PIO.IN_HIGH, out_shiftdir=PIO.SHIFT_RIGHT)
def uart_tx():
    # fmt: off
    # Block with TX deasserted until data available
    pull()
    set(pindirs, 1)
    # Initialise bit counter, assert start bit for 8 cycles
    set(x, 7)  .side(0)       [7]
    # Shift out 8 data bits, 8 execution cycles per bit
    label("bitloop")
    out(pins, 1)              [6]
    jmp(x_dec, "bitloop")
    # Assert stop bit for 8 cycles total (incl 1 for pull())
    nop()      .side(1)       [6]
    set(pindirs, 0)
    set(pins, 1)
    # fmt: on
    
def handler(sm):
    print("break", time.ticks_ms(), end=" ")

#-----------------------------------------------------------------------
# TMC_UART
#
# this class is used to communicate with the TMC via UART
# it can be used to change the settings of the TMC.
# like the current or the microsteppingmode
#-----------------------------------------------------------------------
class TMC_UART:

    mtr_id=0
    ser = None
    rFrame  = [0x55, 0, 0, 0  ]
    wFrame  = [0x55, 0, 0, 0 , 0, 0, 0, 0 ]
    communication_pause = 0
    
#-----------------------------------------------------------------------
# constructor
#-----------------------------------------------------------------------
    def __init__(self, serialport, baudrate, txpin,mtr_id_arg):
        self.baudrate = baudrate
        #self.ser = UART(serialport, baudrate=115200, bits=8, parity=None, stop=1, tx=txpin, rx=None)
        self.mtr_id=mtr_id_arg
        #self.ser.timeout = 20000/baudrate            # adjust per baud and hardware. Sequential reads without some delay fail.
        self.communication_pause = 500/baudrate     # adjust per baud and hardware. Sequential reads without some delay fail.

        #self.ser.reset_output_buffer()
        #self.ser.reset_input_buffer()
        #txpin.init(mode=Pin.IN, pull=Pin.PULL_UP)
        #time.sleep(self.communication_pause)
        #txpin.toggle()
        
        self.pin = txpin
        
        self.tx = StateMachine(
            0, uart_tx, freq=8 * baudrate, set_base=txpin, sideset_base=txpin, out_base=txpin
        )
        self.tx.active(1)
        
        self.rx = StateMachine(
            1,
            uart_rx,
            freq=8 * baudrate,
            in_base=txpin,  # For WAIT, IN
            jmp_pin=txpin,  # For JMP
        )
        #self.rx.irq(handler)
        self.rx.active(1)
        
#-----------------------------------------------------------------------
# destructor
#-----------------------------------------------------------------------
    def __del__(self):
        pass
        #self.tx.active(0)
        #self.rx.active(0)
        #PIO(0).remove_program(uart_tx)
        #PIO(1).remove_program(uart_rx_mini)
        #self.pin.init(mode=Pin.OUT)

#-----------------------------------------------------------------------
# this function calculates the crc8 parity bit
#-----------------------------------------------------------------------
    def compute_crc8_atm(self, datagram, initial_value=0):
        crc = initial_value
        # Iterate bytes in data
        for byte in datagram:
            # Iterate bits in byte
            for _ in range(0, 8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                # Shift to next bit
                byte = byte >> 1
        return crc
    
#-----------------------------------------------------------------------
# reads the registry on the TMC with a given address.
# returns the binary value of that register
#-----------------------------------------------------------------------
    def read_reg(self, reg):
        
        rtn = ""
        #self.ser.reset_output_buffer()
        #self.ser.reset_input_buffer()
        
        self.rFrame[1] = self.mtr_id
        self.rFrame[2] = reg
        self.rFrame[3] = self.compute_crc8_atm(self.rFrame[:-1])

        #rt = self.ser.write(bytes(self.rFrame))
        #if rt != len(self.rFrame):
        #    print("TMC2209: Err in write {}".format(__), file=sys.stderr)
        #    return False
        #self.tx.active(1)
        for r in self.rFrame:
            self.tx.put(r)
        #while self.tx.tx_fifo() != 0:
        #    time.sleep(1/self.baudrate)
        #for i in range(4):
        #    print(hex(self.rx.get()))
        #self.rx.restart()
        #self.rx.active(1)
        #self.pin.high()
        #self.pin.init(mode=Pin.IN, pull=Pin.PULL_UP)
        #time.sleep(self.communication_pause)  # adjust per baud and hardware. Sequential reads without some delay fail.
        #if self.ser.any():
        #    rtn = self.ser.read()#read what it self
        rtn = bytes()
        for i in range(12):
            rtn += (self.rx.get() >> 24).to_bytes(1, 'big')
            #print(hex(self.rx.get() >> 24))
        #print(rtn.hex())
        #print((bytes(self.rFrame) + bytes.fromhex("05ff") + reg.to_bytes(1, 'big')).hex())
        if rtn[0:7] != bytes(self.rFrame) + bytes.fromhex("05ff") + reg.to_bytes(1, 'big'):
            print(rtn.hex())
            print((bytes(self.rFrame) + bytes.fromhex("05ff") + reg.to_bytes(1, 'big')).hex())
            raise ValueError()
        #return ""
        #self.rx.active(0)
        #machine.enable_irq(i)
        time.sleep(self.communication_pause)  # adjust per baud and hardware. Sequential reads without some delay fail.
        if rtn == b'':
            print("TMC2209: Err in read")
            return ""
#         print("received "+str(len(rtn))+" bytes; "+str(len(rtn)*8)+" bits")
        return(rtn[7:11])
#-----------------------------------------------------------------------
# this function tries to read the registry of the TMC 10 times
# if a valid answer is returned, this function returns it as an integer
#-----------------------------------------------------------------------
    def read_int(self, reg):
        tries = 0
        while(True):
            rtn = self.read_reg(reg)
            tries += 1
            if(len(rtn)>=4):
                break
            else:
                print("TMC2209: did not get the expected 4 data bytes. Instead got "+str(len(rtn))+" Bytes")
                raise(ValueError)
            if(tries>=10):
                print("TMC2209: after 10 tries not valid answer. exiting")
                print("TMC2209: is Stepper Powersupply switched on ?")
                raise SystemExit
        val = struct.unpack(">i",rtn)[0]
        return(val)

#-----------------------------------------------------------------------
# this function can write a value to the register of the tmc
# 1. use read_int to get the current setting of the TMC
# 2. then modify the settings as wished
# 3. write them back to the driver with this function
#-----------------------------------------------------------------------
    def write_reg(self, reg, val):
        
        #self.ser.reset_output_buffer()
        #self.ser.reset_input_buffer()
        
        self.wFrame[1] = self.mtr_id
        self.wFrame[2] =  reg | 0x80;  # set write bit
        
        self.wFrame[3] = 0xFF & (val>>24)
        self.wFrame[4] = 0xFF & (val>>16)
        self.wFrame[5] = 0xFF & (val>>8)
        self.wFrame[6] = 0xFF & val
        
        self.wFrame[7] = self.compute_crc8_atm(self.wFrame[:-1])
        self.rx.active(0)
        #rtn = self.ser.write(bytes(self.wFrame))
        for r in self.wFrame:
            self.tx.put(r)
        #if rtn != len(self.wFrame):
        #    print("TMC2209: Err in write {}".format(__), file=sys.stderr)
        #    return False
        time.sleep(self.communication_pause)
        self.rx.active(1)
        return(True)

#-----------------------------------------------------------------------
# this function als writes a value to the register of the TMC
# but it also checks if the writing process was successfully by checking
# the InterfaceTransmissionCounter before and after writing
#-----------------------------------------------------------------------
    def write_reg_check(self, reg, val):
        IFCNT           =   0x02
        ifcnt1 = self.read_int(IFCNT)
        self.write_reg(reg, val)
        ifcnt2 = self.read_int(IFCNT)
        ifcnt2 = self.read_int(IFCNT)
        
        if(ifcnt1 >= ifcnt2):
            print("TMC2209: writing not successful!")
            print("reg:{} val:{}", reg, val)
            print("ifcnt:",ifcnt1,ifcnt2)
            return False
        else:
            return True

#-----------------------------------------------------------------------
# this function clear the communication buffers of the Raspberry Pi
#-----------------------------------------------------------------------
    def flushSerialBuffer(self):
        #self.ser.reset_output_buffer()
        #self.ser.reset_input_buffer()
        return

#-----------------------------------------------------------------------
# this sets a specific bit to 1
#-----------------------------------------------------------------------
    def set_bit(self, value, bit):
        return value | (bit)

#-----------------------------------------------------------------------
# this sets a specific bit to 0
#-----------------------------------------------------------------------
    def clear_bit(self, value, bit):
        return value & ~(bit)

