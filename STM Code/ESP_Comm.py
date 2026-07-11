#class for UART communication with the ESP
from time import sleep
import ustruct


class ESP_Comm:

    def __init__(self, uart_obj, ROMIID):
        #Initializer function
        #Inputs: appropriately configured uart object, id of the Romi we want to track

        self.uart_esp = uart_obj
        #Preallocate buffers
        self.rx_buff = bytearray(12)  # 3 floats * 4 bytes each
        self.tx_buff = bytearray(4)
        self.tx_buff[1] = 0x0A
        self.tx_buff[2] = 0x0A
        self.tx_buff[3] = 0x0A
        self.tx_buff[0] = ROMIID
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.head_offset = 0.0
        #self.initialized = False

    def get_pose(self):
        #Function to send a request to the ESP and get the latest pose estimate
        #Checks if initialization has been set
            #If not, it sets offsets based on the current reading
        #Returns the current pose as a tuple (x, y, heading)
            #If no response from ESP, returns None
        self.uart_esp.write(self.tx_buff) #Request data from ESP
        sleep(0.002) #Give the esp 2 ms to process the request
        if self.uart_esp.any() == 0:
            #If no response from the ESP...
            print("No response from ESP")
            return None
        else:
            #If we did get a response, read it and interpret
            self.uart_esp.readinto(self.rx_buff)
            #print("Received bytes from ESP:", self.rx_buff.hex())
            x, y, heading = ustruct.unpack('<fff', self.rx_buff)
            #if not self.initialized:
            #    #If we haven't set offsets yet, use the first reading to set offsets
            #    self.x_offset = x
            #    self.y_offset = y
            #    self.head_offset = heading
            #    self.initialized = True
            return ((x - self.x_offset)/25.4, (y - self.y_offset)/25.4, heading - self.head_offset)