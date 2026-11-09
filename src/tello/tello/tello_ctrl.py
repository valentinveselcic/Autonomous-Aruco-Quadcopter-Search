# Another Python Library to control a Tello drone using the low level protocol
# A significant part of the code is from telloPy : https://github.com/hanyazou/TelloPy
# All the credits for the low level python implementation is for anyazou (https://github.com/hanyazou)
#
# Compared with TelloPy, this library brings:
# + possibility to downsample images 
# + recording video to file at a specified rate
# + logging all sensors to file at a specified rate
# + viewing live video
# + works with the sister Matlab toolbox
#
# Author : S. Delprat, INSA Hauts-de-France

import threading
import socket
import time
import logging
import av
import math
import os
import fractions
import cv2

from .protocol import *
from .utils import *
from .dispatcher import dispatcher, signal
from . import event
from . import video_stream
from . import state



""" The tello_ctrlException is used to raise exception in the tello_ctrl module"""
class tello_ctrlException(Exception):
    pass


"""The tello_ctrl allows controling a Tello Drone. Essentially, it allows to send joystick controls and receive video.
This object also provide access to all the measured values and the video.

:param ip_address: ip adress of the drone, defaults to '192.168.10.1'
:type ip_address: str, optional
:param port_out: port for sending command to the drone, defaults to 8889
:type port_out: int, optional
param port_in: port for receiving data send by the drone, defaults to 8889
:type port_in: int, optional
"""
class tello_ctrl(object):
    def __init__(self, ip_address='192.168.10.1',port_out=8889, port_in=9000):
        # timeout values
        
        # logger for debugging
        self.__FORMATTER = logging.Formatter('[%(asctime)s %(levelname)s] %(filename)s - %(lineno)d - %(message)s')
        self.__LOGGER_HANDLER_LIST={'console':None,'file':None}
       
        self.__LOGGER = logging.getLogger('tello_ctrl')
        self.__LOGGER.setLevel(logging.DEBUG)
        self.add_console_logger()
        
        # data logger
        self.__DATA_LOGGER = logging.getLogger('Datalogger')
        
        self.__DATA_LOGGER.setLevel(logging.INFO)
        self.__DATA_LOGGER_FORMATTER = None
        self.__DATA_LOGGER_FILEHANDLER = None
        self.__DATA_LOGGER_PERIOD = -1
        self.__DATA_LOGGER_RECORD_ALL=True    # Indicate that all the data needs to be recorded
        self.__DATA_LOGGER_SENSOR_LIST=[]     # List of sensor to be recorded
        self.__DATA_LOGGER_IS_SENSOR=[]       # True if the corresponding sensor is in self.__sensor_list, false for self.__control_list
        self.__DATA_LOGGER_SENSOR_IDX=[]      # Sensor index (either in self.__control_list or self.__sensor_list)
        
        self.__port_in=port_in
        self.__address_in = (ip_address,port_in)
        self.__address_out = (ip_address,port_out)
        
        self.__udpsize = 2000
        
        # Create a dispatcher
        self.__dispatcher=dispatcher()
        
        # custom events
        self.EVENT_CONNECTED = event.Event('connected')
        self.EVENT_WIFI = event.Event('wifi')
        self.EVENT_LIGHT = event.Event('light')
        self.EVENT_FLIGHT_DATA = event.Event('fligt_data')
        self.EVENT_LOG = event.Event('log')
        self.EVENT_TIME = event.Event('time')
        self.EVENT_DISCONNECTED = event.Event('disconnected')
        self.EVENT_FILE_RECEIVED = event.Event('file received')
        self.EVENT_VIDEO_FRAME = event.Event('video frame')
        self.EVENT_VIDEO_DATA = event.Event('video data')
        
        # internal custom events
        self.__EVENT_CONN_REQ = event.Event('conn_req')
        self.__EVENT_CONN_ACK = event.Event('conn_ack')
        self.__EVENT_TIMEOUT = event.Event('timeout')
        self.__EVENT_QUIT_REQ = event.Event('quit_req')

        self.STATE_DISCONNECTED = state.State('disconnected')
        self.STATE_CONNECTING = state.State('connecting')
        self.STATE_CONNECTED = state.State('connected')
        self.STATE_QUIT = state.State('quit')
    
        # state value
        self.__state = self.STATE_DISCONNECTED
        self.__flight_data_received = False
        
        
        # boolean state variables
        self.__flight_data = FlightData()
        self.__wifi_strength = 0.0

        # buffer for file reception
        self.__file_recv={}
        
        
        # threading event
        self.__conected=threading.Event()
        
        
        # current stick command
        self.__left_right          = 0
        self.__forward_backward    = 0
        self.__up_down             = 0
        self.__yaw                 = 0
        self.__fast_mode           = False
        
        
        # Video parameters
        self.__exposure = -9
        self.__video_encoder_bitrate = 4
        self.__pkt_seq_num = 0x01e4
        self.__video_enabled = False
        self.__zoom = False
        self.__video_stream=None
        self.__frame=None
        self.__stream_container = None
        self.__downsample_factor = 1
        self.__video_format='rgb24'
        self.__noframe = -1
        
        # Live view
        self.__live_view_windows_name="Tello live view"
        self.__live_video=False
        
        
        # thread lock
        self.__lock = threading.Lock()
        self.__condition = threading.Condition()

        self.__first_raw_frame_received = False

        # Connect the state machine to all events        
        self.__dispatcher.connect(self.__state_machine, signal.All)
        
        # file recording parameters
        self.__recording_info={'file_name':'','frame_skip':0}
        self.__recording_enabled = False
        self.__recording_container = None
        self.__recording_stream = None
        
        
                    
        """ provide the sensors index given the sensor names (we need to exclude a few attribute from the __flight_data object)"""
        self.__sensor_list = [attr for attr in dir(self.__flight_data) if not callable(getattr(self.__flight_data, attr)) and not attr.startswith('__') and not attr.startswith('ID') and attr!='unknowns_log_msg']
        self.__control_list=['left_right','forward_backward','up_down','yaw','fast_mode']
        

        
    def connect(self, timeout=5):
        """Establish the connection with the droone
        :param timeout: The timeout value for establishing the connection, in seconds. Defaults to 5 seconds if not provided.
        :type timeout: int or float, optional        
        :raises tello_ctrlException: This exception is raised the drone does not respond within the specified timeout interval of time
        
        """
        # Create a UDP socket
        self.__LOGGER.debug('Creating socket')
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__sock.bind(('',self.__port_in))
        self.__sock.settimeout(2.0)

        self.__flight_data_received = False
        # UDP thread reception
        self.__LOGGER.debug('Starting thread')
        threading.Thread(target=self.__data_reception_thread,daemon=True).start()
        
        self.__LOGGER.debug('Connecting')
        self.__send_conn_req()
        
        self.__LOGGER.debug('Wait for connection')
        self.__publish(event=self.__EVENT_CONN_REQ)
        
        self.__conected.clear()
        if not self.__conected.wait(timeout):
            raise tello_ctrlException('Connection timeout')
        
        # wait for the first log packet (so all the variables are available)
        self.__LOGGER.debug('Connect : Wait for first packet')
        tStart=time.time()
        while not self.__flight_data_received:
            time.sleep(0.1)        
        
        self.__LOGGER.debug('End of connection procedure')

    def add_file_logger(self,file_name,mode='w',level="INFO"):
        """This function allows adding a file logger to record message into a text file. The messages are filtered according 
        to the specified level. There exists different levels:
        
        * ``"DEBUG"`` contains all the debugging messages such as packet received, etc.
        * ``"INFO"`` is used to gather the normal operations.
        * ``"ERROR"`` is used to gather only the errors.
            
        The level are ordered: ``"DEBUG"`` will gathers all the messages, ``"INFO"`` will gather ``"INFO"`` and ``"ERROR"``.
        The file logger can be removed using :meth:`~tello_ctrl.tello_ctrl.remove_file_logger`

        :param level: logger level. Can be ``"DEBUG"``, ``"INFO"`` or ``"ERROR"``, defaults to ``"INFO"``
        :type level: str, optional
        :param mode: mode for file creation. ``"a"`` is used to add log at the end of an existing file 
         and ``"w"`` to overwrite the file if it exists.
        
        :raises: tello_ctrlException, exception raised when a file logger has already been added.
        
        """
        if self.__LOGGER_HANDLER_LIST['file'] is None:
            if mode not in ['a','w']:
                raise tello_ctrlException('Mode value must be "a" or "w"')
          
            # add the file handler to the list
            self.__LOGGER_HANDLER_LIST["file"] = logging.FileHandler(file_name,mode=mode)
            self.__LOGGER_HANDLER_LIST["file"].setFormatter(self.__FORMATTER)
        
            self.__LOGGER.addHandler(self.__LOGGER_HANDLER_LIST["file"])
            self.set_log_level("file",level)
        else:
            raise tello_ctrlException('File handler is already active')
    
    def add_console_logger(self,level="ERROR"):
        """This function allows adding a logger to display message on the console. The messages are filtered according 
        to the specified level. There exists different levels:
        
        * ``"DEBUG"`` contains all the debugging messages such as packet received, etc.
        * ``"INFO"`` is used to gather the normal operations.
        * ``"ERROR"`` is used to gather only the errors.
           
        The level are ordered: ``"DEBUG"`` will gathers all the messages, ``"INFO"`` will gather ``"INFO"`` and ``"ERROR"``.
        The file logger can be removed using :meth:`~tello_ctrl.tello_ctrl.remove_file_logger`

        :param level: logger level. Can be ``"DEBUG"``, ``"INFO"`` or ``"ERROR"``, defaults to ``"INFO"``
        :type level: str, optional
        
        :raises: tello_ctrlException, an exception raised when a file logger  has already been added.
        
        """
        
        if self.__LOGGER_HANDLER_LIST['console'] is None:
            # add the console handler to the list
            self.__LOGGER_HANDLER_LIST["console"] =  logging.StreamHandler()
            self.__LOGGER_HANDLER_LIST["console"].setFormatter(self.__FORMATTER)
            self.__LOGGER.addHandler(self.__LOGGER_HANDLER_LIST["console"])
            self.set_log_level("console",level)
        else:
            raise tello_ctrlException('Console handler is already active')
    
    def remove_file_logger(self):
        """ This function removes the file logger. As a result, messages are not written anymore to the file.
        
        raises: tello_ctrlException, an exception is raised if the file logger was not previously added
        
        """
        if self.__LOGGER_HANDLER_LIST['file'] is not None: 
            self.__LOGGER.removeHandler(self.__LOGGER_HANDLER_LIST['file'])
            self.__LOGGER_HANDLER_LIST['file']=None
        else:   
            raise tello_ctrlException('File handler has not been previously set')
    
    def remove_console_logger(self):
        """ This function removes the console logger. As a result, messages are not written anymore to the console.
        
        raises: tello_ctrlException, an exception is raised if the stream logger was not previously added
        
        """
        if self.__LOGGER_HANDLER_LIST['console'] is not None: 
            self.__LOGGER.removeHandler(self.__LOGGER_HANDLER_LIST['console'])
            self.__LOGGER_HANDLER_LIST['console']=None
        else:
            raise tello_ctrlException('Console handler has not been previously set')
        
    
    def set_log_level(self,handler,level):
        """ This function change the level of the selected handler. The level can take three possible values:
            * ``"file"`` : change the file logger
            * ``"console"`` : change the stream logger that writes to the console
            * ``"root"`` : change the level of the root logger. As the root logger sends the messages to the file and console loggers, these two will only receive messages with a level higher than the one selected for root logger.

        :param handler: This parameter indicated which logger level should be changed.
        :type handler: str
        :param level: logger level. Can be ``"DEBUG"``, ``"INFO"`` or ``"ERROR"``, defaults to ``"INFO"``.
        :type level: str
        
        raises: tello_ctrlException, an exception is raised if the selected logger was not previously added.
        
        """
        
        # This function allows to change the log level
        if level not in ['DEBUG','INFO','WARNING','ERROR']:
            raise tello_ctrlException("level must be in ['DEBUG','INFO','WARNING','ERROR']")
        
        if level=="DEBUG":
            level2=logging.DEBUG
        elif level=="INFO":
            level2=logging.INFO
        elif level=="WARNING":
            level2=logging.WARNING
        elif level=="ERROR":
            level2=logging.ERROR
            
        if handler not in ['console','file','root']:
            raise tello_ctrlException('handler must be one of these :  "console", "file", "root"')
        
        if handler != "root":
            # handler is "console" or "file"
            if self.__LOGGER_HANDLER_LIST[handler] is None:
                raise tello_ctrlException('handler is not set')
            self.__LOGGER_HANDLER_LIST[handler].setLevel(level2)
        else:
            # handler is "root"
            self.__LOGGER.setLevel(level2)
        #self.__LOGGER.info('Modifying '+handler+' logger level to {}'.format(level))
            
            
            
    
            
    def __data_reception_thread(self):
        sock = self.__sock
        self.__LOGGER.debug("Starting reception thread")

   
        time_data_logging=time.time()
        tStart=time.time()
        while self.__state != self.STATE_QUIT:
            if self.__state == self.STATE_CONNECTED:
                self.__send_stick_command()  # ignore errors

            try:
                data, server = sock.recvfrom(self.__udpsize)
                #self.__LOGGER.debug("recv: %s" % byte_to_hexstring(data))
                self.__process_packet(data)
                
                now=time.time()
               

                if self.__DATA_LOGGER_PERIOD>0 and now-time_data_logging>self.__DATA_LOGGER_PERIOD:
                    self.data_logging_request()
                    time_data_logging=now
                    
            except socket.timeout as ex:
                if self.__state == self.STATE_CONNECTED:
                    self.__LOGGER.error('data_reception_thread: timeout')
                self.__publish(event=self.__EVENT_TIMEOUT)
                
            except Exception as ex:
                self.__LOGGER.error('data_reception_thread: %s' % str(ex))
                show_exception(ex)

        
        self.__LOGGER.debug('End of drone reception')
    
   
        
    def set_slow_mode(self):
        self.__LOGGER.info('set_slow_mode')
        self.__fast_mode = False
        
    def __send_stick_command(self):
        pkt = Packet(STICK_CMD, 0x60)

        axis1 = int(1024 + 660.0 * self.__left_right) & 0x7ff
        axis2 = int(1024 + 660.0 * self.__forward_backward) & 0x7ff
        axis3 = int(1024 + 660.0 * self.__up_down) & 0x7ff
        axis4 = int(1024 + 660.0 * self.__yaw) & 0x7ff
        axis5 = int(self.__fast_mode) & 0x01        

        '''
        11 bits (-1024 ~ +1023) x 4 axis = 44 bits
        fast_mode takes 1 bit        
        44 bits will be packed in to 6 bytes (48 bits)

                    axis4      axis3      axis2      axis1
             |          |          |          |          |
                 4         3         2         1         0
        98765432109876543210987654321098765432109876543210
         |       |       |       |       |       |       |
             byte5   byte4   byte3   byte2   byte1   byte0
        '''
        #self.__LOGGER.debug("stick command: fast=%d yalrw=%4d ud=%4d fb=%4d rol=%4d" %
        #          (axis5, axis4, axis3, axis2, axis1))
        packed = axis1 | (axis2 << 11) | (
            axis3 << 22) | (axis4 << 33) | (axis5 << 44)
        
        packed_bytes = struct.pack('<Q', packed)
        pkt.add_byte(byte(packed_bytes[0]))
        pkt.add_byte(byte(packed_bytes[1]))
        pkt.add_byte(byte(packed_bytes[2]))
        pkt.add_byte(byte(packed_bytes[3]))
        pkt.add_byte(byte(packed_bytes[4]))
        pkt.add_byte(byte(packed_bytes[5]))
        pkt.add_time()
        pkt.fixup()
        #self.__LOGGER.debug("stick command: %s" % byte_to_hexstring(pkt.get_buffer()))
        return self.__send_packet(pkt)
               
    def __send_packet(self, pkt):
        """Send_packet is used to send a command packet to the drone."""
        try:
            cmd = pkt.get_buffer()
            self.__sock.sendto(cmd, self.__address_out)
            #self.__LOGGER.debug("send_packet: %s" % byte_to_hexstring(cmd))
        except socket.error as err:
            #if self.__state == self.STATE_CONNECTED:
            #    self.__LOGGER.error("send_packet: %s" % str(err))
            #else:
            #    self.__LOGGER.debug("send_packet: %s" % str(err))
            return False

        return True
        
    def __process_packet(self, data):

        if isinstance(data, str):
            data = bytearray([x for x in data])

        if str(data[0:9]) == 'conn_ack:' or data[0:9] == b'conn_ack:':
            self.__LOGGER.info('connected. (port=%2x%2x)' % (data[9], data[10]))
            self.__LOGGER.debug('    %s' % byte_to_hexstring(data))
            self.__publish(self.__EVENT_CONN_ACK, data)
            return True

        if data[0] != START_OF_PACKET:
            self.__LOGGER.info('start of packet != %02x (%02x) (ignored)' % (START_OF_PACKET, data[0]))
            self.__LOGGER.info('    %s' % byte_to_hexstring(data))
            self.__LOGGER.info('    %s' % str(map(chr, data))[1:-1])
            return False

        pkt = Packet(data)
        cmd = uint16(data[5], data[6])
        
        if cmd == LOG_HEADER_MSG:
            id = uint16(data[9], data[10])
            #self.__LOGGER.info("data_reception_thread: log_header: id=%04x, '%s'" % (id, str(data[28:54])))
            #self.__LOGGER.debug("data_reception_thread: log_header: %s" % byte_to_hexstring(data[9:]))
            self.__send_ack_log(id)
            #self.__publish(event=self.EVENT_LOG_HEADER, data=data[9:])
            
                
        elif cmd == LOG_DATA_MSG:
            # This is one of the most interesting message
            self.__flight_data.update_log_message(data[10:],self.__LOGGER)
            self.__flight_data_received = True
            
        elif cmd == WIFI_MSG:
            #self.__LOGGER.debug("data_reception_thread: wifi: %s" % byte_to_hexstring(data[9:]))
            self.__wifi_strength = data[9]
            self.__publish(event=self.EVENT_WIFI, data=data[9:])
       
        elif cmd == LIGHT_MSG:
            #self.__LOGGER.debug("data_reception_thread: light: %s" % byte_to_hexstring(data[9:]))
            self.__publish(event=self.EVENT_LIGHT, data=data[9:])
        
        elif cmd == FLIGHT_MSG:
            self.__flight_data.update_fly_message(data[9:])
            self.__flight_data.wifi_strength = self.__wifi_strength
            #self.__LOGGER.debug("data_reception_thread: flight data: %s" % str(self.__flight_data))
            self.__publish(event=self.EVENT_FLIGHT_DATA, data=self.__flight_data)
            
        elif cmd == TIME_CMD:
            #self.__LOGGER.debug("data_reception_thread: time data: %s" % byte_to_hexstring(data))
            self.__publish(event=self.EVENT_TIME, data=data[7:9])
        
        elif cmd in (TAKEOFF_CMD, LAND_CMD, VIDEO_START_CMD, VIDEO_ENCODER_RATE_CMD, PALM_LAND_CMD,
                     EXPOSURE_CMD, LOG_CONFIG_MSG):
            pass
            #self.__LOGGER.info("data_reception_thread: ack: cmd=0x%02x seq=0x%04x %s" %
            #         (uint16(data[5], data[6]), uint16(data[7], data[8]), byte_to_hexstring(data)))
        elif cmd == TELLO_CMD_FILE_SIZE:
            # Drone is about to send us a file. Get ready.
            # N.b. one of the fields in the packet is a file ID; by demuxing
            # based on file ID we can receive multiple files at once. This
            # code doesn't support that yet, though, so don't take one photo
            # while another is still being received.
            self.__LOGGER.info("data_reception_thread: file size: %s" % byte_to_hexstring(data))
            if len(pkt.get_data()) >= 7:
                (size, filenum) = struct.unpack('<xLH', pkt.get_data())
                self.__LOGGER.info('      file size: num=%d bytes=%d' % (filenum, size))
                # Initialize file download state.
                self.__file_recv[filenum] = DownloadedFile(filenum, size)
            else:
                # We always seem to get two files, one with most of the payload missing.
                # Not sure what the second one is for.
                self.__LOGGER.warn('      file size: payload too small: %s' % byte_to_hexstring(pkt.get_data()))
            # Ack the packet.
            self.__send_packet(pkt)
        elif cmd == TELLO_CMD_FILE_DATA:
            # self.__LOGGERinfo("data_reception_thread: file data: %s" % byte_to_hexstring(data[9:21]))
            # Drone is sending us a fragment of a file it told us to prepare
            # for earlier.
            self.recv_file_data(pkt.get_data())
        else:
            self.__LOGGER.debug('data_reception_thread: unknown packet: %04x %s' % (cmd, byte_to_hexstring(data)))
            return False

        return True
        
    def takeoff(self, blocking=True, timeout=8):
        """Takeoff tells the drones to liftoff and start flying."""
        self.__LOGGER.info('set altitude limit 30m')
        pkt = Packet(SET_ALT_LIMIT_CMD)
        pkt.add_byte(0x1e)  # 30m
        pkt.add_byte(0x00)
        self.__send_packet(pkt)
        self.__LOGGER.info('takeoff (cmd=0x%02x seq=0x%04x)' % (TAKEOFF_CMD, self.__pkt_seq_num))
        pkt = Packet(TAKEOFF_CMD)
        pkt.fixup()
        res=self.__send_packet(pkt)
        if res and blocking:
            # wait for take off: as the command takes time to be executed, 
            # fly mode may be 6 for a while
            tStart=now=time.time()
            
            while self.__flight_data.fly_mode!=11 and now-tStart<timeout:
                time.sleep(0.5)
                now=time.time()
                
            
            # wait until fly_mode change from 12
            while self.__flight_data.fly_mode==11 and now-tStart<timeout:
                time.sleep(0.05)
                now=time.time()

            if now-tStart>=timeout and self.__flight_data.fly_mode==11:
                self.__LOGGER.error('Takeoff timeout')
        return res
        
    def get_fly_mode(self):
        """Returns the current flight mode. Values are:
        
            * ``1`` : the drone is flying
            * ``6`` : the drone is hoovering or landed
            * ``11`` : the drone is taking off
            * ``12`` : the drone is landing
        
        :retunr: The current fly mode.
        :rtype: byte
        
        """
            
        return self.__flight_data.fly_mode
  
    def land(self, blocking = True,timeout=5):
        """Land tells the drone to come in for landing."""
        self.__LOGGER.info('land (cmd=0x%02x seq=0x%04x)' % (LAND_CMD, self.__pkt_seq_num))
        pkt = Packet(LAND_CMD)
        pkt.add_byte(0x00)
        pkt.fixup()
        res= self.__send_packet(pkt)
        
        if res and blocking:
            # wait for fly mode to be 12 (immediately after sending the packet,
            # we may not have a drone response, so the fly_mode may be anything)
            tStart=now=time.time()
            
            while self.__flight_data.fly_mode!=12 and now-tStart<timeout:
                time.sleep(0.5)
                now=time.time()

            
            # wait until fly_mode change from 12
            while self.__flight_data.fly_mode==12 and now-tStart<timeout:
                time.sleep(0.05)
                now=time.time()


            if now-tStart>=timeout and self.__flight_data.fly_mode==12:
                self.__LOGGER.error('Landing timeout')
                
        return res
    def __send_ack_log(self, id):
        pkt = Packet(LOG_HEADER_MSG, 0x50)
        pkt.add_byte(0x00)
        b0, b1 = le16(id)
        pkt.add_byte(b0)
        pkt.add_byte(b1)
        pkt.fixup()
        return self.__send_packet(pkt)
        
    def __send_conn_req(self):
        port = 9617
        port0 = (int(port/1000) % 10) << 4 | (int(port/100) % 10)
        port1 = (int(port/10) % 10) << 4 | (int(port/1) % 10)
        buf = 'conn_req:%c%c' % (chr(port0), chr(port1))
        self.__LOGGER.debug('send connection request (cmd="%s%02x%02x")' % (str(buf[:-2]), port0, port1))
        return self.__send_packet(Packet(buf))
    
    
    def send_rc_control(self,left_right,forward_backward,up_down,yaw):
        """Sets the four sticks values. All the value should be in the -100/100 range.
        Out of range values are clipped.
        
        :param left_right: Position of the left_right stick (-100/100).
        :type left_right: int
        :param forward_backward: Position of the forward_backward stick (-100/100).
        :type forward_backward: int
        :param up_down: Position of the up_down stick (-100/100).
        :type up_down: int
        :param yaw: Position of the yaw stick (-100/100).
        :type yaw: int
        
        """
        # Send RC control via four channels.
        # a,b,c,d as in the SDK, between -100,100
        # They are converted in -1/1  range
        self.__left_right          = self.__saturation(left_right/100)
        self.__forward_backward    = self.__saturation(forward_backward/100)
        self.__up_down             = self.__saturation(up_down/100)
        self.__yaw      = self.__saturation(yaw/100)
        
    def __saturation(self,val):
        # saturate a value between -1/1
        a=val
        if a>1:
            a=1
        elif a<-1:
            a=-1
        return a
        
    def get_sensors_idx(self,sensor_names):
        """Returns the index as a function of the signal name.
            Theses indexes should be used to set ``idx`` in  :meth:`~tello_ctrl.tello_ctrl.get_sensor_values`.
            
            If a sensor name is not found, the corresponding index is set to ``-1```.
            
        :param sensor_name: A list of sensor_names.
        :type sensor_name: [str]
        :return: A list of sensor index.
        :rtype: [int]
        
        """
        if not isinstance(sensor_names,(list,tuple)):
            sensor_names=[sensor_names]

        index=[]
        for name in sensor_names: 
            if name in self.__sensor_list :
                index.append(self.__sensor_list.index(name))
            elif name in self.__control_list:
                index.append(self.__control_list.index(name)+len(self.__sensor_list))
            else:
                index.append(-1)
        return  index
        
    def get_sensor_list(self):
        """Returns the list of all available sensors (it also contains stick control values).
        
        :return: A list of sensor names.
        :rtype: [str]
        
        """
        return self.__sensor_list + self.__control_list
        
    def get_sensor_values_by_index(self,idx=[]):
        """Sends the requested sensor values. The index ``idx`` refers to the position in the list send by :meth:`~tello_ctrl.tello_ctrl.get_sensor_list`.
        
        Alternatively, knowing sensor names, you may get their index with :meth:`~tello_ctrl.tello_ctrl.get_sensors_idx`.
        
        :param idx: List of sensor index, defaults to ``[]`` (send all the known values)
        :return: A list of sensor values.
        :raise: ValueError, An exception if raised if one of the index value is out of range. 
        """
        
       
        
        data=[]
        if idx==[]:
            idx=[i for i in range(len(self.__sensor_list)+len(self.__control_list))]
            
        if not isinstance(idx,(list,tuple)):
            idx=[idx]
            
        for i in idx:
            j=i-len(self.__sensor_list) # index in self.__control_list 
            if i<0 or (i-len(self.__sensor_list)>=len(self.__control_list)):
                raise ValueError('idx must be comprised between 0 and %d'%(len(self.__sensor_list)+len(self.__control_list)-1))
            elif i<len(self.__sensor_list):
                val=getattr(self.__flight_data,self.__sensor_list[i])
                data.append(val)
                
            elif j<len(self.__control_list):
                # in the sensor list, we have first the sensor, then the control
                val=getattr(self,"_tello_ctrl"+'__'+self.__control_list[j])
                if j<4:
                    # stick control are in the -1/1 range, -100/100 for the user
                    val=val*100  
               
                data.append(val)
        
        return data
    
    def get_sensor_values_by_name(self,names=[]):
        """Sends the requested sensor (or control) values.
        The list of sensor and control signal names can be retrived using :meth:`~tello_ctrl.tello_ctrl.get_sensor_list`.
        
        :param names: List of sensor names, defaults to ``[]`` (send all the known values).
        :type names: [str]
        :return: A list of sensor values.
        :raise: ValueError, An exception if raised if sensor or control name is not valid. 
        
        """
        
        if not isinstance(names,(list,tuple)):
            names=[names]
            
        data=[]
        if names==[]:
            # all the values
            names=self.get_sensor_list()
            
       
        for name in names:
            if name in self.__sensor_list :
                val=getattr(self.__flight_data,name)
                data.append(val)

            elif name in self.__control_list:
                val=getattr(self,"_tello_ctrl"+'__'+name)
                idx=self.__control_list.index(name)
                if idx<4:
                    # stick control are in the -1/1 range, -100/100 for the user
                    val=val*100  
                data.append(val)
            else:
                raise ValueError('%s is not a valid sensor or control name'%(name))
        
        return data
        
    def get_battery(self):
        """Returns the battery percentage.
        
        :return: The battery percentage.
        :rtype: int
        
        """
        return self.__flight_data.battery_percentage
        
    def get_drone_velocity(self):
        """Returns the drone velocities in the drone frame. 
        
        :returns: velX,velY,velZ
        :rtype: [float,float,float]
        
        """
        
        return [self.__flight_data.velX,
                self.__flight_data.velY,
                self.__flight_data.velZ]
        
        
    
    def get_accelerometer(self):
        """Returns the drone acceleration in the drone frame. 
        
        :returns: accX,accY,accZ
        :rtype: [float,float,float]
        
        """
        return [self.__flight_data.accX,
                self.__flight_data.accY,
                self.__flight_data.accZ]
                
    def get_gyros(self):
        """Returns the drone rotational velocities in the drone frame. 
        
        :returns: gyroX,gyroY,gyroZ
        :rtype: [float,float,float]
        
        """
        return [self.__flight_data.gyroX,
                self.__flight_data.gyroY,
                self.__flight_data.gyroZ]
                
    def get_ground_velocity(self):
        """Returns the drone ground velocities in the earth frame. It is not sure in which conditions these values are provided by the drone.
        
        :returns: velN,velE,velD
        :rtype: [float,float,float]
        
        """
        return [self.__flight_data.velN,
                self.__flight_data.velE,
                self.__flight_data.velD]
                
    def get_euler_angle(self):
        """Returns the drone Euler angles.
        
        :returns: yaw,pitch,velD
        :rtype: [float,float,roll]
        
        """
        return [self.__flight_data.yaw,
                self.__flight_data.pitch,
                self.__flight_data.roll]
             
    def get_control(self):
        """Returns the actual`` `control values`.` They corresponds to the 4 sticks `left_right`,
        `forward_backward`, `up_down`, `yaw` and the `fast_mode` state.
        
        :returns: The 4 sticks values and the fast_mode state.
        :rtype: [float,float,float,float,bool]
        
        """
        return [self.__left_right,self.__forward_backward,
                self.__up_down,self.__yaw,self.__fast_mode]
        
    def get_position(self):
        """Returns position vector computed using the MVO (Monocular Visual Odometry), i.e. computed using the downward facing camera. 
        Also provide the tof (Time Of Flight) measurement, basically a kind of ultrasonic measurement.
        
        :return: [posX,posY,posZ,tof]
        :rtype: [float,float,float,float]
        
        """
        return [self.__flight_data.posX,
                self.__flight_data.posY,
                self.__flight_data.posZ,
                self.__flight_data.tof]
               
    def get_mvo_pos_valid(self):
        """This property is true when the MVO (Monocular Visual Odometry) position is reliable.
        
        :return: True when MVO position is valid.
        :rtype: bool
        
        """
        return  self.mov_valid_posX and self.mov_valid_posY and self.mov_valid_posZ
    
                
    def get_mvo_vel_valid(self):
        """This property is true when the MVO (Monocular Visual Odometry) velocity is reliable.
        
        :return: True when MVO velocity is valid.
        :rtype: bool
        
        """
        return  self.mov_valid_velX and self.mov_valid_velY and self.mov_valid_velZ
    
        
    def __send_time_command(self):
        self.__LOGGER.info('send_time (cmd=0x%02x seq=0x%04x)' % (TIME_CMD, self.__pkt_seq_num))
        pkt = Packet(TIME_CMD, 0x50)
        pkt.add_byte(0)
        pkt.add_time()
        pkt.fixup()
        return self.__send_packet(pkt)
    
    def quit(self):
        """Stops the reception of all data (sensors & video)."""
        self.__LOGGER.debug('Stopping reception thread')
        if self.__recording_enabled:
            self.stop_recording_to_file()
            
        if self.__video_enabled:
            self.stop_receiving_video()
            
        self.__publish(event=self.__EVENT_QUIT_REQ)
        time.sleep(0.1)
        self.__LOGGER.info('Reception thread stopped')
    
    def __del__(self):
        self.quit()
        
    def __state_machine(self, event, sender, data, **args):
        self.__lock.acquire()
        cur_state = self.__state
        event_connected = False
        event_disconnected = False
        
        if self.__state == self.STATE_DISCONNECTED:
            if event == self.__EVENT_CONN_REQ:
                # connection requested
                self.__send_conn_req()
                self.__state = self.STATE_CONNECTING
                
            elif event == self.__EVENT_QUIT_REQ:
                # disconection requested
                self.__state = self.STATE_QUIT
                event_disconnected = True
                self.__video_enabled = False
            
        elif self.__state == self.STATE_CONNECTING:
            if event == self.__EVENT_CONN_ACK:
                # Connected
                self.__state = self.STATE_CONNECTED
                event_connected = True
                # send time
                self.__send_time_command()
            elif event == self.__EVENT_TIMEOUT:
                # restart
                self.__LOGGER.debug('Restart connection because timeout')
                self.__send_conn_req()
            elif event == self.__EVENT_QUIT_REQ:
                self.__state = self.STATE_QUIT

        elif self.__state == self.STATE_CONNECTED:
            if event == self.__EVENT_TIMEOUT:
                self.__send_conn_req()
                self.__state = self.STATE_CONNECTING
                event_disconnected = True
            elif event == self.__EVENT_QUIT_REQ:
                self.__state = self.STATE_QUIT
                event_disconnected = True
                
        elif self.__state == self.STATE_QUIT:
            pass

        if cur_state != self.__state:
            self.__LOGGER.info('state transit %s -> %s' % (cur_state, self.__state))
        self.__lock.release()

        if event_connected:
            self.__publish(event=self.EVENT_CONNECTED, **args)
            self.__conected.set()
            
        if event_disconnected:
            self.__publish(event=self.EVENT_DISCONNECTED, **args)
            self.__conected.clear()
            
            
            
    def __publish(self, event, data=None, **args):
        args.update({'data': data})
        if 'signal' in args:
            del args['signal']
        if 'sender' in args:
            del args['sender']
        #self.__LOGGER.debug('publish signal=%s, args=%s' % (event, args))
        self.__dispatcher.send(event, sender=self, **args)

    def move_up(self, val):
        """Tells the drone to ascend. Pass in an int from 0-100."""
        self.__LOGGER.info('up(val=%d)' % val)
        self.left_y = val / 100.0

    def move_down(self, val):
        """Moves the drone toward the ground. Pass in an int from 0-100."""
        self.__LOGGER.info('down(val=%d)' % val)
        self.left_y = val / 100.0 * -1

    def move_forward(self, val):
        """Move the drone forward. Pass in an int from 0-100."""
        self.__LOGGER.info('forward(val=%d)' % val)
        self.__forward_backward = val / 100.0

    def move_backward(self, val):
        """Moves the drone in reverse. This command does not modify the drone speed on the other axis.
        :param val: stick position between 0-100
        :type val: float
        
        :raises: ValueError, An exception is raised if val is not between 0 and 100
        
        """
        self.__LOGGER.info('backward(val=%d)' % val)
        if val<0 or val>100:
            raise ValueError('Parameter must be comprised between 0 and 100')
        self.__forward_backward = val / 100.0 * -1

    def move_right(self, val):
        """Moves the drone to the right. This command does not modify the drone speed on the other axis.
        :param val: stick position between 0-100
        :type val: float
        
        :raises: ValueError, An exception is raised if val is not between 0 and 100
        
        """
        self.__LOGGER.info('right(val=%d)' % val)
        self.__left_right = val / 100.0

    def move_left(self, val):
        """Moves the drone to the left. This command does not modify the drone speed on the other axis.
        :param val: stick position between 0-100
        :type val: float
        
        :raises: ValueError, An exception is raised if val is not between 0 and 100
        
        """
        self.__LOGGER.info('left(val=%d)' % val)
        self.__left_right = val / 100.0 * -1

    def move_clockwise(self, val):
        """Rotates the drone clockwise. This command does not modify the drone speed on the other axis.
        :param val: stick position between 0-100
        :type val: float
        
        :raises: ValueError, An exception is raised if val is not between 0 and 100
        
        """
        self.__LOGGER.info('clockwise(val=%d)' % val)
        self.__yaw = val / 100.0

    def move_counter_clockwise(self, val):
        """Rotates the drone counter-clockwise. This command does not modify the drone speed on the other axis.
        :param val: stick position between 0-100
        :type val: float
        
        :raises: ValueError, An exception is raised if val is not between 0 and 100
        
        """
        self.__LOGGER.info('counter_clockwise(val=%d)' % val)
        self.__yaw = val / 100.0 * -1

    def flip_forward(self):
        """flip_forward tells the drone to perform a forwards flip"""
        self.__LOGGER.info('flip_forward (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.__pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipFront)
        pkt.fixup()
        return self.__send_packet(pkt)

    def flip_back(self):
        """flip_back tells the drone to perform a backwards flip"""
        self.__LOGGER.info('flip_back (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.__pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipBack)
        pkt.fixup()
        return self.__send_packet(pkt)

    def flip_right(self):
        """flip_right tells the drone to perform a right flip"""
        self.__LOGGER.info('flip_right (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.__pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipRight)
        pkt.fixup()
        return self.__send_packet(pkt)

    def flip_left(self):
        """flip_left tells the drone to perform a left flip"""
        self.__LOGGER.info('flip_left (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.__pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipLeft)
        pkt.fixup()
        return self.__send_packet(pkt)

    def flip_forwardleft(self):
        """flip_forwardleft tells the drone to perform a forwards left flip"""
        self.__LOGGER.info('flip_forwardleft (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.__pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipForwardLeft)
        pkt.fixup()
        return self.__send_packet(pkt)

    def flip_backleft(self):
        """flip_backleft tells the drone to perform a backwards left flip"""
        self.__LOGGER.info('flip_backleft (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.__pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipBackLeft)
        pkt.fixup()
        return self.__send_packet(pkt)

    def flip_forwardright(self):
        """flip_forwardright tells the drone to perform a forwards right flip"""
        self.__LOGGER.info('flip_forwardright (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.__pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipForwardRight)
        pkt.fixup()
        return self.__send_packet(pkt)

    def flip_backright(self):
        """flip_backleft tells the drone to perform a backwards right flip"""
        self.__LOGGER.info('flip_backright (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.__pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipBackRight)
        pkt.fixup()
        return self.__send_packet(pkt)
        
        
    def __fix_range(self, val, min=-1.0, max=1.0):
        if val < min:
            val = min
        elif val > max:
            val = max
        return val




    def set_up_down(self, val):
        """Set the up-down stick position to the given value (in the -100/100 range). Out of range values are clipped.
        
        :parm val: stick position (-100/100 range)
        :type val: float
        
        """
        up_down=val/100 # self.__up_down is in -1/1
        if self.__up_down != self.__fix_range(up_down):
            self.__LOGGER.info('set_up_down(val=%4.2f)' % val)
        self.__up_down = self.__fix_range(up_down)

    def set_yaw(self, val):
        """Set the yaw stick position to the given value (in the -100/100 range). Out of range values are clipped.
        
        :parm val: yaw position (-100/100 range)
        :type val: float
        
        """
        yaw=val/100
        if self.__yaw!= self.__fix_range(yaw):
            self.__LOGGER.info('set_yaw(val=%4.2f)' % val)
        self.__yaw = self.__fix_range(yaw)

    def set_forward_backward(self, val):
        """Set the forward-backward stick position to the given value (in the -100/100 range). Out of range values are clipped.
        
        :parm val: stick position (-100/100 range)
        :type val: float
        
        """
        forward_backward=val/100
        if self.__forward_backward != self.__fix_range(forward_backward):
            self.__LOGGER.info('set_forward_backward(val=%4.2f)' % val)
        self.__forward_backward = self.__fix_range(forward_backward)

    def set_left_right(self, val):
        """Set the left-right stick position to the given value (in the -100/100 range). Out of range values are clipped.
        
        :parm val: stick position (-100/100 range)
        :type val: float
        
        """
        left_right=val/100
        if self.__left_right != self.__fix_range(left_right):
            self.__LOGGER.info('set_roll(val=%4.2f)' % val)
        self.__left_right = self.__fix_range(left_right)

    def set_fast_mode(self, val):
        """Set the drone fast mode to the specified value.
        
        :param val: Fast mode value (True/False)
        :type val: bool
        :raise ValueError: An exception is raised if the requested mode is not a boolean
        
        """
        if not isinstance(val,bool):
            raise ValueError('Fast mode must be a boolean')
            
        self.__fast_mode = val
        
    def get_fast_mode(self):
        """Get the drone fast mode value.
        
        :return: A boolean value that indicates if the fast mode is set.
        :rtype: bool
        
        """
        return self.fast_mode
 
    def __get_alt_limit(self):
        """..."""
        self.__LOGGER.info('get altitude limit (cmd=0x%02x seq=0x%04x)' % (
            ALT_LIMIT_MSG, self.__pkt_seq_num))
        pkt = Packet(ALT_LIMIT_MSG)
        pkt.fixup()
        return self.__send_packet(pkt)
    
   
    def set_alt_limit(self, limit):
        """ Set the frone altitude limit.
        
        :param limit: Maximum altitude
        :type limit: int
        
        """
        self.__LOGGER.info('set altitude limit=%s (cmd=0x%02x seq=0x%04x)' % (
            int(limit), SET_ALT_LIMIT_CMD, self.__pkt_seq_num))
        pkt = Packet(SET_ALT_LIMIT_CMD)
        pkt.add_byte(int(limit))
        pkt.add_byte(0x00)
        pkt.fixup()        
        self.__send_packet(pkt)
        self.__get_alt_limit()

    def __get_att_limit(self):
        ''' ... '''
        self.__LOGGER.debug('get attitude limit (cmd=0x%02x seq=0x%04x)' % (
            ATT_LIMIT_MSG, self.__pkt_seq_num))
        pkt = Packet(ATT_LIMIT_MSG)
        pkt.fixup()
        return self.__send_packet(pkt)
        
    def __set_att_limit(self, limit):
        self.__LOGGER.info('set attitude limit=%s (cmd=0x%02x seq=0x%04x)' % (
            int(limit), ATT_LIMIT_CMD, self.__pkt_seq_num))
        pkt = Packet(ATT_LIMIT_CMD)
        pkt.add_byte(0x00)        
        pkt.add_byte(0x00)
        pkt.add_byte( int(float.hex(float(limit))[4:6], 16) ) # 'attitude limit' formatted in float of 4 bytes
        pkt.add_byte(0x41)
        pkt.fixup()
        self.__send_packet(pkt)
        self.get_att_limit()
        
    def __get_low_bat_threshold(self):
        ''' ... '''
        self.__LOGGER.debug('get low battery threshold (cmd=0x%02x seq=0x%04x)' % (
            LOW_BAT_THRESHOLD_MSG, self.__pkt_seq_num))
        pkt = Packet(LOW_BAT_THRESHOLD_MSG)
        pkt.fixup()
        return self.__send_packet(pkt)
        
    def set_low_bat_threshold(self, threshold):
        """ Sets the low battery threslhold
        
        :param threshold: Low battery threshold (0-100)
        :type threshold: int
        
        """
        
        self.__LOGGER.info('set low battery threshold=%s (cmd=0x%02x seq=0x%04x)' % (
            int(threshold), LOW_BAT_THRESHOLD_CMD, self.__pkt_seq_num))
        pkt = Packet(LOW_BAT_THRESHOLD_CMD)
        pkt.add_byte(int(threshold))
        pkt.fixup()        
        self.__send_packet(pkt)
        self.get_low_bat_threshold()

    
    def __send_start_video(self):
        pkt = Packet(VIDEO_START_CMD, 0x60)
        pkt.fixup()
        return self.__send_packet(pkt)

    def __send_video_mode(self, mode):
        pkt = Packet(VIDEO_MODE_CMD)
        pkt.add_byte(mode)
        pkt.fixup()
        return self.__send_packet(pkt)

    def get_zoom_state(self):
        """ Get the zoom state of the drone. When the zoom state is ``False``, the video size is 960x720 4:3.
        When the zoom state is ``True``, the video format is  1280x720 16:9.
        4:3 has a wider field of view (both vertically and horizontally), 16:9 is crisper.
        
        :return: A boolean value that indicates if the zoom state.
        :rtype: bool
        
        """
        
        return self.__zooom
     
     
    def set_zoom_state(self, zoom=False):
        """Set the drone video zoom state. When zoom=``False``, the video size is 960x720 4:3, when zoom is ``True``, the video format is  1280x720 16:9.
        4:3 has a wider field of view (both vertically and horizontally), 16:9 is crisper.
        Limitation: You cannot change the zoom state while recording a video file.
        
        :param zoom: Value of the zoom state, defaults to ``False``.
        :type zoom: bool
        :raide tello_ctrlException: An exception is raised if the zoom state is changed while recording a video file.
        
        """
        if self.__recording_enabled and zoom != self.__zoom:
            raise tello_ctrlException('You cannot change zoom setting while recording')
        
        self.__LOGGER.info('set video mode zoom=%s (cmd=0x%02x seq=0x%04x)' % (
            zoom, VIDEO_START_CMD, self.__pkt_seq_num))
        self.__zoom = zoom
        return self.__send_video_mode(int(zoom))
        
    def get_video_exposure(self):
        """Get the video exposure. Values are in the -9..9 range.
        
        :return: Returns the video exposure.
        :rtype: int
        
        """
        
        
        return self.__exposure
    
    def set_video_exposure(self, val):
        """Sets the drone camera exposure level. Valid levels are in -9..9 range.
        
        :param val: exposure level
        :type val: int
        :raises ValueError: An exception is raised when the requested exposure rate is non valid

        """
        if val<-9 or val>9:
            raise ValueError('Invalid exposure level')
        
        level=val+9
        
            
        self.__LOGGER.info('set exposure (cmd=0x%02x seq=0x%04x)' % (EXPOSURE_CMD, self.__pkt_seq_num))
        self.__exposure = level
        self.__send_exposure()
        
    
    def __send_exposure(self):
        pkt = Packet(EXPOSURE_CMD, 0x48)
        pkt.add_byte(self.__exposure)
        pkt.fixup()
        return self.__send_packet(pkt)

    def __send_video_encoder_bitrate(self):
        pkt = Packet(VIDEO_ENCODER_RATE_CMD, 0x68)
        pkt.add_byte(self.__video_encoder_bitrate)
        pkt.fixup()
        return self.__send_packet(pkt)

    def start_receiving_video(self,downsample_factor=1, timeout=15,  video_format='rgb24'):
        """Request video from the drone. It is mandatory to call :meth:`~tello_ctrl.tello_ctrl.start_receiving_video` before accessing the frame with :meth:`~tello_ctrl.tello_ctrl.get_frame`.
        Due to the Tello drone limitations and the pyav video decoder , it can takes a significant amount of time before being able to get an image from. So please use a ``time_out`` greater than 10 seconds.
        
        :param downsample_factor: Allows to downsample the image height&width by the specified factor, defaults to 1.
        :type downsample_factor: integer
        :param time_out: Maximum amount of time allowed to receive the first frame, defaults to 15 seconds.
        :raise tello_ctrlException: An exception is raised if the video is already started.
        :raise tello_ctrlException: An exception is raised if no frame is received within the ``time_out`` perdiod.
        :raise ValueError: An exception is raised if ``downsample_factor`` is not greater or equal to one
        :raise ValueError: An exception is raised if the video_format is not 'rgb24' or 'bgr24'.

        """
        
        if downsample_factor<1:
            raise ValueError('downsample_factor must be  greater or equal to one')
            
        if self.__video_enabled or self.__video_stream is not None:
            raise tello_ctrlException('Video reception is already activated')
        
        if video_format != 'bgr24' and video_format != 'rgb24':
            raise ValueError('Invalid video_format, should be "bgr24" or "rgb24".')
            
        self.__video_format=video_format
      
        self.__downsample_factor=downsample_factor
        self.__LOGGER.info('Start receiving video')
        self.__video_enabled = True
        
        self.__send_exposure()
        self.__send_video_encoder_bitrate()
        res = self.__send_start_video()
        
        self.__LOGGER.debug('  => create video stream')
        self.__video_stream = video_stream.VideoStream(self.__LOGGER)

        self.__first_raw_frame_received=False
        
        # create video thread
        self.__LOGGER.debug('  => create video thread')      
        threading.Thread(target=self.__video_thread,daemon=True).start()
        
        # Start the decoding thread
        self.__LOGGER.debug('  => create video decoding thread') 
        threading.Thread(target=self.__video_decoding_thread, daemon=True).start()
        
        # wait until the first frame is received
        # use a large timeout as it can take some times before receiving the first frame
        img=self.get_frame(timeout)
        if img is None:
            raise tello_ctrlException('Error while receiving video')
        else:
            self.__LOGGER.info('  => First frame received')

    @property
    def is_receiving_video(self):
        """This property (read-only) indicated wether or not the drone is sending video to the base.
        
        :return: Boolean value indicating if the video is being received by the base station.
        :rtype: bool
        """        
        return self.__video_enabled
    
    @property   
    def is_recording(self):
        """This property (read-only) indicated wether the video from the drone is recorded to a file.
        
        :return: Boolean value indicating if the video is being recorded by the base station.
        :rtype: bool

        """ 
        return self.__recording_enabled
        
        
    def stop_receiving_video(self,timeout=5):
        """Stops the video receiption. from the drone. If the video is being recorded, then the recording is also stopped.
        
        :param timeout: Maximum time allowed to closed the video container used to decode the drone video stream, defaults to 5 seconds.
        :type timeout: int
        :param video_format: `rgb24` or `bgr24`` to indicate the order or the R, G, B plane. cv2 uses bgr24.
        :raise tello_ctrlException: An exception is raised if the video receiption was not started.
        :raise tello_ctrlException: An exception is raised if the video container is not closed within the ``time_out`` period.
        """
        
        if self.__recording_enabled:
            # Stop video recording
            self.stop_recording_to_file()
            
        if not self.__video_enabled:
            raise tello_ctrlException('Video is not alreadyh started')
        
          
        # Stop receiving video
        self.__video_enabled = False
        tStart = now = time.time()
        while (self.__stream_container is not None or  self.__video_stream is not None) and (now-tStart<timeout):
            time.sleep(0.05)
            now=time.time()
        
        if (now-tStart>=timeout) and (self.__stream_container is not None or  self.__video_stream is not None):
            if self.__stream_container is not None:
                self.__LOGGER.error('self.__stream_container is not None:')
            else:
                self.__LOGGER.error('self.__video_stream is not None:')
            
            raise tello_ctrlException('Error while stopping the video reception')
        
        

    def __video_thread(self):
        self.__LOGGER.info('start video thread')
        
       # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', 6038))
        sock.settimeout(4.0)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
        self.__LOGGER.debug('video_thread : socket open')
        
        
        prev_video_data = None
        prev_ts = None
        history = []
        
        #  1st byte is slice number
        #  2nd byte, 7 bits is packet number within that frame, 8 bits is end of slice
        frame_no = 0 # counter
        prev_slice_no       = None
        prev_packet_no      = None
        prec_packet_is_last = None
        slice_data=bytes()
        
        self.__LOGGER.debug('video_thread : self.__video_enabled : %r',(self.__video_enabled))
        self.__LOGGER.debug('video_thread : self.__first_raw_frame_received : %r',(self.__first_raw_frame_received))
        time_video_refresh=time.time()
        try:
            while self.__video_enabled:
                try:
                    data, server = sock.recvfrom(self.__udpsize)
                    now=time.time()
                    
                    # extext slice & packet data
                    curr_slice_no  = data[0]
                    curr_packet_no = data[1] & 0x7F  # 7 bits
                    curr_packet_is_last= (data[1] & 0x80) == 0x80 # 8th bit = last packet
                    
                    #self.__LOGGER.debug('Slice : %d packet : %d last : %r' % (curr_slice_no,curr_packet_no,curr_packet_is_last))
                    #self.__LOGGER.info('Raw bytes %d %d' % (data[0],data[1]))
                    
                    # check if data is valid
                    if prev_slice_no == None:
                        # first packet in the first frame
                        packet_ok = curr_packet_no==0
                        prev_packet_no = (curr_packet_no - 1) & 0xFF
                        #self.__LOGGER.info('    => First packet first frame')   
                    else:
                        # any packet
                        # packet is valid if it is a new packet
                        packet_ok = ( (curr_slice_no==prev_slice_no) and (curr_packet_no==prev_packet_no+1) ) \
                                    or (curr_slice_no!=prev_slice_no) and (curr_packet_no==0)
                    
                    if packet_ok:
                        # packet is valid, accumulate slice data
                        #self.__LOGGER.info("    => packet correct")
                        prev_slice_no       = curr_slice_no
                        prev_packet_no      = curr_packet_no
                        prev_packet_is_last = curr_packet_is_last
                        slice_data+=data[2:]
                        
                        
                        if curr_packet_is_last:
                            # send frame to the decoder
                            if self.__video_stream is not None:
                                self.__video_stream.update_raw_data(slice_data)
                            slice_data=bytes()
                            if frame_no==0:
                                # Indicate that the decoding thread can start
                                self.__first_raw_frame_received=True
                                self.__LOGGER.debug('First raw frame received')
                            frame_no+=1
                        
                    else:
                        self.__LOGGER.debug("    => invalid packet")
                        if prev_slice_no is not None:
                            self.__LOGGER.debug('Previous: Slice : %d packet : %d last : %r' % (prev_slice_no,prev_packet_no,prev_packet_is_last))
                        self.__LOGGER.debug('Current: Slice : %d packet : %d last : %r' % (curr_slice_no,curr_packet_no,curr_packet_is_last))
                        self.__LOGGER.debug('Current: Raw bytes %d %d' % (data[0],data[1]))
                        if prev_slice_no is not None:
                            self.__LOGGER.debug('Prev   : Slice : %d packet : %d ' % (prev_slice_no,prev_packet_no))
       
                        # discard all the data as the current packet is not correct
                        slice_data=bytes()
                        
                    # Refresh video request every 2 seconds
                    if  now-time_video_refresh>2:
                        self.__send_start_video()
                        time_video_refresh=now
                        self.__LOGGER.debug('*** __data_reception_thread: __send_start_video')
                    
                except socket.timeout as ex:
                    self.__LOGGER.error('video recv: timeout')
                    self.__send_exposure()
                    self.__send_video_encoder_bitrate()
                    self.__send_start_video()
                    data = None
                
                    
        except Exception as ex:
            self.__LOGGER.error('video recv exception: %s' % str(ex))
            show_exception(ex)
            log_exeception(ex,self.__LOGGER)
            
        finally:
            self.__LOGGER.info('Exit from the video thread (%r).'% (self.__video_enabled))
            sock.close()
            self.__video_stream.end_stream()
            self.__video_stream=None
            

    def __video_decoding_thread(self):
        """ This thread is responsible to decode the h.264 stream using pyav.
            It also send frame to the pyav encoder to record a video file. """
        
        # set py av logging to something low to avoid invalid PPS message at connection
        av.logging.set_level(logging.CRITICAL)
        logging.getLogger("libav").setLevel(logging.CRITICAL)

        self.__LOGGER.debug('Video decoding thread : Wait for the first raw frame')
        while not self.__first_raw_frame_received and self.__video_enabled:
            time.sleep(0.1)
        
        if not self.__video_enabled:
            self.__LOGGER.error('Video decoding thread : video_decoding_thread while recording is not enabled => thread is stopped')
            return
            
        # Try to open the video stream populated by the video thread
        self.__LOGGER.debug('Opening stream_container')
        self.__stream_container =  None
        retry = 5
        while self.__stream_container is None and 0 < retry and self.__video_enabled:
            retry -= 1
            try:
                self.__stream_container =  av.open(self.__video_stream,options={'preset':'ultrafast','tune':'zerolatency '},timeout=(5,0.5))
            except av.AVError as ave:
                # py av will receive incomplete frame (PPS is missing) so it will complain
                # after a few trials it will be ok. So this is logged as debug
                log_exeception(ave,self.__LOGGER,"DEBUG")
                self.__LOGGER.debug('======> retry')
                
                
        # Check if stream container was opened 
        if self.__stream_container is None and self.__video_enabled:
            raise tello_ctrlException('Impossible to retrieve video stream using pyav')
        elif not self.__video_enabled:
            # For some reason decoding is not needed due to the delay taken by the stream opening
            self.__stream_container.close()
            self.__LOGGER.error('video_decoding_thread : __video_enabled is not enabled => thread is stopped')
            return    

        self.__LOGGER.info('Video decoding start now')
        frame_no=0
        
        while self.__video_enabled :
            try:
                self.__LOGGER.info('try decoding')
                for raw_frame in self.__stream_container.decode(video=0):
                    start_time = time.time()
                    
                    self.__condition.acquire()     
                    #self.__frame = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                    self.__frame = raw_frame.to_ndarray(format=self.__video_format)
                    self.__noframe = frame_no

                    # resize if needed
                    if self.__downsample_factor>1:
                        # compute resize (the image size may change depeding on the zoom factor or for edu on which camera is used)
                        new_width = int(self.__frame.shape[1] / self.__downsample_factor)  
                        new_height = int(self.__frame.shape[0] / self.__downsample_factor) 

                        # Downsample the image
                        self.__frame = cv2.resize(self.__frame, (new_width, new_height))
                        
                    frame_copy=self.__frame.copy() # create a copy so we can free the ressource for other threads
                    self.__condition.notifyAll()
                    self.__condition.release() 
                    
                    # display live view
                    if self.__live_video:
                        # CV2 works with BGR image, but the frame is RGB, we need to convert
                        print('refresh')
                        cv2.imshow(self.__live_view_windows_name,  cv2.cvtColor(frame_copy, cv2.COLOR_RGB2BGR))
                        # force CV2 to refresh the image
                        cv2.waitKey(1)
                    
                    # record if needed
                    if (self.__recording_enabled and 
                        self.__recording_container is not None and
                        frame_no%(1+self.__recording_info['frame_skip'])==0):
                        # We need to record frame
                        # Add pts (NB: self.__recording_stream.time_base changes after a few frame)
                        frame_time=frame_no/30 # Theoretical time in seconds
                        newframe = av.VideoFrame.from_ndarray(frame_copy, format=self.__video_format)
                        newframe.pts=round(frame_time/self.__recording_stream.time_base)
                        newframe.time_base=self.__recording_stream.time_base
                        
                        for packet in self.__recording_stream.encode(newframe):
                            self.__recording_container.mux(packet)
                    frame_no+=1
            except Exception as e:
                # error, stop recording
                self.__LOGGER.error('Error during frame encoding');
                log_exeception(e,self.__LOGGER)
           
        self.__LOGGER.info('Cleaning before end of video decoding thread');     
        self.__close_recording_container()        
        self.__stream_container.close()
        self.__stream_container=None           
        
        
    def __close_recording_container(self):
        # flush
        if  self.__recording_container is not None:
            self.__LOGGER.info('Closing video container')        
        
            packet=self.__recording_stream.encode(None)
            self.__recording_container.mux(packet) 
            self.__recording_container.close()
            self.__recording_enabled = False
        self.__recording_enabled = False
         
        
    
    def get_frame_with_no(self, timeout=1):
        """This function returns an RGB frame as a numpy array. The array size is H x W x 3.
        If the frame is not recived within the ``time_out`` period, the last available frame is returned (may be ``None`` if no frame has already been received).
        If the video reception is not started using :meth:`~tello_ctrl.tello_ctrl.start_receiving_video`, then the last available frame is returned.
        
        :param time_out: Maximum amount of time allowed to receive a frame, defaults to 1 seconds.
        :type time_out: int
        :return: (frame, frame_no) The frame and the frame number
        :rtype: (numpy.ndarray, int)
        """
        if self.__video_enabled:
            with self.__condition:
                # wait for a frame (or video stopped for whatever reason)
                tStart=now=time.time()
                while self.__frame is None and self.__video_enabled and now-tStart<timeout:
                    self.__condition.wait(0.1)
                    now=time.time()
                if now-tStart>=timeout and self.__frame is None:
                    self.__LOGGER.error('Time out when receiving a frame')
                tmp=self.__frame
                no=self.__noframe
            return tmp, no
        else:
            # return the last available frame (eventually None if video was never activated)
            return self.__frame, self.__noframe

    def get_frame(self, timeout=1):
        """This function returns an RGB frame as a numpy array. The array size is H x W x 3.
        If the frame is not recived within the ``time_out`` period, the last available frame is returned (may be ``None`` if no frame has already been received).
        If the video reception is not started using :meth:`~tello_ctrl.tello_ctrl.start_receiving_video`, then the last available frame is returned.
        
        :param time_out: Maximum amount of time allowed to receive a frame, defaults to 1 seconds.
        :type time_out: int
        :return: frame The frame and the frame number
        :rtype: numpy.ndarray
        """
        if self.__video_enabled:
            with self.__condition:
                # wait for a frame (or video stopped for whatever reason)
                tStart=now=time.time()
                while self.__frame is None and self.__video_enabled and now-tStart<timeout:
                    self.__condition.wait(0.1)
                    now=time.time()
                if now-tStart>=timeout and self.__frame is None:
                    self.__LOGGER.error('Time out when receiving a frame')
                tmp=self.__frame
                no=self.__noframe
            return tmp
        else:
            # return the last available frame (eventually None if video was never activated)
            return self.__frame
            
    def subscribe(self, signal, handler):
        """Subscribe a event such as EVENT_CONNECTED, EVENT_FLIGHT_DATA, EVENT_VIDEO_FRAME and so on."""
        self.__dispatcher.connect(handler, signal)


    
    def set_video_encoder_bitrate(self, bitrate):
        """Set the video encoder bit rate.
        
        :param bitrate: value of the bitrate in Mbps units. 0 means auto. Should be in the 0..5 range.
        :type bitrate: int
        :raise ValueError: An exception is raised if the bitrate is invalid..
        
        """
        if bitrate<0 or bitrate>5:
            raise ValueError('Invalid bitrate (should be in the 0..5 range)')
            
        self.__LOGGER.info('set video encoder rate (cmd=0x%02x seq=%04x)' %
                 (VIDEO_ENCODER_RATE_CMD, self.__pkt_seq_num))
        self.video_encoder_bitrate = bitrate
        return self.__send_video_encoder_bitrate()


    def start_recording_video_to_file(self,file_name, frame_skip=0):
        """Starts recording the video to the specified file. The video should be already started using :meth:`~tello_ctrl.tello_ctrl.start_receiving_video`.
        The Tello Drone sends the video at a nominal 30 FPS rate. It is possible to skip some frame by indicating a positive ``frame_skip`` value. 
        It is not possible to change the zoom state while recording a video.
        
        The video is encoded using the ``pyav`` library (which uses FFMPEG under the hood). It will save the H.264 stream send by the drone into a container (video file). 
        MKV files are working well, but other may also work. 
        
        :param file_name: File used to save the video. If folders are specified and do not exist, they are created. If the file exist, it is overwritten.
        :type file_name: str
        :param frame_skip: Only one frame every ``frame_skip`` will be saved in the video file. It defaults to 0 (all the frames are kept).
        :type frame_skip: int
        :raise tello_ctrlException: An exception is raised if the video is not started yet using :meth:`~tello_ctrl.tello_ctrl.start_receiving_video`.
        :raise tello_ctrlException: An exception is raised if a video is alrady being recorded.
        :raise ValueError: An exception is raised if frame_skip is not positive or null.
        
        """
        
        # Record the frames into the specified video file
        # frame_skip explains how many frames should be skip (original fps is 30)
        
        # check that video is on
        if not self.__video_enabled:
            raise tello_ctrlException('You must first start receiving video (by calling start_receiving_video)')
            
        if self.__recording_enabled:
            raise tello_ctrlException('A video file is already being recorded')

            
        if frame_skip<0:
            raise ValueError('frame_skip must be positive or null')
            
        # check extension
        base, ext = os.path.splitext(file_name)
        if not ext:
           file_name=file_name+'.mkv'
        
        directory = os.path.dirname(file_name)
        print(os.path.dirname(file_name))
        if directory != '' and not os.path.exists(directory):
            os.makedirs(directory) 

        # Store data for the thread
        self.__recording_info={'file_name':file_name,'frame_skip':frame_skip}
        
        # create the container : specify fps & image size
        fps = 30 / (1+frame_skip)
        
        self.__recording_container = av.open(file_name,"w")
        self.__recording_stream = self.__recording_container.add_stream("libx264", str(fps))
        self.__recording_stream.time_base = fractions.Fraction(1+frame_skip,30)
        
        if self.__zoom:
            image_size=[1280,720]
        else:
            image_size=[960,720]

   
        self.__recording_stream.width = image_size[0]
        self.__recording_stream.height = image_size[1]
        
        self.__recording_enabled=True
        
         
        self.__LOGGER.info('Start recording using %d fps'%(fps))
        
    def stop_recording_video_to_file(self):
        """Stops recording a video file."""
        
        
        if  not self.__recording_enabled:
            self.__LOGGER.error('Trying to stop recording but video is not currently recorded')
            return
        self.__recording_enabled=False
        time.sleep(0.5)  
        self.__close_recording_container()
        self.__LOGGER.info('Video recording stopped')


    def start_data_logging(self, file_name, sampling_time=0.1, mode='w', sensor_list=[]):
        """Starts logging received data to the specified CSV file with a specified ``sampling_time``. 
        When sampling time is negative, the data are not logged automatically but only when :meth:`~tello_ctrl.tello_ctrl.data_logging_request` is called.
        The ``mode`` parameter can take 2 values:
        
            * ``"w"``: overwrite the file if it already exists.
            * ``"a"``: append new line if the file already exists.
        
        :param file_name: File used to save the data. If folders are specified and do not exist, they are created.
        :type file_name: str
        :param sampling_time: interval of time between two record in the log file.
        :type sampling_time: float
        :param sensor_list: List of sensor to be recorded, default to [] (all the available sensors). Available sensors can be obtained using :meth:`~tello_ctrl.tello_ctrl.get_sensor_list`
        :type sensor_list: [str]
        :raise tello_ctrlException: An exception is raised if the logger is already started.
        :raise ValueError: An exception is raised if mode is not ``"a"`` or ``"w"``.
        
        """
        
        if self.__DATA_LOGGER_FILEHANDLER is not None:
            raise tello_ctrlException('Data are already logged. Use stop_data_logging first')
        
        if mode!='w' and mode!='a':
            raise ValueError('mode must be "a" or "w", not %s.' % (mode))
            
        directory = os.path.dirname(file_name)
        if directory != '' and not os.path.exists(directory):
            os.makedirs(directory) 
            
        # check extension
        base, ext = os.path.splitext(file_name)
        if not ext:
           file_name=file_name+'.CSV'
           
        if sensor_list==[]:
            self.__DATA_LOGGER_SENSOR_LIST=self.__sensor_list + self.__control_list
            self.__DATA_LOGGER_RECORD_ALL=True
            self.__DATA_LOGGER_IS_SENSOR=[]  # Not used
            self.__DATA_LOGGER_SENSOR_IDX=[] # Not used
        else:
            # check that the provided list is correct
            self.__DATA_LOGGER_IS_SENSOR=[]
            self.__DATA_LOGGER_SENSOR_IDX=[]
            for sensor in sensor_list:
                if sensor in self.__sensor_list:
                    self.__DATA_LOGGER_IS_SENSOR.append(True)
                    self.__DATA_LOGGER_SENSOR_IDX.append(self.__sensor_list.index(sensor))
                elif sensor in self.__control_list:
                    self.__DATA_LOGGER_IS_SENSOR.append(False)
                    self.__DATA_LOGGER_SENSOR_IDX.append(self.__control_list.index(sensor))
                else:
                    raise ValueError('The requested sensor "%s" does not exists.' % (sensor))
            self.__DATA_LOGGER_SENSOR_LIST=sensor_list
            self.__DATA_LOGGER_RECORD_ALL=False

            
        self.__DATA_LOGGER_FILEHANDLER =  logging.FileHandler(file_name,mode=mode)
        
        # Use a format such that we can write the header without the date
        self.__DATA_LOGGER_FORMATTER=logging.Formatter('%(message)s')

        self.__DATA_LOGGER_FILEHANDLER.setFormatter(self.__DATA_LOGGER_FORMATTER)
        
        self.__DATA_LOGGER.addHandler(self.__DATA_LOGGER_FILEHANDLER)
        self.__DATA_LOGGER_FILEHANDLER.setLevel(logging.INFO)
        self.__DATA_LOGGER_PERIOD = sampling_time
        self.__LOGGER.info('Data logger started into file %s with sampling time %.2f and mode "%s"' % (file_name,sampling_time,mode))



        # write header of the csv file
        data_str='date;time;'
        if self.__DATA_LOGGER_RECORD_ALL:
            # part 1 sensor data
            for i in range (len(self.__sensor_list)):
                data_str+='%s;'%(self.__sensor_list[i])
            # part 2 : control values
            for i in range(len(self.__control_list)):
                data_str+='%s;'%(self.__control_list[i])
        else:
            # Only some sensors
            for sensor in sensor_list:
                data_str+='%s;'%(sensor)
        
        self.__DATA_LOGGER.info(data_str)
 
        # Mofidy the formater to autoamtically include the exact time of logging
        self.__DATA_LOGGER_FORMATTER=logging.Formatter('%(asctime)s;%(message)s')
        self.__DATA_LOGGER_FILEHANDLER.setFormatter(self.__DATA_LOGGER_FORMATTER)

    def stop_data_logging(self):
        """Stops the data logger.
        
        :raise tello_ctrlException: An exception is raised if the data logger was not previously started using :meth:`~tello_ctrl.tello_ctrl.start_data_logging`.
        
        """
        if self.__DATA_LOGGER_FILEHANDLER is  None: 
            raise tello_ctrlException('Data logger is not logged yet. Use start_data_logging first')
        self.__DATA_LOGGER.removeHandler(self.__DATA_LOGGER_FILEHANDLER)
        
        self.__DATA_LOGGER_FILEHANDLER = None
        self.__DATA_LOGGER_PERIOD = -1
        self.__LOGGER.info('Data logger stopped')
        
    def data_logging_request(self):
        """Manually log the data in the log file.
        
        :raise tello_ctrlException: An exception is raised if the data logger was not previously started using :meth:`~tello_ctrl.tello_ctrl.start_data_logging`.
        
        """
        
        # log data upon request
        if  self.__DATA_LOGGER_FILEHANDLER is None:
            raise tello_ctrlException('Data logger is not started yet. Use start_data_logging first')

        data_str='{0:.3f};'.format(time.time())
        
        
        if self.__DATA_LOGGER_RECORD_ALL:
            # Record all the data
            # part 1 sensor data
            for i in range (len(self.__sensor_list)):
                data_str+='%.10e;'%(getattr(self.__flight_data,self.__sensor_list[i]))
                
            # part 2 : control values
            for i in range(len(self.__control_list)):
                if i<4:
                    # controls values needs to be shifted in the -100/100 range
                    data_str+='%.10e;'%(100*getattr(self,"_tello_ctrl"+'__'+self.__control_list[i]))
                else:
                    data_str+='%.10e;'%(getattr(self,"_tello_ctrl"+'__'+self.__control_list[i]))
                
        else:
            # Record some data in the order of self.__DATA_LOGGER_SENSOR_LIST
            for i in range(len(self.__DATA_LOGGER_SENSOR_LIST)):
                idx=self.__DATA_LOGGER_SENSOR_IDX[i]
                if self.__DATA_LOGGER_IS_SENSOR[i]:
                   # Current sensor is in self.__sensor_list
                   data_str+='%.10e;'%(getattr(self.__flight_data,self.__sensor_list[idx]))
                else:
                    # Current sensor is in self.__control_list
                    if idx<4:
                        # controls values needs to be shifted in the -100/100 range
                        data_str+='%.10e;'%(100*getattr(self,"_tello_ctrl"+'__'+self.__control_list[idx]))
                    else:
                        data_str+='%.10e;'%(getattr(self,"_tello_ctrl"+'__'+self.__control_list[idx]))
        
        # Add data to the recorder
        self.__DATA_LOGGER.info(data_str)

    def __start_live_video(self, position=None, size=None, stay_on_top=False):
        """ not working yet as cv2 crashes when called within a thread"""
        if not self.__video_enabled:
            raise tello_ctrlException('You must first start receiving video (by calling start_receiving_video)')
        
        cv2.namedWindow(self.__live_view_windows_name, cv2.WINDOW_NORMAL)
        if size is not None:
            cv2.resizeWindow(self.__live_view_windows_name, size[0], size[1])

        #cv2.setWindowProperty(self.__live_view_windows_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        if position is not None:
            cv2.moveWindow(self.__live_view_windows_name, position[0], position[1])
        
        if stay_on_top:
            cv2.setWindowProperty(self.__live_view_windows_name, cv2.WND_PROP_TOPMOST, 1)

        self.__live_video=True
    
    def __stop_live_video(self):
        """ not working yet as cv2 crashes when called within a thread"""
        self.__live_video=False
        time.sleep(2/30)
        cv2.destroyWindow(self.__live_view_windows_name)



