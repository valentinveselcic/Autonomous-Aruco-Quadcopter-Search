import datetime
from io import BytesIO
import struct
from . import crc
from . utils import *
import math
# low-level Protocol (https://tellopilots.com/wiki/protocol/#MessageIDs)

START_OF_PACKET                     = 0xcc
SSID_MSG                            = 0x0011
SSID_CMD                            = 0x0012
SSID_PASSWORD_MSG                   = 0x0013
SSID_PASSWORD_CMD                   = 0x0014
WIFI_REGION_MSG                     = 0x0015
WIFI_REGION_CMD                     = 0x0016
WIFI_MSG                            = 0x001a
VIDEO_ENCODER_RATE_CMD              = 0x0020
VIDEO_DYN_ADJ_RATE_CMD              = 0x0021
EIS_CMD                             = 0x0024
VIDEO_START_CMD                     = 0x0025
VIDEO_RATE_QUERY                    = 0x0028
TAKE_PICTURE_COMMAND                = 0x0030
VIDEO_MODE_CMD                      = 0x0031
VIDEO_RECORD_CMD                    = 0x0032
EXPOSURE_CMD                        = 0x0034
LIGHT_MSG                           = 0x0035
JPEG_QUALITY_MSG                    = 0x0037
ERROR_1_MSG                         = 0x0043
ERROR_2_MSG                         = 0x0044
VERSION_MSG                         = 0x0045
TIME_CMD                            = 0x0046
ACTIVATION_TIME_MSG                 = 0x0047
LOADER_VERSION_MSG                  = 0x0049
STICK_CMD                           = 0x0050
TAKEOFF_CMD                         = 0x0054
LAND_CMD                            = 0x0055
FLIGHT_MSG                          = 0x0056
SET_ALT_LIMIT_CMD                   = 0x0058
FLIP_CMD                            = 0x005c
THROW_AND_GO_CMD                    = 0x005d
PALM_LAND_CMD                       = 0x005e
TELLO_CMD_FILE_SIZE                 = 0x0062  # pt50
TELLO_CMD_FILE_DATA                 = 0x0063  # pt50
TELLO_CMD_FILE_COMPLETE             = 0x0064  # pt48
SMART_VIDEO_CMD                     = 0x0080
SMART_VIDEO_STATUS_MSG              = 0x0081
LOG_HEADER_MSG                      = 0x1050
LOG_DATA_MSG                        = 0x1051
LOG_CONFIG_MSG                      = 0x1052
BOUNCE_CMD                          = 0x1053
CALIBRATE_CMD                       = 0x1054
LOW_BAT_THRESHOLD_CMD               = 0x1055
ALT_LIMIT_MSG                       = 0x1056
LOW_BAT_THRESHOLD_MSG               = 0x1057
ATT_LIMIT_CMD                       = 0x1058 # Stated incorrectly by Wiki (checked from raw packets)
ATT_LIMIT_MSG                       = 0x1059


#Flip commands taken from Go version of code
#FlipFront flips forward.
FlipFront = 0
#FlipLeft flips left.
FlipLeft = 1
#FlipBack flips backwards.
FlipBack = 2
#FlipRight flips to the right.
FlipRight = 3
#FlipForwardLeft flips forwards and to the left.
FlipForwardLeft = 4
#FlipBackLeft flips backwards and to the left.
FlipBackLeft = 5
#FlipBackRight flips backwards and to the right.
FlipBackRight = 6
#FlipForwardRight flips forwards and to the right.
FlipForwardRight = 7

class Packet(object):
    def __init__(self, cmd, pkt_type=0x68, payload=b''):
        if isinstance(cmd, str):
            self.buf = bytearray()
            for c in cmd:
                self.buf.append(ord(c))
        elif isinstance(cmd, (bytearray, bytes)):
            self.buf = bytearray()
            self.buf[:] = cmd
        else:
            self.buf = bytearray([
                START_OF_PACKET,
                0, 0,
                0,
                pkt_type,
                (cmd & 0xff), ((cmd >> 8) & 0xff),
                0, 0])
            self.buf.extend(payload)

    def fixup(self, seq_num=0):
        buf = self.get_buffer()
        if buf[0] == START_OF_PACKET:
            buf[1], buf[2] = le16(len(buf)+2)
            buf[1] = (buf[1] << 3)
            buf[3] = crc.crc8(buf[0:3])
            buf[7], buf[8] = le16(seq_num)
            self.add_int16(crc.crc16(buf))

    def get_buffer(self):
        return self.buf

    def get_data(self):
        return self.buf[9:len(self.buf)-2]

    def add_byte(self, val):
        self.buf.append(val & 0xff)

    def add_int16(self, val):
        self.add_byte(val)
        self.add_byte(val >> 8)

    def add_time(self, time=datetime.datetime.now()):
        self.add_int16(time.hour)
        self.add_int16(time.minute)
        self.add_int16(time.second)
        self.add_int16(int(time.microsecond/1000) & 0xff)
        self.add_int16((int(time.microsecond/1000) >> 8) & 0xff)

    def get_time(self, buf=None):
        if buf is None:
            buf = self.get_data()[1:]
        hour = int16(buf[0], buf[1])
        min = int16(buf[2], buf[3])
        sec = int16(buf[4], buf[5])
        millisec = int16(buf[6], buf[8])
        now = datetime.datetime.now()
        return datetime.datetime(now.year, now.month, now.day, hour, min, sec, millisec)

""" Info about Fligth data decoding :
+     // https://github.com/Kragrathea/TelloLib/blob/master/TelloLib/parsedRecSpecs.json
+     // https://github.com/o-gs/dji-firmware-tools/blob/master/comm_dissector/wireshark/dji-mavic-flyrec-proto.lua
+    //  https://github.com/bgromov/TelloSwift/blob/master/Sources/TelloSwift/TelloProtocol/Constants.swift
+    //  From https://github.com/BudWalkerJava/DatCon/blob/master/DatCon/src/DatConRecs/FromViewer/new_mvo_feedback_29.java#L49

"""



class FlightData(object):

    def __init__(self):
    
        self.ID_NEW_MVO_FEEDBACK                = 29    # 
        self.ID_IMU_ATTI                        = 2048  # 0x800 IMU Only
        self.ID_IMU_EXT                         = 2064  # 0x810 IMU Extended
        self.unknowns_log_msg = []

    
    
        self.battery_low = 0
        self.battery_lower = 0
        self.battery_percentage = 0
        self.battery_state = 0
        self.camera_state = 0
        self.down_visual_state = 0
        self.drone_battery_left = 0
        self.drone_fly_time_left = 0
        self.drone_hover = 0
        self.em_open = 0
        self.em_sky = 0
        self.em_ground = 0
        self.east_speed = 0
        self.electrical_machinery_state = 0
        self.factory_mode = 0
        self.fly_mode = 0
        self.fly_speed = 0
        self.fly_time = 0
        self.front_in = 0
        self.front_lsc = 0
        self.front_out = 0
        self.gravity_state = 0
        self.ground_speed = 0
        self.height = 0
        self.imu_calibration_state = 0
        self.imu_state = 0
        self.light_strength = 0
        self.north_speed = 0
        self.outage_recording = 0
        self.power_state = 0
        self.pressure_state = 0
        self.smart_video_exit_mode = 0
        self.temperature_height = 0
        self.throw_fly_timer = 0
        self.wifi_disturb = 0
        self.wifi_strength = 0
        self.wind_state = 0

        # MVO
        self.velX=0
        self.velY=0
        self.velZ=0
        self.posX=0
        self.posY=0
        self.posZ=0
        
        self.posCov1=0
        self.posCov2=0
        self.posCov3=0
        self.posCov4=0
        self.posCov5=0
        self.posCov6=0

        self.velCov1=0
        self.velCov2=0
        self.velCov3=0
        self.velCov4=0
        self.velCov5=0
        self.velCov6=0
        
        self.tof=0
        self.tofUncertainty=0
        
        self.mov_valid_velX = False
        self.mov_valid_velY = False
        self.mov_valid_velZ = False
        self.mov_valid_posX = False
        self.mov_valid_posY = False
        self.mov_valid_posZ = False
        
        # IMU
        self.longitude=0
        self.latitude=0
        self.baro=0
        
        self.accX=0
        self.accY=0
        self.accZ=0
        self.gyroX=0
        self.gyroY=0
        self.gyroZ=0
        self.baro_smooth=0
        self.qW=0
        self.qX=0
        self.qY=0
        self.qZ=0
        self.velN=0
        self.velE=0
        self.velD=0
        
        # IMU to Euler
        self.yaw=0
        self.pitch=0
        self.roll=0
        self.vgX=0
        self.vgY=0
        self.vgZ=0
        
        # IMU EXTENDED WITH VISUAL ODOMETRY
        self.velX_VO=0
        self.velY_VO=0
        self.velZ_VO=0
        self.posX_VO=0
        self.posY_VO=0
        self.posZ_VO=0
        self.vel_VO=0
        self.dist_V0=0
        self.rtkLong_VO=0
        self.rtkLat_VO=0
        self.rtkAlt_VO=0
        self.error_flag_VO=0 #  To DO  decoding
        
    def update_fly_message(self,data):
        if len(data) < 24:
            return

        self.height = int16(data[0], data[1])
        self.north_speed = int16(data[2], data[3])
        self.east_speed = int16(data[4], data[5])
        self.ground_speed = int16(data[6], data[7])
        self.fly_time = int16(data[8], data[9])

        self.imu_state = ((data[10] >> 0) & 0x1)
        self.pressure_state = ((data[10] >> 1) & 0x1)
        self.down_visual_state = ((data[10] >> 2) & 0x1)
        self.power_state = ((data[10] >> 3) & 0x1)
        self.battery_state = ((data[10] >> 4) & 0x1)
        self.gravity_state = ((data[10] >> 5) & 0x1)
        self.wind_state = ((data[10] >> 7) & 0x1)

        self.imu_calibration_state = data[11]
        self.battery_percentage = data[12]
        self.drone_battery_left = int16(data[13], data[14])
        self.drone_fly_time_left = uint16(data[15], data[16]) #vérifier int ou uint

        self.em_sky = ((data[17] >> 0) & 0x1)
        self.em_ground = ((data[17] >> 1) & 0x1)
        self.em_open = ((data[17] >> 2) & 0x1)
        self.drone_hover = ((data[17] >> 3) & 0x1)
        self.outage_recording = ((data[17] >> 4) & 0x1)
        
        self.battery_low = ((data[17] >> 5) & 0x1)
        self.battery_lower = ((data[17] >> 6) & 0x1)
        self.factory_mode = ((data[17] >> 7) & 0x1)

        self.fly_mode = data[18]
        self.throw_fly_timer = data[19]
        self.camera_state = data[20]
        self.electrical_machinery_state = data[21]

        self.front_in = ((data[22] >> 0) & 0x1)
        self.front_out = ((data[22] >> 1) & 0x1)
        self.front_lsc = ((data[22] >> 2) & 0x1)

        self.temperature_height = ((data[23] >> 0) & 0x1)
         
       
            
        
    def __str__(self):
        return (
            ("ALT: %2d" % self.height) +
            (" | SPD: %2d" % self.ground_speed) +
            (" | BAT: %2d" % self.battery_percentage) +
            (" | WIFI: %2d" % self.wifi_strength) +
            (" | CAM: %2d" % self.camera_state) +
            (" | MODE: %2d" % self.fly_mode) +
            # (", drone_battery_left=0x%04x" % self.drone_battery_left) +
            "")
            
            
    def update_log_message(self,data,LOGGER):
        #LOGGER.debug('*** Beging log packet processing')
        if isinstance(data, bytearray):
            data = str(data)
        pos = 0
        while (pos < len(data) - 2):
            #LOGGER.debug('LogNewMvoFeedback: pos : %d' % (pos))
               
            if (struct.unpack_from('B', data, pos+0)[0] != 0x55):
                #raise Exception('LogData: corrupted data at pos=%d, data=%s'
                #               % (pos, byte_to_hexstring(data[pos:])))
                return
            length = struct.unpack_from('<h', data, pos+1)[0]
            checksum = data[pos+3]
            id = struct.unpack_from('<H', data, pos+4)[0]
            # 4bytes data[6:9] is tick
            # last 2 bytes are CRC
            # length-12 is the byte length of payload
            xorval = data[pos+6]
            if isinstance(data, str):
                payload = bytearray([ord(x) ^ ord(xorval) for x in data[pos+10:pos+10+length-12]])
            else:
                payload = bytearray([x ^ xorval for x in data[pos+10:pos+10+length-12]])

            if id==self.ID_NEW_MVO_FEEDBACK:
                # info from : https://github.com/bgromov/TelloSwift/blob/06cfb548de787ba373472df381ee49224d2ca89c/Sources/TelloSwiftObjC/include/Protocol.h
                # Decoding of speed & position
                #LOGGER.debug('LogNewMvoFeedback: length=%d %s' % (len(payload), byte_to_hexstring(payload)))
                (self.velX, self.velY, self.velZ) = struct.unpack_from('<hhh', payload, 2)
                self.velX /= 1000.0
                self.velY /= 1000.0
                self.velZ /= 1000.0
                (self.posX, self.posY, self.posZ,self.posUncertainty) = struct.unpack_from('ffff', payload, 8)
                (self.posCov1,self.posCov2,self.posCov3)= struct.unpack_from('fff', payload, 20)
                (self.posCov4,self.posCov5,self.posCov6)= struct.unpack_from('fff', payload, 32)
                (self.velCov1,self.velCov2,self.velCov3)= struct.unpack_from('fff', payload, 44)
                (self.velCov4,self.velCov5,self.velCov6)= struct.unpack_from('fff', payload, 56)
                
                (self.tof,self.tofUncertainty) = struct.unpack_from('ff',payload,68);
                self.posUncertainty*=10000
                mov_valid_data=payload[76] # bit 4 and 8 unused
                self.mov_valid_velX = (mov_valid_data & 1) == 1  
                self.mov_valid_velY = (mov_valid_data & 2) == 2 
                self.mov_valid_velZ = (mov_valid_data & 4) == 4
                self.mov_valid_posX = (mov_valid_data & 16) == 16 
                self.mov_valid_posY = (mov_valid_data & 32) == 32 
                self.mov_valid_posZ = (mov_valid_data & 64) == 64
                
                
                #LOGGER.debug('LogNewMvoFeedback: velX : %.2f velY :%.2f velZ : %.2f posX : %.2f posY : %.2f posZ : %.2f' % (self.velX,self.velY,self.velZ,self.posX,self.posY,self.posZ))
                
            elif id == self.ID_IMU_ATTI:
                #LOGGER.debug('LogImuAtti: length=%d %s' % (len(payload), byte_to_hexstring(payload)))
                (self.longitude,self.latitude,self.baro)= struct.unpack_from('ddf', payload, 0)
                (self.accX, self.accY, self.accZ) = struct.unpack_from('fff', payload, 20)
                (self.gyroX, self.gyroY, self.gyroZ,self.baro_smooth) = struct.unpack_from('ffff', payload, 32)
                (self.qW,self.qX,self.qY, self.qZ) = struct.unpack_from('ffff', payload, 48)
                (self.velN, self.velE, self.velD) = struct.unpack_from('fff', payload, 76)
                self.convertAngle()
            elif id == self.ID_IMU_EXT:
                #IMU extended with visual oddometry
                (self.velX_VO,self.velY_VO,self.velZ_VO)= struct.unpack_from('fff', payload, 0)
                (self.posX_VO,self.posY_VO,self.posZ_VO)= struct.unpack_from('fff', payload, 12)
                (self.vel__VO,self._dist_V0)= struct.unpack_from('ff', payload, 24)
                (self.rtkLong_VO,self.rtkLat_VO,self.rtkAlt_VO)= struct.unpack_from('ddf', payload, 32)
                self.error_flag_VO= struct.unpack_from('h', payload, 52)[0]#indicate if the vel & pos ar valid
                #print('self.error_flag_VO',self.error_flag_VO)
            else:
                if not id in self.unknowns_log_msg:
                    LOGGER.debug('LogData: UNHANDLED LOG DATA: id=%5d, length=%4d' % (id, length-12))
                    self.unknowns_log_msg.append(id)
            pos += length
            
        if pos != len(data) - 2:
            
            #raise Exception('LogData: corrupted data at pos=%d, data=%s'
            #            % (pos, byte_to_hexstring(data[pos:])))
            return
        #LOGGER.debug('*** End log packet processing')
        
    def convertAngle(self):
        # Convert quaternion to Euler
        sqW = self.qW * self.qW
        sqX = self.qX * self.qX
        sqY = self.qY * self.qY
        sqZ = self.qZ * self.qZ
     
                
        # if normalised is one, otherwise is correction factor
        unit = sqX + sqY + sqZ + sqW 
        test = self.qW *  self.qX +  self.qY *  self.qZ
        
        if test > 0.499 * unit:
            # singularity at north pole
            self.yaw = 2 * math.atan2(self.qY, self.qW)
            self.pitch = math.pi / 2
            self.roll = 0
        elif test < -0.499 * unit:
            # singularity at south pole
            self.yaw = -2 * math.atan2(self.qY, self.qW)
            self.pitch = -math.pi / 2
            self.roll = 0
        else:
            self.yaw = math.atan2(2.0 * (self.qW * self.qZ - self.qX * self.qY),
                    1.0 - 2.0 * (sqZ + sqX))
            if unit==0:
                self.roll=0.0
            else:
                self.roll = math.asin(2.0 * test / unit)
            self.pitch = math.atan2(2.0 * (self.qW * self.qY - self.qX * self.qZ),
                    1.0 - 2.0 * (sqY + sqX))
 
        
        
        
class DownloadedFile(object):
    def __init__(self, filenum, size):
        self.filenum = filenum
        self.size = size
        self.bytes_recieved = 0
        self.chunks_received = [0x00] * int((size / 1024 + 1) / 8 + 1)
        self.buffer = BytesIO()

    def done(self):
        return self.bytes_recieved >= self.size

    def data(self):
        return self.buffer.getvalue()

    def haveFragment(self, chunk, fragment):
        return self.chunks_received[chunk] & (1<<(fragment%8))

    def recvFragment(self, chunk, fragment, size, data):
        if self.haveFragment(chunk, fragment):
            return False
        # Mark a fragment as received.
        # Returns true if we have all fragments making up that chunk now.
        self.buffer.seek(fragment*1024)
        self.buffer.write(data)
        self.bytes_recieved += size
        self.chunks_received[chunk] |= (1<<(fragment%8))
        return self.chunks_received[chunk] == 0xFF


class VideoData(object):
    packets_per_frame = 0
    def __init__(self, data):
        self.h0 = byte(data[0])
        self.h1 = byte(data[1])
        if VideoData.packets_per_frame < (self.h1 & 0x7f):
            VideoData.packets_per_frame = (self.h1 & 0x7f)

    def gap(self, video_data):
        if video_data is None:
            return 0

        v0 = self
        v1 = video_data

        loss = 0
        if ((v0.h0 != v1.h0 and v0.h0 != ((v1.h0 + 1) & 0xff))
            or (v0.h0 != v1.h0 and (v0.h1 & 0x7f) != 00)
            or (v0.h0 == v1.h0 and (v0.h1 & 0x7f) != (v1.h1 & 0x7f) + 1)):
            loss = v0.h0 - v1.h0
            if loss < 0:
                loss = loss + 256
            loss = loss * VideoData.packets_per_frame + ((v0.h1 & 0x7f) - (v1.h1 & 0x7f) - 1)

        return loss
