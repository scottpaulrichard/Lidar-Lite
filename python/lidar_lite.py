import smbus
import time

#http:#static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf
# The simplest method of obtaining measurement results from the I2C interface is as follows:
# 1 Write 0x04 to register 0x00.
# 2 Read register 0x01. Repeat until bit 0 (LSB) goes low.
# 3 Read two bytes from 0x8f (High byte 0x0f then low byte 0x10) to obtain the 16-bit measured distance in centimeter

class Lidar_Lite():
    def __init__(self,config_mode=0):
    self.config_mode = config_mode
    self.distance = 0
    self.recvbiascount = 0
    # LIDAR-LITE default address 
    self.LLv3_DEFAULT_ADDRESS = 0x62
    # LIDAR-LITE write values
    self.LLv3_RESET_ALL_DEFAULT  = 0x00
    self.LLv3_VEL_WRITE          = 0x08
    self.LLv3_DIST_WRITE_WO_BIAS = 0x03
    self.LLv3_DIST_WRITE_W_BIAS  = 0x04
    # LIDAR-Lite internal register addresses
    self.LLv3_ACQ_CMD       = 0x00
    self.LLv3_STATUS        = 0x01
    self.LLv3_SIG_CNT_VAL   = 0x02
    self.LLv3_ACQ_CONFIG    = 0x04
    self.LLv3_VELOCITY      = 0x09
    self.LLv3_DISTANCE      = 0x8f
    self.LLv3_DISTANCE_HIGH = 0x0f
    self.LLv3_DISTANCE_LOW  = 0x10
    self.LLv3_REF_CNT_VAL   = 0x12
    self.LLv3_UNIT_ID_HIGH  = 0x16
    self.LLv3_UNIT_ID_LOW   = 0x17
    self.LLv3_I2C_ID_HIGH   = 0x18
    self.LLv3_I2C_ID_LOW    = 0x19
    self.LLv3_I2C_SEC_ADR   = 0x1a
    self.LLv3_THRESH_BYPASS = 0x1c
    self.LLv3_I2C_CONFIG    = 0x1e
    self.LLv3_COMMAND       = 0x40
    self.LLv3_CORR_DATA     = 0x52
    self.LLv3_ACQ_SETTINGS  = 0x5d

  def configure(self):
      '''
      ------------------------------------------------------------------------------
        Configure
        Selects one of several preset configurations.
        Parameters
        ------------------------------------------------------------------------------
        configuration:  Default 0.
          0: Default mode, balanced performance.
          1: Short range, high speed. Uses 0x1d maximum acquisition count.
          2: Default range, higher speed short range. Turns on quick termination
              detection for faster measurements at short range (with decreased
              accuracy)
          3: Maximum range. Uses 0xff maximum acquisition count.
          4: High sensitivity detection. Overrides default valid measurement detection
              algorithm, and uses a threshold value for high sensitivity and noise.
          5: Low sensitivity detection. Overrides default valid measurement detection
              algorithm, and uses a threshold value for low sensitivity and noise.
        lidarliteAddress: Default 0x62. Fill in new address here if changed. See
          operating manual for instructions.
      ------------------------------------------------------------------------------
      '''

      if self.config_mode == 0: # Default mode, balanced performance
        sigCountMax     = 0x80 # Default
        acqConfigReg    = 0x08 # Default
        refCountMax     = 0x05 # Default
        thresholdBypass = 0x00 # Default

      elif self.config_mode == 1: # Short range, high speed
        sigCountMax     = 0x1d
        acqConfigReg    = 0x08 # Default
        refCountMax     = 0x03
        thresholdBypass = 0x00 # Default

      elif self.config_mode == 2: # Default range, higher speed short range
        sigCountMax     = 0x80 # Default
        acqConfigReg    = 0x00
        refCountMax     = 0x03
        thresholdBypass = 0x00 # Default

      elif self.config_mode == 3: # Maximum range
        sigCountMax     = 0xff
        acqConfigReg    = 0x08 # Default
        refCountMax     = 0x05 # Default
        thresholdBypass = 0x00 # Default

      elif self.config_mode == 4: # High sensitivity detection, high erroneous measurements
        sigCountMax     = 0x80 # Default
        acqConfigReg    = 0x08 # Default
        refCountMax     = 0x05 # Default
        thresholdBypass = 0x80

      elif self.config_mode == 5: # Low sensitivity detection, low erroneous measurements
        sigCountMax     = 0x80 # Default
        acqConfigReg    = 0x08 # Default
        refCountMax     = 0x05 # Default
        thresholdBypass = 0xb0

      elif self.config_mode == 6: # Short range, high speed, higher error
        sigCountMax     = 0x04
        acqConfigReg    = 0x01 # turn off short_sig, mode pin = status output mode
        refCountMax     = 0x03
        thresholdBypass = 0x00

      else: # Default mode, balanced performance - same as configure(0)
        sigCountMax     = 0x80 # Default
        acqConfigReg    = 0x08 # Default
        refCountMax     = 0x05 # Default
        thresholdBypass = 0x00 # Default

      self.bus.write_byte_data(self.LLv3_DEFAULT_ADDRESS, self.LLv3_SIG_CNT_VAL, sigCountMax)
      self.bus.write_byte_data(self.LLv3_DEFAULT_ADDRESS, self.LLv3_ACQ_CONFIG, acqConfigReg)
      self.bus.write_byte_data(self.LLv3_DEFAULT_ADDRESS, self.LLv3_REF_CNT_VAL, refCountMax)
      self.bus.write_byte_data(self.LLv3_DEFAULT_ADDRESS, self.LLv3_THRESH_BYPASS, thresholdBypass)

  def connect(self, bus=0):
    # bus 0 indicates /dev/i2c-0
    try:
      self.bus = smbus.SMBus(bus)
      time.sleep(0.5)
      self.configure()
      return 0
    except:
      return -1

  def waitForBusy(self):
    while(True):
      res = self.bus.read_byte_data(self.LLv3_DEFAULT_ADDRESS, self.LLv3_STATUS)
      # least significant bit should be zero
      #
      #   01010101   <---- res
      # & 00000001
      # ----------
      #          1
      #
      #   01010100   <---- res
      # & 00000001
      # ----------
      #          0
      #
      if(res & 1 == 0):
        break

  def writeAndWait(self, register, value):
    self.bus.write_byte_data(self.LLv3_DEFAULT_ADDRESS, register, value)
    self.waitForBusy()

  def read(self, register):
    res = self.bus.read_byte_data(self.LLv3_DEFAULT_ADDRESS, register)
    return res

  def readDist(self, register):
    res = self.bus.read_i2c_block_data(self.LLv3_DEFAULT_ADDRESS, register, 2)
    return (res[0] << 8 | res[1])

  def getDistance(self):
    # every 100 measurements take one with recv bias correction
    if(self.recvbiascount > 99):
        self.recvbiascount = 0
        self.writeAndWait(self.LLv3_ACQ_CMD, 0x04)
    else:
        self.recvbiascount += 1
        self.writeAndWait(self.LLv3_ACQ_CMD, 0x03)
    dist = self.readDist(self.LLv3_DISTANCE)
    return dist

  def getVelocity(self):
    self.distance = self.getDistance()
    self.writeAndWait(self.LLv3_ACQ_CONFIG, self.LLv3_VEL_WRITE)
    vel = self.read(self.LLv3_VELOCITY)
    return self.signedInt(vel)

  def signedInt(self, value):
    if value > 127:
      return (256-value) * (-1)
    else:
      return value


