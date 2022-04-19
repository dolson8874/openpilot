import copy
from cereal import car
from selfdrive.car.landrover.values import DBC, STEER_THRESHOLD
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV


def get_can_parser_landrover(CP):
  signals = [
    # sig_name, sig_address, default
    ("STEER_RATE00", "EPS_00", 0),
    ("STEER_ANGLE01", "EPS_01", 0),
    ("STEER_SPEED01", "EPS_01", 0),
    ("STEER_TORQUE_DRIVER02", "EPS_02", 0),
    ("STEER_TORQUE_MOTOR02", "EPS_02", 0),
    ("STEER_TORQUE_DRIVER03", "EPS_03", 0),
    ("STEER_TQ", "EPS_04", 0),
    ("GEAR_SHIFT", "GEAR_PRND", 0),
    ("CRUISE_ON", "CRUISE_CONTROL", 0),
    ("DRIVER_BRAKE", "CRUISE_CONTROL", 0),
    ("SPEED_CRUISE_RESUME", "CRUISE_CONTROL", 1),
    ("ACCELATOR_DRIVER", "ACCELATOR_DRIVER", 0),
    ("SEAT_BELT_DRIVER", "SEAT_BELT", 0),
    ("RIGHT_TURN", "TURN_SIGNAL", 0),
    ("LEFT_TURN", "TURN_SIGNAL", 0),
    ("LEFT_BLINK", "TURN_SIGNAL", 0),
    ("RIGHT_BLINK", "TURN_SIGNAL", 0),
    ("COUNTER", "LKAS_RUN", -1),
    ("SPEED01", "SPEED_01", 0),
    ("SPEED02", "SPEED_02", 0),
    ("LEFT_ALERT_1", "LEFT_ALERT", 0),
    ("RIGHT_ALERT_1", "RIGHT_ALERT", 0),
    ("WHEEL_SPEED_FR", "SPEED_04", 0),
    ("WHEEL_SPEED_FL", "SPEED_04", 0),
    ("WHEEL_SPEED_RR", "SPEED_03", 0),
    ("WHEEL_SPEED_RL", "SPEED_03", 0),
    ("GREEN2WHITE_RIGHT", "LKAS_STATUS", 1),
    ("GREEN2WHITE_LEFT", "LKAS_STATUS", 1),
    ("FRONT_CAR_DISTANCE", "LKAS_HUD_STAT", 0),
    ("LADAR_DISTANCE", "LKAS_HUD_STAT", 0),
    ("HIBEAM", "HEAD_LIGHT", 0),
  ]

# It's considered invalid if it is not received for 10x the expected period (1/f).
  checks = [
    # sig_address, frequency
    ("EPS_00", 0),
    ("EPS_01", 0),
    ("EPS_02", 0),
    ("EPS_03", 0),
    ("EPS_04",  0),
    ("SPEED_01", 0),
    ("SPEED_02", 0),
    ("SPEED_03", 0),
    ("SPEED_04", 0),
    ("CRUISE_CONTROL", 1),
    ("SEAT_BELT", 0),
    ("LKAS_HUD_STAT", 0),
    ("HEAD_LIGHT", 0),
    ("LKAS_STATUS", 0),
  ]

  return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

def get_cam_can_parser_landrover(CP):
    signals = [
      # sig_name, sig_address, default
      # TODO read in all the other values
      ( "ALLFFFF", "LKAS_RUN", 0),
      ( "A1", "LKAS_RUN", 0),
      ( "HIGH_TORQ", "LKAS_RUN", 0),
      ( "ALL11", "LKAS_RUN", 0),
      ( "COUNTER", "LKAS_RUN", -1),
      ( "STEER_TORQ", "LKAS_RUN", 0),
      ( "LKAS_GREEN", "LKAS_RUN", 0),
    ]

    checks = []

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)

def parse_gear_shifter(can_gear):
  if can_gear == 0x0:
    return "P"
  elif can_gear == 0x9:
    return "R"
  elif can_gear == 0x12:
    return "N"
  elif can_gear == 0x1b:
    return "D"
  elif can_gear == 0xbb:
    return "S"
  return "unknown"


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["GEAR_PRND"]["GEAR_SHIFT"]

    self.left_blinker_on = 0
    self.right_blinker_on = 0
    self.left_alert = 0
    self.right_alert = 0

    self.angle_steers = 0.0

    self.shifter_values = can_define.dv["GEAR_PRND"]["GEAR_SHIFT"]

    self.brake_error = False
    self.park_brake = False

    self.prev_angle_steers = 0.0
    self.angle_rate_multi = 1.0

    self.v_wheel = 0
    self.angle_steers_diff = 0

  @staticmethod
  def get_can_parser(CP):
     return get_can_parser_landrover(CP)

  @staticmethod
  def get_cam_can_parser(CP):
     return get_cam_can_parser_landrover(CP)


  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    self.prev_angle_steers = float(self.angle_steers)
    ret.gas = 0
    ret.gasPressed = ret.gas > 1e-3
    ret.brakePressed = (cp.vl["CRUISE_CONTROL"]['DRIVER_BRAKE'] == 1)
    #ret.doorOpen = 0
    ret.seatbeltUnlatched = (cp.vl["SEAT_BELT"]["SEAT_BELT_DRIVER"]  == 0)

    gear = cp.vl["GEAR_PRND"]["GEAR_SHIFT"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    self.v_wheel = (cp.vl["SPEED_01"]["SPEED01"] + cp.vl["SPEED_02"]["SPEED02"]) / 2.

    ret.vEgoRaw = self.v_wheel # * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not self.v_wheel > 0.001

    ret.steeringAngleDeg = cp.vl["EPS_01"]["STEER_ANGLE01"]
    ret.steeringRateDeg = cp.vl["EPS_01"]["STEER_SPEED01"]
    ret.steeringTorque = cp.vl["EPS_02"]["STEER_TORQUE_DRIVER02"]
    ret.steeringTorqueEps = cp.vl["EPS_02"]["STEER_TORQUE_MOTOR02"] / 10.  # scale to Nm
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # HANIL for landrover steers rate
    """
    self.angle_steers_diff = float(ret.steeringAngleDeg - self.prev_angle_steers)

    if self.angle_steers_diff < 0.0:
           self.angle_rate_multi = -4
    else:
        if self.angle_steers_diff > 0.0:
           self.angle_rate_multi = 4

    ret.steeringRateDeg *= self.angle_rate_multi
    """

    # self.steer_override = False # abs(self.steer_torque_driver) > STEER_THRESHOLD
    steer_state = 1 #cp.vl[""]["LKAS_STATE"]
    self.steer_error = steer_state == 4 or (steer_state == 0 and self.v_ego > self.CP.minSteerSpeed)


    #ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["TURN_SIGNAL"]['LEFT_TURN'],cp.vl["TURN_SIGNAL"]['RIGHT_TURN'])
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["TURN_SIGNAL"]['LEFT_BLINK'],cp.vl["TURN_SIGNAL"]['RIGHT_BLINK'])

    ret.cruiseState.available = True
    ret.cruiseState.enabled = cp.vl["CRUISE_CONTROL"]["CRUISE_ON"] == 1
    ret.cruiseState.standstill = False

    #speed_conv = CV.MPH_TO_MS  CV.KPH_TO_MS
    ret.cruiseState.speed = round(cp.vl["CRUISE_CONTROL"]['SPEED_CRUISE_RESUME']) * CV.KPH_TO_MS

    self.lkas_run = copy.copy(cp_cam.vl["LKAS_RUN"])
    self.lkas_status = copy.copy(cp_cam.vl["LKAS_STATUS"])

    self.lkas_counter = cp_cam.vl["LKAS_RUN"]['COUNTER']

    return ret
