from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from common.conversions import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits, apply_toyota_steer_torque_limits
from selfdrive.car.landrover.landrovercan import create_lkas_command, create_lkas_hud
from selfdrive.car.landrover.values import CarControllerParams, CAR, STATIC_MSGS
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):
  sys_warning = (visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.p = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)

    self.apply_steer_last = 0
    self.ccframe = 0
    self.CP = CP
    self.steer_rate_limited = False
    self.last_resume_frame = 0
    self.accel = 0

    self.controls_allowed = True
    self.apply_steer_last = 0
    self.lkascnt = 0
    self.prev_frame = -1
    self.hud_count = 0
    self.alert_active = False
    self.send_chime = False
    self.gone_fast_yet = False


  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed,
             left_lane, right_lane, left_lane_depart, right_lane_depart):

    # Steering Torque
    new_steer = int(round(actuators.steer * 1024))
    #new_steer = int(round(actuators.steer * self.p.STEER_MAX))
    #new_steer = int(round(actuators.steer * 150))
    #apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    # disable when temp fault is active, or below LKA minimum speed
    lkas_active = enabled and not CS.out.steerFaultTemporary and CS.out.vEgo >= CS.CP.minSteerSpeed
    if CS.leftBlinker or CS.rightBlinker:
      apply_steer = 0

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    #sys_warning, sys_state, left_lane_warning, right_lane_warning = \
    #  process_hud_alert(enabled, self.CP.carFingerprint, visual_alert,
    #                    left_lane, right_lane, left_lane_depart, right_lane_depart)

    can_sends = []

    for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
       if(frame % fr_step == 0):  # 25 0.25s period
           can_sends.append([addr, bus, vl, 0])

    if (self.ccframe % 20 == 0):
       can_sends.append(create_lkas_hud(self.packer, CS.lkas_status, left_lane, right_lane, left_lane_depart, right_lane_depart ))
       #can_sends.append(create_lkas_hud(self.packer, CS.lkas_status, enabled, frame))

    if (self.ccframe % 2 == 0):
       new_msg = create_lkas_command(self.packer, CS.lkas_run, self.lkascnt, int(apply_steer))
       self.lkascnt += 1
       #cloudlog.warning("LANDROVER LKAS (%s)", new_msg)
       can_sends.append(new_msg)

    self.ccframe += 1
    self.prev_frame = frame

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.p.STEER_MAX

    return new_actuators, can_sends
