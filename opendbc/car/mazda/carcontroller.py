from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams, Buttons

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.hold_timer = ControlsTimer(6.0)
    self.hold_delay = ControlsTimer(.5) # delay before we start holding as to not hit the brakes too hard

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams)

    if CC.cruiseControl.cancel:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if CC.cruiseControl.resume and self.frame % 5 == 0:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    self.apply_torque_last = apply_torque

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      #steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      #steer_required = steer_required and CS.lkas_allowed_speed
      steer_required = CS.out.steerFaultTemporary
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send acc commands
    if self.CP.openpilotLongitudinalControl:
      hold = False
      accel_cmd = 4094
      if CS.out.standstill:
        hold = self.hold_timer.active()
      else:
        self.hold_timer.reset()

        accel_cmd = CC.actuators.accel * 1150
        accel_cmd = max(-1000, min(accel_cmd, 1000))

      if self.frame % 2 == 0:
        can_sends.extend(mazdacan.create_acc_command(self.packer, self.CP, CS, self.frame, CC.longActive, hold, accel_cmd))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    ControlsTimer.tick()
    return new_actuators, can_sends


class DurationTimer:
  def __init__(self, duration=0, step=0.01) -> None:
    self.step = step
    self.duration = duration
    self.was_reset = False
    self.timer = 0
    self.min = float("-inf") # type: float
    self.max = float("inf") # type: float

  def tick_obj(self) -> None:
    self.timer += self.step
    # reset on overflow
    self.timer = 0 if (self.timer == (self.max or self.min)) else self.timer

  def reset(self) -> None:
    """Resets this objects timer"""
    self.timer = 0
    self.was_reset = True

  def active(self) -> bool:
    """Returns true if time since last reset is less than duration"""
    return bool(round(self.timer,2) < self.duration)

  def adjust(self, duration) -> None:
    """Adjusts the duration of the timer"""
    self.duration = duration

  def once_after_reset(self) -> bool:
    """Returns true only one time after calling reset()"""
    ret = self.was_reset
    self.was_reset = False
    return ret

  @staticmethod
  def interval_obj(rate, frame) -> bool:
    if frame % rate == 0: # Highlighting shows "frame" in white
      return True
    return False


class ModelTimer(DurationTimer):
  frame: int = 0
  objects: list = []

  def __init__(self, duration=0) -> None:
    self.step = 0.05
    super().__init__(duration, self.step)
    self.__class__.objects.append(self)

  @classmethod
  def tick(cls) -> None:
    cls.frame += 1
    for obj in cls.objects:
      ModelTimer.tick_obj(obj)

  @classmethod
  def reset_all(cls) -> None:
    for obj in cls.objects:
      obj.reset()

  @classmethod
  def interval(cls, rate) -> bool:
    return ModelTimer.interval_obj(rate, cls.frame)


class ControlsTimer(DurationTimer):
  frame = 0
  objects = [] # type: list[DurationTimer]

  def __init__(self, duration=0) -> None:
    self.step = 0.01
    super().__init__(duration=duration, step=self.step)
    self.__class__.objects.append(self)

  @classmethod
  def tick(cls) -> None:
    cls.frame += 1
    for obj in cls.objects:
      ControlsTimer.tick_obj(obj)

  @classmethod
  def reset_all(cls) -> None:
    for obj in cls.objects:
      obj.reset()

  @classmethod
  def interval(cls, rate) -> bool:
    return ControlsTimer.interval_obj(rate, cls.frame)
