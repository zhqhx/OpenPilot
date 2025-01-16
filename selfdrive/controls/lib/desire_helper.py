from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL

from openpilot.common.params import Params

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}
TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

def calculate_lane_width_frog(lane, current_lane, road_edge):
  lane_x, lane_y = np.array(lane.x), np.array(lane.y)
  edge_x, edge_y = np.array(road_edge.x), np.array(road_edge.y)
  current_x, current_y = np.array(current_lane.x), np.array(current_lane.y)

  lane_y_interp = np.interp(current_x, lane_x[lane_x.argsort()], lane_y[lane_x.argsort()])
  road_edge_y_interp = np.interp(current_x, edge_x[edge_x.argsort()], edge_y[edge_x.argsort()])

  distance_to_lane = np.mean(np.abs(current_y - lane_y_interp))
  distance_to_road_edge = np.mean(np.abs(current_y - road_edge_y_interp))

  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge

def calculate_lane_width(lane, lane_prob, current_lane, road_edge):
  index = 10 # 약 1초 앞의 차선.
  distance_to_lane = abs(current_lane.y[index] - lane.y[index])
  #if lane_prob < 0.3:# 차선이 없으면 없는것으로 간주시킴.
  #  distance_to_lane = min(2.0, distance_to_lane)
  distance_to_road_edge = abs(current_lane.y[index] - road_edge.y[index]);
  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge, lane_prob > 0.5

class ExistCounter:
  def __init__(self):
    self.counter = 0
    self.true_count = 0
    self.false_count = 0
    self.threshold = int(0.2 / DT_MDL)  # 노이즈를 무시하기 위한 임계값 설정

  def update(self, exist_flag):
    if exist_flag:
      self.true_count += 1
      self.false_count = 0  # false count 초기화
      if self.true_count >= self.threshold:
          self.counter = max(self.counter + 1, 1)
    else:
      self.false_count += 1
      self.true_count = 0  # true count 초기화
      if self.false_count >= self.threshold:
          self.counter = min(self.counter - 1, -1)

    return self.true_count

class DesireHelper:
  def __init__(self):
    self.params = Params()
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.turn_direction = TurnDirection.none
    self.enable_turn_desires = True
    self.atc_active = 0
    self.desireLog = ""
    self.lane_width_left = 0
    self.lane_width_right = 0
    self.distance_to_road_edge_left = 0
    self.distance_to_road_edge_right = 0
    self.blinker_ignore = False

    self.lane_exist_left_count = ExistCounter()
    self.lane_exist_right_count = ExistCounter()
    self.lane_width_left_count = ExistCounter()
    self.lane_width_right_count = ExistCounter()
    self.road_edge_left_count = ExistCounter()
    self.road_edge_right_count = ExistCounter()
    self.available_left_lane = False
    self.available_right_lane = False
    self.available_left_edge = False
    self.available_right_edge = False

    self.lane_available_last = False
    self.edge_available_last = False
    self.object_detected_count = 0

    self.laneChangeNeedTorque = False
    
  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan, radarState):

    self.laneChangeNeedTorque = self.params.get_bool("LaneChangeNeedTorque")


    
    v_ego = carstate.vEgo
    #one_blinker = carstate.leftBlinker != carstate.rightBlinker
    leftBlinker = carstate.leftBlinker
    rightBlinker = carstate.rightBlinker
    one_blinker = leftBlinker != rightBlinker
    driver_one_blinker = one_blinker
    
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if not leftBlinker and not rightBlinker:
      atc_type = carrotMan.atcType
      if atc_type in ["turn left", "turn right"]:
        if self.atc_active != 2:
          below_lane_change_speed = True
          self.lane_change_timer = 0.0
          leftBlinker = True if atc_type == "turn left" else False
          rightBlinker = not leftBlinker          
          self.atc_active = 1
          self.blinker_ignore = False
      elif atc_type in ["fork left", "fork right"]:
        if self.atc_active != 2:
          below_lane_change_speed = False
          leftBlinker = True if atc_type == "fork left" else False
          rightBlinker = not leftBlinker
          self.atc_active = 1
      else:
        self.atc_active = 0
      one_blinker = leftBlinker != rightBlinker
    if not one_blinker:
      self.blinker_ignore = False
    one_blinker &= not self.blinker_ignore
    
    self.lane_width_left, self.distance_to_road_edge_left, lane_prob_left = calculate_lane_width(modeldata.laneLines[0], modeldata.laneLineProbs[0], modeldata.laneLines[1], modeldata.roadEdges[0])
    self.lane_width_right, self.distance_to_road_edge_right, lane_prob_right = calculate_lane_width(modeldata.laneLines[3], modeldata.laneLineProbs[3], modeldata.laneLines[2], modeldata.roadEdges[1])
    self.lane_exist_left_count.update(lane_prob_left)
    self.lane_exist_right_count.update(lane_prob_right)
    min_lane_width = 2.5
    self.lane_width_left_count.update(self.lane_width_left > min_lane_width)
    self.lane_width_right_count.update(self.lane_width_right > min_lane_width)
    self.road_edge_left_count.update(self.distance_to_road_edge_left > min_lane_width)
    self.road_edge_right_count.update(self.distance_to_road_edge_right > min_lane_width)
    available_count = int(0.2 / DT_MDL)
    self.available_left_lane = self.lane_width_left_count.counter > available_count
    self.available_right_lane = self.lane_width_right_count.counter > available_count
    self.available_left_edge = self.road_edge_left_count.counter > available_count
    self.available_right_edge = self.road_edge_right_count.counter > available_count

    if one_blinker:
      lane_available = self.available_left_lane if leftBlinker else self.available_right_lane
      edge_available = self.available_left_edge if leftBlinker else self.available_right_edge
      lane_appeared = self.lane_exist_left_count.counter == int(0.2 / DT_MDL) if leftBlinker else self.lane_exist_right_count.counter == int(0.2 / DT_MDL)

      radar = radarState.leadLeft if leftBlinker else radarState.leadRight
      side_object_dist = radar.dRel + radar.vLead * 4.0 if radar.status else 255
      object_detected = side_object_dist < v_ego * 3.0
      self.object_detected_count = max(1, self.object_detected_count + 1) if object_detected else min(-1, self.object_detected_count - 1)
      
    else:
      lane_available = True
      edge_available = True
      lane_appeared = False
      self.object_detected_count = 0

    auto_lane_change_blocked = leftBlinker
    lane_availabled = not self.lane_available_last and lane_available
    edge_availabled = not self.edge_available_last and edge_available
    side_object_detected = self.object_detected_count > -0.3 / DT_MDL
    auto_lane_change_available = not auto_lane_change_blocked and lane_availabled and edge_availabled and not side_object_detected

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
    elif one_blinker and below_lane_change_speed and not carstate.standstill and self.enable_turn_desires:
      self.lane_change_state = LaneChangeState.off
      self.turn_direction = TurnDirection.turnLeft if leftBlinker else TurnDirection.turnRight
      self.lane_change_direction = self.turn_direction #LaneChangeDirection.none
    else:
      self.turn_direction = TurnDirection.none     
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif not blindspot_detected:
          if self.laneChangeNeedTorque:
            if torque_applied:
              self.lane_change_state = LaneChangeState.laneChangeStarting
          # 운전자가 깜박이켠경우는 바로 차선변경 시작
          elif driver_one_blinker and lane_available:
            self.lane_change_state = LaneChangeState.laneChangeStarting
          # ATC작동인경우 차선이 나타나거나 차선이 생기면 차선변경 시작
          # lane_appeared: 차선이 생기는건 안함.. 위험.  
          elif torque_applied or auto_lane_change_available:
            self.lane_change_state = LaneChangeState.laneChangeStarting

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL


    self.lane_available_last = lane_available
    self.edge_available_last = edge_available
    
    self.prev_one_blinker = one_blinker
    steering_pressed = carstate.steeringPressed and \
                     ((carstate.steeringTorque < 0 and leftBlinker) or (carstate.steeringTorque > 0 and rightBlinker))
    if steering_pressed and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_ignore = True

    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    self.desireLog = f"desire = {self.desire}"
      
    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
