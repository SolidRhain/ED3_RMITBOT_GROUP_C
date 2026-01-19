export type SpeedMode = "eco" | "normal" | "fast";
export type NavigationStatus = "idle" | "navigating" | "completed" | "failed";

export interface SavedPosition {
  id: string;
  name: string;
  x: number;
  y: number;
  theta: number;
  timestamp: number;
}

export interface RobotPose {
  x: number;
  y: number;
  theta: number;
}

export interface RobotState {
  batteryLevel: number;
  speed: number;
  currentPose: RobotPose;
  navigationStatus: NavigationStatus;
}

export interface TwistMessage {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

export interface TwistStampedMessage {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  twist: TwistMessage;
}

export interface OdometryMessage {
  header: {
    seq?: number;
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  child_frame_id: string;
  pose: {
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
    covariance?: number[];
  };
  twist: {
    twist: {
      linear: { x: number; y: number; z: number };
      angular: { x: number; y: number; z: number };
    };
    covariance?: number[];
  };
}

export interface NavigateThroughPositionsGoal {
  position_names: string[];
}
export interface NavigateThroughPositionsFeedback {
  current_position_index: number;
  current_position_name: string;
  progress_percentage: number;
}
export interface NavigateThroughPositionsResult {
  success: boolean;
  message: string;
  positions_reached: number;
}