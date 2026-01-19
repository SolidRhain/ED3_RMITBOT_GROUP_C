export const ROSBRIDGE_PORT = 9090;

export const ROBOT_OPTIONS = [
  { ip: "100.118.27.83", name: "RPI" },
  { ip: "100.68.218.48", name: "Ubuntu Lesson 6" },
  { ip: "100.114.20.41", name: "Triet" },
  { ip: "100.75.217.119", name: "Hoa" },
  { ip: "100.91.20.95", name: "Loc" },
] as const;

const DEFAULT_ROBOT_IP = ROBOT_OPTIONS[0].ip;

export const ROS_CONFIG = {
  DEFAULT_IP: DEFAULT_ROBOT_IP,
  ROSBRIDGE_PORT,
  RECONNECT_INTERVAL: 3000,
  SPEED: 0.5,
  TURN: 1.0,
  PUBLISH_RATE: 10,
};
