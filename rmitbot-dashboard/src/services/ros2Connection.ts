import * as ROSLIB from "roslib";
import { ROS_CONFIG } from "@/config";
import type { TwistStampedMessage, OdometryMessage } from "@/types";

class RosService {
  private static instance: RosService;
  ros: ROSLIB.Ros;
  isConnected: boolean = false;
  currentIp: string = ROS_CONFIG.DEFAULT_IP;
  url: string = `ws://${ROS_CONFIG.DEFAULT_IP}:${ROS_CONFIG.ROSBRIDGE_PORT}`;
  reconnectInterval: number = ROS_CONFIG.RECONNECT_INTERVAL;
  private isIntentionalDisconnect: boolean = false;
  private connectionChangeCallbacks: Array<() => void> = [];

  private cmdVelPublisher: ROSLIB.Topic<TwistStampedMessage> | null = null;
  private odomSubscriber: ROSLIB.Topic<OdometryMessage> | null = null;
  private odomCallback: ((data: OdometryMessage) => void) | null = null;

  private constructor() {
    this.ros = new ROSLIB.Ros({ url: this.url });
    this.setupListeners();
  }

  public static getInstance(): RosService {
    if (!RosService.instance) {
      RosService.instance = new RosService();
    }
    return RosService.instance;
  }

  private setupListeners() {
    this.ros.on("connection", () => {
      console.log("Connected to ROS");
      this.isConnected = true;
      this.isIntentionalDisconnect = false;

      setTimeout(() => {
        this.initPublishers();
        this.initSubscribers();
      }, 500);
    });

    this.ros.on("close", () => {
      console.log("Disconnected from ROS");
      this.isConnected = false;
      this.cmdVelPublisher = null;
      this.odomSubscriber = null;

      if (!this.isIntentionalDisconnect) {
        setTimeout(() => this.connect(), this.reconnectInterval);
      }
    });

    this.ros.on("error", (error) => {
      console.error("ROS Error:", error);
    });
  }

  private initPublishers() {
    this.cmdVelPublisher = new ROSLIB.Topic({
      ros: this.ros,
      name: "/cmd_vel_keyboard",
      messageType: "geometry_msgs/TwistStamped",
    });
  }

  private initSubscribers() {
    this.odomSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: "/odom_ekf",
      messageType: "nav_msgs/Odometry",
    });

    try {
      this.odomSubscriber.subscribe((message: any) => {
        if (this.odomCallback) {
          this.odomCallback(message as OdometryMessage);
        }
      });
    } catch (error) {
      console.error("Subscriber init failed:", error);
    }
  }

  public setOdomCallback(callback: (data: OdometryMessage) => void) {
    this.odomCallback = callback;
  }

  public publishVelocity(
    linear_x: number,
    linear_y: number,
    angular_z: number
  ) {
    if (!this.isConnected || !this.cmdVelPublisher) {
      console.warn("Cannot publish velocity - not connected or publisher not initialized");
      return;
    }

    const twistStamped: TwistStampedMessage = {
      header: {
        stamp: { sec: 0, nanosec: 0 },
        frame_id: "base_footprint",
      },
      twist: {
        linear: { x: linear_x, y: linear_y, z: 0 },
        angular: { x: 0, y: 0, z: angular_z },
      },
    };

    try {
      this.cmdVelPublisher.publish(twistStamped);
      console.log(`Published velocity - linear: [${linear_x.toFixed(2)}, ${linear_y.toFixed(2)}, 0], angular: [0, 0, ${angular_z.toFixed(2)}]`);
    } catch (error) {
      console.error("Publish failed:", error);
    }
  }

  connect() {
    this.isIntentionalDisconnect = false;
    if (this.isConnected) return;
    try {
      this.ros.connect(this.url);
    } catch (e) {
      console.error("Connection failed:", e);
    }
  }

  disconnect() {
    this.isIntentionalDisconnect = true;
    this.ros.close();
  }

  switchRobot(newIp: string) {
    if (newIp === this.currentIp) return;

    this.isIntentionalDisconnect = true;
    this.ros.close();

    this.currentIp = newIp;
    this.url = `ws://${newIp}:${ROS_CONFIG.ROSBRIDGE_PORT}`;

    this.cmdVelPublisher = null;
    this.odomSubscriber = null;

    this.ros = new ROSLIB.Ros({ url: this.url });
    this.setupListeners();

    this.connectionChangeCallbacks.forEach(cb => cb());

    this.isIntentionalDisconnect = false;
    this.connect();
  }

  onConnectionChange(callback: () => void) {
    this.connectionChangeCallbacks.push(callback);
    return () => {
      this.connectionChangeCallbacks = this.connectionChangeCallbacks.filter(cb => cb !== callback);
    };
  }

  getROS() {
    return this.ros;
  }
}

export const rosService = RosService.getInstance();
