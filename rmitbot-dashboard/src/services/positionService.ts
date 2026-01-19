import * as ROSLIB from "roslib";
import { rosService } from "./ros2Connection";
import type {
  SavedPosition,
  RobotPose,
  NavigateThroughPositionsGoal,
  NavigateThroughPositionsFeedback,
  NavigateThroughPositionsResult,
} from "@/types";

class PositionService {
  private static instance: PositionService;
  private currentMultiGoal: ROSLIB.Goal<
    NavigateThroughPositionsGoal,
    NavigateThroughPositionsFeedback,
    NavigateThroughPositionsResult
  > | null = null;

  private constructor() {}

  public static getInstance(): PositionService {
    if (!PositionService.instance) {
      PositionService.instance = new PositionService();
    }
    return PositionService.instance;
  }

  public cancelNavigation(): void {
    console.log("Cancelling navigation...");
    try {
      if (this.currentMultiGoal) {
        console.log("Cancelling multi goal");
        this.currentMultiGoal.cancel();
      }
    } catch (e) {
      console.warn("Failed to cancel navigation:", e);
    } finally {
      this.currentMultiGoal = null;
    }
  }

  // Save current position (Service)
  async savePosition(
    name: string
  ): Promise<{ success: boolean; message: string }> {
    return new Promise((resolve, reject) => {
      const ros = rosService.getROS();

      if (!rosService.isConnected) {
        console.error("Cannot save position: ROS not connected");
        reject(new Error("ROS not connected"));
        return;
      }

      console.log(`Saving position: ${name}`);

      const savePositionService = new ROSLIB.Service({
        ros: ros,
        name: "/save_position",
        serviceType: "position_manager_msgs/srv/SavePosition",
      });

      const request = { name };

      // Add timeout for service call
      let responded = false;
      const timeout = setTimeout(() => {
        if (!responded) {
          console.error("Save position service timeout");
          reject(new Error("Service call timeout - is the ROS2 node running?"));
        }
      }, 10000);

      savePositionService.callService(
        request,
        (response: any) => {
          responded = true;
          clearTimeout(timeout);
          console.log("Save response:", response);

          resolve({
            success: response.success,
            message: response.message || "Unknown response",
          });
        },
        (error: any) => {
          responded = true;
          clearTimeout(timeout);
          console.error("Save position service error:", error);
          reject(error);
        }
      );
    });
  }

  // Get all positions (Service)
  async getPositions(): Promise<SavedPosition[]> {
    return new Promise((resolve, reject) => {
      const ros = rosService.getROS();

      if (!rosService.isConnected) {
        console.error("Cannot get positions: ROS not connected");
        reject(new Error("ROS not connected"));
        return;
      }

      console.log("Getting positions...");

      const getPositionsService = new ROSLIB.Service({
        ros: ros,
        name: "/get_positions",
        serviceType: "position_manager_msgs/srv/GetPositions",
      });

      const request = {};

      let responded = false;
      const timeout = setTimeout(() => {
        if (!responded) {
          console.error("Get positions service timeout");
          reject(new Error("Service call timeout"));
        }
      }, 10000);

      getPositionsService.callService(
        request,
        (response: any) => {
          responded = true;
          clearTimeout(timeout);
          console.log("Get positions response:", response);

          const names = response.names || [];
          const x_coords = response.x_coords || [];
          const y_coords = response.y_coords || [];
          const theta_coords = response.theta_coords || [];

          if (!Array.isArray(names)) {
            console.error("Invalid response structure: names is not an array");
            reject(new Error("Invalid response from get_positions service"));
            return;
          }

          const positions: SavedPosition[] = names.map(
            (name: string, index: number) => ({
              id: `pos-${index}`,
              name,
              x: x_coords[index] || 0,
              y: y_coords[index] || 0,
              theta: theta_coords[index] || 0,
              timestamp: Date.now(),
            })
          );
          resolve(positions);
        },
        (error: any) => {
          responded = true;
          clearTimeout(timeout);
          console.error("Get positions service error:", error);
          reject(error);
        }
      );
    });
  }

  // Delete position (Service)
  async deletePosition(name: string): Promise<boolean> {
    return new Promise((resolve, reject) => {
      const ros = rosService.getROS();

      if (!rosService.isConnected) {
        reject(new Error("ROS not connected"));
        return;
      }

      console.log(`Deleting position: ${name}`);

      const deletePositionService = new ROSLIB.Service({
        ros: ros,
        name: "/delete_position",
        serviceType: "position_manager_msgs/srv/DeletePosition",
      });

      const request = { name };

      deletePositionService.callService(
        request,
        (response: any) => {
          console.log("Delete response:", response);
          resolve(response.success);
        },
        (error: any) => {
          console.error("Delete position failed:", error);
          reject(error);
        }
      );
    });
  }

  // Navigate through multiple positions (call Service Wrapper)
  async navigateThroughPositions(
    names: string[],
    onFeedback?: (status: string) => void
  ): Promise<{
    success: boolean;
    message: string;
    positions_reached?: number;
  }> {
    return new Promise((resolve, reject) => {
      const ros = rosService.getROS();

      if (!rosService.isConnected) {
        reject(new Error("ROS not connected"));
        return;
      }

      console.log(
        `====== Starting multi-navigation: ${names.join(", ")} ======`
      );

      // 1. Setup Feedback Listener
      const feedbackTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/navigation_web_status",
        messageType: "std_msgs/String",
      });

      // Define cleanup function
      const cleanup = () => {
        feedbackTopic.unsubscribe(feedbackHandler);
      };

      const feedbackHandler = (msg: any) => {
        const data = msg.data;
        if (onFeedback) onFeedback(data);

        if (data.includes("Multi-navigation complete")) {
          cleanup();
          resolve({ success: true, message: data });
        } else if (
          data.includes("Error:") ||
          data.includes("Navigation failed")
        ) {
          console.log("Error when navigating to multiple positions");
        }
      };
      feedbackTopic.subscribe(feedbackHandler);

      // 2. Call the Service to Start
      const navService = new ROSLIB.Service({
        ros: ros,
        name: "/start_multi_navigation_web",
        serviceType: "position_manager_msgs/srv/NavigateMulti",
      });

      const request = { names: names };

      navService.callService(
        request,
        (response: any) => {
          if (!response.success) {
            // If the service itself failed to start the thread
            cleanup();
            resolve({ success: false, message: response.message });
          } else {
            console.log("Multi-navigation background task started...");
          }
        },
        (error: any) => {
          cleanup();
          console.error("Multi-navigation service error:", error);
          reject(error);
        }
      );
    });
  }
  // Get current robot pose in the map frame using TF
  async getCurrentPose(): Promise<RobotPose> {
    return new Promise((resolve, reject) => {
      const ros = rosService.getROS();

      if (!rosService.isConnected) {
        reject(new Error("ROS not connected"));
        return;
      }

      const tfClient = new ROSLIB.TFClient({
        ros: ros,
        fixedFrame: "map",
        angularThres: 0.01,
        transThres: 0.01,
      });

      const timeout = setTimeout(() => {
        tfClient.unsubscribe("base_footprint");
        reject(new Error("Timeout getting current pose from TF"));
      }, 5000);

      tfClient.subscribe("base_footprint", (transform: ROSLIB.Transform) => {
        clearTimeout(timeout);
        tfClient.unsubscribe("base_footprint");

        if (!transform || !transform.translation || !transform.rotation) {
          reject(new Error("Invalid TF transform"));
          return;
        }

        const position = transform.translation;
        const orientation = transform.rotation;

        // Convert quaternion to yaw
        const siny_cosp =
          2 * (orientation.w * orientation.z + orientation.x * orientation.y);
        const cosy_cosp =
          1 -
          2 * (orientation.y * orientation.y + orientation.z * orientation.z);
        const theta = Math.atan2(siny_cosp, cosy_cosp);

        resolve({
          x: position.x,
          y: position.y,
          theta: theta,
        });
      });
    });
  }
}

export const positionService = PositionService.getInstance();
