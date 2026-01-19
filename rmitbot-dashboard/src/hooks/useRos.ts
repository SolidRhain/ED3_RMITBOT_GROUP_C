import { useEffect, useState, useCallback } from "react";
import { rosService } from "../services/ros2Connection";
import { ROS_CONFIG } from "../config";

export type RosStatus = "CONNECTED" | "DISCONNECTED" | "ERROR" | "CONNECTING";

export function useRos() {
  const [status, setStatus] = useState<RosStatus>("DISCONNECTED");
  const [currentIp, setCurrentIp] = useState<string>(ROS_CONFIG.DEFAULT_IP);

  const setupListeners = useCallback(() => {
    const ros = rosService.getROS();

    const handleConnect = () => setStatus("CONNECTED");
    const handleClose = () => setStatus("DISCONNECTED");
    const handleError = () => setStatus("ERROR");

    ros.on("connection", handleConnect);
    ros.on("close", handleClose);
    ros.on("error", handleError);

    return () => {
      ros.off("connection", handleConnect);
      ros.off("close", handleClose);
      ros.off("error", handleError);
    };
  }, []);

  useEffect(() => {
    const cleanup = setupListeners();

    const ros = rosService.getROS();
    if (rosService.isConnected || ros.isConnected) {
      setStatus("CONNECTED");
    } else {
      rosService.connect();
    }

    const unsubscribe = rosService.onConnectionChange(() => {
      cleanup();
      setupListeners();
      setCurrentIp(rosService.currentIp);
    });

    return () => {
      cleanup();
      unsubscribe();
    };
  }, [setupListeners]);

  const connect = () => {
    setStatus("CONNECTING");
    rosService.connect();
  };

  const disconnect = () => {
    rosService.disconnect();
  };

  const switchRobot = (newIp: string) => {
    if (newIp === currentIp) return;
    setStatus("CONNECTING");
    setCurrentIp(newIp);
    rosService.switchRobot(newIp);
  };

  return {
    status,
    isConnected: status === "CONNECTED",
    currentIp,
    connect,
    disconnect,
    switchRobot,
  };
}
