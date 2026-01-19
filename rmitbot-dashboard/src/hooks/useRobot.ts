import { useState, useEffect, useRef, useCallback } from "react";
import type { RobotState, SpeedMode, NavigationStatus } from "../types";
import { useRos } from "./useRos";
import { rosService } from "../services/ros2Connection";
import { ROS_CONFIG } from "../config";

export function useRobot() {
  const { status, isConnected, connect, disconnect, switchRobot, currentIp } = useRos();

  const [state, setState] = useState<Omit<RobotState, "isConnected">>({
    batteryLevel: 78,
    speed: 0,
    navigationStatus: "idle",
    currentPose: { x: 0, y: 0, theta: 0 },
  });

  // --- REFS ---
  const speedModeRef = useRef<SpeedMode>("normal");
  const publishIntervalRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const currentVelocityRef = useRef<{ x: number; y: number; z: number }>({ x: 0, y: 0, z: 0 });

  // --- HELPERS ---
  // Base speed/turn from config, then multiply by mode
  const getSpeedMultiplier = () => {
    switch (speedModeRef.current) {
      case "eco":
        return 0.6; // 60% of base speed
      case "normal":
        return 1.0; // 100% of base speed
      case "fast":
        return 1.6; // 160% of base speed
      default:
        return 1.0;
    }
  };

  // Helper to stop publishing
  const stopPublishing = useCallback((updateSpeed: boolean = true) => {
    console.log("Stopping velocity publishing");
    if (publishIntervalRef.current) {
      clearInterval(publishIntervalRef.current);
      publishIntervalRef.current = null;
    }
    // Send stop command
    currentVelocityRef.current = { x: 0, y: 0, z: 0 };
    rosService.publishVelocity(0, 0, 0);
    if (updateSpeed) {
      setState((prev) => ({ ...prev, speed: 0 }));
    }
  }, []);

  // --- ACTIONS ---

  const toggleConnection = () => (isConnected ? disconnect() : connect());

  const setNavigationStatus = (navigationStatus: NavigationStatus) =>
    setState((prev) => ({ ...prev, navigationStatus }));

  // Set speed mode (Eco/Normal/Fast)
  const setSpeedMode = (mode: SpeedMode) => {
    speedModeRef.current = mode;
  };

  // --- THE MOVE FUNCTION ---
  // Continuous publishing approach: maintains velocity until explicitly stopped
  // x (forward), y (strafe), z (turn)
  const move = useCallback(
    (x: number, y: number, z: number) => {
      if (!isConnected) {
        console.warn("Cannot move - connected:", isConnected);
        // Stop publishing if disconnected
        if (!isConnected && publishIntervalRef.current) {
          clearInterval(publishIntervalRef.current);
          publishIntervalRef.current = null;
        }
        return;
      }

      console.log(`Move command received - x: ${x}, y: ${y}, z: ${z}`);

      const multiplier = getSpeedMultiplier();
      // Apply base speed and turn from config, then multiply by speed mode
      // x, y are linear (use SPEED), z is angular (use TURN)
      const vel_x = x * ROS_CONFIG.SPEED * multiplier;
      const vel_y = y * ROS_CONFIG.SPEED * multiplier;
      const vel_z = z * ROS_CONFIG.TURN * multiplier;

      console.log(`Calculated velocities - linear: [${vel_x.toFixed(2)}, ${vel_y.toFixed(2)}], angular: ${vel_z.toFixed(2)}, multiplier: ${multiplier}`);

      // Store current velocity
      currentVelocityRef.current = { x: vel_x, y: vel_y, z: vel_z };

      const currentSpeed = Math.sqrt(
        Math.pow(vel_x, 2) + Math.pow(vel_y, 2)
      );
      setState((prev) => ({ ...prev, speed: currentSpeed }));

      // If stop command (0,0,0), stop publishing
      if (x === 0 && y === 0 && z === 0) {
        console.log("Stop command received");
        stopPublishing();
        return;
      }

      // If already publishing, the continuous loop will pick up the new velocity
      // If not publishing, start the continuous publishing loop
      if (!publishIntervalRef.current) {
        console.log(`Starting continuous publishing at ${ROS_CONFIG.PUBLISH_RATE}Hz`);
        const publishInterval = 1000 / ROS_CONFIG.PUBLISH_RATE; // Convert Hz to ms

        publishIntervalRef.current = setInterval(() => {
          const { x, y, z } = currentVelocityRef.current;
          rosService.publishVelocity(x, y, z);
        }, publishInterval);

        // Publish immediately as well
        rosService.publishVelocity(vel_x, vel_y, vel_z);
      }
    },
    [isConnected, stopPublishing]
  );

  // --- LOOPS ---
  // No continuous loop needed anymore - publishing is handled in move() function

  // Cleanup on disconnect or unmount
  useEffect(() => {
    return () => {
      // Only cleanup on unmount, not on disconnect
      if (publishIntervalRef.current) {
        clearInterval(publishIntervalRef.current);
        publishIntervalRef.current = null;
      }
      currentVelocityRef.current = { x: 0, y: 0, z: 0 };
      rosService.publishVelocity(0, 0, 0);
    };
  }, []);

  // Battery Simulation
  useEffect(() => {
    if (!isConnected) return;
    const simInterval = setInterval(() => {
      setState((prev) => ({
        ...prev,
        batteryLevel: Math.max(0, prev.batteryLevel - 0.01),
      }));
    }, 1000);
    return () => clearInterval(simInterval);
  }, [isConnected]);

  return {
    ...state,
    isConnected,
    status,
    currentIp,
    toggleConnection,
    setNavigationStatus,
    move,
    setSpeedMode,
    switchRobot,
  };
}
