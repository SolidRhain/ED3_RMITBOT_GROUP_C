import { useState } from "react";
import { Toaster, toast } from "sonner";
import { Wifi } from "lucide-react";

import { ControlButtons } from "./components/ControlButtons";
import { SpeedControl } from "./components/SpeedControl";
import { PositionManager } from "./components/PositionManager";
import { StatusDisplay } from "./components/StatusDisplay";
import { Card } from "./components/ui/card";
import { Switch } from "./components/ui/switch";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "./components/ui/select";

import { useRobot } from "./hooks/useRobot";
import type { SpeedMode } from "./types";
import { ROBOT_OPTIONS } from "./config";

export default function App() {
  const robot = useRobot();
  const [speedMode, setSpeedMode] = useState<SpeedMode>("normal");

  const handleSpeedChange = (mode: SpeedMode) => {
    setSpeedMode(mode);
    robot.setSpeedMode(mode);
  };

  const handleCommand = (cmd: string) => {
    if (!robot.isConnected) return toast.error("Robot Offline");

    switch (cmd) {
      case "forward":
        robot.move(1.0, 0, 0);
        break;
      case "backward":
        robot.move(-1.0, 0, 0);
        break;
      case "left":
        robot.move(0, 1.0, 0);
        break;
      case "right":
        robot.move(0, -1.0, 0);
        break;
      case "stop":
        robot.move(0, 0, 0);
        break;
      case "forward-left":
        robot.move(0.707, 0.707, 0);
        break;
      case "forward-right":
        robot.move(0.707, -0.707, 0);
        break;
      case "backward-left":
        robot.move(-0.707, 0.707, 0);
        break;
      case "backward-right":
        robot.move(-0.707, -0.707, 0);
        break;
      case "rotate-left":
        robot.move(0, 0, 1.0);
        break;
      case "rotate-right":
        robot.move(0, 0, -1.0);
        break;
    }
  };

  const getStatusColor = () => {
    switch (robot.status) {
      case "CONNECTED":
        return "text-green-400";
      case "CONNECTING":
        return "text-yellow-400";
      case "ERROR":
        return "text-red-500";
      default:
        return "text-slate-600";
    }
  };

  const getStatusText = () => {
    switch (robot.status) {
      case "CONNECTED":
        return "ONLINE";
      case "CONNECTING":
        return "CONNECTING...";
      case "ERROR":
        return "ERROR";
      default:
        return "OFFLINE";
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-950 via-slate-900 to-slate-950 text-white">
      <Toaster theme="dark" />

      {/* Header */}
      <div className="border-b border-slate-800/50 bg-slate-900/30 backdrop-blur-sm sticky top-0 z-10">
        <div className="container mx-auto px-4 py-3 max-w-7xl flex items-center justify-between">
          <div className="flex items-center gap-2 md:gap-3">
            <h1 className="text-sm md:text-lg font-bold text-white">
              Robot Control Panel
            </h1>
            <Select value={robot.currentIp} onValueChange={robot.switchRobot}>
              <SelectTrigger className="w-[140px] md:w-[180px] h-8 text-xs">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                {ROBOT_OPTIONS.map((option) => (
                  <SelectItem key={option.ip} value={option.ip}>
                    {option.name} ({option.ip.split(".").pop()})
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>

          <div className="flex items-center gap-2">
            <Wifi className={`w-3 h-3 md:w-4 md:h-4 ${getStatusColor()}`} />
            <span
              className={`text-[10px] md:text-xs font-bold ${getStatusColor()}`}
            >
              {getStatusText()}
            </span>
            <Switch
              checked={robot.isConnected}
              onCheckedChange={robot.toggleConnection}
            />
          </div>
        </div>
      </div>

      {/* Main Content - Responsive Grid */}
      <div className="container mx-auto px-4 py-4 max-w-7xl">
        <div className="grid grid-cols-1 lg:grid-cols-12 gap-4 min-h-[calc(100vh-120px)]">
          {/* Left Column: Position Manager - Full width on mobile, 8 cols on desktop */}
          <div className="lg:col-span-8">
            <PositionManager isConnected={robot.isConnected} />
          </div>

          {/* Right Column: Status + Control - Full width on mobile, 4 cols on desktop */}
          <div className="lg:col-span-4 flex flex-col gap-4">
            {/* Status Display */}
            <StatusDisplay isConnected={robot.isConnected} />

            {/* Control Card */}
            <Card className="bg-slate-900/50 border-slate-700/50 backdrop-blur-sm p-4">
              {/* Speed Control */}
              <h2 className="text-sm font-semibold mb-3 text-white">
                Speed Control
              </h2>
              <SpeedControl
                selectedMode={speedMode}
                onModeChange={handleSpeedChange}
                disabled={false}
              />

              <div className="mt-4">
                <h2 className="text-sm font-semibold mb-3 text-white">
                  Movement Controls
                </h2>
                <ControlButtons
                  onCommand={handleCommand}
                  disabled={!robot.isConnected}
                />
              </div>
            </Card>

            {/* Quick Start Guide */}
            <Card className="bg-slate-900/50 border-slate-700/50 backdrop-blur-sm p-4">
              <h2 className="text-sm font-semibold mb-3 text-white">
                Quick Start
              </h2>
              <ol className="text-slate-400 space-y-2 list-decimal list-inside text-xs md:text-sm">
                <li>Drive robot to a location</li>
                <li>Save position with a name</li>
                <li>Select positions to navigate</li>
                <li>View visualization in Foxglove</li>
              </ol>
            </Card>
          </div>
        </div>
      </div>
    </div>
  );
}
