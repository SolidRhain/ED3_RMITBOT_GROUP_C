import { Button } from "./ui/button";
import {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  RotateCcw,
  RotateCw,
  Square,
  MoveUpRight,
  MoveUpLeft,
  MoveDownRight,
  MoveDownLeft,
} from "lucide-react";

interface ControlButtonsProps {
  onCommand: (command: string) => void;
  disabled?: boolean;
}

export function ControlButtons({
  onCommand,
  disabled = false,
}: ControlButtonsProps) {
  // Helper to handle button press (both mouse and touch)
  const handleButtonPress = (command: string) => {
    if (!disabled) {
      console.log(`Button pressed: ${command}`);
      onCommand(command);
    }
  };

  // Helper to handle button release (both mouse and touch)
  const handleButtonRelease = () => {
    if (!disabled) {
      console.log(`Button released - sending stop`);
      onCommand("stop");
    }
  };

  // Create button props for directional buttons (hold to move)
  const createDirectionalButtonProps = (command: string) => ({
    onMouseDown: () => handleButtonPress(command),
    onMouseUp: handleButtonRelease,
    onMouseLeave: handleButtonRelease,
    onTouchStart: () => handleButtonPress(command),
    onTouchEnd: handleButtonRelease,
  });

  return (
    <div className="space-y-2">
      {/* Directional Controls with Diagonals */}
      <div className="grid grid-cols-3 gap-1.5">
        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("forward-left")}
          disabled={disabled}
          className="h-11 bg-purple-600/80 hover:bg-purple-700 text-white border-0 select-none"
        >
          <MoveUpLeft className="w-4 h-4" />
        </Button>
        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("forward")}
          disabled={disabled}
          className="h-11 bg-blue-600/80 hover:bg-blue-700 text-white border-0 select-none"
        >
          <ArrowUp className="w-4 h-4" />
        </Button>
        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("forward-right")}
          disabled={disabled}
          className="h-11 bg-purple-600/80 hover:bg-purple-700 text-white border-0 select-none"
        >
          <MoveUpRight className="w-4 h-4" />
        </Button>

        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("left")}
          disabled={disabled}
          className="h-11 bg-blue-600/80 hover:bg-blue-700 text-white border-0 select-none"
        >
          <ArrowLeft className="w-4 h-4" />
        </Button>
        <Button
          size="sm"
          variant="destructive"
          onClick={() => onCommand("stop")}
          disabled={disabled}
          className="h-11 select-none"
        >
          <Square className="w-4 h-4" />
        </Button>
        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("right")}
          disabled={disabled}
          className="h-11 bg-blue-600/80 hover:bg-blue-700 text-white border-0 select-none"
        >
          <ArrowRight className="w-4 h-4" />
        </Button>

        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("backward-left")}
          disabled={disabled}
          className="h-11 bg-purple-600/80 hover:bg-purple-700 text-white border-0 select-none"
        >
          <MoveDownLeft className="w-4 h-4" />
        </Button>
        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("backward")}
          disabled={disabled}
          className="h-11 bg-blue-600/80 hover:bg-blue-700 text-white border-0 select-none"
        >
          <ArrowDown className="w-4 h-4" />
        </Button>
        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("backward-right")}
          disabled={disabled}
          className="h-11 bg-purple-600/80 hover:bg-purple-700 text-white border-0 select-none"
        >
          <MoveDownRight className="w-4 h-4" />
        </Button>
      </div>

      {/* Rotation Controls */}
      <div className="grid grid-cols-2 gap-1.5">
        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("rotate-left")}
          disabled={disabled}
          className="h-8 bg-indigo-600/80 hover:bg-indigo-700 text-white border-0 text-[10px] select-none"
        >
          <RotateCcw className="w-3 h-3 mr-1" />
          Left
        </Button>
        <Button
          size="sm"
          variant="secondary"
          {...createDirectionalButtonProps("rotate-right")}
          disabled={disabled}
          className="h-8 bg-indigo-600/80 hover:bg-indigo-700 text-white border-0 text-[10px] select-none"
        >
          <RotateCw className="w-3 h-3 mr-1" />
          Right
        </Button>
      </div>
    </div>
  );
}
