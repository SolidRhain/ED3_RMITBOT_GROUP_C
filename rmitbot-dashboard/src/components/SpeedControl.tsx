import { Leaf, Gauge, Zap } from 'lucide-react';
import { Button } from './ui/button';

export type SpeedMode = 'eco' | 'normal' | 'fast';

interface SpeedControlProps {
  selectedMode: SpeedMode;
  onModeChange: (mode: SpeedMode) => void;
  disabled?: boolean;
}

const speedModes = [
  { mode: 'eco' as SpeedMode, label: 'Eco', icon: Leaf, color: 'from-green-600 to-green-700' },
  { mode: 'normal' as SpeedMode, label: 'Normal', icon: Gauge, color: 'from-blue-600 to-blue-700' },
  { mode: 'fast' as SpeedMode, label: 'Fast', icon: Zap, color: 'from-purple-600 to-purple-700' },
];

export function SpeedControl({ selectedMode, onModeChange, disabled = false }: SpeedControlProps) {
  return (
    <div className="grid grid-cols-3 gap-2">
      {speedModes.map(({ mode, label, icon: Icon, color }) => {
        const isSelected = selectedMode === mode;
        return (
          <Button
            key={mode}
            onClick={() => onModeChange(mode)}
            disabled={disabled}
            variant={isSelected ? 'default' : 'outline'}
            className={`flex items-center justify-center gap-1 h-9 px-2 ${
              isSelected
                ? `bg-gradient-to-br ${color} border-0 hover:opacity-90`
                : 'bg-slate-800/30 border-slate-700/50 hover:bg-slate-700/30'
            }`}
          >
            <Icon className={`w-4 h-4 ${isSelected ? 'text-white' : 'text-slate-400'}`} />
            <span className={`text-xs font-medium ${isSelected ? 'text-white' : 'text-slate-400'}`}>
              {label}
            </span>
          </Button>
        );
      })}
    </div>
  );
}
