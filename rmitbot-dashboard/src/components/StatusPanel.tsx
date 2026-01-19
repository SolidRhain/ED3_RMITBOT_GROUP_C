import { Badge } from './ui/badge';
import { Wifi, WifiOff, Gauge } from 'lucide-react';
import { Card } from './ui/card';

interface StatusPanelProps {
  isConnected: boolean;
  speed: number;
}

export function StatusPanel({ isConnected, speed }: StatusPanelProps) {
  return (
    <div className="grid grid-cols-2 gap-3">
      {/* Connection Status */}
      <Card className="p-3 bg-slate-900/50 border-slate-700">
        <div className="flex flex-col items-center justify-center gap-2 text-center">
          <div className="flex items-center gap-1.5">
            {isConnected ? (
              <Wifi className="w-4 h-4 text-green-500" />
            ) : (
              <WifiOff className="w-4 h-4 text-red-500" />
            )}
            <span className="text-xs text-slate-400">Connection</span>
          </div>
          <Badge
            variant={isConnected ? "default" : "destructive"}
            className={`text-xs ${isConnected ? "bg-green-600" : ""}`}
          >
            {isConnected ? 'Connected' : 'Disconnected'}
          </Badge>
        </div>
      </Card>

      {/* Speed */}
      <Card className="p-3 bg-slate-900/50 border-slate-700">
        <div className="flex flex-col items-center justify-center gap-2 text-center">
          <div className="flex items-center gap-1.5">
            <Gauge className={`w-4 h-4 ${isConnected ? 'text-emerald-500' : 'text-slate-600'}`} />
            <span className="text-xs text-slate-400">Speed</span>
          </div>
          <span className={isConnected ? 'text-emerald-400' : 'text-slate-600'}>
            {isConnected ? `${speed.toFixed(2)} m/s` : '--'}
          </span>
        </div>
      </Card>
    </div>
  );
}
