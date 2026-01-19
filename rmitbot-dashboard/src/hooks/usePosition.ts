import { useState, useEffect } from "react";
import { toast } from "sonner";
import { positionService } from "@/services/positionService";
import type { SavedPosition } from "@/types";

interface UsePositionProps {
  isConnected: boolean;
}

export function usePosition({ isConnected }: UsePositionProps) {
  const [positions, setPositions] = useState<SavedPosition[]>([]);
  const [selectedPositions, setSelectedPositions] = useState<Set<string>>(
    new Set()
  );
  const [selectedOrder, setSelectedOrder] = useState<string[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [isNavigating, setIsNavigating] = useState(false);
  const [newPositionName, setNewPositionName] = useState("");
  const [currentNavigatingIndex, setCurrentNavigatingIndex] =
    useState<number>(-1);
  const [navigationStatus, setNavigationStatus] = useState<string>("");

  useEffect(() => {
    if (isConnected) {
      loadPositions();
    }
  }, [isConnected]);

  const loadPositions = async () => {
    if (!isConnected) return;

    setIsLoading(true);
    try {
      const loadedPositions = await positionService.getPositions();
      setPositions(loadedPositions);
    } catch (error) {
      console.error("Failed to load positions:", error);
      // toast.error("Failed to load positions");
    } finally {
      setIsLoading(false);
    }
  };

  const handleSavePosition = async () => {
    if (!newPositionName.trim()) {
      toast.error("Please enter a position name");
      return;
    }

    setIsSaving(true);
    try {
      const result = await positionService.savePosition(newPositionName.trim());
      if (result.success) {
        toast.success(`Position "${newPositionName}" saved`);
        setNewPositionName("");
        await loadPositions();
      } else {
        toast.error(result.message || "Failed to save position");
      }
    } catch (error) {
      console.error("Error saving position:", error);
      toast.error("Error saving position");
    } finally {
      setIsSaving(false);
    }
  };

  const handleDeletePosition = async (name: string) => {
    try {
      const success = await positionService.deletePosition(name);
      if (success) {
        toast.success(`Position "${name}" deleted`);
        await loadPositions();
        selectedPositions.delete(name);
        setSelectedPositions(new Set(selectedPositions));
        setSelectedOrder(selectedOrder.filter((n) => n !== name));
      } else {
        toast.error("Failed to delete position");
      }
    } catch (error) {
      console.error("Error deleting position:", error);
      toast.error("Error deleting position");
    }
  };

  const handleNavigateSelected = async () => {
    if (selectedPositions.size === 0) {
      toast.error("No positions selected");
      return;
    }

    if (isNavigating) {
      toast.warning("Navigation already in progress");
      return;
    }

    const selectedNames = selectedOrder;
    setIsNavigating(true);
    setCurrentNavigatingIndex(0);
    setNavigationStatus(
      `Starting navigation through ${selectedNames.length} positions...`
    );

    try {
      toast.info(`Navigating through ${selectedNames.length} positions...`);
      console.log(
        `[PositionManager] Starting multi-navigation: ${selectedNames.join(
          ", "
        )}`
      );

      const result = await positionService.navigateThroughPositions(
        selectedNames,
        (feedback) => {
          console.log(`[PositionManager] Multi-feedback:`, feedback);
          const currentName = (feedback as any)?.current_position_name as
            | string
            | undefined;

          if (currentName) {
            const idx = selectedNames.indexOf(currentName);
            if (idx !== -1) {
              setCurrentNavigatingIndex(idx);
              setNavigationStatus(
                `At position ${idx + 1}/${selectedNames.length}: ${currentName}`
              );
              toast.info(
                `At position ${idx + 1}/${selectedNames.length}: ${currentName}`
              );
            } else {
              setNavigationStatus(`Navigating: ${currentName}`);
              toast.info(`Navigating: ${currentName}`);
            }
          }
        }
      );

      console.log(`[PositionManager] Multi-navigation result:`, result);

      if (result.success) {
        toast.success(`Completed!`);
      } else {
        toast.error(`Navigation failed: ${result.message}`);
      }
    } catch (error: any) {
      console.error("[PositionManager] Multi-navigation error:", error);
      toast.error(error?.message || "Navigation error");
    } finally {
      setIsNavigating(false);
      setCurrentNavigatingIndex(-1);
      setNavigationStatus("");
    }
  };

  const handleCancelNavigation = () => {
    console.log("[PositionManager] Cancelling navigation...");
    positionService.cancelNavigation();
    setIsNavigating(false);
    setCurrentNavigatingIndex(-1);
    setNavigationStatus("");
    toast.info("Navigation cancelled");
  };

  const toggleSelection = (name: string) => {
    const newSelected = new Set(selectedPositions);
    if (newSelected.has(name)) {
      newSelected.delete(name);
      setSelectedOrder(selectedOrder.filter((n) => n !== name));
    } else {
      newSelected.add(name);
      setSelectedOrder([...selectedOrder, name]);
    }
    setSelectedPositions(newSelected);
  };

  return {
    // State
    positions,
    selectedPositions,
    selectedOrder,
    isLoading,
    isSaving,
    isNavigating,
    newPositionName,
    currentNavigatingIndex,
    navigationStatus,

    // Setters
    setNewPositionName,

    // Handlers
    loadPositions,
    handleSavePosition,
    handleDeletePosition,
    handleNavigateSelected,
    handleCancelNavigation,
    toggleSelection,
  };
}
