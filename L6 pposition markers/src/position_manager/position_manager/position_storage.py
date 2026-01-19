"""
Position storage utility for saving/loading waypoints to/from YAML
"""
import yaml
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

class PositionStorage:
    def __init__(self, logger):
        self.logger = logger
        
        positions_dir = os.path.join(os.path.expanduser('~'), '.ros', 'positions')
        os.makedirs(positions_dir, exist_ok=True)
        self.file_path = os.path.join(positions_dir, 'positions.yaml')
        
        self.logger.info(f'Using positions file: {self.file_path}')
        
        # Create file if it doesn't exist
        if not os.path.exists(self.file_path):
            self.save_positions([])
            self.logger.info(f'Created new positions file: {self.file_path}')
    
    def load_positions(self):
        """Load all positions from YAML file"""
        try:
            with open(self.file_path, 'r') as f:
                data = yaml.safe_load(f)
                if data is None:
                    return []
                positions = data.get('positions')
                if positions is None: 
                    return []
                self.logger.info(f'Loaded {len(positions)} positions from file')
                return positions
        except FileNotFoundError:
            self.logger.warn(f'Positions file not found: {self.file_path}')
            return []
        except Exception as e:
            self.logger.error(f'Error loading positions: {str(e)}')
            return []
    
    def save_positions(self, positions):
        """Save all positions to YAML file"""
        try:
            with open(self.file_path, 'w') as f:
                yaml.dump({'positions': positions}, f, default_flow_style=False)
            self.logger.info(f'Saved {len(positions)} positions to {self.file_path}')
        except Exception as e:
            self.logger.error(f'Error saving positions: {str(e)}')
            raise  
    
    def add_position(self, name, x, y, theta):
        """Add a new position"""
        try:
            positions = self.load_positions()
            
            # Check if name already exists
            for pos in positions:
                if pos['name'] == name:
                    return False, f"Position '{name}' already exists"
            
            # Validate coordinates
            if not all(isinstance(v, (int, float)) for v in [x, y, theta]):
                return False, "Invalid coordinates"
            
            # Add new position
            positions.append({
                'name': name,
                'x': float(x),
                'y': float(y),
                'theta': float(theta)
            })
            
            # Save immediately
            self.save_positions(positions)
            self.logger.info(f"Added position '{name}' at ({x:.2f}, {y:.2f}, θ={theta:.2f})")
            return True, f"Position '{name}' saved successfully"
        except Exception as e:
            error_msg = f"Error adding position: {str(e)}"
            self.logger.error(error_msg)
            return False, error_msg
    
    def delete_position(self, name):
        """Delete a position by name"""
        try:
            positions = self.load_positions()
            initial_count = len(positions)
            
            positions = [p for p in positions if p['name'] != name]
            
            if len(positions) == initial_count:
                return False, f"Position '{name}' not found"
            
            self.save_positions(positions)
            self.logger.info(f"Deleted position '{name}'")
            return True, f"Position '{name}' deleted"
        except Exception as e:
            error_msg = f"Error deleting position: {str(e)}"
            self.logger.error(error_msg)
            return False, error_msg
    
    def get_position(self, name):
        """Get a single position by name"""
        try:
            positions = self.load_positions()
            for pos in positions:
                if pos['name'] == name:
                    self.logger.info(f"Retrieved position '{name}'")
                    return pos
            self.logger.warn(f"Position '{name}' not found")
            return None
        except Exception as e:
            self.logger.error(f"Error getting position: {str(e)}")
            return None
    
    def clear_all(self):
        """Clear all positions"""
        try:
            self.save_positions([])
            self.logger.info('Cleared all positions')
        except Exception as e:
            self.logger.error(f"Error clearing positions: {str(e)}")