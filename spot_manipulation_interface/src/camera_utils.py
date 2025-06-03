"""Camera coordinate transformation utilities."""

from typing import Tuple


class CameraCoordinateTransformer:
    """Handles coordinate transformations for different camera orientations."""
    
    # Camera resolution mappings
    CAMERA_RESOLUTIONS = {
        'frontleft': (480, 640),
        'frontright': (480, 640),
        'left': (640, 480),
        'right': (640, 480),
        'back': (640, 480),
    }
    
    @classmethod
    def transform_coordinates(cls, x: float, y: float, camera_name: str) -> Tuple[float, float]:
        """Transform pixel coordinates based on camera orientation.
        
        Args:
            x: X coordinate
            y: Y coordinate
            camera_name: Name of the camera
            
        Returns:
            Tuple of transformed (x, y) coordinates
            
        Raises:
            ValueError: If camera_name is invalid
        """
        if camera_name not in cls.CAMERA_RESOLUTIONS:
            raise ValueError(f"Invalid camera name: {camera_name}. "
                           f"Valid options: {list(cls.CAMERA_RESOLUTIONS.keys())}")
        
        if camera_name in ['frontleft', 'frontright']:
            width, height = cls.CAMERA_RESOLUTIONS[camera_name]
            # 90 degree counter-clockwise rotation
            return y, width - x
        elif camera_name == 'right':
            width, height = cls.CAMERA_RESOLUTIONS[camera_name]
            # 180 degree rotation
            return width - x, height - y
        else:
            # No rotation for other cameras
            return x, y
    
    @classmethod
    def validate_camera_name(cls, camera_name: str) -> bool:
        """Validate camera name."""
        return camera_name in cls.CAMERA_RESOLUTIONS and camera_name != 'hand'

