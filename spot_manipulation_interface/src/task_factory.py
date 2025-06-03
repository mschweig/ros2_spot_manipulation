"""Factory for creating manipulation tasks."""

from typing import Dict, Type

from .manipulation_tasks import ManipulationTask, PickTask, PlaceTask, WalkTask
from .spot_client import SpotClient


class TaskFactory:
    """Factory for creating manipulation tasks."""
    
    _TASK_REGISTRY: Dict[str, Type[ManipulationTask]] = {
        'pick': PickTask,
        'place': PlaceTask,
        'walk': WalkTask,
    }
    
    @classmethod
    def create_task(cls, task_type: str, spot_client: SpotClient) -> ManipulationTask:
        """Create a manipulation task.
        
        Args:
            task_type: Type of task to create
            spot_client: Initialized SpotClient instance
            
        Returns:
            ManipulationTask instance
            
        Raises:
            ValueError: If task_type is not supported
        """
        if task_type not in cls._TASK_REGISTRY:
            available_tasks = list(cls._TASK_REGISTRY.keys())
            raise ValueError(f"Unsupported task type: {task_type}. "
                           f"Available tasks: {available_tasks}")
        
        task_class = cls._TASK_REGISTRY[task_type]
        return task_class(spot_client)
    
    @classmethod
    def get_supported_tasks(cls) -> list:
        """Get list of supported task types."""
        return list(cls._TASK_REGISTRY.keys())
