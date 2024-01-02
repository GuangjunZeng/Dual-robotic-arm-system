#!/usr/bin/env python3
"""
Robotic Arm Orchestration System (RAOS)
A sophisticated multi-arm control framework with real-time monitoring
"""

from dataclasses import dataclass
import socket
import time
import threading
from queue import Queue
import json
from typing import Dict, List, Optional, Callable
import logging

# Configure system logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('arm_controller.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('RAOS')

@dataclass
class ArmConfiguration:
    """Robotic arm network and behavior parameters"""
    hostname: str
    command_port: int = 12222
    motion_port: int = 13333
    feedback_port: int = 17333
    trajectory_port: int = 2000
    default_speed: float = 2.0
    collision_sensitivity: int = 1
    payload_mass: float = 3.0
    payload_offset: float = 1.0

class RoboticArmController:
    """High-level interface for individual robotic arm control"""
    
    def __init__(self, config: ArmConfiguration):
        self.config = config
        self._connections = {}
        self._feedback_active = False
        self._command_queue = Queue()
        self._feedback_listeners = []
        self._establish_connections()
        
    def _establish_connections(self):
        """Create all required socket connections"""
        ports = {
            'command': self.config.command_port,
            'motion': self.config.motion_port,
            'feedback': self.config.feedback_port,
            'trajectory': self.config.trajectory_port
        }
        
        for name, port in ports.items():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.config.hostname, port))
                self._connections[name] = sock
                logger.info(f"Connected to {self.config.hostname}:{port} for {name}")
            except socket.error as e:
                logger.error(f"Connection failed to {self.config.hostname}:{port}: {e}")
                raise

    def initialize(self):
        """Prepare arm for operation"""
        init_sequence = [
            f"SpeedL({self.config.default_speed})",
            f"SetCollisionLevel({self.config.collision_sensitivity})",
            f"PayLoad({self.config.payload_mass},{self.config.payload_offset})",
            "Tool(0)",
            "EnableRobot()"
        ]
        
        for cmd in init_sequence:
            self.send_command(cmd)
            time.sleep(0.1)
        
        time.sleep(3)  # Warm-up period
        logger.info(f"Arm {self.config.hostname} initialized and enabled")

    def send_command(self, command: str, await_response: bool = False):
        """Queue a command for execution"""
        self._command_queue.put((command, await_response))

    def _command_dispatcher(self):
        """Process commands from queue"""
        while True:
            cmd, needs_response = self._command_queue.get()
            try:
                self._connections['command'].sendall(cmd.encode('utf-8'))
                if needs_response:
                    return self._connections['command'].recv(1024)
            except Exception as e:
                logger.error(f"Command failed: {cmd} - {e}")

    def start_feedback_monitor(self, callback: Optional[Callable] = None):
        """Begin streaming pose feedback"""
        if callback:
            self._feedback_listeners.append(callback)
            
        if not self._feedback_active:
            self._feedback_active = True
            threading.Thread(
                target=self._feedback_processor,
                daemon=True
            ).start()

    def _feedback_processor(self):
        """Process incoming feedback data"""
        while self._feedback_active:
            try:
                data = self._connections['feedback'].recv(1440)
                for listener in self._feedback_listeners:
                    listener(data)
            except Exception as e:
                logger.error(f"Feedback error: {e}")
                time.sleep(1)

    def move_to(self, position: Dict[str, float], speed: Optional[float] = None):
        """Execute coordinated movement"""
        speed = speed or self.config.default_speed
        cmd = (
            f"MovL("
            f"{position['x']},{position['y']},{position['z']},"
            f"{position['rx']},{position['ry']},{position['rz']},"
            f"SpeedL={speed})"
        )
        self.send_command(cmd)

    def emergency_stop(self):
        """Immediately halt all motion"""
        self.send_command("Stop()", await_response=True)
        logger.warning(f"Emergency stop triggered on {self.config.hostname}")

    def __del__(self):
        """Cleanup resources"""
        self._feedback_active = False
        for conn in self._connections.values():
            try:
                conn.close()
            except:
                pass
        logger.info(f"Arm {self.config.hostname} connections closed")

class ArmOrchestrator:
    """Centralized control for multiple robotic arms"""
    
    def __init__(self, arm_configs: List[ArmConfiguration]):
        self.arms = [RoboticArmController(cfg) for cfg in arm_configs]
        self._operation_mode = 'idle'
        self._command_threads = []
        
    def initialize_all(self):
        """Prepare all arms simultaneously"""
        init_threads = []
        for arm in self.arms:
            t = threading.Thread(target=arm.initialize)
            t.start()
            init_threads.append(t)
            
        for t in init_threads:
            t.join()
            
        logger.info("All arms initialized")

    def start_feedback_monitoring(self):
        """Begin receiving data from all arms"""
        for arm in self.arms:
            arm.start_feedback_monitor(self._handle_feedback)

    def _handle_feedback(self, data):
        """Process incoming feedback from any arm"""
        if self._operation_mode == 'debug':
            logger.debug(f"Feedback data: {data.decode('utf-8')}")

    def coordinated_move(self, movements: Dict[str, Dict[str, float]]):
        """Execute synchronized movement across arms"""
        move_threads = []
        for arm_id, position in movements.items():
            arm = next(a for a in self.arms if a.config.hostname == arm_id)
            t = threading.Thread(target=arm.move_to, args=(position,))
            t.start()
            move_threads.append(t)
            
        for t in move_threads:
            t.join()

    def set_operation_mode(self, mode: str):
        """Change system behavior mode"""
        valid_modes = ['idle', 'debug', 'precision', 'rapid']
        if mode in valid_modes:
            self._operation_mode = mode
            logger.info(f"System mode changed to {mode}")
        else:
            logger.error(f"Invalid mode: {mode}")

    def shutdown(self):
        """Safely power down all arms"""
        for arm in self.arms:
            arm.send_command("DisableRobot()")
        logger.info("All arms safely disabled")

def main():
    """Demonstration of multi-arm coordination"""
    
    # System configuration
    arm_specs = [
        ArmConfiguration("192.168.5.1"),
        ArmConfiguration("192.168.5.2")
    ]
    
    # Create control system
    system = ArmOrchestrator(arm_specs)
    
    try:
        # Initialize system
        system.initialize_all()
        system.start_feedback_monitoring()
        system.set_operation_mode('debug')
        
        # Example coordinated movement
        movement_plan = {
            "192.168.5.1": {
                "x": 100, "y": 50, "z": 25,
                "rx": 0, "ry": 0, "rz": 0
            },
            "192.168.5.2": {
                "x": 150, "y": -30, "z": 40,
                "rx": 0, "ry": 0, "rz": 0
            }
        }
        
        logger.info("Beginning coordinated movement")
        system.coordinated_move(movement_plan)
        
        # Keep system running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Shutdown signal received")
    finally:
        system.shutdown()

if __name__ == "__main__":
    main()