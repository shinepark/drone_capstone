"""
Dual Drone Flight Path Generator with Resume Capability
Generates optimized flight paths for two drones with Mission Planner integration
"""

import json
import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, asdict
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import split
import numpy as np


@dataclass
class FlightState:
    """Tracks the current state of a drone's mission"""
    drone_id: int
    current_waypoint_index: int
    total_waypoints: int
    elapsed_flight_time: float  # seconds
    remaining_battery: float  # percentage
    last_position: Tuple[float, float]  # (lat, lon)
    mission_complete: bool = False


@dataclass
class Waypoint:
    """Individual waypoint with metadata"""
    lat: float
    lon: float
    altitude: float
    index: int
    drone_id: int


class DualDroneFlightPlanner:
    def __init__(
        self,
        field_points: List[Tuple[float, float]],
        flight_altitude: float = 50.0,  # meters
        max_flight_time: float = 900.0,  # 15 minutes in seconds
        sweep_spacing: float = 0.0001,  # degrees (~11m at equator)
        cruise_speed: float = 20.0,  # m/s
        battery_reserve: float = 0.20  # 15% safety margin
    ):
        """
        Initialize the flight planner
        
        Args:
            field_points: List of (lat, lon) tuples defining field boundary
            flight_altitude: Flight altitude in meters AGL
            max_flight_time: Maximum flight time in seconds
            sweep_spacing: Distance between sweep lines in degrees
            cruise_speed: Drone cruise speed in m/s
            battery_reserve: Battery safety margin (0.20 = 20%)
        """
        self.field_points = field_points
        self.flight_altitude = flight_altitude
        self.max_flight_time = max_flight_time * (1 - battery_reserve)
        self.sweep_spacing = sweep_spacing
        self.cruise_speed = cruise_speed
        
        self.polygon = Polygon(field_points)
        self.drone1_waypoints = []
        self.drone2_waypoints = []
        self.drone1_state = None
        self.drone2_state = None
        
    def generate_flight_paths(self) -> Tuple[List[Waypoint], List[Waypoint]]:
        """
        Generate optimized flight paths for both drones
        """
        # Split the polygon into two regions
        region1, region2 = self._split_polygon()
        
        # Generate sweep patterns for each region
        self.drone1_waypoints = self._generate_sweep_pattern(region1, drone_id=1)
        self.drone2_waypoints = self._generate_sweep_pattern(region2, drone_id=2)
        
        # Initialize flight states
        self.drone1_state = FlightState(
            drone_id=1,
            current_waypoint_index=0,
            total_waypoints=len(self.drone1_waypoints),
            elapsed_flight_time=0.0,
            remaining_battery=100.0,
            last_position=(self.drone1_waypoints[0].lat, self.drone1_waypoints[0].lon)
        )
        
        self.drone2_state = FlightState(
            drone_id=2,
            current_waypoint_index=0,
            total_waypoints=len(self.drone2_waypoints),
            elapsed_flight_time=0.0,
            remaining_battery=100.0,
            last_position=(self.drone2_waypoints[0].lat, self.drone2_waypoints[0].lon)
        )
        
        return self.drone1_waypoints, self.drone2_waypoints
    
    def _split_polygon(self) -> Tuple[Polygon, Polygon]:
        """
        Split the field polygon into two roughly equal regions
        """
        minx, miny, maxx, maxy = self.polygon.bounds
        
        # Try vertical split first
        mid_x = (minx + maxx) / 2
        split_line = LineString([(mid_x, miny - 1), (mid_x, maxy + 1)])
        
        try:
            result = split(self.polygon, split_line)
            if len(result.geoms) == 2:
                return result.geoms[0], result.geoms[1]
        except:
            pass
        
        # If vertical split fails, try horizontal
        mid_y = (miny + maxy) / 2
        split_line = LineString([(minx - 1, mid_y), (maxx + 1, mid_y)])
        
        result = split(self.polygon, split_line)
        if len(result.geoms) >= 2:
            return result.geoms[0], result.geoms[1]
        
        # Fallback: return polygon twice (both drones cover same area)
        return self.polygon, self.polygon
    
    def _generate_sweep_pattern(self, region: Polygon, drone_id: int) -> List[Waypoint]:
        """
        Generate lawnmower sweep pattern for a region
        """
        waypoints = []
        minx, miny, maxx, maxy = region.bounds
        
        direction = 1  # 1 for left-to-right, -1 for right-to-left
        y = miny
        waypoint_index = 0
        
        while y <= maxy:
            # Create horizontal sweep line
            sweep = LineString([(minx - 0.0001, y), (maxx + 0.0001, y)])
            intersection = sweep.intersection(region)
            
            if not intersection.is_empty:
                coords = []
                
                if intersection.geom_type == 'LineString':
                    coords = list(intersection.coords)
                elif intersection.geom_type == 'MultiLineString':
                    # Sort segments left to right
                    for segment in sorted(intersection.geoms, key=lambda s: s.bounds[0]):
                        coords.extend(list(segment.coords))
                
                # Reverse every other pass for efficient lawnmower pattern
                if direction == -1:
                    coords.reverse()
                
                # Add waypoints
                for lat, lon in coords:
                    waypoints.append(Waypoint(
                        lat=lat,
                        lon=lon,
                        altitude=self.flight_altitude,
                        index=waypoint_index,
                        drone_id=drone_id
                    ))
                    waypoint_index += 1
                
                direction *= -1
            
            y += self.sweep_spacing
        
        return waypoints
    
    def get_next_flight_segment(self, drone_id: int) -> List[Waypoint]:
        """
        Get the next flight segment that fits within max flight time
        """
        if drone_id == 1:
            waypoints = self.drone1_waypoints
            state = self.drone1_state
        else:
            waypoints = self.drone2_waypoints
            state = self.drone2_state
        
        if state.mission_complete or state.current_waypoint_index >= len(waypoints):
            state.mission_complete = True
            return []
        
        # how many waypoints fit in remaining flight time
        segment = []
        cumulative_time = 0.0
        start_idx = state.current_waypoint_index
        
        # return-to-home time estimate
        home_position = (waypoints[0].lat, waypoints[0].lon)
        
        for i in range(start_idx, len(waypoints)):
            if i > start_idx:
                # flight time between waypoints
                dist = self._haversine_distance(
                    waypoints[i-1].lat, waypoints[i-1].lon,
                    waypoints[i].lat, waypoints[i].lon
                )
                flight_time = dist / self.cruise_speed
                
                # Check if reach waypoint AND return home
                rth_dist = self._haversine_distance(
                    waypoints[i].lat, waypoints[i].lon,
                    home_position[0], home_position[1]
                )
                rth_time = rth_dist / self.cruise_speed
                
                if cumulative_time + flight_time + rth_time > self.max_flight_time:
                    break
                
                cumulative_time += flight_time
            
            segment.append(waypoints[i])
        
        # Update state
        if segment:
            state.current_waypoint_index = start_idx + len(segment)
            state.last_position = (segment[-1].lat, segment[-1].lon)
            state.elapsed_flight_time += cumulative_time
            state.remaining_battery = 100.0 * (1 - state.elapsed_flight_time / (self.max_flight_time * len(waypoints) / len(segment)))
        
        if state.current_waypoint_index >= len(waypoints):
            state.mission_complete = True
        
        return segment
    
    def _haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate distance between two GPS coordinates in meters
        """
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def export_to_mission_planner(self, drone_id: int, segment: Optional[List[Waypoint]] = None, 
                                   filename: Optional[str] = None) -> str:
        """
        Export waypoints to Mission Planner waypoint file format (.waypoints)
        """
        if segment is None:
            segment = self.drone1_waypoints if drone_id == 1 else self.drone2_waypoints
        
        if filename is None:
            filename = f"drone{drone_id}_mission.waypoints"
        
        with open(filename, 'w') as f:
            f.write("QGC WPL 110\n")  # Mission Planner header
            
            # Waypoint 0: Home position
            if segment:
                f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{segment[0].lat}\t{segment[0].lon}\t{self.flight_altitude}\t1\n")
            
            # Mission waypoints
            for i, wp in enumerate(segment, start=1):
                # Format: index, current_wp, coord_frame, command, param1-4, lat, lon, alt, autocontinue
                # Command 16 = NAV_WAYPOINT
                f.write(f"{i}\t0\t3\t16\t0\t0\t0\t0\t{wp.lat}\t{wp.lon}\t{wp.altitude}\t1\n")
            
            # Return to launch
            if segment:
                f.write(f"{len(segment)+1}\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1\n")  # RTL command
        
        return filename
    
    def save_flight_state(self, filename: str = "flight_state.json"):
        """Save current flight state for both drones"""
        state_data = {
            "drone1": asdict(self.drone1_state) if self.drone1_state else None,
            "drone2": asdict(self.drone2_state) if self.drone2_state else None,
        }
        
        with open(filename, 'w') as f:
            json.dump(state_data, f, indent=2)
        
        return filename
    
    def load_flight_state(self, filename: str = "flight_state.json"):
        """Load saved flight state"""
        with open(filename, 'r') as f:
            state_data = json.load(f)
        
        if state_data["drone1"]:
            self.drone1_state = FlightState(**state_data["drone1"])
        if state_data["drone2"]:
            self.drone2_state = FlightState(**state_data["drone2"])
    
    def get_mission_progress(self, drone_id: int) -> Dict:
        """Get current mission progress"""
        state = self.drone1_state if drone_id == 1 else self.drone2_state
        
        if not state:
            return {"error": "Flight state not initialized"}
        
        progress_pct = (state.current_waypoint_index / state.total_waypoints) * 100
        
        return {
            "drone_id": state.drone_id,
            "progress_percent": round(progress_pct, 2),
            "waypoints_completed": state.current_waypoint_index,
            "total_waypoints": state.total_waypoints,
            "mission_complete": state.mission_complete,
            "last_position": state.last_position,
            "elapsed_flight_time_seconds": round(state.elapsed_flight_time, 2),
        }


# example usage
if __name__ == "__main__":
    field_boundary = [
        (-35.3610561, 149.1672027),
        (-35.3607236, 149.1704965),
        (-35.3626485, 149.1707969),
        (-35.3629285103559, 149.16759967804),
    ]
    
    # planner
    planner = DualDroneFlightPlanner(
        field_points=field_boundary,
        flight_altitude=50.0,  # 50 meters
        max_flight_time=900.0,  # 15 minutes
        sweep_spacing=0.00005,  # ~5.5m spacing
        cruise_speed=20.0  # 20 m/s
    )
    
    # full flight paths
    print("Generating flight paths...")
    drone1_path, drone2_path = planner.generate_flight_paths()
    print(f"Drone 1: {len(drone1_path)} waypoints")
    print(f"Drone 2: {len(drone2_path)} waypoints")
    
    # multiple flights
    print("\n=== FLIGHT 1 ===")
    segment1_d1 = planner.get_next_flight_segment(drone_id=1)
    segment1_d2 = planner.get_next_flight_segment(drone_id=2)
    
    print(f"Drone 1 Flight 1: {len(segment1_d1)} waypoints")
    print(f"Drone 2 Flight 1: {len(segment1_d2)} waypoints")
    
    # export
    mp_file1 = planner.export_to_mission_planner(drone_id=1, segment=segment1_d1)
    mp_file2 = planner.export_to_mission_planner(drone_id=2, segment=segment1_d2)
    print(f"\nMission files exported:")
    print(f"  {mp_file1}")
    print(f"  {mp_file2}")
    
    state_file = planner.save_flight_state()
    print(f"\nFlight state saved to: {state_file}")
    
    # progress
    print("\n=== MISSION PROGRESS ===")
    print(f"Drone 1: {planner.get_mission_progress(1)}")
    print(f"Drone 2: {planner.get_mission_progress(2)}")
    
    # flight 2
    print("\n=== FLIGHT 2 ===")
    segment2_d1 = planner.get_next_flight_segment(drone_id=1)
    segment2_d2 = planner.get_next_flight_segment(drone_id=2)
    
    print(f"Drone 1 Flight 2: {len(segment2_d1)} waypoints")
    print(f"Drone 2 Flight 2: {len(segment2_d2)} waypoints")
    
    if segment2_d1:
        planner.export_to_mission_planner(drone_id=1, segment=segment2_d1, 
                                         filename="drone1_mission_flight2.waypoints")
    if segment2_d2:
        planner.export_to_mission_planner(drone_id=2, segment=segment2_d2,
                                         filename="drone2_mission_flight2.waypoints")
    
    print("\n=== FINAL PROGRESS ===")
    print(f"Drone 1: {planner.get_mission_progress(1)}")
    print(f"Drone 2: {planner.get_mission_progress(2)}")
