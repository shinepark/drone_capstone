from drone_flight_planner import DualDroneFlightPlanner
import json


def example_workflow():
    
    # define field boundary
    my_field = [
        (-35.3610561, 149.1672027),
        (-35.3607236, 149.1704965),
        (-35.3626485, 149.1707969),
        (-35.3629285103559, 149.16759967804),
    ]
    
    planner = DualDroneFlightPlanner(
        field_points=my_field,
        flight_altitude=60.0,          # 60 meters altitude
        max_flight_time=15 * 60,       # 15 minutes in seconds
        sweep_spacing=0.00005,         # ~5.5m spacing between lines
        cruise_speed=20.0,             # 20 m/s cruise speed
        battery_reserve=0.20           # 20% battery safety margin
    )
    
    # complete flight paths
    print("\n Generating optimized flight paths...")
    drone1_waypoints, drone2_waypoints = planner.generate_flight_paths()
    
    print(f"    Drone 1 path: {len(drone1_waypoints)} waypoints")
    print(f"    Drone 2 path: {len(drone2_waypoints)} waypoints")
    
    # calculate estimated total flights needed based on field size
    total_flights = 0
    drone_id = 1
    
    temp_state = planner.drone1_state.current_waypoint_index
    while not planner.drone1_state.mission_complete:
        segment = planner.get_next_flight_segment(drone_id=1)
        if segment:
            total_flights += 1
        else:
            break
    
    planner.drone1_state.current_waypoint_index = temp_state
    planner.drone1_state.mission_complete = False
    
    print(f"\n Mission Planning Summary:")
    print(f"    Estimated flights per drone: ~{total_flights}")
    print(f"    Total field coverage time: ~{total_flights * 15} minutes per drone")
    print(f"    sFlight altitude: {planner.flight_altitude}m AGL")
    
    # first flight segments
    print(f"\n Preparing Flight #1 for both drones...")
    
    flight1_drone1 = planner.get_next_flight_segment(drone_id=1)
    flight1_drone2 = planner.get_next_flight_segment(drone_id=2)
    
    print(f"    Drone 1 - Flight 1: {len(flight1_drone1)} waypoints")
    print(f"    Drone 2 - Flight 1: {len(flight1_drone2)} waypoints")
    
    print(f"\n Exporting to Mission Planner...")
    
    mp1 = planner.export_to_mission_planner(
        drone_id=1, 
        segment=flight1_drone1,
        filename="drone1_flight1.waypoints"
    )
    
    mp2 = planner.export_to_mission_planner(
        drone_id=2,
        segment=flight1_drone2, 
        filename="drone2_flight1.waypoints"
    )
    
    print(f"\n Saving flight state...")
    state_file = planner.save_flight_state("mission_state.json")
    
    print(f"\n Mission Progress After Flight 1:")
    progress1 = planner.get_mission_progress(1)
    progress2 = planner.get_mission_progress(2)
    
    print(f"    Drone 1: {progress1['progress_percent']:.1f}% complete")
    print(f"             ({progress1['waypoints_completed']}/{progress1['total_waypoints']} waypoints)")
    print(f"    Drone 2: {progress2['progress_percent']:.1f}% complete")
    print(f"             ({progress2['waypoints_completed']}/{progress2['total_waypoints']} waypoints)")
    
    print(f"\n battery swap")    
    flight2_drone1 = planner.get_next_flight_segment(drone_id=1)
    flight2_drone2 = planner.get_next_flight_segment(drone_id=2)
    
    if flight2_drone1:
        print(f"    Drone 1 - Flight 2: {len(flight2_drone1)} waypoints")
        planner.export_to_mission_planner(
            drone_id=1,
            segment=flight2_drone1,
            filename="drone1_flight2.waypoints"
        )
    
    if flight2_drone2:
        print(f"    Drone 2 - Flight 2: {len(flight2_drone2)} waypoints")
        planner.export_to_mission_planner(
            drone_id=2,
            segment=flight2_drone2,
            filename="drone2_flight2.waypoints"
        )
    
    print(f"\n Final Mission Status:")
    final_progress1 = planner.get_mission_progress(1)
    final_progress2 = planner.get_mission_progress(2)
    
    print(f"    Drone 1: {final_progress1['progress_percent']:.1f}% complete")
    print(f"    Drone 2: {final_progress2['progress_percent']:.1f}% complete")
    
    if final_progress1['mission_complete']:
        print(f"drone 1 done")
    if final_progress2['mission_complete']:
        print(f"drone 2 done")
    
    planner.save_flight_state("mission_state_final.json")


def resuming_from_saved_state():
    
    my_field = [
        (-35.3610561, 149.1672027),
        (-35.3607236, 149.1704965),
        (-35.3626485, 149.1707969),
        (-35.3629285103559, 149.16759967804),
    ]
    
    planner = DualDroneFlightPlanner(
        field_points=my_field,
        flight_altitude=60.0,
        max_flight_time=15 * 60,
        sweep_spacing=0.00005,
        cruise_speed=20.0
    )
    
    planner.generate_flight_paths()
    
    # Load saved state
    print("\n Loading saved mission state...")
    try:
        planner.load_flight_state("mission_state.json")
        
        # Check where left off
        progress1 = planner.get_mission_progress(1)
        progress2 = planner.get_mission_progress(2)
        
        print(f"\n Current Progress:")
        print(f"    Drone 1: {progress1['progress_percent']:.1f}% complete")
        print(f"    Drone 2: {progress2['progress_percent']:.1f}% complete")
        
        # next segments
        print(f"\n Generating next flight segments...")
        next_segment1 = planner.get_next_flight_segment(drone_id=1)
        next_segment2 = planner.get_next_flight_segment(drone_id=2)
        
        if next_segment1:
            print(f"    Drone 1: {len(next_segment1)} waypoints")
            planner.export_to_mission_planner(
                drone_id=1,
                segment=next_segment1,
                filename="drone1_next_flight.waypoints"
            )
        
        if next_segment2:
            print(f"    Drone 2: {len(next_segment2)} waypoints")
            planner.export_to_mission_planner(
                drone_id=2,
                segment=next_segment2,
                filename="drone2_next_flight.waypoints"
            )
    except FileNotFoundError:
        print("Start a new mission first.")


if __name__ == "__main__":
    example_workflow()
    
    # resuming_from_saved_state()
