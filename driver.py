import socket
import json
import time
import math
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/E7E7E7E7E7'

def main():
    # 1. Setup UDP Listener
    udp_ip = "0.0.0.0" # Listen on all network interfaces
    udp_port = 9000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((udp_ip, udp_port))
    sock.settimeout(0.5) # Don't block forever waiting for data

    print(f"👂 Listening for Docker commands on UDP port {udp_port}...")

    # 2. Init Crazyflie drivers
    cflib.crtp.init_drivers()

    print(f"🚁 Connecting to Crazyflie at {URI}...")
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("✅ Connected!")
        
        # MotionCommander handles taking off nicely! Setting default_height to 0.6 meters (2x normal)
        with MotionCommander(scf, default_height=0.6) as mc:
            print("🚀 Took off! Waiting for vision commands...")
            
            while True:
                try:
                    # Catch the UDP packet from the Docker Container
                    data, addr = sock.recvfrom(1024)
                    telemetry = json.loads(data.decode('utf-8'))
                    
                    vx = telemetry['vx']
                    vy = telemetry['vy']
                    vz = telemetry['vz']
                    yaw = telemetry['yaw']
                    
                    # Convert rad/s yaw to degrees/sec for cflib
                    yaw_deg = math.degrees(yaw)

                    # Send velocity vector to MotionCommander
                    mc.start_linear_motion(vx, vy, vz, yaw_deg)
                    
                    # print(f"📡 Commanding -> VX: {vx:.2f}, VY: {vy:.2f}, VZ: {vz:.2f}, YAW: {yaw_deg:.2f}")

                except socket.timeout:
                    # If we don't hear from the controller for 0.5s, hover safely
                    print("⚠️ Command timeout, hovering...")
                    mc.stop()
                except KeyboardInterrupt:
                    print("\n🛑 Landing requested...")
                    break
                    
            print("🛬 Landing...")

if __name__ == '__main__':
    main()
