#!/usr/bin/env python3
"""
Convert binary IMU SD card logs to CSV format
FeaturePacket structure: 32 bytes per packet
- sampleIndex (uint32): 4 bytes
- timestamp (uint32): 4 bytes
- ax, ay, az (int16 each): 6 bytes
- gx, gy, gz (int16 each): 6 bytes
- rms (int16): 2 bytes
- pitch (int16): 2 bytes
- roll (int16): 2 bytes
- stepCount (uint32): 4 bytes
"""

import struct
import os
import sys
from pathlib import Path

# ------------- CONFIGURATION ----------
# Folder containing .bin files
INPUT_FOLDER = "bin_to_csv/bin_files"

# Folder where CSV files will be saved (will be created if doesn't exist)
OUTPUT_FOLDER = "bin_to_csv/csv_files"

def convert_bin_to_csv(bin_file, csv_file=None):
    """Convert a single binary file to CSV"""
    
    if not os.path.exists(bin_file):
        print(f"Error: File not found: {bin_file}")
        return False
    
    # Default CSV filename if not provided
    if csv_file is None:
        csv_file = os.path.splitext(bin_file)[0] + '.csv'
    
    # Create output directory if it doesn't exist
    os.makedirs(os.path.dirname(csv_file), exist_ok=True)
    
    try:
        with open(bin_file, 'rb') as f:
            data = f.read()
        
        file_size = len(data)
        packet_size = 30  # FeaturePacket: sampleIndex(4) + timestamp(4) + ax/ay/az(6) + gx/gy/gz(6) + rms(2) + pitch(2) + roll(2) + stepCount(4) = 30 bytes
        num_packets = file_size // packet_size
        
        print(f"Converting {bin_file}...")
        print(f"File size: {file_size} bytes")
        print(f"Packet size: {packet_size} bytes")
        print(f"Found {num_packets} packets")
        
        if file_size % packet_size != 0:
            print(f"Warning: File size {file_size} is not multiple of {packet_size} (remainder: {file_size % packet_size} bytes)")
        
        if num_packets == 0:
            print(f"Warning: No complete packets found in file")
        
        # Write CSV
        with open(csv_file, 'w') as f:
            # Header
            f.write("SampleIndex,Timestamp,Ax,Ay,Az,Gx,Gy,Gz,RMS,Pitch,Roll,StepCount\n")
            
            packets_written = 0
            errors = 0
            first_sample = None
            last_sample = None
            
            # Parse packets
            for i in range(num_packets):
                offset = i * packet_size
                packet_data = data[offset:offset + packet_size]
                
                if len(packet_data) < packet_size:
                    break
                
                # Unpack: < means little-endian
                # I=uint32(4), h=int16(2)
                # Format: sampleIndex(I) + timestamp(I) + ax/ay/az(hhh) + gx/gy/gz(hhh) + rms(h) + pitch(h) + roll(h) + stepCount(I)
                try:
                    sampleIndex, timestamp, ax, ay, az, gx, gy, gz, rms, pitch, roll, stepCount = \
                        struct.unpack('<II hhhhhh hhh I', packet_data)
                    
                    # Convert scaled values (rms, pitch, roll were stored as int16 * 100)
                    rms_val = rms / 100.0
                    pitch_val = pitch / 100.0
                    roll_val = roll / 100.0
                    
                    f.write(f"{sampleIndex},{timestamp},{ax},{ay},{az},{gx},{gy},{gz},{rms_val},{pitch_val},{roll_val},{stepCount}\n")
                    packets_written += 1
                    
                    # Track first and last sample numbers
                    if first_sample is None:
                        first_sample = sampleIndex
                    last_sample = sampleIndex
                except struct.error as e:
                    print(f"Error parsing packet {i}: {e}")
                    errors += 1
                    if errors > 5:
                        print("Too many errors, stopping conversion")
                        break
        
        print(f"Successfully wrote {packets_written} packets to: {csv_file}")
        if errors > 0:
            print(f"Encountered {errors} parsing errors")
        if packets_written > 0:
            print(f"Sample range: {first_sample} to {last_sample}")
        return True
        
    except Exception as e:
        print(f"Error converting {bin_file}: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main entry point"""
    
    # Create output folder if it doesn't exist
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    print(f"Output folder: {OUTPUT_FOLDER}\n")
    
    # Check if input folder exists
    if not os.path.isdir(INPUT_FOLDER):
        print(f"Error: Input folder not found: {INPUT_FOLDER}")
        return
    
    # Find all .bin files in input folder
    bin_files = list(Path(INPUT_FOLDER).glob("*.bin"))
    
    if not bin_files:
        print(f"No .bin files found in: {INPUT_FOLDER}")
        return
    
    print(f"Input folder: {INPUT_FOLDER}")
    print(f"Found {len(bin_files)} binary file(s) to convert\n")
    
    success_count = 0
    total_packets = 0
    for bin_file in sorted(bin_files):
        # Create output filename with original name in output folder
        output_csv = os.path.join(OUTPUT_FOLDER, os.path.splitext(os.path.basename(bin_file))[0] + '.csv')
        if convert_bin_to_csv(str(bin_file), output_csv):
            success_count += 1
            # Count packets from file size
            try:
                file_size = os.path.getsize(str(bin_file))
                packets = file_size // 30
                total_packets += packets
            except:
                pass
        print()
    
    print(f"Conversion complete: {success_count}/{len(bin_files)} files converted")
    print(f"Total packets: {total_packets}")


if __name__ == "__main__":
    main()
