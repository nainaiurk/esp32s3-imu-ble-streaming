# Binary to CSV Converter

Converts binary IMU log files from SD card into human-readable CSV format.

## Setup

1. Place `.bin` files from SD card into `bin_files/` folder

2. Run the converter (requires Python 3, no external dependencies):
```bash
python bin_to_csv.py
```

3. Check `csv_files/` folder for generated CSV files

## Output

Each CSV file contains:
- **sampleIndex**: Packet sequence number
- **timestamp**: Unix timestamp (seconds)
- **ax, ay, az**: Accelerometer (raw LSB)
- **gx, gy, gz**: Gyroscope (raw LSB)
- **rms**: RMS magnitude (fixed-point, divide by 100)
- **pitch, roll**: Orientation angles (fixed-point, divide by 100)
- **stepCount**: Total step counter
