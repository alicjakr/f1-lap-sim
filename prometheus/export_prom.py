import argparse
from typing import List
import datetime
from math import modf

from datetime import date, datetime, time

def get_metrics(csvline: str) -> List[str]:
    result = []
    values = csvline.split(',')

    # divide time into seconds and microseconds
    if values[0] != "time_s":
        b,a = modf(float(values[0]))
        mins = int(a / 60)
        secs = int(a % 60)
        msecs = int(b*10e5) # accuracy up to 6 numbers
        dt = datetime.combine(date.today(), time(0, mins, secs, msecs)).timestamp()

    if values[0] != "time_s":
        result.append(f'distance_m{{track="{slug}"}} {values[1]} {dt}')
    if values[0] != "time_s":
        result.append(f'speed_ms{{track="{slug}"}} {values[2]} {dt}')
    if values[0] != "time_s":
        result.append(f'lon_acc_ms2{{track="{slug}"}} {values[3]} {dt}')
    if values[0] != "time_s":
        result.append(f'lat_acc_ms2{{track="{slug}"}} {values[4]} {dt}')
    if values[0] != "time_s":
        result.append(f'grip_usage{{track="{slug}"}} {values[5]} {dt}')

    return result

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--track", default="singapore", help="driven lap location, e.g. singapore, suzuka")
args = parser.parse_args()

slug = args.track.lower()

with open(f'data/{slug}_lap_telemetry.csv') as infile:
    with open(f'prometheus/{slug}_lap_telemetry.prom', 'w') as outfile:
        for line in infile:
            metrics = get_metrics(line.rstrip())
            for metric in metrics:
                outfile.write(metric + '\n')
        outfile.write('# EOF')