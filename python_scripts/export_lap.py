import argparse
import csv
import math

# get solved lap - x_racingline_path.csv - convert speed to m/s
# time
# distance
# speed
# longitudinal acceleration (u) - change in speed over distance multiplied by speed, form rust optimal.r
# lateral acceleration - speed^2 times curvature, form rust optimal.r
# grip usage,  mu · (G + c_l · v²) - form rust optimal.rs

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--track", default="singapore", help="driven lap location, e.g. singapore, suzuka")
args = parser.parse_args()

slug = args.track.lower()

speeds = []
xs = []
ys = []
a_lats = []
a_lons = [] # u
grip = []
kappa = []
with open(f'data/{slug}_racingline_path.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        speeds.append(float(row['speed_kmh']) / 3.6)
        xs.append(float(row['x_line']))
        ys.append(float(row['y_line']))
        a_lats.append(float(row['a_lat']))
        a_lons.append(float(row['u']))
        grip.append(float(row['grip_usage']))
        kappa.append(float(row['kappa']))

times = []
distance_m = []
total_t = 0.0
total_d = 0.0

# computes time and distance per step
for i in range(0,len(xs)):
    j = (i + 1) % len(xs)
    times.append(total_t)
    sl_dist = math.hypot((xs[j] - xs[i]), (ys[j] - ys[i]))
    total_t += sl_dist * 0.5 * (1 / speeds[i] + 1 / speeds[j])

    distance_m.append(total_d)
    total_d += sl_dist


# writing to file
with open(f'data/{slug}_lap_telemetry.csv', 'w', newline='') as csvfile:
    spamwriter = csv.writer(csvfile)
    spamwriter.writerow(['time_s']+['distance_m']+['speed_ms']+['lon_acc_ms2']+['lat_acc_ms2']+['grip_usage'])
    for i in range (len(times)):
        spamwriter.writerow([times[i], distance_m[i], speeds[i], a_lons[i], a_lats[i], grip[i]])