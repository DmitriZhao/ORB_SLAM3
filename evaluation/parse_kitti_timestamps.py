import os
import numpy as np
import pandas as pd
from pathlib import Path


#%%
raw_sequence_name = [
    '2011_10_03_drive_0027', '2011_10_03_drive_0042',
    '2011_10_03_drive_0034', None,
    '2011_09_30_drive_0016', '2011_09_30_drive_0018',
    '2011_09_30_drive_0020', '2011_09_30_drive_0027',
    '2011_09_30_drive_0028', '2011_09_30_drive_0033',
    '2011_09_30_drive_0034'
]
raw_sequence_start = [0] * 8 + [1100] + [0] * 2
raw_sequence_end = [
    4540, 1100, 4660, 800, 270, 2760, 1100, 1100, 5170, 1590, 1200, 382]

kitti_root = Path('/data/datasets/kitti/')
kitti_unsynced = kitti_root / "raw_unsynced"
kitti_synced = kitti_root / "raw_synced"
kitti_fixed = kitti_root / "raw_fixed"
kitti_odom = kitti_root / "data_odometry_gray"

#%%
# Odometry Seq
SEQUENCE = 9
kitti_subset = raw_sequence_name[SEQUENCE]
kitti_set = kitti_subset[:10]


# Read image timestamps
image_timestamps_file = kitti_synced / kitti_set / f"{kitti_subset}_sync" / 'image_00' / "timestamps.txt"
def read_timestamp_file(file):
    with open(file, 'r') as file:
        data = file.readlines()
    return [pd.to_datetime(ts, format="%Y-%m-%d %H:%M:%S.%f\n") 
        for ts in data]

def datetimes_to_unix_timestamps(datetimes):
    return np.array([
        (t - np.datetime64('1970-01-01T00:00:00')).astype(int)
            for t in pd.to_datetime(datetimes).to_numpy()])

# read timestamps
image_timestamps = read_timestamp_file(image_timestamps_file)
# convert to unix timestamps
image_timestamps = datetimes_to_unix_timestamps(image_timestamps)  # in nanoseconds
print(image_timestamps.shape)

index_start = raw_sequence_start[SEQUENCE]
index_end = raw_sequence_end[SEQUENCE] + 1
# select timestamps
image_timestamps = image_timestamps[index_start:index_end]
print(image_timestamps.shape)

#%%
# write timestamps in nanoseconds without decimals
output_file = kitti_fixed / "KITTI_Images" / f'{SEQUENCE:02}_timestamps.txt'
output_file.parent.mkdir(parents=True, exist_ok=True)
with open(output_file, 'w') as file:
    for timestamp in image_timestamps:
        file.write(f'{timestamp:.0f}\n')