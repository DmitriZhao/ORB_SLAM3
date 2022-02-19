#!/bin/bash
pathDatasetKITTI='/Datasets/kitti_dataset/data_odometry_color' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Monocular Examples
echo "Launching KITTI00 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI00-02.yaml "$pathDatasetKITTI"/dataset/sequences/00 dataset-KITTI00_mono

echo "Launching KITTI01 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI00-02.yaml "$pathDatasetKITTI"/dataset/sequences/01 dataset-KITTI01_mono

echo "Launching KITTI02 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI00-02.yaml "$pathDatasetKITTI"/dataset/sequences/02 dataset-KITTI02_mono

echo "Launching KITTI03 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI03.yaml "$pathDatasetKITTI"/dataset/sequences/03 dataset-KITTI03_mono

echo "Launching KITTI04 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI04-12.yaml "$pathDatasetKITTI"/dataset/sequences/04 dataset-KITTI04_mono

echo "Launching KITTI05 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI04-12.yaml "$pathDatasetKITTI"/dataset/sequences/05 dataset-KITTI05_mono

echo "Launching KITTI06 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI04-12.yaml "$pathDatasetKITTI"/dataset/sequences/06 dataset-KITTI06_mono

echo "Launching KITTI07 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI04-12.yaml "$pathDatasetKITTI"/dataset/sequences/07 dataset-KITTI07_mono

echo "Launching KITTI08 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI04-12.yaml "$pathDatasetKITTI"/dataset/sequences/08 dataset-KITTI08_mono

echo "Launching KITTI09 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI04-12.yaml "$pathDatasetKITTI"/dataset/sequences/09 dataset-KITTI09_mono

echo "Launching KITTI10 with Monocular sensor"
./Monocular/mono_kitti_old ../Vocabulary/ORBvoc.txt ./Monocular/KITTI04-12.yaml "$pathDatasetKITTI"/dataset/sequences/10 dataset-KITTI10_mono