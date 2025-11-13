#!/bin/bash

cd ../../

echo "Running ns3"
./ns3 run scratch/ns3-sionna/example-sionna-sensing-mobile

echo "Plotting results"
python3 ./scratch/ns3-sionna/plot3d_mobile_csi.py example-sionna-sensing-mobile.csv example-sionna-sensing-mobile-pathloss.csv example-sionna-sensing-mobile-time-pos.csv