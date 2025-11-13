#!/bin/bash

cd ../../

echo "Running ns3"
./ns3 run scratch/ns3-sionna/example-mobility-sionna

echo "Plotting results"
python3 ./scratch/ns3-sionna/plot_snr.py example-mobility-sionna-snr.csv