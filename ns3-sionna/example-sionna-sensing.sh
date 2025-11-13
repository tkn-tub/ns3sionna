#!/bin/bash

cd ../../

echo "Running ns3"
./ns3 run scratch/ns3-sionna/example-sionna-sensing

echo "Plotting results"
python3 ./scratch/ns3-sionna/plot_csi.py csi_node0.csv csi_node1.csv
