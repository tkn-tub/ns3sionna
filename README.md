ns3sionna
============

<img src="./res/ns3sionna_logo_small_sw.jpg" width="15%" alt="Logo" style="text-align: left; vertical-align: middle;">

ns3sionna is a software module that brings realistic channel simulation using ray tracing from 
Sionna (https://nvlabs.github.io/sionna/) to the widely used ns-3 network simulator (https://github.com/nsnam).

<table style="width:70%">
<tr>
<td><img src="./res/munich2.png" width="75%" alt="Outdoor scenario: area around Frauenkirche in Munich" style="text-align: center; vertical-align: middle;"></td>
<td><img src="./res/ex2_munich_paper.jpg" width="90%" alt="Results for outdoor scenario: trajectory of STA, CSI, Prx over time, distance vs. Prx." style="text-align: center; vertical-align: middle;"></td>
</tr>
<tr>
<td>Outdoor scenario (Munich)</td>
<td>Results from simulation</td>
</tr>
</table>


More information can be found in our paper: [Ns3Sionna paper](https://arxiv.org/abs/2412.20524)

Installation
============

We recommend using Linux (e.g. Ubuntu 22 or higher) and a multi-core/GPU compute node.

1. Download ns3

```
wget https://www.nsnam.org/releases/ns-allinone-3.40.tar.bz2
tar xf ns-allinone-3.40.tar.bz2
cd ns-allinone-3.40
```

2. Download ns3-sionna

```
git clone https://github.com/tkn-tub/ns3sionna.git
cd ns3sionna/
./install_packages.sh
./bootstrap.sh
```

3. Build everything in ns3
```
cd ../ns-3.40/
./ns3 configure -d debug --enable-examples
./ns3 build
```

4. Build sionna server
```
cd ../ns3sionna/sionna_server/
python3 -m venv sionna-venv
source sionna-venv/bin/activate
python3 -m pip install -r requirements.txt # or requirements_gpu.txt if you want to use GPUs
python3 test_imports.py # all packages should be correctly installed
```

5. Start sionna server
```
cd ./ns3sionna/sionna_server/
source sionna-venv/bin/activate
python3 sionna_server.py
```

5. Start a ns-3 example script
```
cd ns-3.40/
./ns3 run scratch/ns3-sionna/example-sionna-sensing-mobile
```

6. Plot the results
```
cd ns-3.40/
python3 ./scratch/ns3-sionna/plot3d_mobile_csi.py example-sionna-sensing-mobile.csv example-sionna-sensing-mobile-pathloss.csv example-sionna-sensing-mobile-time-pos.csv
```

Examples
========

All examples can be found [here](./ns3-sionna/).

Current limitations
========
* SISO only

Contact
============
* Anatolij Zubow, TU-Berlin, zubow@tkn
* Sascha Roesler, TU-Berlin, zubow@tkn
* tkn = tkn.tu-berlin.de

How to reference ns3sionna?
============

Please use the following bibtex:

```
@techreport{zubow2024ns3-preprint,
    author = {Zubow, Anatolij and Pilz, Yannik and R{\"{o}}sler, Sascha and Dressler, Falko},
    doi = {10.48550/arXiv.2412.20524},
    title = {{Ns3 meets Sionna: Using Realistic Channels in Network Simulation}},
    institution = {arXiv},
    month = {12},
    number = {2412.20524},
    type = {cs.NI},
    year = {2024},
   }
```
