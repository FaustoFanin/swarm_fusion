# swarm_fusion
This repo contains the ROS sub-network for data fusion of an autonomous swarm developed for a [Group Design Project](https://www.cranfield.ac.uk/press/news-2019/bae-competition-challenges-students-to-counter-threat--from-uavs) that was part of my MSc thesis at Cranfield University. We had to design and develop an autonomous UAV swarm that was capable of detecting, reacting and beating an enemy swarm by either delivering payload and/or eliminating enemy agents.

The autonomous system was divided into the following sections:
- GNC - Local ROS network on-board each UAV to manage flight computer
- System Perception - Sensor station capable of detecting and locating drones in the environment
- Mission Planning - Intelligence aspect of system that interpreted situational awareness (friendly & enemy drones) and generated commands for the swarm
- Situational Awareness - Subsystem that fused data from friendly swarm GNC modules and System Perception infomation

My contribution to this project was developing the ROS framework, communication network and the Situational Awareness systems. The project contains my contribution to this project, the other sections of the system haven't been included as requested by their respective creators.

### Project Breakdown
This project contains the following subsections:
###### Communication
Our communication network utilised XBee 2.4GHz modules and the following files handles this section of the system:
- comm_agent_node.py
- comm_gnd_node.py
- comm_module.py
- xbee_module.py

###### Data Fusion
This subsection received the estimated location of detected drones and the swarm information from the on-board GNC modules and fused this data, delivering an estimate of the enemy swarms position and velocity. The rest of the files belong to this subsection:
- filter_module.py
- frame_module.py
- fusion_module.py
- detection_node.py
