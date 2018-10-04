# PAWS Low Energy Front-End README #

This folder contains the pcb files necessary to produce the PAWS LE PCB, which uses a custom ASIC, as well as the firmware for the BLE chip that is used to communicate with the smartphone application.

* Note: The PAWS project was at some point known as SEUS, which is why SEUS is used at some points rather than PAWS.

# Reproducing the PAWS Low Energy Front-End System

Fabricate the pcb according to the schematic and Altium Circuit Maker Files in **./pcb/AltiumCMFiles**.
Alternatively, you can directly access the project on the Altium CircuitMaker software or your web browser [here](https://circuitmaker.com/Projects/Details/Stephen-Xia/ICSL-PS-REV-02).

Once the PCB is assembled, the Nordic nRF52 chip can be flashed using the firmware located in **./nordic_nRF52/pca10040/s132/arm5_no_packs/_build/nrf52832_xxaa.hex**.

# Building and Flashing Your Own Source Code

Instructions on building your own source code and required libraries for the Nordic nRF52 Chip for wireless transmission can be found in the [Nordic website](https://infocenter.nordicsemi.com/index.jsp).
The SDK used to develop this project was the **nRF5 SDK v12.0.0** with SoftDevice **S132 v3.0.0**.

# More About the PAWS Project
To learn more about PAWS, please visit our [project page](http://icsl.ee.columbia.edu/projects/seus), or contact us at: [stephen.xia@columbia.edu](stephen.xia@columbia.edu). 	 
	
This repository is part of the **Pedestrian Audio Wearable System (PAWS)** project of the **Intelligent and Connected Systems Lab (ICSL)**, Columbia University.
For more information about our latest projects, please visit our [group website](http://icsl.ee.columbia.edu).