# README #

This repository contains the code, pcb design, and documentation for the **Pedestrian Audio Wearable System (PAWS)** front-end, headset, embedded platform.

PAWS is a low-power connected system for improving pedestrian safety in current and future smart cities. PAWS uses microphones embedded into a headset combined with low-power feature extraction, signal processing, and machine learning, for detecting, localizing, and providing alerts of oncoming vehicles to pedestrians in noisy environments.

* Note: The PAWS project was at some point known as SEUS, which is why SEUS is used at some points rather than PAWS.

The contents of this repository are summarized below.

* **PAWS**: Contains the PAWS front-end source code and designs.
	* nordic_nRF52: Contains the project files for building and flashing the firmware for the Nordic nRF52 BLE chip used for wireless communication with the PAWS smartphone application.
	* stm32f4: Contains the project files for building and flashing the firmware for the STM32F4 chip used for feature extraction in PAWS.
	* pcb: Contains the pcb schematics, designed using Altium Circuitmaker, for the PAWS embedded platform as well as documentation for how to maintain the PAWS headset. The platform uses a STM32F4 Chip for feature extraction + the Nordic nRF52 BLE chip for transmission to the smartphone.
* **PAWS_LE**: Contains the PAWS LE front-end source code and designs. PAWS LE is the low-energy variant of PAWS and has finer granularity car localization, achieved by using a technique called *Angle via Polygonal Regression (AvPR)*.
	* nordic_nRF52: Contains the project files for building and flashing the firmware for the Nordic nRF52 BLE chip used for wireless communication with the PAWS LE smartphone application.
	* pcb: Contains the pcb schematics, designed using Altium Circuitmaker, for the PAWS LE embedded platform. The platform uses a custom-build ASIC for relative delay extraction + the Nordic nRF52 BLE chip for transmission to the smartphone.

# Reproducing the PAWS system

To reproduce the PAWS system, you must first install the PAWS smartphone application; details on how to do this can be found in the PAWS-Smartphone repository.

Next, you must reproduce the PAWS front-end PCB. Details on how to do this can be found in the **./PAWS** folder.

# Reproducing the PAWS LE System

To reproduce the PAWS LE system, you must first install the PAWS LE smartphone application; details on how to do this can be found in the PAWS-Smartphone repository.

Next, you must reproduce the PAWS LE front-end PCB. Details on how to do this can be found in the **./PAWS_LE** folder.

# More About the PAWS Project
To learn more about PAWS, please visit our [project page](http://icsl.ee.columbia.edu/projects/seus), or contact us at: [stephen.xia@columbia.edu](stephen.xia@columbia.edu). 	 
	
This repository is part of the **Pedestrian Audio Wearable System (PAWS)** project of the **Intelligent and Connected Systems Lab (ICSL)**, Columbia University.
For more information about our latest projects, please visit our [group website](http://icsl.ee.columbia.edu).