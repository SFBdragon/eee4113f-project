# EEE4113F

A repository containing additional data and raw files used in the EEE4113F final report.

## Overview

This repository includes code, videos, images, and logs used to better understand Antarctic climate conditions and the conditions under which research operations are carried out. These files informed multiple critical design decisions and trade-offs that impacted the final design of the connector pod.

## Key Evidence Files

The following files are key to understanding the design justifications:

- `Media/Videos/20220724_SB04_retrival.mov`  
  Shows retrieval from the stand after a 5-day data collection deployment on the buoy. This supports the assumption that ice does not accumulate significantly over a short deployment.

- `Media/Images/20220724_SB01_01_Taylor_N.JPG`  
- `Media/Images/20220724_SB01_02_Taylor_N.JPG`  
- `Media/Images/Sunken_SharcBuoy.jpg`  

  These images show the gradual sink event and support the assumption that the sink event occurs over approximately 15 minutes.

## MATLAB Simulation Code

The MATLAB hydrostatic simulation code used to model buoyancy, waterline height, connector seam clearance, metacentric height, and ballast sensitivity is available at:

- `MATLAB_Code/Hydrostatic_Simulation.m`

## Model Files

STL files for all models are available inside the `/Modelfiles` directory.

## Print Settings

### Print Setting 1

Used for the following components:

- `/Model_Files/ballast_body.stl`
- `/Model_Files/floatation_Ring.stl`
- `/Model_Files/sb_cover.stl`
- `/Model_Files/connector_pod.stl`

Settings:

- Infill: 5%
- Layer height: 0.28 mm
- Supports: Tree (Auto)
- Material: Generic PLA

### Print Setting 2

Used for the following components:

- `/Model_Files/ballast_lid.stl`
- `/Model_Files/ballast_nut.stl`
- `/Model_Files/ballast_screw.stl`
- `/Model_Files/component_holder.stl`

Settings:

- Infill: 20%
- Layer height: 0.28 mm
- Supports: Tree (Auto)
- Material: Generic PLA