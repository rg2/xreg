# Simulation of Fluoroscopy Images During Hip Surgery
This tool creates synthetic fluoroscopy images using geometries typically associated with hip surgery.
The tool will create a series of three-view collections, where each collection has an approximate AP view and two additional views at orbital rotations.
Nominal parameters for a [Siemens CIOS Fusion C-arm](https://www.siemens-healthineers.com/en-us/surgical-c-arms-and-navigation/mobile-c-arms/cios-fusion) with 30 cm detector are used.
Randomness is introduced by sampling the movement of the C-arm, the pose of the volume, and the detected X-ray intensities (Poisson sampling).
Although a single volume is used as input, extensions are possible that would enable multiple volumes and analytic objects.

A comprehensive listing of the program's usage may be obtained by passing `-h` or `--help`.

Several demonstrations of this tool are given in the walkthrough [here](https://github.com/rg2/xreg/wiki/Walkthrough%3A-Simulated-Fluoroscopy).