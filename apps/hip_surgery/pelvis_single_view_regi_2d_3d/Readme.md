# 2D/3D Pelvis Registration Using a Single-View
This tool will perform a registration of a single 3D object (e.g. the pelvis), represented by a CT scan, to a single 2D fluoroscopy view.
The tool first computes a paired point registration using anatomical landmarks previously identified in the 3D volume and the 2D view.
A multiple-resolution intensity-based registration is then performed using the pose produced by the paired point method as an initial estimate.
The final pose estimate is written to disk.
Optionally, some debug information is also written which is useful for determining a successful registration or for debugging any problems with the processing.

The tool requires the following information as input:
  * 3D volume of the object to be registered (stored in an ITK compatible file format, such as NIFTI/`.nii`/`.nii.gz`)
  * 3D landmarks of the object to be registered (stored in FCSV format (`.fcsv`))
  * A 2D view along with 2D landmarks (stored in the [xReg projection data format](https://github.com/rg2/xreg/wiki/Projection-Data-HDF5-File)

The following information is optional input:
  * 3D segmentation of the object to be registered - this is used to mask the object in the input volume file and tightly crop about it (stored in an ITK compatible file format, such as NIFTI/`.nii`/`.nii.gz`)
  * The projection index to be registered (if the projection data contains multiple projections), this defaults to the first projection

A comprehensive listing of the program's usage may be obtained by passing `-h` or `--help`.

An example demonstrating this tool's usage is given in the walkthrough [here](https://github.com/rg2/xreg/wiki/Walkthrough%3A-Single-View-Pelvis-Registration).