# 2D/3D Pelvis, Femur and PAO Fragment Registration Using Multiple Views
This tool performs a 2D/3D registration of multiple bones using information from multiple views.
The motivating example used here is to obtain and intraoperative estimate the relative poses of the femur and periacetabular (PAO) bone fragment with respect to the anterior pelvic plane (APP) of the pelvis.

Much like the [single-view pelvis tool](../../pelvis_single_view_regi_2d_3d), the processing begins by first solving a single-view PnP problem for the pelvis and solving intensity-based registrations.
The pelvis pose is then registered using an intensity-based registration using all available views.
Note that the relative poses between each view are assumed to be known in this example.
The femur is then registered using multiple views, keeping the pelvis fixed at its current estimate.
Next, the fragment is registered while keeping the poses of the pelvis and femur fixed in the background.
This strategy continues at a finer resolution level: the pose of each object is refined while keeping the poses of the remaining two objects fixed.
Finally, the poses of all objects are refined simultaneous.
This is a similar strategy employed in (Grupp 2020) for registering a PAO fragment once the relative motion of the C-arm was recovered and the shape of the fragment was refined.
The videos produced by the example below also help to visualize the different registration stages.

The tool requires the following information as input:
  * 3D volume of the objects to be registered (stored in an ITK compatible file format, such as NIFTI/`.nii`/`.nii.gz`)
  * 3D segmentation labeling the pelvis, femur, and fragment to be registered - this is used to mask the objects in the input volume file and tightly crop about them (stored in an ITK compatible file format, such as NIFTI/`.nii`/`.nii.gz`)
  * The side which contains the fragment and femur to be registered; either "left" or "right"
  * 3D landmarks of the pelvis which are used to define the APP with an origin at the center of the ipsilateral femoral head (stored in FCSV format (`.fcsv`))
  * 3D landmarks of the pelvis which are used to solve the PnP problem (stored in FCSV format (`.fcsv`))
  * A collection of 2D views along with 2D landmarks in at least one view (stored in the [xReg projection data format](https://github.com/rg2/xreg/wiki/Projection-Data-HDF5-File))

The following information is optional input:
  * The projection index to be used for performing PnP initialization, this defaults to the first projection

Each registered pose of the pelvis, femur, and fragment are written to disk along with the two relative poses of the femur and fragment.

Optionally, a path to an output debug file may be provided.

A comprehensive listing of the program's usage may be obtained by passing `-h` or `--help`.

An example of this program's usage is given in the walkthrough [here](https://github.com/rg2/xreg/wiki/Walkthrough%3A-Multiple-View-PAO-Fragment-Registration).