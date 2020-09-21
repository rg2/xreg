# Point Cloud to Surface Registration
This tool performs a registration between a point cloud and a surface using the iterative most-likely point (ICP) approach.
The method optimizes over a rigid tranformation, but may also attempt to recover scale.
ICP is intialized using the identity transformation by default, but two FCSV files may be provided in order to perform a paired-point registration to provide a better initialization.
A KD-Tree of the surface is constructed in order to efficiently perform closest-point lookups.

An example of this program's usage are given in the walkthough [here](https://github.com/rg2/xreg/wiki/Walkthrough%3A-Point-Cloud-to-Surface-Registration).