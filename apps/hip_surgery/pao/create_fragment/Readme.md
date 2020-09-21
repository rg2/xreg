# Modeling and Planning Periacetabular Osteotomies
This tool may be used to construct volumetric labelings of periacetabular osteotomy (PAO) fragment shapes.
The fragment shape is defined by a series of four cuts through the pelvis and about the acetabulum.
Six 3D anatomical landmarks are the primary mechanism used to define each PAO cut.
An anterior point on the ilium (AIl), typically on the iliac spine, and a more posteriorly located ilium point (PIl) are used to define the ilium cut.
The ischial cut is also defined with anterior (AIs) and posterior (PIs) points on the cutting plane.
The posterior points of the ilium and ischial cuts (PIl and PIs) are used to define the posterior cut.
Finally, the pubic cut is defined by a point on the superior aspect of the pubis (SP) and another point on the inferior aspect of the pubis (IP).
When annotating these landmarks, the left and right side is indicated by appending "-r" or "-l" to each landmark's name.
For example, "AIs-r" is the right anterior ischial cut landmark and "PIl-l" is the posterior ilium cut landmark.
The following landmarks must also be annotated in order to define the Anterior Pelvic Plane (APP) coordinate frame:
* Left and right anterior superior iliac spine (ASIS-{l,r})
* Left and right superior pubic symphyses (SPS-{l,r})
* Left and right inferior pubic symphyses (IPS-{l,r})
* Left and right femoral head centers (FH-{l,r})

A comprehensive listing of the program's usage may be obtained by passing `-h` or `--help`.

Examples of this program's usage is provided in the walkthrough [here](https://github.com/rg2/xreg/wiki/Walkthrough%3A-PAO-Planning).