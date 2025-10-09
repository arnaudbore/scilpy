Estimate Q-ball orientation distribution functions
==================================================

Q-ball, a timeless classic. When you cannot make assumptions about the nature of the fiber response function, e.g. for small animal or infant brains, the Q-ball model can be useful. The script :ref:`scil_qball_metrics` is available for computing Q-ball orientation distribution functions (ODF) and their derived metrics.

To run the script, you need a DWI image with its corresponding b-values and b-vectors. Optionally, a mask can be provided to speed up the computation. You can run the command as follows:

::

    scil_qball_metrics dwi.nii.gz dwi.bval dwi.bvec --mask brainmask.nii.gz

By default, the script will output the following files in the current directory:

- Anisotropic power map (``anisotropic_power.nii.gz``): This metric quantifies the anisotropic component of the ODF (for more details, see Dell'Acqua et al, ISMRM, 2014).
- Generalized fractional anisotropy (GFA) map (``gfa.nii.gz``): GFA is a measure of the anisotropy of the diffusion process, similar to FA in DTI, but derived from the ODF.
- Number of fiber orientations (NuFO) map (``nufo.nii.gz``): This metric indicates the number of distinct fiber populations within a voxel, which can be useful for identifying complex fiber configurations.
- Peaks image (``peaks.nii.gz``): The ODF peaks representing the main diffusion directions within each voxel.
- Peaks indices image (``peaks_indices.nii.gz``): The indices on a discretized sphere corresponding to the ODF peaks.
- Spherical harmonics coefficients image (``sh.nii.gz``): The coefficients of the spherical harmonics representation of the ODF.

To skip some outputs, you can use the ``--not_all`` flag. For example, if you only want the GFA and NuFO maps, you can run:

::

    scil_qball_metrics dwi.nii.gz dwi.bval dwi.bvec --mask brainmask.nii.gz --not_all --gfa gfa.nii.gz --nufo nufo.nii.gz
