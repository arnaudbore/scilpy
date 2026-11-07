#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert fiber response function (FRF) text files between Scilpy/DIPY format
(tensor eigenvalues + S0) and MRtrix3 format (spherical harmonic coefficients).
"""

import argparse
import logging
import numpy as np

from dipy.core.gradients import gradient_table
from dipy.reconst.shm import real_sh_descoteaux_from_index
from dipy.sims.voxel import multi_tensor

from scilpy.io.utils import (add_sh_basis_args, add_overwrite_arg, add_verbose_arg,
                             assert_inputs_exist, assert_outputs_exist)


def _build_arg_parser():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter)

    # Required Positional Arguments
    p.add_argument('in_response',
                   help='Path to the input response function text file.')
    p.add_argument('out_response',
                   help='Path to the output converted response function text file.')

    # Conversion Direction Toggle (Mutually Exclusive Required Group)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--scil_to_mrtrix', action='store_true',
                      help='Convert from Scilpy tensor format [eval1 eval2 eval3 S0]\n'
                           'to MRtrix spherical harmonic coefficient profile format.')
    mode.add_argument('--mrtrix_to_scil', action='store_true',
                      help='Convert from MRtrix SH coefficient profile format\n'
                           'to Scilpy tensor format [eval1 eval2 eval3 S0].')

    # Sequence Metadata Needed for Scil -> MRtrix
    p.add_argument('--bvals', type=float, nargs='+',
                   help='The sequence b-values matching each row (shell) in the \n'
                        'input file. Required for --scil_to_mrtrix.')

    # FIX 2: --sh_order was used in main() but never added to the parser.
    p.add_argument('--sh_order', type=int, default=8,
                   help='Maximum even spherical harmonic order for conversion. '
                        'Default: %(default)s')

    add_sh_basis_args(p)
    add_verbose_arg(p)
    add_overwrite_arg(p)

    return p


def _zonal_basis(sh_order, theta):
    """Return the zonal (m=0, even l) SH basis evaluated at polar angles theta.

    For m=0 the Descoteaux and Tournier (MRtrix) bases are identical, so the
    coefficients fitted with this basis can be written as-is in MRtrix format.
    """
    l_values = np.arange(0, sh_order + 1, 2)
    m_values = np.zeros_like(l_values)
    return real_sh_descoteaux_from_index(m_values, l_values,
                                         theta[:, None], np.zeros_like(theta)[:, None])


def scil_to_mrtrix(data, bvals, sh_order):
    """
    Simulate the signal of Scilpy tensor parameters [eval1, eval2, eval3, S0]
    along the polar angle and fit it with the zonal (m=0, even l) SH basis,
    which is the format MRtrix3 response function files expect.
    """
    if len(bvals) != len(data):
        raise ValueError(
            f"Mismatch: Provided {len(bvals)} bvals for {len(data)} data shells.")

    num_projections = 300
    theta = np.arccos(np.linspace(-1, 1, num_projections))
    vertices = np.column_stack([
        np.sin(theta),
        np.zeros(num_projections),
        np.cos(theta)
    ])

    # The response is axially symmetric around z, so only the zonal terms are
    # fitted. Fitting the full basis on a single meridian is ill-posed.
    B = _zonal_basis(sh_order, theta)
    n_zonal = B.shape[1]

    mrtrix_rows = []

    for idx, shell_params in enumerate(data):
        evals = shell_params[0:3]
        S0 = shell_params[3]
        bval = bvals[idx]

        # Explicitly handle b=0: response is isotropic, only l=0 is non-zero.
        if bval <= 10:
            row_coefs = np.zeros(n_zonal)
            row_coefs[0] = S0 / np.sqrt(4 * np.pi)
            mrtrix_rows.append(row_coefs)
            continue

        # Simulate diffusion attenuation profile along the virtual sphere.
        gtab_virtual = gradient_table(np.ones(num_projections) * bval,
                                      bvecs=vertices)
        # FIX 1: multi_tensor returns (signal, sticks); unpack accordingly.
        # FIX 6b: fractions are in percent (must sum to ≤100); use 100, not 1.0.
        signal, _ = multi_tensor(gtab_virtual, [evals], S0=S0,
                                 fractions=[100], angles=[(0, 0)], snr=None)

        zonal_coefs, _, _, _ = np.linalg.lstsq(B, signal, rcond=None)
        mrtrix_rows.append(zonal_coefs)

    return np.vstack(mrtrix_rows)


def mrtrix_to_scil(data):
    """
    Convert MRtrix response profile SH parameters backward into an analytical
    Scilpy tensor representation.  Estimates eigenvalues using log-linear system.

    MRtrix response files store only the zonal (m=0, even l) SH coefficients,
    which are evaluated directly with the zonal SH basis.
    Note: the backward b-value is assumed to be 1000 s/mm² for scaling purposes,
    matching common single-shell acquisitions.
    """
    # FIX 4: MRtrix stores n_zonal = lmax/2 + 1 zonal coefficients per shell,
    # so lmax = 2*(n_zonal - 1).  The previous formula solved for full-SH order
    # and was wrong.
    n_zonal = data.shape[1] if data.ndim > 1 else data.shape[0]
    lmax = 2 * (n_zonal - 1)

    num_projections = 100
    theta = np.linspace(0, np.pi / 2, num_projections)
    B = _zonal_basis(lmax, theta)

    scil_rows = []

    for zonal_coefs in data:
        # Reconstruct simulated signal along theta.
        signal = B @ zonal_coefs

        # S0 baseline from the isotropic (l=0) SH coefficient.
        S0 = zonal_coefs[0] * np.sqrt(4 * np.pi)

        log_s = np.log(np.clip(signal / S0, 1e-5, 1.0))

        # FIX 3: theta[0] = 0 (along fiber, parallel); theta[-1] = π/2 (perpendicular).
        # The previous code had val_parallel and val_perpendicular swapped.
        val_parallel = -log_s[0] / 1000.0        # θ=0  → along fiber axis
        val_perpendicular = -log_s[-1] / 1000.0  # θ=π/2 → perpendicular to fiber

        # Safe clipping to protect against non-physical negative eigenvalues.
        val_parallel = max(val_parallel, 1e-6)
        val_perpendicular = max(val_perpendicular, 1e-6)

        scil_rows.append([val_parallel, val_perpendicular, val_perpendicular, S0])

    return np.vstack(scil_rows)


def main():
    parser = _build_arg_parser()
    args = parser.parse_args()

    # Configure logger behavior.
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format='%(levelname)s: %(message)s')

    # Handle environment safety checks.
    assert_inputs_exist(parser, args.in_response)
    assert_outputs_exist(parser, args, args.out_response)

    logging.info(f"Loading input response: {args.in_response}")
    input_data = np.loadtxt(args.in_response)
    if input_data.ndim == 1:
        input_data = np.atleast_2d(input_data)

    # Execution Logic Flow.
    if args.scil_to_mrtrix:
        if not args.bvals:
            parser.error(
                "--bvals parameter list is strictly required when using --scil_to_mrtrix.")
        logging.info("Executing conversion: Scilpy (Tensor) -> MRtrix (SH)")
        output_data = scil_to_mrtrix(input_data, args.bvals, args.sh_order)
        fmt = '%.6f'
    else:
        logging.info("Executing conversion: MRtrix (SH) -> Scilpy (Tensor)")
        output_data = mrtrix_to_scil(input_data)
        fmt = '%.6e'

    # Write Out Array.
    logging.info(f"Saving output response: {args.out_response}")
    np.savetxt(args.out_response, output_data, fmt=fmt)


if __name__ == '__main__':
    main()
