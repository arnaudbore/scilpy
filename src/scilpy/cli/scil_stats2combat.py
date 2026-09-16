#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert ROI-level DTI metrics from a wide-format Excel file into long-format
TSV files compatible with the CamCAN reference dataset structure.

One output file is produced per site and per metric/statistic combination:
    {site}_{metric}_{stat}.tsv   (e.g. SH_fa_mean.tsv, QC_md_std.tsv)

The input Excel file must contain one sheet per (metric, statistic) pair,
named as:
    {metric}_metric_{stat}
where {metric} is a DTI metric name (e.g. fa, md) and {stat} is mean or std.
Each sheet has subjects as rows (first column = subject ID) and ROIs as
columns.

Each output TSV contains one row per (subject, bundle) for subjects belonging
to that site, following the CamCAN reference column structure:
    sid, site, bundle, metric, mean, age, sex, handedness, disease

Since the Excel file contains no disease label, a default value is used
(see --disease). If the participants file contains a 'handedness' column its
values are encoded as right->1 and left->2; otherwise all subjects are
assigned handedness=1.

Sex encoding (participants TSV -> output):
    F -> 2  (female)
    H -> 1  (male / Homme)

Example usage:
    scil_stats2combat.py metrics.xlsx participants.tsv \\
        output_dir/ --disease HC
"""

import argparse
import logging
import os

import pandas as pd

from scilpy.io.utils import (add_overwrite_arg,
                             add_verbose_arg,
                             assert_inputs_exist,
                             assert_output_dirs_exist_and_empty)
from scilpy.version import version_string


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SEX_ENCODING = {'F': 2, 'H': 1}
HANDEDNESS_ENCODING = {'right': 1, 'left': 2}
DEFAULT_HANDEDNESS = 1


# ---------------------------------------------------------------------------
# Argument parser
# ---------------------------------------------------------------------------

def _build_arg_parser():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=version_string)

    p.add_argument('in_metrics',
                   help='Excel file (.xlsx) with one sheet per metric/stat '
                        'combination, named {metric}_metric_{mean|std}.')
    p.add_argument('in_participants',
                   help='Tab-separated participants file with columns: '
                        'participant_id, site, age, sex, M1.')
    p.add_argument('out_dir',
                   help='Output directory where TSV files will be written.')

    p.add_argument('--disease', default='HC', metavar='LABEL',
                   help='Disease label assigned to all subjects in the '
                        'output. The Excel file contains no disease '
                        'information so this value is used for '
                        'every subject. [%(default)s]')
    p.add_argument('--subject_col', default='participant_id', metavar='COL',
                   help='Name of the subject ID column in the participants '
                        'file. [%(default)s]')

    add_verbose_arg(p)
    add_overwrite_arg(p)

    return p


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_participants(args, parser):
    """Load, validate, and encode the participants TSV.

    Parameters
    ----------
    args : argparse.Namespace
    parser : argparse.ArgumentParser

    Returns
    -------
    pandas.DataFrame
        Table with columns: sample, sex, age, site, handedness.
    """
    df = pd.read_csv(args.in_participants, sep='\t')

    if args.subject_col not in df.columns:
        parser.error('Column \'{}\' not found in {}. Available columns: {}'
                     .format(args.subject_col, args.in_participants,
                             list(df.columns)))

    df = df.rename(columns={args.subject_col: 'sample'})

    # Encode sex: F -> 2, H -> 1
    df['sex'] = df['sex'].map(SEX_ENCODING)
    if df['sex'].isna().any():
        logging.warning('Unknown sex value(s) found in participants file '
                        '— will be encoded as NaN. Expected F (female=2) '
                        'or H (male/Homme=1).')

    # Handedness: encode right->1, left->2 if column exists, else default to 1
    if 'handedness' in df.columns:
        df['handedness'] = (df['handedness'].str.lower()
                                            .map(HANDEDNESS_ENCODING))
    else:
        df['handedness'] = DEFAULT_HANDEDNESS

    return df[['sample', 'sex', 'age', 'site', 'handedness']]


def _parse_sheet_name(sheet_name):
    """Parse '{metric}_metric_{stat}' into (metric, stat).

    Returns (None, None) if the name does not match the convention.
    """
    parts = sheet_name.split('_metric_')
    if len(parts) != 2:
        return None, None
    metric, stat = parts
    if stat not in ('mean', 'std'):
        return None, None
    return metric.lower(), stat


def _load_metrics(xlsx_path, parser):
    """Load all valid metric sheets from the Excel file.

    Parameters
    ----------
    xlsx_path : str
    parser : argparse.ArgumentParser

    Returns
    -------
    dict
        {(metric, stat): pandas.DataFrame}
        Each DataFrame is indexed by subject ID; columns are ROI names.
    """
    xl = pd.ExcelFile(xlsx_path)
    sheets = {}

    for sheet_name in xl.sheet_names:
        metric, stat = _parse_sheet_name(sheet_name)
        if metric is None:
            logging.warning('Sheet \'{}\' does not match the expected naming '
                            'convention {{metric}}_metric_{{mean|std}} '
                            '— skipped.'.format(sheet_name))
            continue

        df = xl.parse(sheet_name, index_col=0)
        df.index.name = 'sample'
        df.index = df.index.astype(str)
        sheets[(metric, stat)] = df
        logging.debug('  Loaded sheet \'{}\': {} subjects x {} ROIs.'
                      .format(sheet_name, len(df), len(df.columns)))

    if not sheets:
        parser.error('No valid metric sheets found in {}. Sheet names must '
                     'follow the pattern {{metric}}_metric_{{mean|std}}.'
                     .format(xlsx_path))

    return sheets


def _wide_to_long(df, metric):
    """Melt a wide (subjects x ROIs) DataFrame into long format.

    Parameters
    ----------
    df : pandas.DataFrame
        Wide-format DataFrame indexed by subject ID, ROI names as columns.
    metric : str
        Name of the metric (becomes the value column name).

    Returns
    -------
    pandas.DataFrame
        Long-format DataFrame with columns: sample, roi, {metric}.
    """
    return (df.reset_index()
              .melt(id_vars='sample', var_name='roi', value_name=metric))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = _build_arg_parser()
    args = parser.parse_args()
    logging.getLogger().setLevel(logging.getLevelName(args.verbose))

    # --- Input / output validation -------------------------------------------
    assert_inputs_exist(parser, [args.in_metrics, args.in_participants])
    assert_output_dirs_exist_and_empty(parser, args, args.out_dir)

    # --- Load participants ---------------------------------------------------
    logging.debug('Loading participants: {}'.format(args.in_participants))
    participants = _load_participants(args, parser)

    participants['disease'] = args.disease
    sites = sorted(participants['site'].dropna().unique())
    logging.debug('Sites found: {}'.format(sites))

    # --- Load metrics --------------------------------------------------------
    logging.debug('\nLoading metrics: {}'.format(args.in_metrics))
    sheets = _load_metrics(args.in_metrics, parser)

    metrics_stats = sorted(sheets.keys())
    logging.debug('Metric/stat combinations: {}'.format(
        ['{0}_{1}'.format(m, s) for m, s in metrics_stats]))

    # --- Reshape and write one file per (site, metric, stat) -----------------
    logging.debug('\nWriting output files to: {}'.format(args.out_dir))
    for (metric, stat), wide_df in sorted(sheets.items()):
        long_df = _wide_to_long(wide_df, metric)

        for site in sites:
            out_fname = '{}_{}_{}_raw.csv'.format(site, metric, stat)
            out_path = os.path.join(args.out_dir, out_fname)

            # Filter participants and data to this site
            site_demo = participants.loc[
                participants['site'] == site].copy()
            site_samples = set(site_demo['sample'])
            site_long = long_df.loc[
                long_df['sample'].isin(site_samples)].copy()

            if site_long.empty:
                logging.warning('No data found for site \'{}\' in sheet '
                                '{}_metric_{} — skipped.'
                                .format(site, metric, stat))
                continue

            # Merge demographics
            result = site_long.merge(site_demo, on='sample', how='left')

            # Warn about subjects present in metrics but absent from
            # the participants file
            missing = result.loc[result['sex'].isna(), 'sample'].unique()
            if len(missing):
                logging.warning('{} subject(s) found in metrics but not in '
                                'the participants file: {}.'
                                .format(len(missing), list(missing)))

            # Rename columns, add metric label, and set final column order
            result = result.rename(columns={'sample': 'sid',
                                            'roi': 'bundle',
                                            metric: 'mean'})
            result['metric'] = metric
            col_order = ['sid', 'site', 'bundle', 'metric', 'mean',
                         'age', 'sex', 'handedness', 'disease']
            result = (result[col_order]
                      .sort_values(['sid', 'bundle'])
                      .reset_index(drop=True))

            result.to_csv(out_path, index=False)
            logging.debug('  Written: {} ({} rows, {} subjects).'
                          .format(out_fname, len(result),
                                  result['sid'].nunique()))

    logging.debug('\nDone.')


if __name__ == '__main__':
    main()
