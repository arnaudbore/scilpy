# -*- coding: utf-8 -*-

import numpy as np
from scipy.ndimage import binary_erosion

class RAP:
    def __init__(self, mask_rap, propagator, max_nbr_pts):
        """
        RAP_mask: DataVolume
            HRegion-Adaptive Propagation tractography volume.
        """
        self.rap_mask = mask_rap
        self.propagator = propagator
        self.max_nbr_pts = max_nbr_pts

    def is_in_rap_region(self, curr_pos, space, origin):
        return self.rap_mask.get_value_at_coordinate(
            *curr_pos, space=space, origin=origin) > 0

    def rap_multistep_propagate(self, line, prev_direction):
        """
        All child classes must implement this method. Must receive and return
        the parameters as defined here:


        Params
        ------
        line: list
            The beginning of the streamline

        Returns
        -------
        line: list
            The streamline extended with RAP in the RAP neighborhood.
        prev_direction: tuple
            The last direction (x, y, z).
        is_line_valid: bool
            If the line generated with RAP is valid.
        """
        raise NotImplementedError


class RAPContinue(RAP):
    """Dummy RAP class for tests. Goes straight"""
    def __init__(self, mask_rap, propagator, max_nbr_pts, step_size):
        """
        Step size: float
            The step size inside the RAP mask. Could be different from the step
            size elsewhere. In voxel world.
        """
        super().__init__(mask_rap, propagator, max_nbr_pts)
        self.step_size = step_size

    def rap_multistep_propagate(self, line, prev_direction):
        is_line_valid = True
        if len(line)>3:
            pos = line[-2] + self.step_size * np.array(prev_direction)
            line[-1] = pos
            return line, prev_direction, is_line_valid
        return line, prev_direction, is_line_valid


class RAPGraph(RAP):
    def __init__(self, mask_rap, propagator, max_nbr_pts, distance_max,):
        super().__init__(mask_rap, propagator, max_nbr_pts)
        self.distance_max = distance_max



    def rap_multistep_propagate(self, line, prev_direction):
        self.end_point_mask = self.get_end_points_mask(
            line[-1], prev_direction)


        raise NotImplementedError
    
    def get_end_points_mask(self, last_point, last_direction, max_angle=10):
        """
        Get a mask of the RAP region around the end point.
        """
        # Get the coordinates of the last point
        x, y, z = last_point

        # Create a mask for the endpoint RAP region
        rap_end_point_mask = np.zeros_like(self.rap_mask.data, dtype=bool)
        rap_end_point_mask[self.rap_mask.data == 1] = 1
        rap_end_point_mask[binary_erosion(self.rap_mask.data, iterations=1) == 1] = 0

        distance_from_last_point = np.abs(rap_end_point_mask - last_point)
        rap_end_point_mask[distance_from_last_point > self.distance_max] = 0

        angle = np.dot(vox_directions_norm, directions[0])


        for point in end_points:
            if self.rap_mask.is_in_bounds(point):
                end_point_mask[point] = True
        
        self.end_point_mask = end_point_mask

        return 