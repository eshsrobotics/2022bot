#!/usr/bin/env python3

import sys
import math
import argparse

from typing import List, NamedTuple, Optional, IO, Tuple

FRONT_LEFT: int = 0
BACK_LEFT: int = 1
BACK_RIGHT: int = 2
FRONT_RIGHT: int = 3

N: int = 0x01
NE: int = 0x02
E: int = 0x04
SE: int = 0x08
S: int = 0x10
SW: int = 0x20
W: int = 0x40
NW: int = 0x80

"""
Multiplying a degree value by this constant returns its equivalent value
in radians.
"""
DEGREES_TO_RADIANS: float = math.pi / 180

"""
Multiplying a value in radians by this constant returns its equivalent
value in degrees.
"""
RADIANS_TO_DEGREES: float = 1.0 / DEGREES_TO_RADIANS


class AsciiCanvas:
    """A simple 2D grid of characters.  Used to help visualize the vectors this
program calculates."""
    def __init__(self, width: int, height: int):
        """
        Initializes the grid with blank spaces.

        Arguments:
        - Width: The width of the grid in character cells.
        - Height: The height of trhe grid in character cells.
        """
        self._grid: List[int] = [ord(' ')] * (width * height)
        self._width: int = width
        self._height: int = height
        self._table: Optional[List[int]] = None

    @property
    def width(self): return self._width

    @property
    def height(self): return self._height

    def print(self, stream: IO[str] = sys.stdout):
        """
        'Renders' the grid to the given stream.  It will be printed in the
        obvious way: one row at a time, separated by newlines.

        Arguments:
        - stream: The output stream to render to (stdout by default.)
        """
        for i in range(self._width * self._height):
            print(chr(self._grid[i]), file=stream, sep="", end="")
            if (i + 1) % self._width == 0:
                print()

    def _set_pixel(self, grid: List[int], x: int, y: int, c: str):
        """
        Helper function that actually sets a character cell on an arbitrary
        grid.  This function performs no bounds checking.

        Arguments:
        - grid: An integer array with as many elements in it as self_grid.
        - x: The X coordinate of the 'pixel' to set.
        - y: The Y coordinate of the 'pixel' to set.
        - c: The character to set at the given cell position.
        """
        grid[self._width * y + x] = ord(c[0])

    def _get_pixel(self, grid: List[int], x: int, y: int) -> str:
        """
        Helper function that does the opposite of _set_pixel().  Returns the
        character at the given coordinate of the given grid, performing no
        bounds checking.

        Arguments:
        - grid: An integer array with as many elements in it as self_grid.
        - x: The X coordinate of the 'pixel' to set.
        - y: The Y coordinate of the 'pixel' to set.

        Returns:
          Return the character at the given cell position.
        """
        return chr(grid[self._width * y + x])

    def _get_neighbor_mask(self, grid: List[int], x: int, y: int,
                           exclude: str = " ", include: str = "") -> int:
        """
        A helper function that simplifies the process of obtaining a 'neighbor
        mask' (that is to say, a group of ORed bitflags representing the
        presence or absence of interesting neighbors.)

        Arguments:
        - grid: An integer array with as many elements in it as self_grid.
        - x: The X coordinate of the 'pixel' to examine.
        - y: The Y coordinate of the 'pixel' to examine.
        - exclude: Any adjacent cell that matches a character from this string
                   is not considered to be a neighbor.  By default, this is a
                   blank space, so adjacent characters that are not
                   blank spaces will become part of the neighbor mask.
        - include: If this string is non-empty, only adjacent cells that match
                   a character from this string are considered to be a
                   neighbor.  By default, this is empty.  The exclude argument
                   takes precedence if a character is present in both.

        Returns:
          Returns an integer between 0 and 255 representing the presence or
          absence of all eight surrounding neighbors.

          Limitations: coordinates that are out of bounds will always return
          0.  There are never neighbors beyond the edge of the grid.
        """
        neighbor_mask: int = 0x00
        mask_data = [
            (W, x - 1, y),
            (NW, x - 1, y - 1),
            (SW, x - 1, y + 1),
            (E, x + 1, y),
            (NE, x + 1, y - 1),
            (SE, x + 1, y + 1),
            (N, x, y - 1),
            (S, x, y + 1),
        ]
        for i in range(len(mask_data)):
            mask: int = mask_data[i][0]
            current_x: int = mask_data[i][1]
            current_y: int = mask_data[i][2]
            if current_x >= self._width or current_x < 0 or \
               current_y >= self._height or current_y < 0:
                continue

            c: str = self._get_pixel(grid, current_x, current_y)

            if c in exclude or (include != "" and c not in include):
                # No neighbor at this position.
                continue
            neighbor_mask |= mask

        return neighbor_mask

    def set_pixel(self, x: float, y: float, neighbor_mask: int = 0):
        """
        My crude attempt at setting a connected 'pixel' on an ASCII grid.

        Arguments:
        - x: The X coordinate of the pixel.  Fractional values are rounded
             (for now.)
        - y: The Y coordinate of the pixel.  Fractional values are rounded
             (again, for now.)
        - neighbor_mask: A bitmask indicating the desired connectivity of the
                         neighbors for this pixel.  This determines what the
                         'pixel' will will look like.  For example:

                         UP_MASK & DOWN_MASK: '|'
                         RIGHT_MASK & UP_MASK: '`'
        """
        # A simple adjacency matrix for connected ASCII characters.
        #
        # I actually have some old C++ code that has been sitting on my hard
        # drive since at least 2004 which "solves" this adjacency problem, so
        # I'm just going to use that.  The comments are from 2004, too.
        if not self._table:
            self._table = [' '] * 256

            # The "obvious" choices follow.
            self._table[0x00] = '.'                               # Isolated point
            self._table[NW] = '`'
            self._table[N] = '|'
            self._table[NE] = '\''
            self._table[E] = '-'
            self._table[SE] = '.'                                 # Think of this as a backwards comma.
            self._table[S] = '|'
            self._table[SW] = ','
            self._table[W] = '-'
            self._table[NW | SE] = '\\'
            self._table[NW | SW] = ')'                            # '>' doesn't look good
            self._table[N | E] = '`'
            self._table[N | S] = '|'
            self._table[NE | NW] = '\"'                           # 'v' doesn't look good
            self._table[NE | SE] = '('                            # '<' doesn't look good
            self._table[NE | SW] = '/'
            self._table[E | S] = ','
            self._table[E | W] = '-'
            self._table[SE | SW] = '.'                            # '^' doesn't look good
            self._table[S | W] = '.'                              # Backwards comma again.
            self._table[W | N] = '\''
            self._table[NW | NE | S] = 'Y'
            self._table[NW | NE | SW] = 'y'
            self._table[NW | E | SW] = '}'
            self._table[N | SE | SW] = 'A'                        # '^' is sometimes more effective, but not always
            self._table[NE | SE | W] = '{'
            self._table[SE | S | SW] = 'm'                        # '_' works, but it's too flat
            self._table[NW | NE | SE | SW] = 'X'
            self._table[N | NE | SE | S] = 'K'
            self._table[N | E | S | W] = '+'

                                                                  # The following choices are less clear-cut than the above.
            self._table[NW | N] = '`'
            self._table[NW | E] = '`'
            self._table[NW | S] = ':'
            self._table[NW | W] = '`'
            self._table[N | NE] = '\''
            self._table[N | SE] = ':'
            self._table[N | SW] = ';'
            self._table[NE | E] = '\''
            self._table[NE | S] = ':'
            self._table[NE | W] = '\''                            # In preference to the double quote
            self._table[E | SE] = '.'
            self._table[E | SW] = ','
            self._table[SE | W] = '.'
            self._table[SE | S] = '.'
            self._table[S | SW] = ','
            self._table[SW | W] = ','
            self._table[NW | N | NE] = '\"'
            self._table[NW | N | SW] = ';'
            self._table[NW | N | E] = '`'
            self._table[NW | N | SE] = ':'
            self._table[NW | N | S] = ':'
            self._table[NW | N | W] = '\''
            self._table[NW | NE | E] = '`'
            self._table[NW | NE | SE] = '7'                       # Should really be a backwards 'y'
            self._table[NW | NE | W] = '\"'
            self._table[NW | E | SE] = ':'
            self._table[NW | E | S] = ':'                         # 'b' looks too "fat?"
            self._table[NW | E | W] = '`'
            self._table[NW | SE | S] = ':'
            self._table[NW | SE | SW] = 'h'                       # Upside-down 'y'; 'L' works less effectively
            self._table[NW | SE | W] = ':'
            self._table[NW | S | SW] = ':'
            self._table[NW | S | W] = ':'
            self._table[NW | SW | W] = 'D'                        # Fine-tuning: was ')'
            self._table[N | NE | E] = '`'
            self._table[N | NE | SE] = ':'                        # Previous candidate was '('
            self._table[N | NE | S] = ':'
            self._table[N | NE | W] = '\"'
            self._table[N | NE | SW] = ';'
            self._table[N | E | SE] = ':'
            self._table[N | E | S] = ':'
            self._table[N | E | SW] = ';'                         # Looks better than 'p' (too fat?) or 'L'
            self._table[N | E | W] = '\"'
            self._table[N | SE | S] = ':'
            self._table[N | SE | W] = ':'                         # 'q' looks too "fat?"
            self._table[N | S | SW] = ';'
            self._table[N | S | W] = ':'
            self._table[N | SW | W] = ';'
            self._table[NE | E | SE] = 'C'                        # Fine-tuning: was '('
            self._table[NE | E | S] = ':'
            self._table[NE | E | SW] = ';'
            self._table[NE | E | W] = '\''
            self._table[NE | S | SW] = ';'
            self._table[NE | S | W] = ':'                         # 'd' looks too "fat?"
            self._table[NE | SE | S] = ':'
            self._table[NE | SE | SW] = 'j'                       # Upside-down reverse 'y'; use '1' or 'J' here?
            self._table[NE | SW | W] = ';'
            self._table[E | SE | S] = ','
            self._table[E | SE | SW] = ','
            self._table[E | SE | W] = '.'
            self._table[E | S | SW] = ','
            self._table[E | S | W] = '.'
            self._table[E | SW | W] = ','
            self._table[SE | S | W] = '.'
            self._table[SE | SW | W] = '.'
            self._table[S | SW | W] = '.'
            self._table[NW | NE | E | SW] = 'D'                   # Works better than ']'
            self._table[NW | NE | E | W] = '\"'
            self._table[NW | NE | SE | S] = 'U'
            self._table[NW | NE | SE | W] = 'C'                   # Works better than '['
            self._table[NW | NE | S | SW] = 'U'
            self._table[NW | N | NE | E] = '`'
            self._table[NW | N | NE | SE] = '7'                   # Could also be 'q'
            self._table[NW | N | NE | S] = 'T'
            self._table[NW | N | NE | SW] = 'r'                   # Could also be 'F' or 'p'
            self._table[NW | N | NE | W] = '\''
            self._table[NW | N | E | SE] = 'q'                    # Previously '7'
            self._table[NW | N | E | S] = '7'                     # Closely resembles an upside-down, reverse 'L'
            self._table[NW | N | E | SW] = ']'                    # More feasible than ')'
            self._table[NW | N | E | W] = '\"'
            self._table[NW | N | SE | S] = 'Z'                    # Backwards 'S'
            self._table[NW | N | SE | SW] = 'n'
            self._table[NW | N | SE | W] = 'q'
            self._table[NW | N | S | SW] = ']'                    # Closely resembles a backwards 'K'
            self._table[NW | N | S | W] = ':'                     # Previous candidate was 'q', but it looked too fat
            self._table[NW | N | SW | W] = ';'                    # Could only be 'p' if table[NW | w|S | SW | W] was '8'
            self._table[NW | NE | E | SE] = '7'                   # Could be 'q'
            self._table[NW | NE | E | S] = 'U'
            self._table[NW | NE | S | W] = 'U'
            self._table[NW | NE | SW | W] = 'r'                   # Could also be 'F' or 'p'
            self._table[NW | E | SE | S] = 'b'
            self._table[NW | E | SE | SW] = 'D'                   # Works better than ']'
            self._table[NW | E | SE | W] = ':'                    # Previously '~', which looked inconsistent
            self._table[NW | E | S | SW] = ']'                    # More feasible than ')'
            self._table[NW | E | S | W] = 'b'
            self._table[NW | E | SW | W] = '}'
            self._table[NW | SE | S | SW] = 'L'                   # Could also be 'b'
            self._table[NW | SE | S | W] = 'b'                    # Previously 'L' (which was effective, but inconsistent)
            self._table[NW | SE | SW | W] = 'L'                   # 'h' works too, but this is more consistent
            self._table[NW | S | SW | W] = ':'                    # Could only be 'b' if table[NW | N | S | SW | W] was '8'
            self._table[N | NE | E | SE] = ':'                    # Could only be 'q' if table[N | NE | E | SE | S] was '8'
            self._table[N | NE | E | S] = ':'                     # Previous candidate was 'p', but it looked too fat
            self._table[N | NE | E | SW] = 'p'
            self._table[N | NE | E | W] = '\"'
            self._table[N | NE | SE | SW] = 'n'
            self._table[N | NE | SE | W] = '['                    # More feasible than '(', more consistent than 'C'
            self._table[N | NE | S | SW] = 'S'
            self._table[N | NE | S | W] = 'F'                     # More closely resembles an upside-down 'L' than 'f' does
            self._table[N | NE | SW | W] = 'p'                    # Previously 'F'; could also be 'P', 'r' or ';'
            self._table[N | E | SE | S] = ':'                     # Previous candidate was 'b', but it looked too fat
            self._table[N | E | SE | SW] = 'n'
            self._table[N | E | SE | W] = 'q'
            self._table[N | E | S | SW] = 'j'                     # Closely resembes a reverse 'L'
            self._table[N | E | SW | W] = 'p'
            self._table[N | SE | S | SW] = 'i'                    # Or should we use 'A' or 'l'?
            self._table[N | SE | S | W] = 'L'
            self._table[N | SE | SW | W] = 'n'
            self._table[N | S | SW | W] = ':'                     # Previous candidate was 'd', but it looked too fat
            self._table[NE | E | SE | S] = ':'                    # Could only be 'd' if table[N | NE | E | SE | S] was '8'
            self._table[NE | E | SE | SW] = 'j';                  # Could also be 'd'
            self._table[NE | E | SE | W] = '{';
            self._table[NE | E | S | SW] = 'd';                   # Previously 'j' (which was effective, but inconsistent)
            self._table[NE | E | S | W] = 'd';
            self._table[NE | E | SW | W] = '~';                   # Won't look good if the tilde isn't centered (maybe ';'?)
            self._table[NE | SE | S | W] = '['                    # More feasible than '(', more consistent than 'C'
            self._table[NE | SE | SW | W] = 'C'                   # Works better than '['
            self._table[NE | S | SW | W] = 'd'
            self._table[E | SE | S | W] = '.'
            self._table[E | SE | S | SW] = ','
            self._table[E | SE | SW | W] = '.'
            self._table[E | S | SW | W] = '.'
            self._table[SE | S | SW | W] = '.'
            self._table[NW | N | NE | E | SE] = '7'               # Right triangle with corner at upper right (maybe 'q'?)
            self._table[NW | N | NE | E | S] = '7'
            self._table[NW | N | NE | E | SW] = 'D'               # ']' and ')' also work
            self._table[NW | N | NE | E | W] = '\"'
            self._table[NW | N | NE | SE | S] = 'L'
            self._table[NW | N | NE | SE | SW] = 'n'
            self._table[NW | N | NE | SE | W] = 'C'               # '[' and '(' also work
            self._table[NW | N | NE | S | SW] = 'J'
            self._table[NW | N | NE | S | W] = 'F'                # Better than 'r'
            self._table[NW | N | NE | SW | W] = 'r'               # Right triangle with corner at upper left ('f' or ';'?)
            self._table[NW | N | E | S | W] = 'S'
            self._table[NW | N | E | SE | S] = '7'                # 'q' does not work as well here
            self._table[NW | N | E | SE | SW] = 'D'               # ')' works here also
            self._table[NW | N | E | SE | W] = 'q'                # '7' does not work here
            self._table[NW | N | E | S | SW] = ']'                # Worse (?) alternatives are 'D', ')'
            self._table[NW | N | E | SW | W] = 'p'
            self._table[NW | N | SE | S | SW] = '7'               # Previous candidate was 'i'
            self._table[NW | N | SE | S | W] = 'L'                # 'b' does not work as well here
            self._table[NW | N | SE | SW | W] = 'R'               # 'n' and 'h' also work
            self._table[NW | N | S | SW | W] = ':'
            self._table[NW | NE | E | SE | S] = 'U'
            self._table[NW | NE | E | SE | SW] = 'D'              # Most closely resembles a backwards 'C', I guess
            self._table[NW | NE | E | SE | W] = 'q'
            self._table[NW | NE | E | S | SW] = 'U'
            self._table[NW | NE | E | S | W] = 'U'
            self._table[NW | NE | E | SW | W] = 'p'
            self._table[NW | NE | SE | S | SW] = 'U'              # 'u' works, but is not consistent
            self._table[NW | NE | SE | S | W] = 'C'               # '(' works here also
            self._table[NW | NE | SE | SW | W] = 'C'              # A 'U' tilted to the right
            self._table[NW | NE | S | SW | W] = 'U'               # An upside-down 'R' would work better
            self._table[NW | E | SE | S | SW] = 'D'
            self._table[NW | E | SE | S | W] = 'b'                # 'L' does not work here
            self._table[NW | E | SE | SW | W] = 'b'               # Previous candidate was 'h', which was asymmetric
            self._table[NW | E | S | SW | W] = 'b'
            self._table[NW | SE | S | SW | W] = 'L'               # Right triangle with corner at lower left
            self._table[N | NE | E | SE | S] = ':'
            self._table[N | NE | E | SE | SW] = 'n'               # A backwards 'R' would work better
            self._table[N | NE | E | SE | W] = 'q'
            self._table[N | NE | E | S | SW] = 'j'                # 'd' does not work as well here
            self._table[N | NE | E | S | W] = 'Z'                 # Backwards 'S' '2' also works
            self._table[N | NE | E | SW | W] = 'p'                # 'r' does not work here
            self._table[N | NE | SE | S | SW] = 'f'               # Previous candidate was 'F'
            self._table[N | NE | SE | S | W] = '['                # Worse (?) alternatives are 'C', '('
            self._table[N | NE | SE | SW | W] = 'R'               # 'n' works here also
            self._table[N | NE | S | SW | W] = 'r'                # 'p' does not work as well here
            self._table[N | E | SE | S | SW] = 'j'
            self._table[N | E | SE | S | W] = 'S'
            self._table[N | E | SE | SW | W] = 'n'                # Upside-down 'U'. Should I use 'A' here?
            self._table[N | E | S | SW | W] = 'Z'                 # Backwards 'S' '2' also works
            self._table[N | SE | S | SW | W] = 'L'
            self._table[NE | E | SE | S | SW] = 'j'               # Right triangle with corner at lower right
            self._table[NE | E | SE | S | W] = 'd'
            self._table[NE | E | SE | SW | W] = 'd'
            self._table[NE | E | S | SW | W] = 'd'                # 'j' does not work here
            self._table[NE | SE | S | SW | W] = 'C'               # '[' and '(' also work
            self._table[E | SE | S | SW | W] = '.'
            self._table[NW | N | NE | E | SE | S] = '7'           # Closely resembles an upside-down, reverse 'L' ('1'?)
            self._table[NW | N | NE | E | SE | SW] = 'D'          # A backwards 'R' would work better
            self._table[NW | N | NE | E | SE | W] = 'q'
            self._table[NW | N | NE | E | S | SW] = ']'           # Better than ')'
            self._table[NW | N | NE | E | S | W] = '8'            # This has to match table[255], I think
            self._table[NW | N | NE | E | SW | W] = 'p'
            self._table[NW | N | NE | SE | S | SW] = 'I'
            self._table[NW | N | NE | SE | S | W] = '['           # Better than 'C' or '('
            self._table[NW | N | NE | SE | SW | W] = 'R'          # 'n' also works
            self._table[NW | N | NE | S | SW | W] = 'F'           # Treating it like an upside-down 'L'
            self._table[NW | N | E | SE | S | SW] = ']'           # Better than ')'
            self._table[NW | N | E | SE | S | W] = 'S'            # Better than '8'
            self._table[NW | N | E | SE | SW | W] = 'n'           # Closest thing to an upside-down 'U'
            self._table[NW | N | E | S | SW | W] = '8'            # This has to match table[255], I think
            self._table[NW | N | SE | S | SW | W] = 'L'
            self._table[NW | NE | E | SE | S | SW] = 'U'
            self._table[NW | NE | E | SE | S | W] = 'U'
            self._table[NW | NE | E | SE | SW | W] = 'H'
            self._table[NW | NE | E | S | SW | W] = 'U'
            self._table[NW | NE | SE | S | SW | W] = 'C'          # '[' and '(' also work
            self._table[NW | E | SE | S | SW | W] = 'b'
            self._table[N | NE | E | SE | S | SW] = 'j'           # Closely resembles a reverse 'L'
            self._table[N | NE | E | SE | S | W] = '8'            # This has to match table[255], I think
            self._table[N | NE | E | SE | SW | W] = 'n'           # Closest thing to an upside-down 'U'
            self._table[N | NE | E | S | SW | W] = 'Z'            # Backwards 'S' '2' also works; both are better than '8'
            self._table[N | NE | SE | S | SW | W] = '['           # Better than 'C' or '(', more consistent than 'f'
            self._table[N | E | SE | S | SW | W] = '8'            # This has to match table[255], I think
            self._table[NE | E | SE | S | SW | W] = 'd'
            self._table[NW | N | NE | E | SE | S | SW] = ')'
            self._table[NW | N | NE | E | SE | S | W] = '8'       # This has to match table[255], I think
            self._table[NW | N | NE | E | SE | SW | W] = 'n'      # Most closely resembles upside-down capital 'U'
            self._table[NW | N | NE | E | S | SW | W] = '8'       # This has to match table[255], I think
            self._table[NW | N | NE | SE | S | SW | W] = '('
            self._table[NW | N | E | SE | S | SW | W] = '8'       # This has to match table[255], I think
            self._table[NW | NE | E | SE | S | SW | W] = 'U'
            self._table[N | NE | E | SE | S | SW | W] = '8'       # This has to match table[255], I think
            self._table[NW | N | NE | E | SE | S | SW | W] = '8'  # Or maybe 'O' or '*'?

            # In case you're counting, there are 255 assigned entries in that
            # table.
            #
            # I'm missing one.  I don't know which.

        px: int = round(x)
        py: int = round(y)
        if px < 0 or px >= self._width or py < 0 or py >= self._height:
            return
        self._set_pixel(self._grid, px, py, self._table[neighbor_mask & 0xFF])

    def draw_ellipse(self, center_x: float, center_y: float,
                     radius_x: float, radius_y: float):
        """
        Renders an axis-aligned ellipse onto the ASCII canvas.

        Stolen, without shame, from
        https://www.geeksforgeeks.org/midpoint-ellipse-drawing-algorithm/.  I
        used to know how to do this stuff, but it's not 2004 anymore.

        Arguments:
        - center_x: The X coordinate of the ellipse's center on the grid.
        - center_y: The Y coordinate of the ellipse's center on the grid.
        - radius_x: The length of the ellipse's axis in the X direction.
        - radius_y: The length of the ellipse's axis in the Y direction.
        """
        x: float = 0
        y: float = radius_y

        # Initial decision parameter for region 1.
        d1: float = (radius_y ** 2) - \
                    (radius_y * (radius_x ** 2)) + \
                    (0.25 * (radius_x ** 2))
        dx: float = 2 * (radius_y ** 2) * x
        dy: float = 2 * (radius_x ** 2) * y

        # Gather the generated coordinates into a list.  The ellipse is
        # rendered symmetrically about the four quadrants.
        points: List[Tuple[int, int]] = []

        while dx < dy:  # Region 1.
            points.append((center_x + x, center_y + y))
            points.append((center_x - x, center_y + y))
            points.append((center_x + x, center_y - y))
            points.append((center_x - x, center_y - y))
            if d1 < 0:
                x += 1
                dx += 2 * (radius_y ** 2)
                d1 += (dx + (radius_y ** 2))
            else:
                x += 1
                y -= 1
                dx += 2 * (radius_y ** 2)
                dy -= 2 * (radius_x ** 2)
                d1 += (dx - dy + (radius_y ** 2))

        # Decision parameter for region 2.
        d2: float = ((radius_y ** 2) * (x + 0.5) ** 2) + \
                    ((radius_x ** 2) * (y - 1) ** 2) - \
                    ((radius_x ** 2) + (radius_y ** 2))

        while y >= 0:
            points.append((center_x + x, center_y + y))
            points.append((center_x - x, center_y + y))
            points.append((center_x + x, center_y - y))
            points.append((center_x - x, center_y - y))
            if d2 > 0:
                y -= 1
                dy -= 2 * (radius_x ** 2)
                d2 += (radius_x ** 2) - dy
            else:
                y -= 1
                x += 1
                dx += 2 * (radius_y ** 2)

        # Render in two passes: a first pass just to set the character cells,
        # and a second pass to set them based on their observed connectivity.
        grid_copy: List[int] = [ord(' ')] * self._width * self._height
        for index in range(len(points)):
            x: int = int(points[index][0] + 0.5)
            y: int = int(points[index][1] + 0.5)
            if x >= self._width or x < 0 or y >= self._height or y < 0:
                continue
            self._set_pixel(grid_copy, x, y, '*')

        for index in range(len(points)):
            x: int = int(points[index][0] + 0.5)
            y: int = int(points[index][1] + 0.5)
            if x >= self._width or x < 0 or y >= self._height or y < 0:
                continue
            self.set_pixel(x, y, self._get_neighbor_mask(grid_copy, x, y))


class Vector(NamedTuple):
    """A very simple 2D vector."""
    x: float
    y: float


def rotate(v: Vector, theta: float) -> Vector:
    """
    Rotates the given vector by theta radians counterclockwise around the
    origin.
    """
    costheta: float = math.cos(theta)
    sintheta: float = math.sin(theta)
    return Vector(v.x * costheta - v.y * sintheta,
                  v.x * sintheta + v.y * costheta)


def normalize(v: Vector) -> Vector:
    """
    Returns the unit vector having the same direction as the given vector.
    Undefined if it is the zero vector.
    """
    dist: float = math.dist(Vector(0, 0), v)
    return Vector(v.x / dist, v.y / dist)


def get_crab_rotation_thetas(length: float, width: float) -> List[float]:
    """
    Calculates the rotation angles that the four swerve modules of a swerve
    drive with the given drive base dimensions needs in order to rotate in a
    circle.

    All four swerve modules are presumed to point forward prior to the
    rotations.  Clockwise angles are negative and counterclockwise angles are
    positive, following classroom conventions.
    """
    theta: float = math.atan(length / width)
    return [-theta,  # FRONT_LEFT
            +theta,  # FRONT_RIGHT
            -theta,  # BACK_LEFT
            +theta]  # BACK_RIGHT


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="""
    Calculates the swerve module angles that will cause a given drive train to
    pivot in place.  It can also calculate the vectors that result from adding
    these rotated vectors to an overall chassis direction.
    """, usage="%(prog)s [-h] width height [forwardBack leftRight rotation]")

    mandatory_group = parser.add_argument_group("Mandatory arguments")
    mandatory_group.add_argument("length",
                                 type=float,
                                 help="Length of the chassis.")
    mandatory_group.add_argument("width",
                                 type=float,
                                 help="Width of the chassis.")

    joystick_group = parser.add_argument_group("Joystick channels")
    joystick_group.add_argument("forwardBack",
                                type=float,
                                nargs="?",
                                default=None,
                                help="Value of left vertical joystick channel (-1.0 to 1.0)")
    joystick_group.add_argument("leftRight",
                                type=float,
                                nargs="?",
                                default=None,
                                help="Value of left horizontal joystick channel (-1.0 to 1.0)")
    joystick_group.add_argument("rotation",
                                type=float,
                                nargs="?",
                                default=None,
                                help="Value of right horizontal joystick channel (-1.0 to 1.0)")

    parser.add_argument("--verbose", "-v",
                        action="store_true",
                        default=False,
                        help="Prints more debugging information to the terminal")
    args = parser.parse_args()

    params = [
        ["verbose",     args.verbose,     0],
        ["length",      args.length,      0],
        ["width",       args.width,       0],
        ["forwardBack", args.forwardBack, 1],
        ["leftRight",   args.leftRight,   1],
        ["rotation",    args.rotation,    1],
    ]

    channels = 0
    for param in params:
        if args.verbose:
            print(f"{param[0]}: {param[1]}")
        if param[1] is not None:
            channels += param[2]

    if channels > 0:
        if channels < 3:
            print("Error: Since at least 1 joystick channel was supplied, all three joystick channels must have values.", file=sys.stderr)
            exit(1)
        else:
            args.forwardBack = max(-1.0, min(1.0, args.forwardBack))
            args.leftRight = max(-1.0, min(1.0, args.leftRight))
            args.rotation = max(-1.0, min(1.0, args.rotation))

    if args.length <= 0:
        print("Error: Length must be positive")
        exit(1)

    if args.width <= 0:
        print("Error: Width must be positive")
        exit(1)

    thetas = get_crab_rotation_thetas(args.length, args.width)
    if args.verbose:
        degrees = [angle * RADIANS_TO_DEGREES for angle in thetas]
        print(f"Thetas: Front left = {degrees[FRONT_LEFT]:.3f}°, Front right = {degrees[FRONT_RIGHT]:.3f}°, Back left = {degrees[BACK_LEFT]:.3f}°, Back right = {degrees[BACK_RIGHT]:.3f}°,")

    grid = AsciiCanvas(79, 25)
    grid.draw_ellipse(grid.width / 2,
                      grid.height / 2,
                      grid.width * 0.4,
                      grid.height * 0.4)
    grid.print()