#!/usr/bin/env python3

import sys
import math
import argparse

from enum import Enum
from typing import List, NamedTuple, Optional, IO, Tuple, Mapping


class Vector(NamedTuple):
    """A very simple 2D vector."""
    x: float
    y: float

    def normalized(self):
        """
        Returns a Vector pointing in the same direction as this Vector, but
        with a length of 1.  Vectors that have a length of 0 are returned
        unchanged.
        """
        length = math.dist((0, 0), (self.x, self.y))
        if length == 0:
            return self
        return Vector(self.x / length, self.y / length)

    def rotate(self, theta: float):
        """
        Returns a copy of this vector rotated by theta radians
        counterclockwise around the origin.
        """
        costheta: float = math.cos(theta)
        sintheta: float = math.sin(theta)
        return Vector(self.x * costheta - self.y * sintheta,
                      self.x * sintheta + self.y * costheta)


    def __mul__(self, factor: float):
        """Returns this vector scaled by the given factor."""
        return Vector(self.x * factor, self.y * factor)

    # Make sure 2 * v works as well as v * 2.
    __rmul__ = __mul__

    def __add__(self, v):
        """Returns the sum of the given vector and this one."""
        return Vector(self.x + v.x, self.y + v.y)

    def __neg__(self):
        """Returns a vector pointed in the opposite direction from this one."""
        return Vector(-self.x, -self.y)


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


def round_nearest(x: float) -> int:
    """Returns the argument rounded to the nearest whole number."""
    if x > 0:
        return int(x + 0.5)
    elif x < 0:
        return int(x - 0.5)
    else:
        return 0


class OverwriteBehavior(Enum):
    """
    What to do when a character from a primitive we are trying to draw is in
    the same place as an existing character on a grid.  The choices are:

    - ALWAYS: Render our character, overwriting the existing character.
    - MERGE: Look at the existing neighbors and render a character that
             appears to connect to them.
    - NEVER: Take no action (that is, ignore our character.)
    """
    ALWAYS = 1
    MERGE = 2
    NEVER = 3

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
        self._overwrite: OverwriteBehavior = OverwriteBehavior.ALWAYS

    @property
    def width(self): return self._width

    @property
    def height(self): return self._height

    @property
    def overwrite(self):
        """Returns the canvas's current overwrite policy."""
        return self._overwrite

    @overwrite.setter
    def overwrite(self, value: OverwriteBehavior):
        """
        Changes the overwrite policy for any new primitives that are rendered
        onto the canvas (the draw_* functions.  Note that set_pixel() is not
        affected.)
        """
        self._overwrite = value

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
        if exclude == "" and include == "":
            return neighbor_mask

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

    def _is_occupied(self, grid: List[int], x: int, y: int,
                     exclude: str = " ", include: str = "") -> bool:
        """
        A helper function that extends the notion of adjacent neighbors in
        order to tell if a given position itself is occupied.

        - grid: An integer array with as many elements in it as self_grid.
        - x: The X coordinate of the 'pixel' to examine.
        - y: The Y coordinate of the 'pixel' to examine.
        - exclude: A grid cell that matches a character from this string
                   is considered to be unoccupied.  By default, this is a
                   blank space.
        - include: If this string is non-empty, only grid cells matching a
                   character from this string are considered to be occupied.
                   By default, this is empty.  The exclude argument takes
                   precedence if a character is present in both.

        Returns:
          Returns True if the position is occupied and False if it is not.
          Positions out of bounds are always considered unoccupied.
        """
        if x >= self._width or x < 0 or y >= self._height or y < 0:
            return False

        c: str = self._get_pixel(grid, x, y)
        if c in exclude or (include != "" and c not in include):
            # No occupied cell at this position.
            return False

        return True

    def overwrite_pixel(self, x: float, y: float, neighbor_mask: int = 0,
                        background: str = " ", foreground: str = ""):
        """
        This function (which is largely a helper function for drawing graphics
        primitives) works like set_pixel() except that it pays attention to
        the current value of the canvas's overwrite property.

        * If self.overwrite is ALWAYS, this function is equivalent to
          set_pixel().
        * If self.overwrite is NEVER, this function will only set pixels on
          unoccupied grid cells (with the exclude and include arguments
          controlling what characters are considered occupied.)
        * If self.overwrite is MERGE, this function will compute the existing
          neighbor_mask on the grid cell at (x, y) and OR it with the
          neighbor_mask argument, then it will call set_pixel() with that.

          I found this useful for a single use case: drawing a primitive onto
          some other empty grid before drawing to the main one.  The existing
          grid will have its own neighbor_mask that you can combine with the
          neighbor_mask on the main grid so that the primitive will look
          correctly 'connected' to the main grid when superimposed.

        Arguments:
        - x: The X coordinate of the pixel.  Fractional values are rounded
             (for now.)
        - y: The Y coordinate of the pixel.  Fractional values are rounded
             (again, for now.)
        - neighbor_mask: A bitmask indicating the desired connectivity of the
                         neighbors for this pixel.
        - background: Any grid cell that matches a character from this string
                      is not considered occupied.  By default, this is a
                      blank space.
        - foreground: If this string is non-empty, only cells that match a
                      character from this string are considered to be
                      occupied.  neighbor.  By default, this is empty.  The
                      exclude argument takes precedence if a character is
                      present in both.

                      Having a non-empty foreground string that doesn't match
                      any characters that are already on the grid is
                      equivalent to having self.overwrite == ALWAYS.
        """
        if self._overwrite == OverwriteBehavior.ALWAYS:
            self.set_pixel(x, y, neighbor_mask)
        elif self._overwrite == OverwriteBehavior.NEVER:
            if not self._is_occupied(self._grid, x, y, background, foreground):
                self.set_pixel(x, y, neighbor_mask)
        else:  # self._overwrite == OverwriteBehavior.MERGE
            neighbor_mask |= \
                self._get_neighbor_mask(self._grid, x, y, background, foreground)
            self.set_pixel(x, y, neighbor_mask)

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

    def _draw_primitive(self, points: List[Tuple[int, int]],
                        exclude: str = " ", include: str = ""):
        """
        Helper function for draw_*().

        Once you have generated a list of points for a particular rendering
        primitive, you can call this function to draw those points to the
        canvas.  It employs a 2-pass algorithm so that the initial characters
        are laid down on the first pass and the actual connectivity-sensitive
        characters are laid down on the second pass using
        self.overwrite_pixel().
        """
        grid_copy: List[int] = [ord(' ')] * self._width * self._height
        exclude: str = " "
        include: str = ""
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
            neighbor_mask: int = \
                self._get_neighbor_mask(grid_copy, x, y, exclude, include)

            self.overwrite_pixel(x, y, neighbor_mask,
                                 background=exclude, foreground=include)

    def draw_ellipse(self, center_x: float, center_y: float,
                     radius_x: float, radius_y: float):
        """
        Renders an axis-aligned ellipse onto the ASCII canvas.  The ellipse
        can be partially off-screen.

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

        exclude: str = " "
        include: str = ""
        self._draw_primitive(points, exclude, include)

    def draw_line(self, start_x: float, start_y: float,
                  end_x: float, end_y: float):
        """
        Draws a straight line that goes through the given coordinates.  The
        line can be partially off-screen.

        Shamelessly stolen from
        https://en.wikipedia.org/wiki/Bresenham's_line_algorithm.  I am
        calling the version that works for all octants, though it is now
        always guaranteed from draw starting from (x1, y1) (which I don't care
        about.)

        Arguments:
        - start_x: The X coordinate of the line's first endpoint.
        - start_y: The Y coordinate of the line's first endpoint.
        - end_x: The X coordinate of the line's second endpoint.
        - end_y: The Y coordinate of the line's second endpoint.
        """

        x1: int = int(start_x + 0.5)
        y1: int = int(start_y + 0.5)
        x2: int = int(end_x + 0.5)
        y2: int = int(end_y + 0.5)

        points: List[Tuple[int, int]] = []

        dx: int = abs(x2 - x1)
        sx: int = 1 if x1 < x2 else -1
        dy: int = -abs(y2 - y1)
        sy: int = 1 if y1 < y2 else -1
        error: int = dx + dy

        x: int = x1
        y: int = y1
        while True:
            points.append((x, y))
            if x == x2 and y == y2:
                break
            e2: int = 2 * error
            if e2 >= dy:
                error += dy
                x += sx
            if e2 <= dx:
                error += dx
                y += sy

        exclude: str = " "
        include: str = ""
        self._draw_primitive(points, exclude, include)

    def draw_rect(self, x1: float, y1: float, x2: float, y2: float):
        """
        Draws an axis-aligned rectangle in the most straightforward way.

        Arguments:
        - x1: The X coordinate of the first corner.
        - y1: The Y coordinate of the first corner.
        - x2: The X coordinate of the second corner.
        - y2: The y coordinate of the second corner.
        """

        x1: int = int(x1 + 0.5)
        y1: int = int(y1 + 0.5)
        x2: int = int(x2 + 0.5)
        y2: int = int(y2 + 0.5)
        sx: int = 1 if x1 < x2 else -1
        sy: int = 1 if y1 < y2 else -1

        points: List[Tuple[int, int]] = []

        for x in range(x1, x2 + sx, sx):
            points.append((x, y1))
            points.append((x, y2))
        for y in range(y1 + sy, y2, sy):
            points.append((x1, y))
            points.append((x2, y))

        exclude: str = " "
        include: str = ""
        self._draw_primitive(points, exclude, include)

    def draw_rotated_rect(self, x1: float, y1: float, x2: float, y2: float,
                          width: float):
        """
        Draws an oriented bounding box (for an axis-aligned one, see
        draw_rect().)

        Arguments:
        - x1: The X coordinate of the front center of the rectangle (the
              midpoint of its leading edge.)
        - y1: The Y coordinate of the front center of the rectangle.
        - x2: The X coordinate of the rear center of the rectangle (the
              midpoint of its trailing edge.)
        - y2: The Y coordinate of the rear center of the rectangle.
        - width: The width of the rectangle in characters.  A width of 0
                 should produce a narrow line from (x1, y1) to (x2, y2).
        """
        points: List[Tuple[int, int]] = []
        main_axis_length: float = math.dist((x1, y1), (x2, y2))

        if main_axis_length == 0:
            # Degenerate rectangle.
            points.append((round_nearest(x1), round_nearest(y1)))
        else:
            main_axis_normalized: Tuple[float, float] = \
                ((x2 - x1) / main_axis_length, (y2 - y1) / main_axis_length)

            cross_axis_normalized: Tuple[float, float] = \
                (-main_axis_normalized[1], main_axis_normalized[0])

            endpoints: List[Tuple[float, float]] = [
                (x1 + cross_axis_normalized[0] * width/2, y1 + cross_axis_normalized[1] * width/2),
                (x1 - cross_axis_normalized[0] * width/2, y1 - cross_axis_normalized[1] * width/2),
                (x2 - cross_axis_normalized[0] * width/2, y2 - cross_axis_normalized[1] * width/2),
                (x2 + cross_axis_normalized[0] * width/2, y2 + cross_axis_normalized[1] * width/2)
            ]
            for i in range(len(endpoints)):
                p1: Tuple[float, float] = endpoints[i - 1 if i > 0 else len(endpoints) - 1]
                p2: Tuple[float, float] = endpoints[i]
                dx: float = p2[0] - p1[0]
                dy: float = p2[1] - p1[1]
                x: float = p1[0]
                y: float = p1[1]
                delta: int = round_nearest(max(abs(dx), abs(dy)))
                for j in range(delta):
                    points.append((round_nearest(x), round_nearest(y)))
                    x += dx / delta
                    y += dy / delta

        exclude: str = " "
        include: str = ""
        self._draw_primitive(points, exclude, include)

    def draw_text(self, x: float, y: float, s: str, justify: int = 0):
        """
        Draws the given string of text at (x, y).

        Arguments:
        - x: The X coordinate.  Which character will be at this position is
             controlled by the justify argument.
        - y: The Y coordinate.  Which character will be at this position is
             controlled by the justify argument.
        - s: The string to render.  Characters beneath will be overwritten
             unless self.overwrite is NEVER.
        - justify: Controls where the text is drawn in relation to the
                   coordinate.  Negative values left-justify (x,y will contain
                   the first character); positive values right-justify (x,y
                   will contain the last character), and 0 centers the string.
        """
        if justify == 0:
            # (x, y) will contain the center character of the string.
            x -= len(s) / 2
        elif justify > 0:
            # (x, y) will contain the rightmost character of the string.
            x -= (len(s) - 1)

        current_x: int = int(x + 0.5)
        current_y: int = int(y + 0.5)

        exclude = " "
        include = ""
        for i in range(len(s)):
            if current_x < 0 or current_x >= self._width or \
               current_y < 0 or current_y >= self._height:
                continue
            if self.overwrite == OverwriteBehavior.NEVER and \
               self._is_occupied(self._grid, current_x, current_y,
                                 exclude, include):
                continue
            self._set_pixel(self._grid, current_x, current_y, s[i])
            current_x += 1


def get_crab_rotation_thetas(length: float, width: float) -> List[float]:
    """
    Calculates the rotation angles that the four swerve modules of a swerve
    drive with the given drive base dimensions needs in order to rotate in a
    circle.

    All four swerve modules are presumed to point forward prior to the
    rotations.  Clockwise angles are negative and counterclockwise angles are
    positive, following classroom conventions.

    Arguments:
    - length: The length of the drive base (that is, the length from the
              center of one of the front wheels to the center of the back
              wheel on the same side.)
    - width:  The width of the drive base (that is, the width from the center
              of one of the left wheels to the center of the right wheel on
              the same end.)
    Returns:
      Returns an array of four angles, in radians, representing the amounts by
      which the front left, front right, back right, and back left swerve
      modules (respectively) need to be rotated so that all four modules are
      tangent to the circumcircle.
    """
    theta: float = math.atan(length / width)
    phi: float = math.pi/2 - theta         # This term cancels out; see below.
    result: List[float] = [0] * 4
    result[FRONT_LEFT] = theta
    result[FRONT_RIGHT] = math.pi - theta  # π/2 + φ = (π/2 - θ) + π/2 = π + θ
    result[BACK_LEFT] = -theta
    result[BACK_RIGHT] = theta - math.pi   # -π/2 - φ = (θ - π/2) - π/2 = θ - π
    return result


def draw_crab_rotation_diagram(canvas: AsciiCanvas,
                               width: float, height: float,
                               params = None):
    """
    Draws an ellipse which circumscribes a rectangle having the aspect ratio of
    width/height.

    The information from https://en.wikipedia.org/wiki/Cyclic_quadrilateral
    was helpful for getting the circumcircle math (roughly) correct on this
    one.

    Arguments:
    - canvas: The AsciiCanvas to "draw" on.  You can inspect the results later
              with canvas.print().
    - width:  The width of the drive base (see get_crab_rotation_thetas for
              details.)
    - height: The _length_ of the drive base (see get_crab_rotation_thetas for
              details.)
    - params: Optional.  An object with any of the following attributes:
      * draw_circle: If True, draw the circumcircle that surrounds the chassis
                     rectangle.
      * forwardBack: The Y-component of the field-oriented direction vector
                     (the front of the field is considered to be the top of
                     the screen.)
                     Range must be from -1.0 to 1.0.
      * leftRight:   The X-component of the field-oriented direction vector
                     (the right side of the field is considered to be the
                     right side of the screen.)
                     Range must be from -1.0 to 1.0.
      * rotation:    The degree to which the crab rotation angles -- or their
                     inverses -- should influence the orientation of the
                     swerve wheels.  At 0, there is no influence, and all four
                     swerve wheels will point in the same direction (namely,
                     the vector [forwardBack, leftRight.])  Increasing the
                     value of the rotation parameter adds a multiple of the
                     crab rotation vectors to that direction vector.
                     Range must be from -1.0 to 1.0.
    """
    forwardBack: float = 0 if not hasattr(params, "forwardBack") or params.forwardBack is None else -params.forwardBack
    leftRight: float = 0 if not hasattr(params, "leftRight") or params.leftRight is None else params.leftRight
    rotation: float = 1.0 if not hasattr(params, "rotation") or params.rotation is None else params.rotation
    direction: Vector = Vector(leftRight, forwardBack)
    direction = direction.normalized()

    # Calculate the directions for all of the individual swerve wheels as
    # vectors.
    thetas: List[float] = get_crab_rotation_thetas(height, width)
    vectors: List[Vector] = [Vector(0, -1),  # FRONT_LEFT
                             Vector(0, -1),  # FRONT_RIGHT
                             Vector(0, -1),  # BACK_LEFT
                             Vector(0, -1)]  # BACK_RIGHT
    for i in range(len(vectors)):
        # Negative rotation parameters should rotate in reverse.
        crabVector: Vector = vectors[i].rotate(thetas[i] if rotation > 0 else thetas[i] + math.pi)

        # Use a combination of the overall direction vector and the rotation
        # parameter.
        r: float = abs(rotation)
        vectors[i] = (1 - r) * direction + (r * crabVector)
        vectors[i] = vectors[i].normalized()  # Probably not needed

    # How much of the canvas height the rectangle representing the chassis
    # will take up.
    HEIGHT_SCALE_FACTOR = 0.7 if not hasattr(params, "zoom") or params.zoom is None else params.zoom
    scaled_height: float = HEIGHT_SCALE_FACTOR * canvas.height

    # Height : scaled_height :: width : scaled_width
    scaled_width: float = scaled_height * width / height

    # Parameshvara's circumradius formula
    perimeter: float = 2 * (scaled_width + scaled_height)
    s: float = 0.5 * perimeter
    a: float = scaled_height
    b: float = scaled_width
    c: float = scaled_height
    d: float = scaled_width
    numerator: float = (a * b + c * d) * (a * c + b * d) * (a * d + b * c)
    denominator: float = (s - a) * (s - b) * (s - c) * (s - d)
    circumradius: float = 0.25 * math.sqrt(numerator / denominator)

    # How much to scale the width of the circumcircle and the rectangle in
    # order to make them look nicer in ASCII.
    ELLIPSE_ASPECT_RATIO = 2.0
    center_x: float = canvas.width / 2
    center_y: float = canvas.height / 2

    # Calculate the corners of the chassis rectangle.
    x1: float = center_x - scaled_width * ELLIPSE_ASPECT_RATIO / 2
    y1: float = center_y - scaled_height / 2
    x2: float = center_x + scaled_width * ELLIPSE_ASPECT_RATIO / 2
    y2: float = center_y + scaled_height / 2

    if hasattr(params, "draw_circle") and params.draw_circle:
        canvas.overwrite = OverwriteBehavior.ALWAYS
        canvas.draw_ellipse(center_x, center_y,
                            circumradius * ELLIPSE_ASPECT_RATIO, circumradius)

    canvas.overwrite = OverwriteBehavior.ALWAYS
    canvas.draw_rect(x1, y1, x2, y2)

    # Draw the chassis wheels as rotated rectangles.
    corners: List[Tuple[float, float]] = [(0, 0)] * 4
    corners[FRONT_LEFT] = (x1, y1)
    corners[FRONT_RIGHT] = (x2, y1)
    corners[BACK_RIGHT] = (x2, y2)
    corners[BACK_LEFT] = (x1, y2)
    for i in range(len(corners)):
        corner_x: float = corners[i][0]
        corner_y: float = corners[i][1]
        WHEEL_LENGTH: float = 7
        WHEEL_WIDTH: float = 5
        # The line segment p1->p2 goes through the long axis of the wheel.
        p1: Tuple[float, float] = (corner_x - (WHEEL_LENGTH/2) * vectors[i].x,
                                   corner_y - (WHEEL_LENGTH/2) * vectors[i].y)
        p2: Tuple[float, float] = (corner_x + (WHEEL_LENGTH/2) * vectors[i].x,
                                   corner_y + (WHEEL_LENGTH/2) * vectors[i].y)
        canvas.overwrite = OverwriteBehavior.ALWAYS
        canvas.draw_rotated_rect(p1[0], p1[1], p2[0], p2[1], WHEEL_WIDTH)

        canvas.draw_line(corner_x,
                         corner_y,
                         corner_x + WHEEL_LENGTH * vectors[i].x,
                         corner_y + WHEEL_LENGTH * vectors[i].y)

    # Label the chassis sides.
    canvas.draw_text(center_x, center_y - scaled_height/2 + 1, f"{width}", 0)
    canvas.draw_text(center_x + scaled_width * ELLIPSE_ASPECT_RATIO / 2 - 1,
                     center_y, f"{height}", 1)

    # Label the swerve angles.
    x_offsets: List[float] = [0] * 4
    x_offsets[FRONT_LEFT] = x_offsets[BACK_LEFT] = -6
    x_offsets[FRONT_RIGHT] = x_offsets[BACK_RIGHT] = 6
    for i in range(len(corners)):
        # Force a vector facing directly upward to have an angle of 0°.
        theta: float = math.atan2(vectors[i].y, vectors[i].x) + math.pi/2

        # Force the overall range to be in the interval [-π, π] with respect
        # to the top (0°).  This represents the minimum rotation that the
        # swerve wheels will have to perform to go from facing forward to
        # facing in the required direction.
        theta = math.fmod(theta + math.pi, 2 * math.pi) - math.pi

        canvas.draw_text(corners[i][0] + x_offsets[i],
                         corners[i][1],
                         f"{theta * RADIANS_TO_DEGREES:.2f}°",
                         +1 if x_offsets[i] < 0 else -1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="""
    Calculates the swerve module angles that will cause a given drive train to
    pivot in place.  It can also calculate the vectors that result from adding
    these rotated vectors to an overall chassis direction.
    """, usage="%(prog)s [-h] width length [forwardBack leftRight rotation]")

    mandatory_group = parser.add_argument_group("Mandatory arguments")
    mandatory_group.add_argument("width",
                                 type=float,
                                 help="Width of the chassis.")
    mandatory_group.add_argument("length",
                                 type=float,
                                 help="Length of the chassis.")

    rendering_group = parser.add_argument_group("Rendering arguments")
    rendering_group.add_argument("--canvas-width", "-W",
                                 type=int,
                                 default=79,
                                 help="Width of the ASCII grid, in characters.  Default %(default)s.")
    rendering_group.add_argument("--canvas-height", "-H",
                                 type=int,
                                 default=25,
                                 help="Height of the ASCII grid, in characters.  Default %(default)s.")
    rendering_group.add_argument("--zoom", "-z",
                                 type=float,
                                 default=0.7,
                                 help="How much to scale chassis length as a percentage of the overall canvas height.  The default is %(default)s.")
    rendering_group.add_argument("--draw-circle", "-c",
                                 action="store_true",
                                 default=False,
                                 help="If true, draw the circumcircle that  the wheels would be tangent to during a full rotation.  Default is %(default)s.")

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
        ["verbose",       args.verbose,       0],
        ["length",        args.length,        0],
        ["width",         args.width,         0],
        ["canvas_width",  args.canvas_width,  0],
        ["canvas_height", args.canvas_height, 0],
        ["draw_circle",   args.draw_circle,   0],
        ["forwardBack",   args.forwardBack,   1],
        ["leftRight",     args.leftRight,     1],
        ["rotation",      args.rotation,      1],
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

    grid = AsciiCanvas(args.canvas_width, args.canvas_height)
    # grid.draw_ellipse(grid.width / 2,
    #                   grid.height / 2,
    #                   grid.width * 0.4,
    #                   grid.height * 0.4)
    # grid.overwrite = OverwriteBehavior.MERGE
    # grid.draw_ellipse(grid.width * 0.75,
    #                   grid.height * 0.25,
    #                   grid.width * 0.3,
    #                   grid.height * 0.3)
    #
    # grid.overwrite = OverwriteBehavior.MERGE
    # grid.draw_line(grid.width - 1, 10, -10, grid.height - 1)

    # grid.overwrite = OverwriteBehavior.NEVER
    # grid.draw_rect(0, 0, grid.width - 1, grid.height - 1)
    #
    # grid.overwrite = OverwriteBehavior.MERGE
    # grid.draw_rotated_rect(0, 0, 79, 25, 1)

    draw_crab_rotation_diagram(grid, args.width, args.length, args)
    grid.print()
