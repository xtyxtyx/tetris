"""
solver -- Tetris puzzle solver.

Copyright (c) 2017 Caleb Begly.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from __future__ import division, generators, print_function
from collections import namedtuple
import numpy as np
import rospy

__all__ = ['TileType', 'Tile', 'TILE_TYPES', 'EMPTY', 'FLAT', 'AR_TAG',
           'rotate', 'solve_puzzle', 'optimize_solution',
           'construct_solution_board', 'display_solution']


EMPTY, FLAT, AR_TAG = 0, 1, 2
TileType = namedtuple('TileType', ['marker_id', 'name', 'rotations', 'x_offset',
                                   'y_offset', 'pattern'])
Tile = namedtuple('Tile', ['tile_name', 'row', 'column', 'rotations'])

TILE_SIZE = rospy.get_param('tile_size')/100
SQUARE_TILE = TileType(0, 'square', 1, 0.9*TILE_SIZE, -0.9*TILE_SIZE, [
    [FLAT, AR_TAG],
    [FLAT, FLAT],
])
LINE_TILE = TileType(1, 'line', 2, 0, -1.5*TILE_SIZE, [
    [AR_TAG],
    [FLAT],
    [FLAT],
    [FLAT],
])
S_TILE = TileType(2, 's', 2, -5/6*TILE_SIZE, 0, [
    [FLAT, EMPTY],
    [FLAT, AR_TAG],
    [EMPTY, FLAT],
])
Z_TILE = TileType(3, 'z', 2, 5/6*TILE_SIZE, 0, [
    [EMPTY, FLAT],
    [AR_TAG, FLAT],
    [FLAT, EMPTY],
])
REVERSE_L_TILE = TileType(4, 'reverse_l', 4, 4.75/6*TILE_SIZE, -TILE_SIZE, [
    [AR_TAG, EMPTY, EMPTY],
    [FLAT, FLAT, FLAT],
])
T_TILE = TileType(5, 't', 4, 0, 5/6*TILE_SIZE, [
    [FLAT, FLAT, FLAT],
    [EMPTY, AR_TAG, EMPTY],
])
L_TILE = TileType(6, 'l', 4, TILE_SIZE, -4.75/6*TILE_SIZE, [
    [AR_TAG, FLAT],
    [EMPTY, FLAT],
    [EMPTY, FLAT],
])

TILE_TYPES = {tile_type.name: tile_type for tile_type in
    [SQUARE_TILE, LINE_TILE, S_TILE, Z_TILE, REVERSE_L_TILE, T_TILE, L_TILE]}


def is_solution(board):
    """ Checks whether the given board has no empty cells. """
    rows, columns = board.shape
    for i in range(rows):
        for j in range(columns):
            if board[i, j] == EMPTY:
                return False
    return True


def rotate_once(pattern):
    """
    Rotates a tile pattern by 90 degrees clockwise.

    >>> rotate(np.array([[1, 1, 1], [0, 1, 0]], dtype=np.int)).tolist()
    [[0, 1], [1, 1], [0, 1]]
    """
    pattern = np.array(pattern, dtype=np.int)
    rows, columns = pattern.shape
    rotated_pattern = np.empty((columns, rows), dtype=np.int)
    for i in range(rows):
        for j in range(columns):
            rotated_pattern[j, rows - 1 - i] = pattern[i, j]
    return rotated_pattern


def rotate(pattern, iters=1):
    for _ in range(iters % 4):
        pattern = rotate_once(pattern)
    return np.array(pattern, dtype=np.int)


def can_place_tile(board, tile_pattern, row, column):
    """
    Checks whether a tile can be placed at the given location.
    """
    rows, columns = board.shape
    pattern_rows, pattern_columns = tile_pattern.shape
    for i in range(pattern_rows):
        for j in range(pattern_columns):
            board_row, board_column = row + i, column + j
            in_bounds = 0 <= board_row < rows and 0 <= board_column < columns
            if not in_bounds or (board[board_row, board_column] != EMPTY
                                 and tile_pattern[i, j] != EMPTY):
                return False
    return True


def place_tile(board, tile_pattern, row, column):
    pattern_rows, pattern_columns = tile_pattern.shape
    for i in range(pattern_rows):
        for j in range(pattern_columns):
            if board[row + i, column + j] == EMPTY:
                board[row + i, column + j] = tile_pattern[i, j]
            else:
                assert tile_pattern[i, j] == EMPTY


def solve_puzzle_partial(board, tiles, solution, row_start=0, col_start=0):
    """ Solve a partially filled board. """
    rows, columns = board.shape
    for i in range(row_start, rows):
        inner_start = col_start if i == row_start else 0
        for j in range(inner_start, columns):
            if board[i, j] == EMPTY:
                for tile_name in filter(tiles.get, tiles):
                    tile_type = TILE_TYPES[tile_name]
                    pattern = np.array(tile_type.pattern, dtype=np.int)
                    assert tile_type.rotations in [1, 2, 4]
                    for rotation in range(tile_type.rotations):
                        if can_place_tile(board, pattern, i, j):
                            board_copy = np.copy(board)
                            place_tile(board_copy, pattern, i, j)
                            tiles_copy = dict(tiles)
                            tiles_copy[tile_name] -= 1
                            if solve_puzzle_partial(board_copy, tiles_copy, solution, i, j):
                                solution.append(Tile(tile_name, i, j, rotation))
                                return True
                        pattern = rotate_once(pattern)
                return False
    return is_solution(board)


def solve_puzzle(rows, columns, tiles):
    """ Solve for an empty board, given a distribution of available tiles. """
    board, solution = np.full((rows, columns), EMPTY), []
    solved = solve_puzzle_partial(board, tiles, solution)
    if solved:
        return solution


def optimize_solution(solution):
    max_col = max(tile.column for tile in solution)
    return list(sorted(solution, key=lambda tile: (max_col + 1)*tile.row + tile.column))


def construct_solution_board(rows, columns, solution):
    board = np.empty((rows, columns), dtype=np.int)
    for k, tile in enumerate(solution):
        pattern = rotate(np.array(TILE_TYPES[tile.tile_name].pattern, np.int), tile.rotations)
        pattern_rows, pattern_columns = pattern.shape
        for i in range(pattern_rows):
            for j in range(pattern_columns):
                if pattern[i, j] != EMPTY:
                    board[tile.row + i, tile.column + j] = k + 1
    return board


def display_solution(rows, columns, solution, scale=1, offset=2):
    board = construct_solution_board(rows, columns, solution)
    if scale > 1:
        rows, columns = rows*scale, columns*scale
        old_board, board = board, np.empty((rows, columns), dtype=np.int)
        for i in range(rows):
            for j in range(columns):
                board[i, j] = old_board[i//scale, j//scale]
    for i in range(rows):
        print(' '*offset, end='')
        for j in range(columns):
            fmt_id = board[i, j]%16
            code = '{}'.format(int(fmt_id/2 + 31))
            if fmt_id%2:
                code += ';1'
            print(u'\x1b[{}m'.format(code) + u'\u2588'*2 + u'\x1b[0m', end='')
        print()


if __name__ == '__main__':
    # Example usage
    print('\n')
    display_solution(6, 8, solve_puzzle(6, 8, {
        'square': 2,
        'line': 2,
        's': 2,
        'z': 1,
        'reverse_l': 2,
        't': 2,
        'l': 1,
    }), scale=4, offset=4)
    print('\n')
