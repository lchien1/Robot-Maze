import numpy as np
import pdb
import heapq
'''A simple module to store and query 2D mazes with grid-aligned walls.'''

DIR_LEFT = 0
DIR_RIGHT = 1
DIR_DOWN = 2
DIR_UP = 3

DIR_RC_DELTAS = [
    (0, -1),
    (0, 1),
    (1, 0),
    (-1, 0)
]

DIR_XY_DELTAS = [
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1)
]

DIR_STRINGS = [
    'left',
    'right',
    'forward',
    'backward'
]

DIR_STRING_LOOKUP = dict(zip(DIR_STRINGS, range(4)))

def split_command(str_or_list):
    '''Splits a command string into five integer tokens. For instance, the
string

  0 0 up 3 5

would return the tuple 0, 0, 2, 3, 5 (because DIR_UP=2).

    '''

    if isinstance(str_or_list, str):
        x0, y0, dir0, x1, y1 = str_or_list.split()
    else:
        x0, y0, dir0, x1, y1 = str_or_list
        
    return int(x0), int(y0), DIR_STRING_LOOKUP[dir0.lower()], int(x1), int(y1)


    
def find_path(maze,x0,y0,x1,y1):
   
    w,h = maze.width(), maze.height()
    g = [[1000000]*(w+2) for i in range(h+2)]
    g[x0][y0] =  0 
    pred = [[None]*(w+2) for i in range(h+2)]
    path = []
    Q = []
    Q.append((g[x0][y0],(x0,y0)))
    while Q:
#Finding minimum cost element in Q
        Q.sort(reverse = True)      
        node_p = Q.pop()
        node = node_p[1]
#        pdb.set_trace();
        if node[0] == x1 and node[1] == y1:
            
            path = construct_path(pred, x1, y1)                
            print("Path found")
            break
        else:
            neighbors = maze.reachable_neighbors(node[0],node[1])
            for neighbor in neighbors:
            
                    curr_cost = g[node[0]][node[1]]
                    x_neigh,y_neigh = neighbor[0],neighbor[1]
                    k = curr_cost + 1
                    try:           
                        if k < g[x_neigh][y_neigh]:
                                g[x_neigh][y_neigh] = k
                                pred[x_neigh][y_neigh] = (node[0],node[1])
                                Q.append((k,(x_neigh,y_neigh)))
                                if x_neigh == x1 and y_neigh == y1: 
                                    path = construct_path(maze,pred, x1, y1)
                                    return path
                    except:
                        print('YO' , x_neigh, y_neigh)
    return path

def construct_path(m,pred,x,y):
    #pdb.set_trace()
    path = []
    while pred[x][y] != None:
        path.append((x,y))
        x, y = pred[x][y][0], pred[x][y][1]
    path.append((x,y))    
    path.reverse()
  
    
    return path

class Maze(object):

    '''Encapsulate a maze and support querying/setting positions of walls.
When a maze is read in from a file or printed, the bottom-left corner
corresponds to (x, y) = (0, 0). The x coordinate increases to the
right and the y coordinate increases going up.

    '''

    def __init__(self, size_or_filename=None):

        '''Initializer. You can pass in a size in as a (width, height) tuple
to create an empty maze of that size, or a filename of a file to
read. Otherwise, creates an empty 0x0 maze. You can call load() or
create() later to finish constructing the maze.

        '''

        self.data = None

        if isinstance(size_or_filename, tuple):
            self.create(*size_or_filename)
        elif isinstance(size_or_filename, str):
            self.load(size_or_filename)
        else:
            self.clear()

    def _rc_valid(self, r, c):
        '''Internal helper function to check if row/col valid.'''
        return (r >= 0 and c >= 0 and
                r < self.data.shape[0] and
                c < self.data.shape[1])

    def _xy_to_rc(self, x, y):
        '''Internal helper function to convert x/y to row/col.'''
        r = self.data.shape[0]-1-2*y
        c = 2*x
        return r, c

    def width(self):
        '''Returns the width of this maze.'''
        return (self.data.shape[1]+1)/2

    def height(self):
        '''Returns the height of this maze.'''
        return (self.data.shape[0]+1)/2

    def clear(self):
        '''Resets the maze to an empty 0x0 maze.'''
        self.data = np.empty((0, 0), dtype=bool)

    def create(self, width, height):
        '''Create a blank maze of the given size. No walls will be added.'''
        if width <= 0 or height <= 0:
            raise RuntimeError('cannot create empty maze; use clear() instead.')
        self.data = np.zeros((2*height-1, 2*width-1), dtype=bool)

    def add_wall(self, x, y, dir_index):
        '''Add a wall to the given (x, y) location on the border with the
given direction. Invalid locations/directions will fail silently.'''
        self.set_wall(x, y, dir_index, True)

    def remove_wall(self, x, y, dir_index):
        '''Remove a wall from the given (x y) location on the border with the
given direction. Invalid locations/directions will fail silently.'''
        self.set_wall(x, y, dir_index, False)

    def set_wall(self, x, y, dir_index, wval):

        '''Add or remove a wall to the given (x, y) location on the border
with the given direction. Invalid locations/directions will fail
silently.

        '''

        r0, c0 = self._xy_to_rc(x, y)

        delta_r, delta_c = DIR_RC_DELTAS[dir_index]

        r1, c1 = r0+delta_r, c0+delta_c

        if self._rc_valid(r0, c0) and self._rc_valid(r1, c1):
            self.data[r1, c1] = bool(wval)

    def can_move(self, x, y, dir_index):

        '''Returns true if it is possible to move in the given direction from
the cell (x, y).'''

        r, c = self._xy_to_rc(x, y)

        delta_r, delta_c = DIR_RC_DELTAS[dir_index]

        for _ in range(3):
            if not self._rc_valid(r, c) or self.data[r, c]:
                return False
            r += delta_r
            c += delta_c

        return True

    def reachable_neighbors(self, x, y):

        '''Return a list of all (xn, yn) positions for neighbors of the cell
at (x, y).'''
     
        r, c = self._xy_to_rc(x, y)

        rval = []

        if not self._rc_valid(r, c):
            return rval

        for dir_index in range(4):
            if self.can_move(x, y, dir_index):
                dx, dy = DIR_XY_DELTAS[dir_index]
                rval.append((x+dx, y+dy))

        return rval

    def pretty(self):

        '''Return a string representing a "pretty-print" of the maze with the
characters +|- as well as space and newline.'''

        outstr = []

        nrows, ncols = self.data.shape

        outstr.append(('+-' * ((ncols+1)/2)) + '+\n')

        for i in range(nrows):
            outstr.append('|')
            for j in range(ncols):
                if not self.data[i, j]:
                    outstr.append(' ')
                elif i % 2 == 0 and j % 2 == 0:
                    outstr.append('#')
                elif i % 2 == 0 and j % 2 == 1:
                    outstr.append('|')
                elif i % 2 == 1 and j % 2 == 0:
                    outstr.append('-')
                else:
                    outstr.append('+')
            outstr.append('|\n')

        outstr.append(('+-' * ((ncols+1)/2)) + '+\n')

        return ''.join(outstr)

    def load(self, filename):

        '''Load a maze from a file with the given name.'''

        with open(filename, 'r') as f:
            array_of_strings = [line.strip() for line in f]
        self.load_from_array(array_of_strings, filename)

    def load_from_string(self, string):
        '''Load a maze from a Python string in memory.'''

        return self.load_from_array(self, string.split('\n'))

    def load_from_array(self, array_of_strings, filename='maze input'):

        '''Load a maze from an array of strings. You can provide an optional
filename which will appear in any error messages produced by this
function.'''

        nrows = len(array_of_strings)
        ncols = len(array_of_strings[0])

        if nrows < 3 or ncols < 3:
            raise RuntimeError('{}: error: empty maze!'.format(filename))

        self.data = np.zeros((nrows-2, ncols-2))

        for i in range(nrows):

            if len(array_of_strings[i]) != ncols:
                raise RuntimeError('{}:{} wrong length '
                                   '(expected {}, but got {})'.format(
                                       filename, i+1, ncols,
                                       len(array_of_strings[i])))

            for j in range(ncols):

                cij = array_of_strings[i][j]

                if (i == 0 or i+1 == nrows or
                    j == 0 or j+1 == ncols or
                    (i % 2 == 0 and j % 2 == 0)):

                    if cij.isspace():
                        print '{}:{}:{} warning: expected non-space'.format(
                            filename, i+1, j)

                elif i % 2 == 1 and j % 2 == 1:

                    if not cij.isspace():
                        print '{}:{}:{} warning: expected space'.format(
                            filename, i+1, j)

                if i > 0 and j > 0 and i+1 < nrows and j+1 < ncols:
                    self.data[i-1, j-1] = not cij.isspace()

    def is_solvable(self):

        '''Return true if the maze is solvable -- that is, every other cell
can be reached from position (0, 0).'''

        init_pos = (0, 0)

        queue = [init_pos]
        visited = set(queue)

        while len(queue):
            x, y = queue.pop()
            for n in self.reachable_neighbors(x, y):
                if n not in visited:
                    visited.add(n)
                    queue.append(n)

        for y in range(self.height()):
            for x in range(self.width()):
                if (x, y) not in visited:
                    return False

        return True



def _do_tests():


    test_cmds = [
        '0 0 up 4 5',
        '3 2 down 1 2',
        '0 2 left 0 4',
        '1 1 right 1 2'
        ]

    for test_cmd in test_cmds:
        print test_cmd, '->', split_command(test_cmd)

    print

    test_maze_1 = [
        '+-+-+-+-+',
        '| |     |',
        '+ + +-+ +',
        '| | |   |',
        '+ + +-+-+',
        '|       |',
        '+-+-+ +-+',
        '|       |',
        '+ + +-+-+',
        '| |     |',
        '+ +-+-+ +',
        '| |     |',
        '+-+-+-+-+',
    ]
    
    test_maze_2 = [
        '+-+-+-+',
        '| | | |',
        '+ + + +',
        '| |   |',
        '+-+-+-+'
    ]

    test_mazes = [
        test_maze_1,
        test_maze_2,
    ]

    for test_maze in test_mazes:

        print '*'*50
        print

        m = Maze()
        m.load_from_array(test_maze)

        h = m.height()
        w = m.width()

        print 'maze is {} by {}:'.format(w, h)
        print
        print m.pretty()

        for y in range(h):
            for x in range(w):
                print 'neighbors of {} are {}'.format(
                    (x, y), m.reachable_neighbors(x, y))

        print
        print 'maze solvable:', m.is_solvable()
        print

        for wval in [True, False]:

            for y in range(h):
                for x in range(w):
                    for d in range(4):
                        m.set_wall(x, y, d, wval)

            print m.pretty()
            print 'maze solvable:', m.is_solvable()
            print

            assert m.is_solvable() == (not wval)


if __name__ == '__main__':

    _do_tests()
