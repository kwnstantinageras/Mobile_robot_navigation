#!/usr/bin/env python

from PIL import Image  # need it for loading the map
import math
import Node


def radians(degree):
    return degree * math.pi / 180


# poso na to valw na kineitai?????????????????
MOVES = [(0.2, radians(0)),  # move ahead
         (-0.2, radians(0)),  # move backwards
         (0, radians(90)),  # turn left
         (0, -radians(90))]  # turn right

# pinakas me antikeimena tis klasis Node
# diladi kathe node ehei tin thesi tou(se world coordinates) kai to varos- diladi poso kostizei na paw se auto apo to start node
nodes = []


class GlobalPathPlanner:
    # start position from the odometry/amcl (?) and goal is supposed to be depended on gesture
    def __init__(self, start, goal):
        print("building map....")
        self.start = start  # start position
        # self.theta = theta  # start orientation
        self.goal = goal  # goal position
        self.map = None  # occupancy grid
        self.width = None
        self.height = None

    # den kserw an hreiazetai pleon
    def plan(self):
        # kalw ti sunartisi gia na mou ftiaksei to graph, ston pinaka nodes
        self.build_graph_from_grid()

        path = self.dijkstra(self.start, self.goal, self.map)
        # tha epistrefei kapoio array-list me ta positions tou path. an einai adeio, den uparhei path gia to shmeio auto
        # to path tha einai 2D list

        if not path:
            print("Path not found.")
        else:
            # ehw to path, ara epistrefw ta shmeia gia na kalesw local planner h na kalesw apo edw?

            """
            # print("Constructing path..")
            # path = self.construct_path(final)  # path in world coordinates
            # print("path: ")
            # points = []
           
            for step in path:
                points.append((step.x, step.y))
            # publish this path - safegoto for each of the path components
            points.reverse()
            points = points[1:]
            points.append((self.goal.x, self.goal.y))
            for p in range(len(points)):
                print("x:", points[p][0], " y:", points[p][1])
            '''
            # first process the points
            translate_x = path[0][0]
            translate_y = path[0][1]
            for p in range(len(path)):
                new_x = path[p][0] - translate_x
                new_y = path[p][1] - translate_y
                if self.theta == math.pi / 2:
                    path[p] = [-new_y, new_x]
                elif self.theta == math.pi:
                    path[p] = [-new_x, -new_y]
                elif self.theta == -math.pi / 2:
                    path[p] = [new_y, -new_x]
                else:
                    path[p] = [new_x, new_y]
            # translate coordinates for theta

    '''
            # run safegoto on the translated coordinates
            robot = SafeGoTo()
            robot.travel(points)
    '''
    """

    # TO-DO

    # metatrepw ta pixels se world coordinates, kai an einai obstacle-free se nodes
    # epistrefei ton pinaka nodes, diladi oles tis world coordinates stis opoies borei na paei to robot
    def build_graph_from_grid(self):
        global nodes
        for x in range(self.width):
            for y in range(self.height):
                coordinate = x, y
                world_points = Node.pixel_to_world((x, y), (self.width, self.height))

                # gia kathe pixel, an einai obstacle free
                # if self.map([x][y]) != 0: # an den einai 0, shmainei oti einai obstacle free
                #   world_points = Node.pixel_to_world((x, y), (self.width, self.height))

                #UPDATE:
                #an uparhei idio world coordinate tote:
                    #an to pixel einai mauro/gkri:
                        #vgale to world coordinate apo ta nodes[]
                #an den uparhei to world coordinate:
                    # an to pixel einai aspro:
                        #vale to world coordinate sta nodes[]


                # an uparhei node ston pinaka me autes tis suntetagmenes kai to pixel einai aspro min kaneis tipota
                # an uparhei node ston pinaka me autes tis suntetagmenes kai to pixel einai mauro vgale to node
                # alliws prosthese to

                for node in range(len(nodes)):
                    if node.x == world_points[0] and node.y == world_points[1]:
                        if self.map([x][y]) == 0:
                            # delete node from array
                            nodes.remove(node)
                    else:
                        new_node = Node.Node(x, y)
                        nodes.append(new_node)

        # PWS BORW NA KRINW AN KAPOIO NODE EINAI H OHI AVAILABLE
        # ME TH SUNARTHSH IS_MOVE_VALID AN ELEGXW AN APO AUTO TO NODE BORW NA PAW KAPOU?
        # AN DEN BORW NA PAW KAPOU DEN EINAI OBSTACLE FREE
        """
        gia kathe pixel vale se ena pinaka tis suntetagmenes tou se world coordinates, efoson
        den uparhei kapoio pixel ston pinaka pou na thewreitai idia i thesi tou kai efoson thewreitai obstacle free
        """

        # return nodes

    def dijkstra(self, start, end, grid_map):
        # start, end are in world coordinates and Node objects

        # before starting Dijkstra, check if goal is reachable - ie, in an obstacle free zone - if not, directly reject
        # borw kai na dw an uparhei san node sto graph
        if not end.is_valid(grid_map):
            print("goal invalid")
            return None
        print("goal valid")

        visited_set = []
        unvisited_set = nodes  # einai mesa kai to start
        #min_distance = math.inf

        for node in range(len(nodes)):
            if node.x == start.x and node.y == start.y:
                node.weight = 0

        while len(unvisited_set) != 0:
            min_distance = math.inf

            # vriskw to node me to mikrotero varos
            for node in range(len(unvisited_set)):

                if node.weight < min_distance:
                    min_distance = node.weight
                    u = node
            unvisited_set.remove(u)
            visited_set.append(u)

            # for each neighbor of u
            for move in MOVES:
                neighbor = u + move  # kati tetoio
                if neighbor in unvisited_set:
                    temp_distance = u.weight + 1
                    if temp_distance < neighbor.weight:
                        neighbor.weight = temp_distance
                        neighbor.parent = u

                # if node.is_move_valid(map, move)

            # enimerwnw to node pou eksetasa ston pinaka nodes h einai pio grigoro na ta valw se neo pinaka?

        # return visited_set

        """
        DIJKSTRA ALGORITHM
        # edw tha valw ta weights sta Node ston pinaka nodes, otan tha trehw ton algorithmo
        EHW TO PATH
        """

        """
        # teliko stadio-epeidi ehw kanei backward search, ta antistrefw sto path
        # DE NOMIZW OTI ISHUEI AUTO APLA HTIZW TO PATH ME VASH TOUS PARENTS
        points = []

        for step in path:
            points.append((step.x, step.y))
        points.reverse()
        points = points[1:]
        points.append((self.goal.x, self.goal.y))
        """

        '''for p in range(len(points)):
            print("x:", points[p][0], " y:", points[p][1])
        '''

        """
        afou teleiwsei h episkepsi se olous tous komvous tou gragimatos
        tha ehw pleon ton ananeomeno pinaka nodes[] pou tha ehei kai ta vari mesa
        epomenws meta ksekinaw apo to end node kai htizw to path mou mesw twn parents
        """
        """
        backtrack from end to construct path
        """
        current = end
        path = []  # path needs to be in world coordinates
        while current != None:
            path.append(current)
            current = current.parent
        # return path

        path.reverse()
        path = path[1:]
        path.append(end)

        """
        points.reverse()
        points = points[1:]
        points.append((self.goal.x, self.goal.y))

        return points
        """
        return path

    def load_map(self, image_path):
        # map_image = Image.open('/home/konstantina/catkin_ws/src/pioneer_gazebo_ros/pioneer_gazebo/map/small_world.pgm')
        map_image = Image.open(image_path)
        self.width, self.height = map_image.size
        # self.pixels = map_image.load()
        # self.map_image.show()
        # width, height = map_image.size
        self.map = map_image.load()

        '''for x in range(width):
            for y in range(height):
                print(grid_map[x, y])
        '''

        # fortwnw ta pixel tis eikonas wste meta na elegxw an einai h timi tou pixel == or != me to 0, me skopo
        # na vlepw an ehei empodio h ohi(mauro pixel = empodio)
        # PROSOHI NA GINEI ME TI SWSTI SEIRA WIDTH-HEIGHT OTAN THA ELEGXW

        return self.map


def main():
    # metatrepw tin arhiki kai tin teliki thesi(tin teliki ehw tha tin ehw orisei vevea opote tha einai default)
    # se nodes gia na ta dwsw ston planner
    # pws tha metatrepsw tin arhiki thesi se node?
    # den tha ehei kapoio sfalma auto?
    # pws na to kanw
    # kapws tha ginetai, se kapoio node tha prepei na anhkei
    path = GlobalPathPlanner(0, 0, 0, 0)
    path.load_map('/home/konstantina/catkin_ws/src/pioneer_gazebo_ros/pioneer_gazebo/map/small_world.pgm')


if __name__ == "__main__":
    main()
