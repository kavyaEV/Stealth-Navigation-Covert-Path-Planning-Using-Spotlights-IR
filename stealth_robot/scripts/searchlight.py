#!/usr/bin/env python3

from sys import argv
from math import sqrt, radians, sin, cos, trunc
import numpy as np

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def mapRound(d):
    return round(round(d / 0.05) * 0.05, 2)


class Map:
    def __init__(self, data, width, height):
        self.map_data = data
        self.width = width
        self.height = height

    def getOccupancy(self, x, y):
        #x and y are in m
        # 0.05 pixels per m
        if self.map_data is None:
            rospy.logwarn("Map data not available yet.")
            return None
        if(trunc(y/0.05) <= self.height and trunc(x/0.05) <= self.width):
            return self.map_data[trunc(y/0.05)*self.width +trunc(x/0.05)]
        else:
            rospy.loginfo("bad map coordinate: ({0}, {1})".format(trunc(x/0.05), trunc(y/0.05)))
            return 100

    def getHeight(self):
        return self.height

    def getWidth(self):
        return self.width

    """
    def mapCallback(self, msg):
        self.map_data = msg.data
        self.width = msg.info.width
        self.height = msg.info.height

        # print("searchlight map width: ", self.width)
    """


class SearchlightCloud:
    def __init__(self, searchlights):
        self.searchlights = searchlights

        # publisher on searchlights topic
        self.position_pub = rospy.Publisher("/searchlights",
                                            PointCloud,
                                            queue_size=10)

        self.visibility_pub = rospy.Publisher("/searchlight_visibility",
                                              MarkerArray, 
                                              queue_size=10) 

        self.rate = rospy.Rate(1)



    def publish(self):
        # set up PointStamped message for Rviz
        # create a MarkerArray message to contin the polygons defining each
        # searchlight's visibility cone
        vis_cone_msg= MarkerArray()

        # create a position array message to publish the searchlight positions
        positionMsg = PointCloud()
        positionMsg.header.seq = 1
        positionMsg.header.frame_id = "map"

        for searchlight_id in range(len(self.searchlights)):
            vis_cone_msg.markers.append(self.searchlights[searchlight_id].get_visibilty_marker(searchlight_id))
            positionMsg.points.append(self.searchlights[searchlight_id].get_position())

        while self.position_pub.get_num_connections() == 0:
            rospy.sleep(1.0)

        self.position_pub.publish(positionMsg)

        while self.visibility_pub.get_num_connections() == 0:
            rospy.sleep(1.0)

        self.visibility_pub.publish(vis_cone_msg)

        """
        # set up PointStamped message for Rviz
        while not rospy.is_shutdown():
            # create a MarkerArray message to contin the polygons defining each
            # searchlight's visibility cone
            vis_cone_msg= MarkerArray()

            # create a position array message t publish the searchlight positions
            positionMsg = PointCloud()
            positionMsg.header.seq = 1
            positionMsg.header.frame_id = "map"


            for searchlight_id in range(len(self.searchlights)):
                vis_cone_msg.markers.append(self.searchlights[searchlight_id].get_visibilty_marker(searchlight_id))
                positionMsg.points.append(self.searchlights[searchlight_id].get_position())


            self.position_pub.publish(positionMsg)
            self.visibility_pub.publish(vis_cone_msg)

            try:
                self.rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                continue
        """


    def total_robot_visibility(self, robot_pos):
        total_visibility = 0
        for id in range(len(self.searchlights)):
            total_visibility += self.searchlights[id].robot_visibility(robot_pos)
        # rospy.loginfo("total visibility of robot at ({0}, {1}) is {2}".format(robot_pos[0], robot_pos[1], total_visibility))
        return total_visibility


class Searchlight:
    def __init__(self, occupancy_map, position, detection_range, beamWidth, cutoffAngle, direction):
        self.occupancy_map = occupancy_map

        # x and y coords
        self.x = position[0]
        self.y = position[1]


        # detection range
        self.range = detection_range
        #angle of the inner beam of light
        self.beamWidth = beamWidth
        #angle of the outer cone where light completely attentuates
        self.cutoffAngle = cutoffAngle
        # direction (degrees)
        self.direction = direction
        self.epsilon = 0.1

        # set up Point32 message
    

    def is_in_range(self, robot_pos):
        """ Check if robot is in range of searchlight """
        robot_x = robot_pos[0]
        robot_y = robot_pos[1]

        dist_x = robot_x - self.x
        dist_y = robot_y - self.y

        # Euclidean distance of robot from searchlight
        robot_dist = sqrt(dist_x**2 + dist_y**2)

        return robot_dist <= self.range

    def is_in_angle(self, robot_pos):
       A = [robot_pos[0], robot_pos[1]]
       B = [self.x, self.y]
       AB = np.array(B) - np.array(A)
       AB_norm = AB / np.linalg.norm(AB)
       dir_rads = np.radians(self.direction)
       dir_norm = np.array([np.cos(dir_rads), np.sin(dir_rads)])

       dot_prod = -np.dot(dir_norm, AB_norm)
       cos_half_fov = np.cos(np.radians(self.cutoffAngle/2))
       return dot_prod >= cos_half_fov

    def detect_robot(self, robot_pos):
        """ Check if robot can be detected by searchlight (is in distance and
        angle range) """
        return self.is_in_range(robot_pos) and self.is_in_angle(robot_pos)
    
    def intensity(self, rx, ry):
        # could improve by calculating outer cone and inner cone intensity drop off too

        #returns an intensity value from the spotlight, satisfying certain physical properties such as conical attenuation
        # light intensity follows basic inverse square law parameterised by "alpha"
        # alpha is calculated such that the light intensity = epsilon (small value close to 0) at the max range of the spotlight
        alpha = (1-self.epsilon) / (self.epsilon*self.range*self.range)

        #calculate the intensity of the light hitting robot at the distance from the spotlight following the customised curve
        i = 1 / (1 + (alpha*((rx-self.x)**2+(ry-self.y)**2)))
        return i
    
    
    def robot_visibility(self, robot_pos):
        if self.detect_robot(robot_pos):
            x0 = mapRound(self.x)
            x1 = mapRound(robot_pos[0])
            y0 = mapRound(self.y)
            y1 = mapRound(robot_pos[1])

            dx = abs(x1-x0)
            dy = abs(y1-y0)
            sx = 0.05 if x0<x1 else -0.05
            sy = 0.05 if y0<y1 else -0.05

            err = dx-dy

            while (mapRound(x0) != mapRound(x1) or mapRound(y0)!= mapRound(y1)):
                #draw line with points unitl the collision. used for visualising in rViz
                #get occupancy grid
                if (self.occupancy_map.getOccupancy (x0, y0) == 100
                    or self.occupancy_map.getOccupancy (x0, y0) == -1):
                    # obstructuion found
                    rospy.loginfo("visibility blocked")
                    return 0
                err2 = 2 * err
                if err2 > -dy:
                    err -= dy
                    x0 += sx

                if err2 < dx:
                    err += dx
                    y0 += sy
            #no obstructuion found
            # return distance intensity calculation
            return self.intensity(x1, y1)
        else:
            return 0
    def coords_between_points(self, x0, y0, x1, y1):
        x0 = mapRound(x0)
        y0 = mapRound(y0)
        x1 = mapRound(x1)
        y1 = mapRound(y1)
        # rospy.loginfo("getting coords between points: ({0},{1}) and ({2},{3})".format(x0, y0, x1, y1))
        coords = []
        dx = abs(x1-x0)
        dy = abs(y1-y0)
        sx = 0.05 if x0<x1 else -0.05
        sy = 0.05 if y0<y1 else -0.05

        err = dx-dy
        coords.append(Point(x=x0, y=y0, z=0))
        while (mapRound(x0) != mapRound(x1) or mapRound(y0)!= mapRound(y1)):
            err2 = 2 * err
            if err2 > -dy:
                err -= dy
                x0 += sx

            if err2 < dx:
                err += dx
                y0 += sy
            coords.append(Point(x=mapRound(x0), y=mapRound(y0), z=0))
        return coords
        
    def collision_correct_point(self, x0, y0, x1, y1):
            x0 = mapRound(x0)
            y0 = mapRound(y0)
            x1 = mapRound(x1)
            y1 = mapRound(y1)

            dx = abs(x1-x0)
            dy = abs(y1-y0)
            sx = 0.05 if x0<x1 else -0.05
            sy = 0.05 if y0<y1 else -0.05

            err = dx-dy
            while (mapRound(x0) != mapRound(x1) or mapRound(y0)!= mapRound(y1)):
                #draw line with points unitl the collision. used for visualising in rViz
                #get occupancy grid
                if (self.occupancy_map.getOccupancy (x0, y0) == 100 or
                        self.occupancy_map.getOccupancy (x0, y0) == -1):
                    # obstructuion found
                    return Point(x=mapRound(x0), y=mapRound(y0), z=0)
                err2 = 2 * err
                if err2 > -dy:
                    err -= dy
                    x0 += sx

                if err2 < dx:
                    err += dx
                    y0 += sy
            return Point(x=x1, y=y1, z=0)

    def get_visibilty_marker(self, polygon_id):
        # pg = Polygon()
        # p1 = Point32(self.x, self.y, 0)
        # pg.points.append(p1)
        # left_bound = (self.direction - self.cutoffAngle / 2) % 360
        # right_bound = (self.direction + self.cutoffAngle / 2) % 360
        # p2 = Point32((self.x + self.range* cos(radians(left_bound))), (self.y + self.range * sin(radians(left_bound))), 0)
        # pg.points.append(p2)
        # p3 = Point32((self.x + self.range* cos(radians(right_bound))), (self.y + self.range * sin(radians(right_bound))), 0)
        # pg.points.append(p3)
        # return pg
        marker = Marker()
        marker.header.frame_id = "map"  # Set the appropriate frame_id
        marker.header.stamp = rospy.Time.now()
        marker.id = polygon_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        left_bound = (self.direction - self.cutoffAngle / 2) % 360
        right_bound = (self.direction + self.cutoffAngle / 2) % 360

        p1 = Point(x=self.x, y=self.y, z=0)
        p2 = Point(x=(self.x + self.range* cos(radians(left_bound))), y=(self.y + self.range * sin(radians(left_bound))), z=0)
        p3 = Point(x=(self.x + self.range* cos(radians(right_bound))), y=(self.y + self.range * sin(radians(right_bound))), z=0)
        coneline = self.coords_between_points(p2.x, p2.y, p3.x, p3.y)
        marker.points = [p1]
        
        for i in range(len(coneline)):
            marker.points.append(self.collision_correct_point(p1.x, p1.y, coneline[i].x, coneline[i].y))
        
        marker.points.append(p1)

        return marker
    
    def get_position(self):
        msg = Point32()
        msg.x = self.x
        msg.y = self.y
        return msg

if __name__ == '__main__':
    # initialise ROS node
    rospy.init_node("searchlights", anonymous=True)

    map = Map()
    searchlights = []

    # dict of all map names and searchlight configurations
    maps_and_searchlights = {
            "lgfloor": [[(20.5, 4.5), 6.5, 45, 45, 75],
                        [(19.6, 19), 4.4, 30, 30, 240],
                        [(10, 25), 9, 30, 30, 270],
                        [(15, 18), 9, 30, 30, 155]],
            "map1": [[],
                     [],
                     []],
            "floorplan": [(20, 9), 9, 30, 30, 30],
            "simple-maze": []
    }

    try:
        # map name as cmdline argument passed in launch file
        map_name = argv[1]
        # construct Searchlight objects according to map name
        for params in maps_and_searchlights[map_name]:
            searchlights.append(Searchlight(position=params[0],
                                            detection_range=params[1],
                                            beamWidth=params[2],
                                            cutoffAngle=params[3],
                                            direction=params[4]))
    except IndexError:
        rospy.logerr("Map name not specified")

    for i in range(10):
        try:
            cloud = SearchlightCloud(searchlights)
        except AttributeError:
            rospy.sleep(i+1)
            continue
        break
