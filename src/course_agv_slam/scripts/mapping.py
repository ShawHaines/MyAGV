import math
import numpy as np


class Mapping():
    def __init__(self,xw,yw,xyreso):
        self.width_x = xw*xyreso
        self.width_y = yw*xyreso
        self.xyreso = xyreso
        self.xw = xw
        self.yw = yw
        self.pmap = np.ones((self.xw, self.yw))/2 # default 0.5 -- [[0.5 for i in range(yw)] for i in range(xw)] 
        self.minx = -self.width_x/2.0
        self.maxx =  self.width_x/2.0
        self.miny = -self.width_y/2.0
        self.maxy =  self.width_y/2.0
        pass

    def update(self, ox, oy, center_x, center_y):
        # TODO
        return self.pmap

def main():
    import rospy
    from nav_msgs.msg import OccupancyGrid
    """
    Example usage
    """
    print(__file__, "start")
    length = 20
    xyreso = 0.1  # x-y grid resolution
    xyw = int(round(length/xyreso))
    mapping = Mapping(xyw,xyw,xyreso)
    lidar_num = 200

    for i in range(10):
        center_x = 3 - i*0.3
        center_y = 3 - i*0.3
        ang = np.linspace(0,2*math.pi,lidar_num)
        dist = np.random.rand(lidar_num)*1+5
        ox = np.sin(ang) * dist # + center_x
        oy = np.cos(ang) * dist # + center_y
        pmap = mapping.update(ox, oy, center_x, center_y)

        rospy.init_node("map_pub")

        map_pub = rospy.Publisher('/slam_map',OccupancyGrid,queue_size=1)

        map_msg = OccupancyGrid()
        map_msg.header.seq = 1
        map_msg.header.stamp = rospy.Time().now()
        map_msg.header.frame_id = "map"

        map_msg.info.map_load_time = rospy.Time().now()
        map_msg.info.resolution = xyreso
        map_msg.info.width = xyw
        map_msg.info.height = xyw
        map_msg.info.origin.position.x = -xyw*xyreso/2.0
        map_msg.info.origin.position.y = -xyw*xyreso/2.0
        map_msg.info.origin.position.z = 0
        map_msg.info.origin.orientation.x = 0
        map_msg.info.origin.orientation.y = 0
        map_msg.info.origin.orientation.z = 0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = list(pmap.reshape(-1)*100)
        
        map_pub.publish(map_msg)
        rospy.sleep(1)

if __name__ == '__main__':
    main()     
