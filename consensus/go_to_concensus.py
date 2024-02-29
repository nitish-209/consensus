#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#let the turtle bots be positioned at points
#(x1,y1),(x2,y2).....(xn,yn)
# and be the list of these points 

locations = [[4,5],[3,5],[2,-1],[4,2],[-2,2]]
point1 = [0,0]
#Finding the furthest point from origin
for point in locations:
    if (point[0]**2+point[1]**2)>(point1[0]**2+point1[1]**2):
        point1 = point.copy()
print("First point",point1)
locations.remove(point1)
print(locations)
#x,y co-ordinate of farthest point 
x1 = point1[0]
y1 = point1[1]

radius = 0

#Finding Next point on circle
for point in locations:
    x = point[0]
    y = point[1]

    xc = ((x1**2+y1**2-x**2-y**2)*x1)/(2*((x1**2-x*x1)+(y1**2-y*y1)))
    yc = xc * y1 / x1

    r = (x1-xc)**2 + (y1-yc)**2

    if r > radius:
        radius = r
        xc1 = xc
        yc1 = yc
        x2 = x
        y2 = y
print(f"Second point   ({x2},{y2})")
print(f"Center of circle   ({xc1},{yc1})")

locations.remove([x2,y2])
print(locations)

## Midpoint of chord joining First and second Point
xm = (x1+x2)/2
ym = (y1+y2)/2

## Finding points outside the circle whoe center is (xm,ym):::
outside_points = []

for point in locations:
    x = point[0]
    y = point[1]
    if (x-xm)**2 + (y-ym)**2 > (xm-x1)**2 + (ym-y1)**2:
        outside_points.append(point)

print(f"Outside points={outside_points}")
## Finding Final Concensus point::::

if not outside_points:
    x_con = xm
    y_con = ym
else:
    print(f"xm={xm},ym={ym}\nxc1={xc1},yc1={yc1}")
    m = (ym-yc1)/(xm-xc1)
    c = m*xm - ym
    print("m =",m)
    print("c =",c)
    radius = 0
    for point in outside_points:
        x = point[0]
        y = point[1]

        xc = ((y1-y)*(y1+2*c+y)+(x1**2-x**2))/(2*((x1-x)+m*(y1-y)))
        yc = m*xc - c
        print(xc,yc)
        r2 = (x1-xc)**2 + (y1-yc)**2
        if r2>radius:
            radius = r2
            x_con = xc
            y_con = yc
            x3 = x
            y3 = y

print(f"Third Point on circle=({x3},{y3})")
print(f"Concensus Point=({x_con},{y_con})")

