#!/usr/bin/python3

# Let the turtle bots be positioned at points
# (x1,y1),(x2,y2).....(xn,yn)
# and be the list of these points 

#robot_locations = [[-2,4.5],[-3,4],[-4,3],[2,2],[1,1]]
#robot_locations = [[3,4],[5,0],[-3,4],[-0.98,-4.9],[-13,6]]
#robot_locations = [[3, 4], [5, 7], [-4, -5], [8, -5], [0, 6]]
#robot_locations = [[8, 6], [5, 3], [9, 0], [-7, 4], [6, 9]]
#robot_locations = [[3, -7], [-4, 5], [5, 6], [6, -4], [-4, 4], [0, 8], [-5, 3]]
robot_locations = [[3, 5], [3, 2], [-4, 10], [-9, -4], [-4, 10], [-7, -5], [-10, -2]]

# # Average Point ::
# n = len(robot_locations)
# xavg, yavg = 0, 0
# for point in robot_locations:
#     xavg += point[0]
#     yavg += point[1]

# xavg = xavg/n
# yavg = yavg/n

# locations =[]

# # Transformation wrt avg point:
# for point in robot_locations:
#     locations.append([point[0]-xavg, point[1]-yavg])
locations = robot_locations
# Finding the furthest point from origin
point1 = [0,0]
for point in locations:
    if (point[0]**2+point[1]**2)>(point1[0]**2+point1[1]**2):
        point1 = point.copy()
print("First point",point1)
locations.remove(point1)
print(locations)
# x,y co-ordinate of farthest point 
x1 = point1[0]
y1 = point1[1]

radius = 0

# Finding Next point on circle
for point in locations:
    x = point[0]
    y = point[1]
    if x1 == 0:
        xc = 0
        yc = (x1**2 + y1**2 - x**2 - y**2) / (2 * (y - y1))
    else:
        xc = ((x1**2 + y1**2 - x**2 - y**2) * x1)/(2 * ((x1**2 - x*x1) + (y1**2 - y*y1)))
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

## Finding points outside the circle whose center is (xm,ym):::
outside_points = []

for point in locations:
    x = point[0]
    y = point[1]
    if (x-xm)**2 + (y-ym)**2 > (xm-x1)**2 + (ym-y1)**2:
        outside_points.append(point)

print(f"Outside points={outside_points}")
## Finding Final Concensus point ::::

if not outside_points:
    x_con = xm
    y_con = ym
else:
    print(f"xm={xm},ym={ym}\nxc1={xc1},yc1={yc1}")
    if xm == xc1:
        radius = 0
        for point in outside_points:
            x = point[0]
            y = point[1]

            xc = xm
            yc = (x1**2 - x**2 + y1**2 - y**2 - 2*xm*(x1 - x)) / (2*y1 - 2*y)

            r2 = (x1-xc)**2 + (y1-yc)**2
            if r2>radius:
                radius = r2
                x_con = xc
                y_con = yc
                x3 = x
                y3 = y
    else:
        m = (ym-yc1)/(xm-xc1)
        c = m*xm - ym
        radius = 0
        for point in outside_points:
            x = point[0]
            y = point[1]

            xc = ((y1-y)*(y1+2*c+y)+(x1**2-x**2))/(2*((x1-x)+m*(y1-y)))
            yc = m*xc - c

            r2 = (x1-xc)**2 + (y1-yc)**2
            if r2>radius:
                radius = r2
                x_con = xc
                y_con = yc
                x3 = x
                y3 = y

# # Transforming back to original System ::
# x_con = x_con + xavg
# y_con = y_con + yavg
print(f"Concensus Point=({x_con},{y_con})")

