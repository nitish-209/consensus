'''

 WELZL ALGORITHM TO FIND MINIMUM ENCLOSING CIRCLE

'''
import random

# Python3 program to find the minimum enclosing 
# circle for N integer points in a 2-D plane 
from math import sqrt 
from random import randint,shuffle 
  
# Defining infinity 
INF = 1e18
  
# Structure to represent a 2D point 
class Point : 
    def __init__(self,X=0,Y=0) -> None: 
        self.X=X 
        self.Y=Y 
  
# Structure to represent a 2D circle 
class Circle : 
    def __init__(self,c=Point(),r=0) -> None:         
        self.C=c 
        self.R=r 
  
  
# Function to return the euclidean distance 
# between two points 
def dist(a, b): 
    return sqrt(pow(a.X - b.X, 2) 
                + pow(a.Y - b.Y, 2)) 
  
  
# Function to check whether a point lies inside 
# or on the boundaries of the circle 
def is_inside(c, p): 
    return dist(c.C, p) <= c.R 
  
  
# The following two functions are used 
# To find the equation of the circle when 
# three points are given. 
  
# Helper method to get a circle defined by 3 points 
def get_circle_center(bx, by, 
                        cx, cy): 
    B = bx * bx + by * by 
    C = cx * cx + cy * cy 
    D = bx * cy - by * cx 
    return Point((cy * B - by * C) / (2 * D), 
             (bx * C - cx * B) / (2 * D)) 
  
# Function to return the smallest circle 
# that intersects 2 points 
def circle_from1(A, B): 
    # Set the center to be the midpoint of A and B 
    C = Point((A.X + B.X) / 2.0, (A.Y + B.Y) / 2.0 ) 
  
    # Set the radius to be half the distance AB 
    return Circle(C, dist(A, B) / 2.0 ) 
  
# Function to return a unique circle that 
# intersects three points 
def circle_from2(A, B, C): 
    I = get_circle_center(B.X - A.X, B.Y - A.Y, 
                                C.X - A.X, C.Y - A.Y) 
  
    I.X += A.X 
    I.Y += A.Y 
    return Circle(I, dist(I, A)) 

# Function to check whether a circle 
# encloses the given points 
def is_valid_circle(c, P): 
  
    # Iterating through all the points 
    # to check  whether the points 
    # lie inside the circle or not 
    for p in P: 
        if (not is_inside(c, p)): 
            return False
    return True
  
  
# Function to return the minimum enclosing 
# circle for N <= 3 
def min_circle_trivial(P): 
    assert(len(P) <= 3) 
    if not P : 
        return Circle()  
      
    elif (len(P) == 1) : 
        return Circle(P[0], 0)  
      
    elif (len(P) == 2) : 
        return circle_from1(P[0], P[1]) 
      
  
    # To check if MEC can be determined 
    # by 2 points only 
    for i in range(3): 
        for j in range(i + 1,3): 
  
            c = circle_from1(P[i], P[j]) 
            if (is_valid_circle(c, P)): 
                return c 
          
      
    return circle_from2(P[0], P[1], P[2]) 
  
  
# Returns the MEC using Welzl's algorithm 
# Takes a set of input points P and a set R 
# points on the circle boundary. 
# n represents the number of points in P 
# that are not yet processed. 
def welzl_helper(P, R, n): 
    # Base case when all points processed or |R| = 3 
    if (n == 0 or len(R) == 3) : 
        return min_circle_trivial(R) 
      
  
    # Pick a random point randomly 
    idx = randint(0,n-1) 
    p = P[idx] 
  
    # Put the picked point at the end of P 
    # since it's more efficient than 
    # deleting from the middle of the vector 
    P[idx], P[n - 1]=P[n-1],P[idx] 
  
    # Get the MEC circle d from the 
    # set of points P - :p 
    d = welzl_helper(P, R.copy(), n - 1) 
  
    # If d contains p, return d 
    if (is_inside(d, p)) : 
        return d 
      
  
    # Otherwise, must be on the boundary of the MEC 
    R.append(p) 
  
    # Return the MEC for P - :p and R U :p 
    return welzl_helper(P, R.copy(), n - 1) 
  
  
def welzl(P): 
    P_copy = P.copy() 
    shuffle(P_copy) 
    return welzl_helper(P_copy, [], len(P_copy)) 
  
  


'''

OUR ALGORITHM TO FIND MINIMUM ENCLOSING CIRCLE
 
'''
def our_algo(robot_locations):
    # Average Point ::
    n = len(robot_locations)
    xavg, yavg = 0, 0
    for point in robot_locations:
        xavg += point[0]
        yavg += point[1]

    xavg = xavg/n
    yavg = yavg/n

    locations =[]

    # Transformation wrt avg point:
    for point in robot_locations:
        locations.append([point[0]-xavg, point[1]-yavg])
    
    # Finding the furthest point from origin
    point1 = [0,0]
    for point in locations:
        if (point[0]**2+point[1]**2)>(point1[0]**2+point1[1]**2):
            point1 = point.copy()
    locations.remove(point1)

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

    locations.remove([x2,y2])

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

    ## Finding Final Concensus point ::::

    if not outside_points:
        x_con = xm
        y_con = ym
    else:
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

    # Transforming back to original System ::
    x_con = x_con + xavg
    y_con = y_con + yavg
    return x_con, y_con



# GENERATE SET OF ORDERED PAIRS 


# Driver code 
if __name__ == '__main__': 
    is_true = []
    for _ in range(100):
        ordered_pairs = [[random.randint(-100, 100), random.randint(-100, 100)] for _ in range(5)]
        OP = ordered_pairs.copy()
        x_con, y_con = our_algo(OP)
        mec2 = welzl([Point(ordered_pairs[0][0],ordered_pairs[0][1]) , 
                            Point(ordered_pairs[1][0],ordered_pairs[1][1]) , 
                            Point(ordered_pairs[2][0],ordered_pairs[2][1]) , 
                            Point(ordered_pairs[3][0],ordered_pairs[3][1]), 
                            Point(ordered_pairs[4][0],ordered_pairs[4][1])] ) 
        if round(mec2.C.X,3) != round(x_con,3) or round(mec2.C.Y,3) != round(y_con,3):
            print(f"Failed for {ordered_pairs}")
            print(f"our algo {[x_con,y_con]}")
            print(f"welzl algo {[mec2.C.X,mec2.C.Y]}")

