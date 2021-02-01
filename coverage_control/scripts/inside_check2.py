# A Python3 program to check if a given point 
# lies inside a given polygon 
# Refer https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ 
# for explanation of functions onSegment(), 
# orientation() and doIntersect() 

# Define Infinite (Using INT_MAX 
# caused overflow problems)
INT_MAX = 10000

# Given three colinear points p, q, r, 
# the function checks if point q lies 
# on line segment 'pr' 
def onSegment(p, q, r):
    
    if ((q[0] <= max(p[0], r[0])) &
        (q[0] >= min(p[0], r[0])) &
        (q[1] <= max(p[1], r[1])) &
        (q[1] >= min(p[1], r[1]))):
        return True
        
    return False

# To find orientation of ordered triplet (p, q, r). 
# The function returns following values 
# 0 --> p, q and r are colinear 
# 1 --> Clockwise 
# 2 --> Counterclockwise 
def orientation(p, q, r):
    
    val = (((q[1] - p[1]) *
            (r[0] - q[0])) -
        ((q[0] - p[0]) *
            (r[1] - q[1])))
            
    if val == 0:
        return 0
    if val > 0:
        return 1 # Collinear
    else:
        return 2 # Clock or counterclock

def doIntersect(p1, q1, p2, q2):
    
    # Find the four orientations needed for 
    # general and special cases 
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if (o1 != o2) and (o3 != o4):
        return True
    
    # Special Cases 
    # p1, q1 and p2 are colinear and 
    # p2 lies on segment p1q1 
    if (o1 == 0) and (onSegment(p1, p2, q1)):
        return True

    # p1, q1 and p2 are colinear and 
    # q2 lies on segment p1q1 
    if (o2 == 0) and (onSegment(p1, q2, q1)):
        return True

    # p2, q2 and p1 are colinear and 
    # p1 lies on segment p2q2 
    if (o3 == 0) and (onSegment(p2, p1, q2)):
        return True

    # p2, q2 and q1 are colinear and 
    # q1 lies on segment p2q2 
    if (o4 == 0) and (onSegment(p2, q1, q2)):
        return True

    return False

# Returns true if the point p lies 
# inside the polygon[] with n vertices 
def is_inside_polygon(points, p):
    
    n = len(points)
    
    # There must be at least 3 vertices
    # in polygon
    if n < 3:
        return False

    xs, ys = zip(*points)
    xmax, xmin = max(xs), min(xs)
    ymax, ymin = max(ys), min(ys)

    if p[0] == 0:
        return ymin <= p[1] <= ymax

    if p[1] == 0:
        return xmin <= p[0] <= xmax
        
    # Create a point for line segment
    # from p to infinite
    extreme = (INT_MAX, p[1])
    count = i = 0
    
    while True:
        next = (i + 1) % n
        
        # Check if the line segment from 'p' to 
        # 'extreme' intersects with the line 
        # segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(points[i],
                        points[next], 
                        p, extreme)):
                            
            # If the point 'p' is colinear with line 
            # segment 'i-next', then check if it lies 
            # on segment. If it lies, return true, otherwise false 
            if orientation(points[i], p, 
                        points[next]) == 0:
                return onSegment(points[i], p, 
                                points[next])
                                
            count += 1
            
        i = next
        
        if (i == 0):
            break
        
    # Return true if count is odd, false otherwise 
    print("Query - Polygon: {}, Point: {}, intersections: {}".format(points, p, count))
    return (count % 2 == 1)

# Driver code
if __name__ == '__main__':
    
    # polygon1 = [ (0, 0), (10, 0), (10, 10), (0, 10) ]
    
    # p = (20, 20)
    # if (is_inside_polygon(points = polygon1, p = p)):
    #     print ('Yes')
    # else:
    #     print ('No')
    
    # p = (5, 5)
    # if (is_inside_polygon(points = polygon1, p = p)):
    #     print ('Yes')
    # else:
    #     print ('No')

    # polygon2 = [ (0, 0), (5, 0), (5, 5), (3, 3) ]
    
    # p = (3, 3)
    # if (is_inside_polygon(points = polygon2, p = p)):
    #     print ('Yes')
    # else:
    #     print ('No')
    
    # p = (5, 1)
    # if (is_inside_polygon(points = polygon2, p = p)):
    #     print ('Yes')
    # else:
    #     print ('No')
    
    # p = (8, 1)
    # if (is_inside_polygon(points = polygon2, p = p)):
    #     print ('Yes')
    # else:
    #     print ('No')
    
    # polygon3 = [ (0, 0), (10, 0), (10, 10), (0, 10) ] 
    
    # p = (-1, 10)
    # if (is_inside_polygon(points = polygon3, p = p)):
    #     print ('Yes')
    # else:
    #     print ('No')

    polygon4 = [ (0, 40), (-40, 0), (0, -40), (40, 0) ] 

    p = (-14, -30)
    if (is_inside_polygon(points = polygon4, p = p)):
        print ('Yes')
    else:
        print ('No')
    
# This code is contributed by Vikas Chitturi
