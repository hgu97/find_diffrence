import cv2 as cv
import numpy as np
import deburr
from robodk import *
from robolink import *
from pypylon import pylon

#%% img read()
# og_color = cv.imread('D:/Python_code/OG.jpg')    #('D:/Python_code/135m_OG6.jpg')             
# burr_color = cv.imread('D:/Python_code/BURR_Rot&Trans.jpg')#('D:/Python_code/135m_BURR6.jpg')

# og_color = cv.imread('D:/Python_code/test_o.bmp')    #('D:/Python_code/135m_OG6.jpg')     
# burr_color = cv.imread('D:/Python_code/test_b.bmp')    #('D:/Python_code/135m_OG6.jpg')     

og_color = cv.imread('C:/Users/GL-NT/opencv/test_o.bmp') 
burr_color = cv.imread('C:/Users/GL-NT/opencv/test.bmp') 


og = cv.cvtColor(og_color, cv.COLOR_BGR2GRAY)
burr = cv.cvtColor(burr_color, cv.COLOR_BGR2GRAY)


#%% 이미지 trim
og_upper, og_lower, og_left, og_right = deburr.imtrim(og,n=1,offset=100)

og_trim = og_color[og_upper:og_lower, og_left:og_right]
og_trim_g = og[og_upper:og_lower, og_left:og_right]
trim_og = og_trim.copy()
test_og= og_trim.copy()
       
burr_left, burr_upper, burr_right, burr_lower = deburr.TempMatch(og_trim_g,burr)

burr_trim = burr_color[burr_upper:burr_lower,burr_left:burr_right]
burr_trim_g = burr[burr_upper:burr_lower,burr_left:burr_right]
trim_burr = burr_trim.copy()


#%% Alinge을 위한 match함수  """ Feature match """
matches,matchesMask, kp1, kp2,src,dst = deburr.F_match(og_trim_g,burr_trim_g)
res2 = cv.drawMatches(og_trim, kp1, burr_trim, kp2, matches, None, \
                    matchesMask = matchesMask,
                    flags=cv.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)


#%% Alinge을 위한 knn match함수
good_matches, knn_matchesMask, kp1, kp2, knn_src, knn_dst = deburr.F_knnmatch(og_trim_g,burr_trim_g)
res4 = cv.drawMatches(og_trim, kp1, burr_trim, kp2, good_matches, None, \
                    matchesMask = knn_matchesMask,
                    flags=cv.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
  
    
#%%   Align
"""###############  (knn, match 수정)  ######################"""
M_theta1 = deburr.Align(src,dst,matchesMask)
# cv.imshow('Matching-Homo', res2)

# M_theta1 = deburr.Align(knn_src,knn_dst,knn_matchesMask)
# cv.imshow('Matching-knn_match-Homo', res4)
rows,cols = burr_trim.shape[0:2]
img_rot = cv.warpAffine(burr_trim, M_theta1,(cols, rows), \
                        flags=cv.INTER_CUBIC, borderMode=cv.BORDER_CONSTANT)
  
    
#%%  Contour drawing
img_rot_g = cv.cvtColor(img_rot, cv.COLOR_BGR2GRAY)

rot_largestcnt,_ = deburr.contour(img_rot_g)
og_largestcnt,_ = deburr.contour(og_trim_g)
        
test_bg = np.zeros(og_trim.shape,dtype=np.uint8) 
o = test_bg.copy()
b = test_bg.copy()

# cv.drawContours(og_trim, og_largestcnt, -1, (255,0,0), 1)    
# cv.drawContours(og_trim, rot_largestcnt, -1, (0,0,255), 1)  
    
cv.drawContours(test_bg, og_largestcnt, -1, (255,0,0), 1)    
cv.drawContours(test_bg, rot_largestcnt, -1, (0,255,255), 1)  
cv.drawContours(o, og_largestcnt, -1, (255,0,0), 1)    
cv.drawContours(b, rot_largestcnt, -1, (0,255,255), 1)  
   
cv.drawContours(trim_og, og_largestcnt, -1, (0,255,0), 1)

#%% burr size
tool_feed = deburr.burr_size(og_largestcnt,rot_largestcnt, test_bg)

# Start the RoboDK API:
RDK = Robolink()

# Get the robot item by name:
robot = RDK.Item('Hyundai HH020-3', ITEM_TYPE_ROBOT)

# Get the reference target by name:
target = RDK.Item('Target 1')
Home = RDK.Item('Target 2')
target_pose = target.Pose()
xyz_ref = target_pose.Pos()

robot.setSpeed(30,5,-1,-1)
robot.MoveL(Home)

# Move the robot to the reference point:
robot.MoveL(target)

# Draw a hexagon around the reference target:
for i in range(len(og_largestcnt)):
    if tool_feed[i]>=5:
        robot.setSpeed(30,-1,-1,-1)
    elif tool_feed[i]>=4:
        robot.setSpeed(30,-1,-1,-1)
    elif tool_feed[i]>=3:
        robot.setSpeed(30,-1,-1,-1)
    elif tool_feed[i]>=2:
        robot.setSpeed(60,-1,-1,-1)
    elif tool_feed[i]>=1:
        robot.setSpeed(60,-1,-1,-1)
        
    # Calculate the new position around the reference:
    x = (float(og_largestcnt[i,:,1])-334)/5.34+870.5 # new X coordinate
    y = (float(og_largestcnt[i,:,0])-291)/5.3+11.9  # new Y coordinate
    z = 500      # new Z coordinate
    
    target_pose.setPos([x,y,z])

    print(x, y, z)

    # Move to the new target:
    robot.MoveL(target_pose)

# Trigger a program call at the end of the movement

# Move back to the reference target:
robot.setSpeed(30,5,-1,-1)
robot.MoveL(target)
robot.MoveL(Home)