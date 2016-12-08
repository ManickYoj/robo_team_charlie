import cv2
import numpy
import time
import rospy
from std_msgs.msg import Int8MultiArray
import math
from operator import itemgetter

PKG = 'numpy_tutorial'
import roslib; #roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg

practiceImg="./sampleImages/trafficCone.jpg"

def talker(val):
    pub = rospy.Publisher('camera', Int8MultiArray,queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        a = Int8MultiArray()
        a.data = val
        pub.publish(a)
        r.sleep()   

cap = cv2.VideoCapture(0) #opening webcam

def findCenter(cnts):
    centers=[] #just an array j chilling
    for c in cnts: #goin' through those contours lyke
        if(cv2.arcLength(c, 1)>30): #filtering out false negatives
        # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centers.append([cX,cY])
 
    # draw the contour and center of the shape on the image
            cv2.circle(output, (cX, cY), 7, (255, 255, 255), -1)
    return centers

def distanceCalc(pt1,pt2): #calculates horizontal distance of cones
    distance=abs(pt1[0]-pt2[0])
    return distance


def calcGap(centers): #finding the largest gap between two centers
    centerAr= sorted(centers, key=itemgetter(0)) #sorts in order of horizontal distance
    maxDistance=0
    for p in range(0, len(centerAr)-1):  #goes through each 2 consecutive points
        if(distanceCalc(centerAr[p], centerAr[p+1])>maxDistance):
            maxDistance= distanceCalc(centerAr[p], centerAr[p+1])
            point1=centerAr[p]
            point2=centerAr[p+1]
    return point1, point2

def headingCalc(pt1, pt2, img):
    #because an image is really just a matrix of r, c...
    numColumns=len(img[0])
    distbWsections=numColumns/11
    #because point1 is closer to the left no matter what, it'll be the lower one
    minSection=pt1[0]/distbWsections+1
    #to be conservative, compensate by subtracting to get the right section
    maxSection=(pt2[0]/distbWsections)
    val=[0,0,0,0,0,0,0,0,0,0,0]
    for i in range(0,10):
        if(i>=minSection and i<=maxSection):
            val[i]=1
    talker(val)
    return val


while not rospy.is_shutdown():
    # Capture frame-by-frame
    #ret, frame = cap.read() #gets the frame
    frame=cv2.imread(practiceImg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imshow('Colors', frame) #display filtered out thing
    orangeLower=numpy.array([5, 42, 160], dtype="uint8") #uint8 necessary for this kind of thing 
    orangeUpper=numpy.array([120, 120, 255], dtype= "uint8") #represents upper and lower bounds of the color "orange"

    mask=cv2.inRange(frame, orangeLower,orangeUpper) #creates mask of all the orange pixels
    output=cv2.bitwise_and(frame,frame,mask=mask) #maps mask onto image
    cv2.imshow('Output', output)
    #getting rid of false negatives and other outliers/smoothing over larger objects
    output = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)
    output = cv2.erode(output, None, iterations=2)
    output = cv2.dilate(output, None, iterations=2)

    #conversion to find contours
    blurred = cv2.GaussianBlur(output, (7, 7), 0)
    edged = cv2.Canny(blurred, 50, 150)

    # find contours in the edge map
    contours, hierarchy= cv2.findContours(edged,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #drawing said contours
    cv2.drawContours(output, contours, -1, (255,0,0),-1)
    centers=findCenter(contours)
    point1, point2=calcGap(centers)
    headingCalc(point1, point2, output)
    ###Showing you what happened
    #cv2.imshow('Output', output) #display filtered out thing
    cv2.waitKey(0)

    # Display the resulting framegit pu
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()