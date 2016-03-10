#library needed on beaglebone
import cv2
import numpy as np
import time
#import matplotlib
import matplotlib.pyplot as plt

def get_360p(file):
    img=cv2.imread(file)
    img=cv2.resize(img,(320,240))
    gray_img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img,gray_img

def wh_det(img,x1=10,x2=630,y1=470,y2=0,xit=10,yit=-10,slope_en=0,er_max=4):    
    #create green pointers on sidewalk
    count=[]
    tot=0
    for x in range(x1,x2,xit):
        test=er_max#used to ignore noise in the image. if the values goes too low the system stops incrementing in that y direction
        for y in range(y1,y2,yit):
            bgr=img[y,x]
            if abs(int(bgr[1])-int(bgr[2]))<14 and ((int(bgr[0])*1.25)>(int(bgr[1])+int(bgr[2]))/2):#works
		print 'white'
                tot+=1
                if test<er_max:
                    test+=1
                #draw where true
                for xn in range(-2,3):
                    for yn in range(-2,3):
                        img[y+yn,x+xn]=[0,200,255]
            else:
                test-=1
                if test<1:
                    break
        count.append(tot)
        tot=0
    #calculate slope,
    slopes=[]
    c1=yit/xit
    print len(count)
    points=6#even multiple of len(count). number of points to evaluate in one loop
    if slope_en==1:
        for x1 in range(0,len(count),points):
            ydiff=[]
            for y1 in range(0,points-1):
                ydiff.append((count[x1+y1]-count[x1+y1+1])*c1)
            ysum=0
            #print y1+x1+1
            for y in ydiff:
                ysum=y+ysum
            slopes.append(float(ysum)/len(ydiff))    
    #print results
    print count

if __name__ == "__main__":
	img,gray=get_360p('debug_img9.jpg')
	#wh_det(img,x1=10,x2=310,y1=235,y2=100,xit=5, yit=5,er_max=5)

	for x in range(10,310,5):
		for y in range(235,200,-5):
			print '1'			
			bgr=img[y,x]
			
			if abs(int(bgr[1])-int(bgr[2]))<14 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
				print '1'
