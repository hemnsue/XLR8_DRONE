import cv2 as cv
import numpy as np
nc=(7,7)
framesize=(1920,1080)
criteria=(cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,30,0.001)
objp=np.zeros((nc[0]*nc[1],3),np.float32)
objp[:,:2]=np.mgrid[0:nc[0],0:nc[1]].T.reshape(-1,2)
objPoints=[]
imgPoints=[]
paths=["7.jpg","8.jpg","9.jpg","12.jpg","16.jpg","18.jpg","19.jpg","21.jpg","24.jpg","27.jpg","29.jpg","33.jpg","37.jpg","40.jpg","41.jpg","65.jpg","66.jpg","67.jpg","69.jpg","71.jpg","104.jpg","106.jpg","107.jpg","109.jpg","110.jpg","111.jpg","118.jpg","120.jpg","123.jpg","125.jpg","127.jpg","171.jpg","172.jpg","173.jpg","175.jpg","1012.jpg","1013.jpg","1022.jpg","1023.jpg","1025.jpg","1026jpg","1027.jpg","1031.jpg","1034.jpg","1035.jpg","1036.jpg","1038.jpg","1039.jpg","1040.jpg","1044.jpg","1045.jpg","1046.jpg","1073.jpg","1074.jpg","1075.jpg","1077.jpg","1080.jpg","1081.jpg"]
for image in paths:
    row=cv.imread(image)
    print(row)
    gray=cv.cvtColor(row,cv.COLOR_BGR2GRAY)
    ret,corners=cv.findChessboardCorners(gray,nc,None)
    if ret==True:
        objPoints.append(objp)
        corners2=cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgPoints.append(corners)
        cv.drawChessboardCorners(row,nc,corners2,ret)
        #plt.imshow(row);
        #plt.show();
ret,cameraMatrix,dist,rvecs,tvecs=cv.calibrateCamera(objPoints,imgPoints,gray.shape[::-1],None,None)
print("Camera Calibrated: ",ret)
print("\nCamera Matrix \n",cameraMatrix)
print("\nDistortion Parameters :\n",dist)
print("\nRotation Vectors:\n",rvecs)
print("\nTranslation Vectors:\n",tvecs)
img=cv.imread("10002.jpg")
h,w=img.shape[:2]
newCameraMatrix,roi=cv.getOptimalNewCameraMatrix(cameraMatrix,dist,(w,h),1,(w,h))
dst=cv.undistort(img,cameraMatrix,dist,None,newCameraMatrix)
x,y,w,h=roi
dst=dst[y:y+h,x:x+w]
cv.imwrite("100021.jpg",dst)
