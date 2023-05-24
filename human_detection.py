import cv2
import mediapipe as mp
import time
import serial

arduino_connection = serial.Serial('/dev/ttyACM0', 9600)
class PoseDetector:
	def __init__(self, mode=False, upBody=False, 		smooth=True, detectionCon=0.5, trackCon=0.5):
	
		self.mode = mode
		self.upBody = upBody
		self.smooth = smooth
		self.enableSegmentation = False
		self.smoothSegmentation = True
		self.detectionCon = detectionCon
		self.trackCon = trackCon
		self.mpDraw = mp.solutions.drawing_utils
		self.mpPose = mp.solutions.pose
		self.pose = self.mpPose.Pose(self.mode, self.upBody, self.smooth, self.enableSegmentation, self.smoothSegmentation, self.detectionCon, self.trackCon)
		
	def findPose(self, img, draw=True):
		imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		self.results = self.pose.process(imgRGB)
		if self.results.pose_landmarks:
			if draw:
								self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
		return img
		
	def getPosition(self, img):
		lmList  = []
		if self.results.pose_landmarks:
			for id, lm in enumerate(self.results.pose_landmarks.landmark):
		 		h, w, c = img.shape
		 		cx, cy = int(lm.x * w), int(lm.y *h)
		 		lmList.append([id, cx, cy])
		return lmList
	 	
		 	
if __name__ == "__main__":

	detector = PoseDetector()
	
	
	camera = cv2.VideoCapture(0)
	camera.set(3,640)
	camera.set(4, 480)
	
	
	while True:
		
		ret, frame = camera.read()
		#print("camera connection", ret)
		
		frame = detector.findPose(frame)
		lmList = detector.getPosition(frame)
		
		
		height, width, channels = frame.shape
		#print(frame.shape)
		center_x = int(width/2)
		center_y = int(height/2)
		
		cv2.circle(frame, (center_x, center_y), 10, (0,0,255), -1)
		
		if lmList:
		
			person_center_x = lmList[0][1]
			person_center_y = lmList[0][2]
			
			cv2.circle(frame, (person_center_x, person_center_y), 10, (0,255,0), -1)
			
			error_x = person_center_x - center_x
			error_y = person_center_y - center_y
			
			print("Error X: ", error_x)
			print("Error Y: ", error_y)
			#print(str(error_x).encode())
			arduino_connection.write(("+"+str(error_x)).encode())	
			
								
			
		cv2.imshow('Frame', frame)
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
			
			
	camera.release()
	cv2.destroyAllWindows()
