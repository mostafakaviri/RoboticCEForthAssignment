#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results
from turtlebot3_object_tracker.srv import GetError, GetErrorRequest, GetErrorResponse

# ROS
import rospy
from sensor_msgs.msg import Image


class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        # TODO: Subscribe on your robot's camera topic
        # self.cmd_publisher = rospy.Publisher('/follower/camera/image' , Image , queue_size=10)
        self.camera_subscriber = rospy.Subscriber("/follower/camera/image" , Image , callback=self.camera_listener)
        # NOTE: Make sure you use the provided listener for this subscription

        self.error_y = 0
        self.error_x = 0
        # self.camera_subscriber = None

        # TODO: Instantiate your YOLO object detector/classifier model

        self.model = YOLO("yolov8m.pt")

        # TODO: You need to update results each time you call your model

        # self.results = Results()

        self.cv2_frame_size = 400, 320
        
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # TODO: Setup your "human detection" service
        self.human_detection_server = rospy.Service('/mammad', GetError, self.send_error)

        self.update_view()


    def camera_listener(self, msg: Image):

        self.image_msg.data = copy.deepcopy(msg.data)

    def send_error(self, req : GetErrorRequest):

        res = GetErrorResponse()

        res.error_x = self.error_x
        res.error_y = self.error_y

        return res
        # self.update_view()


    def update_view(self):
        try:

            
            while not rospy.is_shutdown():

                if len(self.image_msg.data) == 0: # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                frame = copy.deepcopy(self.image_np)

                # print("annnnnnnnnnnnnnnnnnnnnn")
                # rospy.loginfo("annnnnnnnnnnnnn")

                self.results = self.model.predict(frame, verbose=False)

                rospy.logerr(f'---------{[res.boxes.xyxy for res in self.results]}---')

                self.error_y = 120 - self.results[0].boxes.data[0][3]
                self.error_x = 160 - (self.results[0].boxes.data[0][0] + self.results[0].boxes.data[0][2])/2

                # rospy.logerr(self.results[0].probs)
                # TODO: You can use an "Annotator" to draw object bounding boxes on frame
                ann = Annotator(frame)

                data = self.results[0].boxes.data

                persons = [self.results[0].boxes.xyxy[i] for i in range(len(data)) if data[i][-1] == 0.]

                for person in persons:
                    ann.box_label(person, label='Person', color=(200, 20, 20), txt_color=(255, 255, 255))

                cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass


if __name__ == "__main__":

    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    # image_processor.send_error()

    

    # rospy.spin()


