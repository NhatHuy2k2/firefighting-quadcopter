
import rospy
import cv2
from ultralytics import YOLO
import math
import cvzone
from std_msgs.msg import String
from std_msgs.msg import Int32


rospy.init_node("fire_detection_node")
pub_x = rospy.Publisher('fire_detection_x', Int32, queue_size=10)
pub_y = rospy.Publisher('fire_detection_y', Int32, queue_size=10)
rate = rospy.Rate(20)   #10Hz


""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""
count = 0
model = YOLO('/home/jetson/Downloads/fire.pt')
classnames = ['fire']

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=720,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# Tạo đối tượng VideoWriter để ghi video
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Định dạng video (trong ví dụ này sử dụng MP4V)

# Tạo tên file video và đường dẫn tuyệt đối
output_path = '/home/jetson/Downloads/output.mp4'

out = cv2.VideoWriter(output_path, fourcc, 24.0, (720, 480))  # Tên file video, định dạng, FPS và kích thước khung hình

def show_camera():
    window_title = "Recognition Camera"

   
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()
                out.write(frame)
                frame2 = cv2.resize(frame,(720,480))
                result = model(frame2,stream = 1)
                x0=0
                y0=0  
                for info in result:
                     boxes = info.boxes

                     for box in boxes:
                        confidence = box.conf[0]
                        confidence = math.ceil(confidence * 100)
                        Class = int(box.cls[0])
                        if confidence > 60:
                            x1,y1,x2,y2 = box.xyxy[0]
                            x1,y1,x2,y2 = int(x1),int(y1),int(x2),int(y2)
                            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),5)
                            x0 = (x1+x2)//2
                            y0 = (y1+y2)//2
                            cvzone.putTextRect(frame, f'{classnames[Class]} {confidence}%', [x1 + 8, y1 + 100], scale=1.5, thickness=2)
                     pub_x.publish(x0)
                     pub_y.publish(y0)
                     print("x0="+ str(x0), end="\n\n")
                     print("y0="+ str(y0), end="\n\n") 
                     x0=0
                     y0=0
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, frame)
                else:
                    break 
                keyCode = cv2.waitKey(10) & 0xFF

                if keyCode == 27 or keyCode == ord('q'):
                    break
                rate.sleep()
        finally:
            video_capture.release()
            out.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    show_camera()
