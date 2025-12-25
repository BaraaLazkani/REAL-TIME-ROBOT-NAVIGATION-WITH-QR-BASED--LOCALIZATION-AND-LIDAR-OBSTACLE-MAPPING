from picamera2 import Picamera2
import cv2
import os
from datetime import datetime

picam2 = Picamera2()
picam2.start()

photo_counter = 1  # To keep track of photos and avoid overwriting

while True:
    frame = picam2.capture_array()
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    cv2.imshow("Camera", frame_bgr)
    
    key = cv2.waitKey(1)
    
    if key == ord('q'):
        break
    elif key == ord('h'):
        # Generate timestamp for filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"photo_{timestamp}.jpg"
        
        # Save the photo
        cv2.imwrite(filename, frame_bgr)
        print(f"Photo saved as {filename}")
        
        # Alternative naming with counter:
        # filename = f"photo_{photo_counter}.jpg"
        # cv2.imwrite(filename, frame_bgr)
        # print(f"Photo saved as {filename}")
        # photo_counter += 1

picam2.stop()
cv2.destroyAllWindows()