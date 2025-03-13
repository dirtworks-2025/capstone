import cv2

cap = cv2.VideoCapture(0)  # Try 1 if 0 doesn't work

if not cap.isOpened():
    print("Error: Couldn't access the webcam.")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break
        
        cv2.imshow("Webcam Test", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
