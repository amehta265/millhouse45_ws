import cv2
from .detector import detect_object

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("unable to open camera")
        exit()

    # Camera is open at this point
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("unable to grab frame")
                break
            coords, radius, mask = detect_object(frame)

            overlay = frame.copy()
            if coords is not None:
                # Draw bounding box
                cx, cy = int(round(coords[0])), int(round(coords[1]))
                color = (0, 255, 0)
                cv2.circle(overlay, (cx, cy), int(round(radius)), color, 2)
                cv2.circle(overlay, (cx, cy), 4, color, -1)

                label = (f"cx: {cx}, cy: {cy}")
            else:
                label = "no ball"

            cv2.putText(overlay, f"{label}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("frame", overlay)
            cv2.imshow("mask", mask)

            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                 break

    finally:
        cap.release()
        cv2.destroyAllWindows()

    return None


main()