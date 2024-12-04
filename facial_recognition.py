#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import face_recognition
import os


def load_known_faces_and_names():
    """Load known faces and their names from image files in the current directory."""
    known_face_encodings = []
    known_face_names = []
    current_dir = os.path.dirname(os.path.abspath(__file__))

    for file in os.listdir(current_dir):
        if file.lower().endswith((".jpg", ".jpeg")):
            image_path = os.path.join(current_dir, file)
            rospy.loginfo(f"Loading {file} for face recognition...")
            image = face_recognition.load_image_file(image_path)
            face_encodings = face_recognition.face_encodings(image)

            if face_encodings:
                known_face_encodings.append(face_encodings[0])
                name = os.path.splitext(file)[0]
                known_face_names.append(name)
            else:
                rospy.logwarn(f"No faces found in {file}. Skipping.")

    return known_face_encodings, known_face_names


def recognize_faces_in_frame(frame, known_face_encodings, known_face_names, pub):
    """Detect and recognize faces in a given frame."""
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Detect face locations and their encodings
    face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        # Compare faces
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        name = "Unknown"

        # Assign name if a match is found
        if True in matches:
            first_match_index = matches.index(True)
            name = known_face_names[first_match_index]

        # Publish the recognized name
        pub.publish(name)

        # Draw a rectangle around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

        # Label the face
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 0.5, (255, 255, 255), 1)

    return frame


def process_camera_feed(known_face_encodings, known_face_names):
    """Capture frames from the webcam and process for face recognition."""
    recognized_name_pub = rospy.Publisher('recognized_faces', String, queue_size=10)
    cap = cv2.VideoCapture(0)  # Open the default webcam

    if not cap.isOpened():
        rospy.logerr("Unable to access the webcam.")
        return

    rospy.loginfo("Processing video feed from webcam...")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame. Retrying...")
            continue

        # Perform face recognition
        frame = recognize_faces_in_frame(frame, known_face_encodings, known_face_names, recognized_name_pub)

        # Display the frame
        cv2.imshow("Face Recognition - Webcam", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    rospy.init_node('face_recognition_with_camera_node', anonymous=True)
    known_face_encodings, known_face_names = load_known_faces_and_names()

    if not known_face_encodings:
        rospy.logerr("No face images found in the current directory.")
        return

    # Choose to process live camera feed
    process_camera_feed(known_face_encodings, known_face_names)


if __name__ == '__main__':
    main()
