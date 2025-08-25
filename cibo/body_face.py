#https://yoppa.org/mit-design4-22/14113.html
#https://note.com/npaka/n/nd0c1b5d6d31c
#https://blog.futurestandard.jp/entry/2017/03/02/143449#%EF%BC%95%EF%BC%92-%E5%8F%97%E4%BF%A1%E5%81%B4Python%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%A0
#from zmq import Context, REQ
import zmq
import numpy as np
import cv2
import mediapipe as mp
import struct


def main():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PULL)
    sock.connect("tcp://localhost:5554")
    
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind("tcp://*:5555")

    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose
    mp_face_mesh = mp.solutions.face_mesh
    mp_drawing_styles = mp.solutions.drawing_styles

    with mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as face_mesh:
            
            with mp_pose.Pose(
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as pose:
                 
                while True:                   
                    # Receve Data from C++ Program
                    byte_rows, byte_cols, byte_mat_type, data=  sock.recv_multipart()

                    # Convert byte to integer
                    rows = struct.unpack('i', byte_rows)
                    cols = struct.unpack('i', byte_cols)
                    mat_type = struct.unpack('i', byte_mat_type)

                    # BGR Color
                    image = np.frombuffer(data, dtype=np.uint8).reshape((rows[0],cols[0],3))

        
                    image.flags.writeable = False
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

                    # 画像をMediaPipeに渡して検出を行う
                    results = pose.process(image) 
                    resultsFace = face_mesh.process(image)

                    image.flags.writeable = True
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                    if resultsFace.multi_face_landmarks:
                        for face_landmarks in resultsFace.multi_face_landmarks:
                            mp_drawing.draw_landmarks(
                                image=image,
                                landmark_list=face_landmarks,
                                connections=mp_face_mesh.FACEMESH_CONTOURS,
                                landmark_drawing_spec=None,
                                connection_drawing_spec=mp_drawing_styles
                                .get_default_face_mesh_contours_style())
                            
                            mp_drawing.draw_landmarks(
                                image=image,
                                landmark_list=face_landmarks,
                                connections=mp_face_mesh.FACEMESH_IRISES,
                                landmark_drawing_spec=None,
                                connection_drawing_spec=mp_drawing_styles
                                .get_default_face_mesh_iris_connections_style())
                            
                    mp_drawing.draw_landmarks(
                        image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                    
                    cv2.imshow("test", image)
                    
                    if cv2.waitKey(5) & 0xFF == 27:
                            break

                    
        ############server###########                    
                    landmarks = []
                    #i=0
                    if results.pose_landmarks:
                        for landmark in results.pose_landmarks.landmark:
                            landmarks.append(int(landmark.x * 640))
                            landmarks.append(int(landmark.y * 480))
                            #print(i)
                        print(len(landmarks)) 
                    else:
                        print("noData")
                        landmarks = [0, 0]

                    if resultsFace.multi_face_landmarks:      
                        for face_landmarks in resultsFace.multi_face_landmarks:
                            for landmark in face_landmarks.landmark:
                                landmarks.append(int(landmark.x * 640))
                                landmarks.append(int(landmark.y * 480))
                        print(len(landmarks)) 
                    else:
                        print("face_noData")
                        landmarks.append(int(0))
                        landmarks.append(int(0))

                    sendlm = struct.pack(f'{len(landmarks)}i', *landmarks)
                    
                    socket.send(sendlm)

        
            
####################

if __name__ == '__main__':
    main() 
    