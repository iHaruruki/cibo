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
         # Open ZMQ Connection
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PULL)
    sock.connect("tcp://localhost:5557")
    
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind("tcp://*:5558")
    
    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands
   
    with mp_hands.Hands(
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
            
        while True:           
            # Receve Data from C++ Program
            byte_rows, byte_cols, byte_mat_type, data= sock.recv_multipart()

            # Convert byte to integer
            rows = struct.unpack('i', byte_rows)
            cols = struct.unpack('i', byte_cols)
            mat_type = struct.unpack('i', byte_mat_type)

            # BGR Color
            image = np.frombuffer(data, dtype=np.uint8).reshape((rows[0],cols[0],3))


            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # 画像をMediaPipeに渡して検出を行う
            results = hands.process(image) 
            
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image, hand_landmarks, mp_hands.HAND_CONNECTIONS)    
            
            #cv2.imshow("hand", image)
            
            if cv2.waitKey(5) & 0xFF == 27:
                    break

            
############server########### 
            landmarks = []
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    for landmark in hand_landmarks.landmark:
                        landmarks.append(int(landmark.x * 640))
                        landmarks.append(int(landmark.y * 480))
                print(len(landmarks)) 
            else:
                print("noData")
                landmarks = [0, 0]

            sendlm = struct.pack(f'{len(landmarks)}i', *landmarks)
            
            socket.send(sendlm)

        
            
####################

if __name__ == '__main__':
    main() 
    