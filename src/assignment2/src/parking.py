#! /usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import rospy, math
import cv2, time, rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, 
          "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
motor_msg = xycar_motor()

global angle, prevDx, prevDy, prevYaw, prevSpeed, prevAngle, now
angle, prevDx, prevDy, prevYaw, prevSpeed, prevAngle, now = 0, 0, 0, 0, 0, 0, False
global count
refspeed = 20
refangle = 50
retry = 50
count = -50 * retry
retry2 = 100
atime = 0
debugcount = False
Docount2 = False
twocount = False


#=============================================
# 콜백함수 - ar_pose_marker 토픽을 처리하는 콜백함수
# ar_pose_marker 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 AR 정보를 꺼내 arData 변수에 옮겨 담음.
#=============================================
def callback(msg):
    global arData

    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

#=========================================
# ROS 노드를 생성하고 초기화 함.
# AR Tag 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
#=========================================
rospy.init_node('ar_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )

#=========================================
# 메인 루프 
# 끊임없이 루프를 돌면서 
# "AR정보 변환처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
# 작업을 반복적으로 수행함.
#=========================================



while not rospy.is_shutdown():
    global prevDx, prevDy, prevYaw, prevSpeed, prevAngle, count, refspeed, refangle, prevDistance
    global atime, debugcount, Docount2, twocount



    # 쿼터니언 형식의 데이터를 오일러 형식의 데이터로 변환
    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
	
    # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    # Row 100, Column 500 크기의 배열(이미지) 준비
    img = np.zeros((150, 500, 3))

    # 4개의 직선 그리기
    img = cv2.line(img,(25,125),(475,125),(0, 0, 255),2)
    img = cv2.line(img,(25,110),(25,135),(0,0,255),3)
    img = cv2.line(img,(250,110),(250,135),(0,0,255),3)
    img = cv2.line(img,(475,110),(475,135),(0,0,255),3)

    # DX 값을 그림에 표시하기 위한 좌표값 계산
    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25 : 
        point = 25	

    # DX값에 해당하는 위치에 동그라미 그리기 
    img = cv2.circle(img,(point,115),15,(0,255,0),-1)
  
    # DX값과 DY값을 이용해서 거리값 distance 구하기
    distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DY"],2))
    
    # 그림 위에 distance 관련된 정보를 그려넣기
    cv2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))

    # DX값 DY값 Yaw값 구하기
    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                +" Yaw:"+ str(round(yaw,1))
    prevstr = "PX:"+str(prevDx) + " PY:" + str(prevDy)
    arstr = "roll:" + str(roll) + " pitch:" + str(pitch)
    nowstr = "refspeed:" + str(refspeed) + " refangle: " + str(refangle)
    countstr = "count1:" + str(count) + " count2:" + str(atime)


    # 그림 위에 DX값 DY값 Yaw값 관련된 정보를 그려넣기
    cv2.putText(img,dx_dy_yaw, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
    cv2.putText(img,prevstr, (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))
    cv2.putText(img, nowstr, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))
    cv2.putText(img, countstr, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))

    # 만들어진 그림(이미지)을 모니터에 디스플레이 하기
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)

    #벽이 아닌 주차구역을 중심으로 계산하기 위해 Dx와 Dy값을 주차구역 정중앙을 기준으로 바꿔줍니다.
    # nowDx = arData["DX"]
    # nowDy = arData["DY"]


    #==============================#
    #핸들 조향각과 speed 설정하기
    if count < -8 * retry:      #1, 2, 3, 4 키 입력을 위해 잠시 멈췄다 시작함
        temspeed = 0
        temangle = 0
    elif count < -retry:        # 구석에서 시작하는 경우 초기 회전하다가 테두리 충돌하는 경우가 있기 떄문에
                                # 그것을 방지하기 위해서 벽에서 떨어뜨려줍니
        if arData["DX"] < 0:
            temangle = -50
        elif arData["DX"] > 0:
            temangle = 50
        else:
            temangle = 0
        temspeed = 50

    elif count < 0:       #첫번째 반복
        print(1)
        temspeed = refspeed * 2
        temangle = refangle
    elif count == 0:    #반복 기준점
        print(2)
        prevDx = int(arData["DX"])
        prevDy = int(arData["DY"])
    elif count < retry: #두번째 반복
        print(3)
        temspeed = -refspeed * 2.3
        temangle = 0
        pass
    elif count == retry:  #반복 기준점2
        pass
        print(4)
        Docount2 = False if Docount2 == True else True
        if Docount2 == True:
            temangle = refangle
            temspeed = refspeed
        else:
            temangle = 0
            temspeed = -refspeed
        refangle = 30
        if prevDx == int(arData["DX"]) and prevDy == int(arData["DY"]): # 빠져나가 다음으로 가기 위해
            print('in')
            refspeed = 0
            temspeed = refspeed
            temangle = 0
            count += 2 * retry + 1
            atime = -retry
        count -= 2 * retry
    elif retry < count < 2 * retry:
        retry2 == 2 * retry
        if atime < retry :   #다음 반복을 하기 전에
            '''
            멀어지는 경우 -> DX와 DY가 모두 변화 없음
            ->그 때 멈춰보기.
            '''
            print('here')
            refspeed = 50
            temspeed = refspeed
            temangle = 0
            atime += 1
            count -= 1
        else:
            if arData["DX"] > 0:
                refangle = 50
            else:
                refangle = -50
            count = -retry
            atime = 0


    #모터 토픽을 발생시키기 전(다음 반복을 하기 전)에 할 일
    count += 1


    try:        # temspeed가 정의되지 않은 경우
        temspeed != 1
    except:
        temspeed = refspeed
    try:        # temangle가 정의되지 않은 경우
        temangle == 1
    except:
        temangle = refangle

    speed = temspeed
    angle = temangle


    # 조향각값과 속도값을 넣어 모터 토픽을 발행하기
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor_pub.publish(motor_msg)


# while 루프가 끝나면 열린 윈도우 모두 닫고 깔끔하게 종료하기
cv2.destroyAllWindows()





