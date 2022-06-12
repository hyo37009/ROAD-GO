#! /usr/bin/env python
# -*- coding: utf-8 -*-

# =============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
# =============================================
import rospy, math
import cv2, time, rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor


# =============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
# =============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)


# =============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
# =============================================
arData = {"DX": 0.0, "DY": 0.0, "DZ": 0.0,
          "AX": 0.0, "AY": 0.0, "AZ": 0.0, "AW": 0.0}
roll, pitch, yaw = 0, 0, 0
motor_msg = xycar_motor()

global angle, prevDx, prevDy, prevYaw, prevSpeed, prevAngle, now, where, reftime, firstDy
angle, prevDx, prevDy, prevYaw, prevSpeed, prevAngle, now = 0, 0, 0, 0, 0, 0, None
global count, firstDx, finish
refspeed = 25
refangle = 50
retry = 50
count = -50 * retry
retry2 = 100
atime = 0
debugcount = False
Docount2 = False
twocount = False
where = None
reftime = 0
firstDx = 0
firstDy = 0
finish = False

# =============================================
# 콜백함수 - ar_pose_marker 토픽을 처리하는 콜백함수
# ar_pose_marker 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 AR 정보를 꺼내 arData 변수에 옮겨 담음.
# =============================================
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


# =========================================
# ROS 노드를 생성하고 초기화 함.
# AR Tag 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
# =========================================
rospy.init_node('ar_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

# =========================================
# 메인 루프
# 끊임없이 루프를 돌면서
# "AR정보 변환처리 +차선위치찾기 +조향각결정 +모터토픽발행"
# 작업을 반복적으로 수행함.
# =========================================


while not rospy.is_shutdown():
    global prevDx, prevDy, prevYaw, prevSpeed, prevAngle, count, refspeed, refangle, prevDistance
    global atime, debugcount, Docount2, twocount, where, reftime, firstDx, firstDy

    # 쿼터니언 형식의 데이터를 오일러 형식의 데이터로 변환
    (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))

    # 라디안 형식의 데이터를 호도법(각) 형식의 데이터로 변환
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    # Row 100, Column 500 크기의 배열(이미지) 준비
    img = np.zeros((100, 500, 3))

    # 4개의 직선 그리기
    img = cv2.line(img, (25, 65), (475, 65), (0, 0, 255), 2)
    img = cv2.line(img, (25, 40), (25, 90), (0, 0, 255), 3)
    img = cv2.line(img, (250, 40), (250, 90), (0, 0, 255), 3)
    img = cv2.line(img, (475, 40), (475, 90), (0, 0, 255), 3)

    # DX 값을 그림에 표시하기 위한 좌표값 계산
    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25:
        point = 25

    # DX값에 해당하는 위치에 동그라미 그리기
    img = cv2.circle(img, (point, 65), 15, (0, 255, 0), -1)

    # DX값과 DY값을 이용해서 거리값 distance 구하기
    distance = math.sqrt(pow(arData["DX"], 2) + pow(arData["DY"], 2))

    # 그림 위에 distance 관련된 정보를 그려넣기
    cv2.putText(img, str(int(distance)) + " pixel", (350, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

    # DX값 DY값 Yaw값 구하기
    dx_dy_yaw = "DX:" + str(int(arData["DX"])) + " DY:" + str(int(arData["DY"])) \
                + " Yaw:" + str(round(yaw, 1))

    # 그림 위에 DX값 DY값 Yaw값 관련된 정보를 그려넣기
    cv2.putText(img, dx_dy_yaw, (20, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))

    # 만들어진 그림(이미지)을 모니터에 디스플레이 하기
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)


    # ==============================#
    '''
    주차의 페이즈는 총 4가지로 이루어집니다.
    1) 주차구역에 수직 방향으로 제자리 회전
    2) 주차구역의 중앙까지 직진하여 이동
    3) 주차구역에 수평 방향으로 제자리 회전
    4) 주차구역까지 직진
    이후 지정된 주차구역에 들어가면 차가 멈추면서 주차가 완료됩니다.
    
    단, 시뮬레이터의 시작점(숫자 1, 2, 3, 4를 누르면 차량이 이동하는 위치)에서
    1)을 수행하기 위해 제자리 회전을 한다면
    수직으로 회전하기 전에 벽에 부딪혀 차가 멈춰버립니다.
    
    그런 현상을 막기 위해 일련의 주차과정을 수행하기 이전,
    차를 벽에서 띄어놓기 위해 약간의 우회전을 합니다.
    특히, 이 우회전을 한 뒤 차량이 주차구역에 수평으로 정렬됩니다.
    이동하는 값은 시뮬레이터를 여러 번 실행하며 실험적으로 얻은 상숫값입니다.
    
    주차구역에 수직 방향으로 회전한 뒤, 현재 위치에 따라 전진 또는 후진합니다.
    이때, 전진/후진하는 값은 DX 값에 비례합니다.
    (차가 수직으로 놓여있기 때문에 DX 값(단위:픽셀)과 주차구역까지의 정렬하기 위한 거리가 같습니다)
    
    
    마지막으로, 차를 주차구역의 수평 방향으로 제자리 회전시킵니다.
    2)에서 차량의 위치를 적절하게 이동했기 때문에, DX가 0이 된다면 차와 주차구역이 수평하게 됩니다.
    
    이후 DX < 72까지 차를 이동하면 주차가 완료됩니다.
    
    실제로 코드를 실행하면, 차가 주차구역에 대해서 수직/수평이 아닌 다소의 오차값을 가지게 됩니다.
    이는 코드 작업을 진행한 노트북의 사양이 낮아 매번 실행할 때마다 차량이 이동하는 정도가 달라져 정확한 수치를 구하기 어려웠기 때문입니다.
    다만 오차범위 내에서도 차가 잘 주차됩니다.
    '''

    '''
    기본 구조:
        일정한 시간동안 특정한 행동을 반복합니다.
        코드가 한번 반복될 때마다(로스 토픽이 한번 발생될 때마다) count값이 1씩 올라가고
        보통 retry 변수값을 1Hz로 하여 특정한 행동을 반복합니다.
    
    변수 설명:
        atime, count:
            모터 토픽이 한번 발생할 때마다 1씩 증가하여
            실행 시간을 기록하는 역할을 합니다.
        retry : 
            count, 반복주기와 관련된 변수입니다.
            -retry < count < 0일 동안 반복의 첫번째 행동을,
            0 < count < retry일 동안 반복의 두번째 행동을 합니다.
            총 2retry가 1Hz가 되어 차량이 일정한 행동을 수행합니다.
        reftime:
            atime, 반복 시간과 관련된 변수입니다.
            atime이 reftime이 될 때 까지 특정한 행동을 수행합니다.
            
        
        refspeed:
            레퍼런스 스피드, 기준이 되는 속도입니다.
            50으로 설정되어있습니다.
        refangle:
            레퍼런스 앵글, 기준이 되는 조향각입니다.
            50으로 설정되어있습니다.
        temspeed:
            모터 토픽을 발생시킬 속도값입니다.
        temangle:
            모터 토픽을 발생시킬 조향각입니다.
    '''


    # ---------------------------------------------------- #
    '''
    우선, 차가 출발하기 전 1~4의 키 값을 받을 시간이 필요하니, 
    6retry의 시간이 지난 후 차가 출발합니다.
    '''
    if count < -6 * retry:
        temspeed = 0
        temangle = 0
    '''
    제자리에서 회전을 하기 전, 차를 우회전 전진 시킵니다.
    이 코드를 수행한 뒤 차는 주차구역과 수평하게 됩니다.
    '''
    if -9 * retry < count < -retry:
        temspeed = refspeed
        if arData["DX"] > 0:
            temangle = refangle
        else:
            temangle = -refangle
    elif count == -retry:
            now = 1
            count = -retry
            atime = 0
    '''
    어느 페이즈를 수행해야하는지 now값을 검사합니다.
    
    now값은 아래와 같습니다.
    1) 주차구역에 수직 방향으로 제자리 회전
    2) 주차구역의 중앙까지 직진하여 이동
    3) 주차구역에 수평 방향으로 제자리 회전, 
        + 주차구역까지 직진
    
    각 페이즈가 끝나면 now += 1이 되어 다음 페이즈로 넘어갑니다.
    
    
    '''

    if now == 1:                            # 1) 주차구역에 수직 방향으로 제자리 회전
        where = 0
        reftime = 38 * retry
        firstDx = int(arData["DX"])

    elif now == 2:                          # 2) 주차구역의 중앙까지 직진하여 이동
        if yaw > 0:
            where = 3
            reftime = abs(firstDx) * 1.1
        else:
            where = 2
            reftime = abs(firstDx) * 0.9

    elif now == 3:                          # 3) 주차구역에 수평 방향으로 제자리 회전
        reftime = float('inf')
        # DX값이 0이 될 때 까지 회전합니다
        if int(arData["DX"]) > 0:
            where = 0                       # DX값이 0보다 크다면 우회전합니다
            firstDy = int(arData["DY"])
        elif int(arData["DX"]) < 0:
            where = 1                       # DX값이 0보다 작다면 좌회전합니다.
            firstDy = int(arData["DY"])
        else:                               # 4) 주차구역까지 직진
            where = 2                       # 직진합니다.
            if int(arData["DY"]) < 73:      # 만약 DY값이 72이하이면
                finish = True               # 종료합니다.

# ---------------------------------------------------------------------------- #
    '''
    where변수는 어떤 행동을 취할지 정하는 변수입니다.
    각 행동은 아래와 같습니다.
    0) 제자리에서 우회전함
    1) 제자리에서 좌회전함
    2) 직진
    3) 후진
    '''
# ----------------------
    '''
    제자리에서 우회전합니다.
    제자리에서 우회전 하는 것은 1Hz, 즉 모터 토픽이 2retry만큼 발생되었을 때
    -retry < count < 0 만큼은 우회전 직진을 하지만
    0 < count < retry 만큼은 후진을 하여
    제자리에서 회전할 수 있도록 만듭니다.
    '''
    if where == 0:
        if count > retry:
            count = -retry

        if count < 0:                       # -retry < count < 0 일 때,
            temspeed = refspeed             # 직진으로
            temangle = refangle             # 우회전합니다
        elif count < retry:                 # 0 <= count < retry 일 때,
            temspeed = -refspeed            # 후진합니다.
            temangle = 0
        elif count == retry:                #
            Docount2 = False if Docount2 == True else True
            if Docount2 == True:
                temangle = refangle
                temspeed = refspeed
            else:
                temangle = 0
                temspeed = -refspeed
            count = -retry
        if atime > reftime:
            atime = 0
            now += 1


    elif where == 1:            #where1은 제자리에서 좌회전하는 코드입니
        if count > retry:
            count = -retry


        if count < 0:         # 첫번째 반복
            temspeed = refspeed * 2
            temangle = -refangle
        elif count == 0:  # 반복 기준점
            prevDx = int(arData["DX"])
            prevDy = int(arData["DY"])
        elif count < retry:  # 두번째 반복
            temspeed = -refspeed * 2.3
            temangle = 0
        elif count == retry:  # 반복 기준점2
            Docount2 = False if Docount2 == True else True
            if Docount2 == True:
                temangle = refangle
                temspeed = refspeed
            else:
                temangle = 0
                temspeed = -refspeed
            count = -retry
        if atime > reftime:
            atime = 0
            now += 1

    elif where == 2:            #where2은 전진하는 코드입니다.
        temangle = 0
        temspeed = refspeed
        if atime > reftime:
            atime = 0
            now += 1

    elif where == 3:            # where3은 후진하는 코드
        temangle = 0
        temspeed = -refangle
        if atime > reftime:
            atime = 0
            now += 1



    # 모터 토픽을 발생시키기 전(다음 반복을 하기 전)에 할 일
    count += 1
    atime += 1


    try:  # temspeed가 정의되지 않은 경우
        temspeed != 1
    except:
        temspeed = refspeed
    try:  # temangle가 정의되지 않은 경우
        temangle == 1
    except:
        temangle = refangle

    speed = temspeed
    angle = temangle

    if finish == True:
        speed = 0

    # 조향각값과 속도값을 넣어 모터 토픽을 발행하기
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor_pub.publish(motor_msg)

# while 루프가 끝나면 열린 윈도우 모두 닫고 깔끔하게 종료하기
cv2.destroyAllWindows()





