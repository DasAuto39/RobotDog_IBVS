#!/usr/bin/env python

import rospy

import cv2

import numpy as np

import threading

import time

import actionlib

from flask import Flask, Response, request, render_template_string

from sensor_msgs.msg import Image

from geometry_msgs.msg import Twist

from message_transformer.msg import SimpleCMD 

from cv_bridge import CvBridge, CvBridgeError

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import quaternion_from_euler



# --- IMPORT KHUSUS ---

from std_msgs.msg import Bool

from open_manipulator_msgs.srv import SetJointPosition

from open_manipulator_msgs.msg import JointPosition



# --- KONFIGURASI KAMERA ---

TOPIC_BODY    = "/camera/quadruped/front_cam"

TOPIC_GRIPPER = "/camera/arm/gripper_cam"



# --- KONFIGURASI PARAMETER ---

WEB_PORT = 5000

PROCESS_WIDTH = 480

STREAM_FPS = 20

JPEG_QUALITY = 30



MOVE_SPEED   = 0.2

TURN_SPEED   = 0.1

STRAFE_SPEED = 0.15

KP_TURN      = 0.005

KP_STRAFE    = 0.003



# --- ARM POSE ---

POSE_TRACKING_START = [-0.001, -1.57, 1.14, 0.55] 

POSE_ROTATE_LEFT    = [-1.6, -0.33, -0.96, 2.0]

POSE_BEFORE_GRASP = [-1.552389, -0.342078, -0.116583, 1.148952]

# POSE_TRACKING_START = [-0.001, -1.57, 1.14, 0.55] 



# --- GLOBAL VARIABLES ---

arm_srv = None

pub_vel = None

pub_cmd = None

pub_navman = None

camera_sub = None

current_cam_topic = TOPIC_BODY

navman_status = False 



CMD_STAND_SIT  = 0x21010202

CMD_NAV_MODE   = 0x21010C03

CMD_MOVE_MODE  = 0x21010D06



# --- STATE MACHINE ---

STATE_IDLE           = 0

STATE_NAVIGATING     = 1 

STATE_SEARCHING      = 2 

STATE_TRACKING       = 3  # Body Cam Tracking

STATE_MANEUVER       = 4  # Transisi

STATE_ALIGNING       = 5  # Gripper Cam Tracking (Strafe)

STATE_READY_TO_GRASP = 6  # (BARU) Robot Duduk & Tunggu User Gambar Kotak

STATE_GRASPING_CPP   = 7  # (BARU) C++ Node Sedang Bekerja

STATE_COMPLETE       = 8



app = Flask(__name__)

bridge = CvBridge()

output_frame = None

data_lock = threading.Lock()



tracker = None

stash_tracker = None

robot_twist = Twist() 

current_state = STATE_IDLE

nav_client = None 

is_maneuvering = False



current_location_name = ""

search_waypoints = [] 

search_index = 0



# --- HTML FRONTEND ---

index_html = """

<!DOCTYPE html>

<html>

<head>

    <title>Jueying Hybrid Grasping</title>

    <style>

        body { background: #111; color: #ccc; font-family: monospace; text-align: center; margin: 0; }

        #container { position: relative; display: inline-block; border: 3px solid #555; margin-top: 10px; }

        #videoFeed { display: block; max-width: 100%; }

        #canvasLayer { position: absolute; top: 0; left: 0; width: 100%; height: 100%; cursor: crosshair; }

        .panel { margin: 5px; padding: 5px; background: #222; display: inline-block; border-radius: 8px;}

        button { padding: 8px 12px; cursor: pointer; border: none; margin: 3px; color: white; border-radius: 4px; font-weight: bold;}

        .btn-blue { background: #007bff; } .btn-red { background: #d9534f; } .btn-green { background: #28a745; } .btn-purp { background: #6f42c1; }

        

        #indicator { font-size: 20px; font-weight: bold; padding: 10px; margin: 10px; border-radius: 5px; background: #333; color: white;}

    </style>

</head>

<body>

    <div id="indicator">STATUS: IDLE</div>



    <div class="panel">

        <button class="btn-blue" onclick="sendCmd('stand')">STAND</button>

        <button class="btn-blue" onclick="sendCmd('nav')">NAV MODE</button>

        <button class="btn-blue" onclick="sendCmd('move')">MOVE MODE</button>

        <button class="btn-red" onclick="sendCmd('sit')">SIT</button>

    </div>

    <div class="panel">

        <button class="btn-purp" onclick="switchCam('body')">BODY CAM</button>

        <button class="btn-purp" onclick="switchCam('gripper')">GRIPPER CAM</button>

    </div>

    <br>

    <div class="panel">

        <button class="btn-green" onclick="startMission('Posisi_Awal')">POSISI AWAL</button>

        <button class="btn-green" onclick="startMission('Depan_Raisa')">DEPAN RAISA</button>

        <button class="btn-red" onclick="stopMission()">STOP ALL</button>

    </div>



    <div id="container">

        <img id="videoFeed" src="/video_feed">

        <canvas id="canvasLayer"></canvas>

    </div>

    

    <script>

        const canvas = document.getElementById('canvasLayer');

        const img = document.getElementById('videoFeed');

        const ind = document.getElementById('indicator');

        const cont = document.getElementById('container');

        let startX, startY, isDrawing = false;



        function resizeCanvas() { canvas.width = img.clientWidth; canvas.height = img.clientHeight; }

        img.onload = resizeCanvas; window.onresize = resizeCanvas;



        function sendCmd(cmd) { fetch('/robot_cmd/' + cmd); }

        function startMission(loc) { fetch('/mission_goto/' + loc); }

        function stopMission() { fetch('/stop_all'); }

        function switchCam(cam) { fetch('/switch_cam/' + cam); }



        setInterval(() => {

            fetch('/get_state').then(r => r.json()).then(data => {

                ind.innerText = data.text;

                ind.style.background = data.color;

                

                // Ubah warna border container biar user ngeh

                if(data.code == 6) { // READY TO GRASP

                    cont.style.borderColor = "#00FF00"; // HIJAU MENYALA

                } else if (data.code == 7) {

                    cont.style.borderColor = "#FF0000"; // MERAH (JANGAN GANGGU)

                } else {

                    cont.style.borderColor = "#555";

                }

            });

        }, 500);



        canvas.addEventListener('mousedown', e => {

            const rect = canvas.getBoundingClientRect();

            startX = e.clientX - rect.left; startY = e.clientY - rect.top; isDrawing = true;

        });

        canvas.addEventListener('mousemove', e => {

            if (!isDrawing) return;

            const rect = canvas.getBoundingClientRect();

            const ctx = canvas.getContext('2d');

            ctx.clearRect(0, 0, canvas.width, canvas.height);

            ctx.strokeStyle = '#00FF00'; ctx.lineWidth = 2;

            ctx.strokeRect(startX, startY, e.clientX - rect.left - startX, e.clientY - rect.top - startY);

        });

        canvas.addEventListener('mouseup', e => {

            isDrawing = false;

            const rect = canvas.getBoundingClientRect();

            const endX = e.clientX - rect.left; const endY = e.clientY - rect.top;

            const scaleX = img.naturalWidth / img.clientWidth; const scaleY = img.naturalHeight / img.clientHeight;

            

            let realX = Math.floor(Math.min(startX, endX) * scaleX);

            let realY = Math.floor(Math.min(startY, endY) * scaleY);

            let realW = Math.floor(Math.abs(endX - startX) * scaleX);

            let realH = Math.floor(Math.abs(endY - startY) * scaleY);



            if (realW > 5 && realH > 5) {

                fetch('/init_tracker', {

                    method: 'POST', headers: {'Content-Type': 'application/json'},

                    body: JSON.stringify({x: realX, y: realY, w: realW, h: realH})

                });

                const ctx = canvas.getContext('2d'); ctx.clearRect(0, 0, canvas.width, canvas.height);

            }

        });

    </script>

</body>

</html>

"""



# --- CALLBACK UNTUK C++ NODE (NAVMAN) ---

def navman_callback(msg):

    global navman_status

    navman_status = msg.data



# --- HELPER FUNCTIONS ---

def set_camera_topic(topic_name):

    global camera_sub, current_cam_topic

    if camera_sub: camera_sub.unregister()

    current_cam_topic = topic_name

    camera_sub = rospy.Subscriber(current_cam_topic, Image, image_callback, queue_size=1, buff_size=2**24)



def move_arm_service(joint_angles, time_sec=2.0):

    global arm_srv

    if arm_srv is None: return

    try:

        req = SetJointPosition._request_class()

        req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']

        req.joint_position.position = joint_angles

        req.path_time = time_sec

        arm_srv(req)

        time.sleep(time_sec)

    except: pass



def get_waypoints_from_param(param_name):

    waypoints = []

    if rospy.has_param(param_name):

        d = rospy.get_param(param_name)

        for k in sorted(d.keys()): waypoints.append(d[k])

    return waypoints



def send_map_goal(x, y, theta):

    global nav_client

    if nav_client is None: return False

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "map"; goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x; goal.target_pose.pose.position.y = y

    q = quaternion_from_euler(0, 0, theta)

    goal.target_pose.pose.orientation.x=q[0]; goal.target_pose.pose.orientation.y=q[1]

    goal.target_pose.pose.orientation.z=q[2]; goal.target_pose.pose.orientation.w=q[3]

    nav_client.send_goal(goal); return True



# --- LOGIKA MANUVER ---

def perform_transition_maneuver():

    global robot_twist, is_maneuvering, current_state, tracker

    is_maneuvering = True; 

    # stash_tracker = tracker

    tracker = None 

     

    # Putar Body

    robot_twist.linear.x = 0; robot_twist.angular.z = 0; time.sleep(1.0)

    robot_twist.angular.z = 0.45; time.sleep(4.0)

    robot_twist.linear.x = 0.13

    robot_twist.angular.z = 0

    time.sleep(1.0)

    print("Rotating Body 90 deg...") 

    robot_twist.linear.x = 0

    robot_twist.angular.z = 0

    time.sleep(1.0) 

    # Putar Arm & Ganti Kamera

    move_arm_service(POSE_ROTATE_LEFT, 2.5)

    set_camera_topic(TOPIC_GRIPPER)

    

    current_state = STATE_ALIGNING

    tracker = stash_tracker

    is_maneuvering = False



# --- LOGIKA SELESAI ALIGNMENT (SIAP UNTUK GRASPNET) ---

def perform_alignment_complete():

    global robot_twist, current_state, is_maneuvering, tracker

    

    is_maneuvering = True

    tracker = None # Stop Tracker OpenCV

    

    print("ALIGNMENT OK. Sitting Down...")

    

    robot_twist.linear.y = -0.11; robot_twist.linear.x = 0

    time.sleep(1.0)

 





    # 1. Stop Robot

    robot_twist.linear.y = 0; robot_twist.linear.x = 0; robot_twist.angular.z = 0

    time.sleep(1.0)



    

    # 2. Duduk

    msg = SimpleCMD(); msg.type=0; msg.cmd_code=CMD_STAND_SIT

    pub_cmd.publish(msg)

    time.sleep(2.0)

    

    # move_arm_service(POSE_BEFORE_GRASP, 2.5)

    

    # 3. Pindah State ke READY

    current_state = STATE_COMPLETE

    is_maneuvering = False

    print(">>> ROBOT READY. PLEASE DRAW BOX TO START GRASPNET <<<")



# --- LOGIKA HANDOVER KE C++ ---

def perform_cpp_handover():

    global current_state, navman_status

    

    current_state = STATE_GRASPING_CPP

    print("[PY] Sending START to C++ Node...")

    

    # Trigger C++

    pub_navman.publish(True)

    time.sleep(2.0)

    

    # Tunggu C++ Selesai

    while navman_status:

        time.sleep(1.0)

    

    print("[PY] Grasping Completed.")

    current_state = STATE_COMPLETE



# --- MISSION LOOP ---

def mission_logic_thread():

    global current_state, nav_client, search_index, search_waypoints

    nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    rate = rospy.Rate(10)



    while not rospy.is_shutdown():

        if current_state == STATE_NAVIGATING:

            if nav_client.get_state() == actionlib.GoalStatus.SUCCEEDED:

                sp_param = f"/{current_location_name}_spoints"

                search_waypoints = get_waypoints_from_param(sp_param)

                if len(search_waypoints) > 0:

                    current_state = STATE_SEARCHING; search_index = 0

                    pt = search_waypoints[0]; send_map_goal(pt[0], pt[1], pt[2])

                else: current_state = STATE_COMPLETE



        elif current_state == STATE_SEARCHING:

            if tracker is not None:

                print("LOCKED FRONT! Stop Nav."); nav_client.cancel_all_goals(); current_state = STATE_TRACKING

            elif nav_client.get_state() in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED]:

                search_index = (search_index + 1) % len(search_waypoints)

                pt = search_waypoints[search_index]; send_map_goal(pt[0], pt[1], pt[2])

                rospy.sleep(0.5)

        rate.sleep()



# --- CALLBACK KAMERA ---

def image_callback(msg):

    global output_frame, tracker, robot_twist, current_state, is_maneuvering

    try:

        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        h, w = cv_image.shape[:2]

        scale = PROCESS_WIDTH / float(w)

        cv_image = cv2.resize(cv_image, (PROCESS_WIDTH, int(h * scale)))

    except: return



    lin_x = 0.0; lin_y = 0.0; ang_z = 0.0

    should_upd_twist = True



    with data_lock:

        if is_maneuvering:

            cv2.putText(cv_image, "PROCESSING...", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            should_upd_twist = False 

        

        # --- BODY TRACKING ---

        elif current_state == STATE_TRACKING and tracker and current_cam_topic == TOPIC_BODY:

            success, box = tracker.update(cv_image)

            if success:

                x, y, w, h = [int(v) for v in box]; cx, cy = x+w/2, y+h

                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                th_y = int(cv_image.shape[0] * 0.65)

                th_x_lo = int(PROCESS_WIDTH * 0.475); th_x_hi = int(PROCESS_WIDTH * 0.525)

                # cv2.line(cv_image, (0, th_y), (PROCESS_WIDTH, th_y), (0, 255, 255), 1)



                if (cy > th_y) and (th_x_lo < cx < th_x_hi):

                    cv2.putText(cv_image, "REACHED!", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    current_state = STATE_MANEUVER

                    threading.Thread(target=perform_transition_maneuver).start()

                    should_upd_twist = False

                else:

                    if not (th_x_lo < cx < th_x_hi):

                        ang_z = np.clip(KP_TURN * ((PROCESS_WIDTH/2)-cx), -TURN_SPEED, TURN_SPEED)

                    else: ang_z = 0.0; lin_x = MOVE_SPEED

            else:

                cv2.putText(cv_image, "LOST FRONT", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)



        # --- GRIPPER TRACKING (ALIGNMENT) ---

        elif current_state == STATE_ALIGNING and tracker and current_cam_topic == TOPIC_GRIPPER:

            success, box = tracker.update(cv_image)

            if success:

                x, y, w, h = [int(v) for v in box]; cx, cy = x+w*0.2, y+h

                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

                th_y = int(cv_image.shape[0]); th_x_lo = int(PROCESS_WIDTH * 0.475); th_x_hi = int(PROCESS_WIDTH * 0.525)

                cv2.line(cv_image, (0, th_y), (PROCESS_WIDTH, th_y), (0, 255, 255), 1)

                

                # JIKA SUDAH PAS TENGAH

                if (th_x_lo < cx < th_x_hi) and (cy > th_y):

                    cv2.putText(cv_image, "CENTERED! SITTING...", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # Panggil fungsi stop & duduk

                    threading.Thread(target=perform_alignment_complete).start()

                    should_upd_twist = False

                else:

                    if not (th_x_lo < cx < th_x_hi):

                        err_x = (PROCESS_WIDTH/2) - cx

                        lin_x = np.clip(KP_STRAFE * err_x, -STRAFE_SPEED, STRAFE_SPEED)

                    else : lin_y = -0.4; lin_x = 0.0

            else:

                cv2.putText(cv_image, "ALIGNING...", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)



        # --- INDIKATOR STATUS LAIN ---

        elif current_state == STATE_READY_TO_GRASP:

             cv2.putText(cv_image, "DRAW BOX TO GRASP!", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        

        elif current_state == STATE_GRASPING_CPP:

             cv2.putText(cv_image, "GRASPING IN PROGRESS...", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)



        output_frame = cv_image.copy()



    if should_upd_twist and (current_state in [STATE_TRACKING, STATE_ALIGNING]):

        robot_twist.linear.x = lin_x; robot_twist.linear.y = lin_y; robot_twist.angular.z = ang_z



# --- FLASK ROUTES ---

@app.route("/")

def index(): return render_template_string(index_html)



@app.route("/get_state")

def get_state():

    # Return JSON untuk indikator warna

    st_text = ["IDLE","NAV","SEARCH","TRACK_BODY","MANEUVER","ALIGN_GRIPPER","READY TO GRASP","GRASPING (C++)","COMPLETE"][current_state]

    color = "#333"

    if current_state == STATE_TRACKING: color = "#0000FF" # Biru

    if current_state == STATE_ALIGNING: color = "#FFA500" # Orange

    if current_state == STATE_READY_TO_GRASP: color = "#00FF00" # Hijau (User Action Needed)

    if current_state == STATE_GRASPING_CPP: color = "#FF0000" # Merah (Busy)

    

    return {"text": st_text, "color": color, "code": current_state}



@app.route("/robot_cmd/<cmd>")

def robot_cmd(cmd):

    msg = SimpleCMD(); msg.type=0

    if cmd=='stand': msg.cmd_code=CMD_STAND_SIT

    elif cmd=='nav': msg.cmd_code=CMD_NAV_MODE

    elif cmd=='move': msg.cmd_code=CMD_MOVE_MODE

    elif cmd=='sit': msg.cmd_code=CMD_STAND_SIT

    pub_cmd.publish(msg)

    return "OK"



@app.route("/switch_cam/<cam>")

def switch_c(cam):

    if cam == 'body': set_camera_topic(TOPIC_BODY)

    elif cam == 'gripper': set_camera_topic(TOPIC_GRIPPER)

    return "OK"



@app.route("/mission_goto/<loc>")

def mission_goto(loc):

    global current_state, current_location_name

    current_location_name = loc

    set_camera_topic(TOPIC_BODY)

    wp = get_waypoints_from_param(f"/{loc}")

    if len(wp) > 0:

        pt = wp[0]; send_map_goal(pt[0], pt[1], pt[2]); current_state = STATE_NAVIGATING

        return "OK"

    return "Error"



@app.route("/stop_all")

def stop_all():

    global current_state, tracker

    current_state = STATE_IDLE; tracker = None

    if nav_client: nav_client.cancel_all_goals()

    robot_twist.linear.x = 0; robot_twist.linear.y = 0; robot_twist.angular.z = 0

    set_camera_topic(TOPIC_BODY) 

    return "STOPPED"



@app.route("/init_tracker", methods=['POST'])

def init_track():

    global tracker, current_state, stash_tracker

    d = request.json

    print(f"DEBUG: Draw Box at State: {current_state}")



    with data_lock:

        if output_frame is not None:

            # 1. JIKA SEDANG READY TO GRASP -> INI TRIGGER UNTUK GRASPNET C++

            if current_state == STATE_READY_TO_GRASP:

                print("TRIGGERING GRASPNET...")

                # Kita tidak perlu init tracker OpenCV disini, cukup start thread C++

                threading.Thread(target=perform_cpp_handover).start()

                return "OK"



            # 2. JIKA NORMAL TRACKING -> INIT OPENCV

            try: 

                tracker = cv2.TrackerCSRT_create()

                stash_tracker = tracker



            except: tracker = cv2.legacy.TrackerCSRT_create()

            tracker.init(output_frame, (d['x'], d['y'], d['w'], d['h']))

            

            if current_state == STATE_SEARCHING:

                if nav_client: nav_client.cancel_all_goals()

                current_state = STATE_TRACKING

                threading.Thread(target=move_arm_service, args=(POSE_TRACKING_START, 2.0)).start()

            elif current_state == STATE_ALIGNING:

                print("Aligning Tracker Started")

            elif current_state == STATE_IDLE or current_state == STATE_COMPLETE:

                 if current_cam_topic == TOPIC_BODY:

                    current_state = STATE_TRACKING

                    threading.Thread(target=move_arm_service, args=(POSE_TRACKING_START, 2.0)).start()

                 else:

                    current_state = STATE_ALIGNING

    return "OK"



def gen():

    while True:

        time.sleep(1.0/STREAM_FPS)

        with data_lock:

            if output_frame is None: continue

            _, enc = cv2.imencode(".jpg", output_frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])

        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(enc) + b'\r\n')



@app.route("/video_feed")

def vid(): return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")



def pub_thread():

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():

        # Kirim CMD VEL hanya saat tracking/aligning/navigasi

        if current_state in [STATE_TRACKING, STATE_MANEUVER, STATE_ALIGNING, STATE_COMPLETE]:

            if pub_vel: pub_vel.publish(robot_twist)

        rate.sleep()



def arm_init_thread():

    global arm_srv

    try:

        rospy.wait_for_service('/goal_joint_space_path', timeout=5.0)

        arm_srv = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)

    except: pass



if __name__ == '__main__':

    rospy.init_node('yaml_mission_commander')

     

    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    pub_cmd = rospy.Publisher('/simple_cmd', SimpleCMD, queue_size=1)

    

    pub_navman = rospy.Publisher('/navman_comm', Bool, queue_size=1)

    rospy.Subscriber('/navman_comm', Bool, navman_callback)



    t_arm = threading.Thread(target=arm_init_thread); t_arm.daemon = True; t_arm.start()

    set_camera_topic(TOPIC_BODY)

    

    t1 = threading.Thread(target=app.run, kwargs={'host':'0.0.0.0','port':WEB_PORT,'debug':False,'use_reloader':False})

    t1.daemon = True; t1.start()

    

    t2 = threading.Thread(target=pub_thread); t2.daemon = True; t2.start()

    t3 = threading.Thread(target=mission_logic_thread); t3.daemon = True; t3.start()

    

    print(f"COMMANDER READY: http://<IP_JETSON>:{WEB_PORT}")

    rospy.spin()
