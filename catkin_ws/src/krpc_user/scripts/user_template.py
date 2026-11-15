#!/usr/bin/python3
# -*- coding:utf-8 -*-
import json
import rospy
import actionlib
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
from geometry_msgs.msg import PoseStamped
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType

# Optional IF examples. Uncomment only what your program uses.
# from std_msgs.msg import Time
# from platform_msgs.msg import UserNodeStatus
# from sensor_msgs.msg import Image
# from ib2_msgs.msg import Navigation


class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')

        # Action client
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # Service client
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE

        # Optional subscriber examples.
        # /sensor_fusion/navigation provides the current Int-Ball2 pose, velocity and acceleration.
        # self.__subscriber_nav = rospy.Subscriber('/sensor_fusion/navigation', Navigation, self.__callback_nav)
        # /camera_main/image_raw provides the main camera image.
        # self.__subscriber_camera = rospy.Subscriber('/camera_main/image_raw', Image, self.__callback_camera_image)

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing) # DO NOT REMOVE THIS LINE

        self.__started = False
        self.__action_active = False
        rospy.set_param('/ib2_user/ready', True) # DO NOT REMOVE THIS LINE

    def __notify_mission_start(self):
        rospy.wait_for_service('/event/mission_start')
        res = self.__mission_start()
        rospy.loginfo('mission_start result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_mission_finished(self):
        rospy.wait_for_service('/event/mission_finished')
        res = self.__mission_finished()
        rospy.loginfo('mission_finished result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_checkpoint_passed(self, checkpoint_id):
        rospy.loginfo('notify checkpoint_id=%d passed', checkpoint_id)
        self.__checkpoint_client.wait_for_server()
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        self.__checkpoint_client.send_goal(goal)
        while not rospy.is_shutdown():
            if self.__checkpoint_client.wait_for_result(rospy.Duration(0.2)):
                result = self.__checkpoint_client.get_result()
                rospy.loginfo('checkpoint_passed result: %s', result)
                return result.success
        if rospy.is_shutdown():
            self.__checkpoint_client.cancel_goal()
            rospy.logwarn('checkpoint_passed action cancelled by node shutdown')
        return False

    def __get_checkpoint_id(self, checkpoint):
        for key in ('checkpoint_id', 'checkpointid', 'id'):
            if key in checkpoint:
                return int(checkpoint[key])
        return None

    def __resolve_first_checkpoint_target(self):
        default_target = {
            'checkpoint_id': 1,
            'position': [0.0, 0.0, 0.0],
            'quaternion': [0.0, 0.0, 0.0, 1.0],
        }
        rospy.wait_for_service('/mission/get_checkpoints')
        res = self.__checkpoints()
        rospy.loginfo('get_checkpoints result: success=%s message=%s', res.success, res.message)
        if not res.success:
            return default_target

        try:
            payload = json.loads(res.message)
            checkpoints = payload.get('checkpoints', payload.get('Checkpoints', []))
            checkpoints = [cp for cp in checkpoints if isinstance(cp, dict) and self.__get_checkpoint_id(cp) is not None]
            if not checkpoints:
                rospy.logwarn('No checkpoints found in payload')
                return default_target

            checkpoint = min(checkpoints, key=self.__get_checkpoint_id)
            position = checkpoint.get('position', default_target['position'])

            if not (isinstance(position, list) and len(position) >= 3):
                rospy.logwarn('Invalid checkpoint position format')
                return default_target

            quaternion = default_target['quaternion']
            attitude = checkpoint.get('attitude')
            if isinstance(attitude, dict):
                attitude_quaternion = attitude.get('quaternion')
                if isinstance(attitude_quaternion, list) and len(attitude_quaternion) >= 4:
                    quaternion = attitude_quaternion

            return {
                'checkpoint_id': self.__get_checkpoint_id(checkpoint),
                'position': [float(position[0]), float(position[1]), float(position[2])],
                'quaternion': [float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3])],
            }
        except Exception as e:
            rospy.logwarn('Failed to parse checkpoints payload: %s', e)

        return default_target

    def __move_to_checkpoint(self, checkpoint_target):
        position = checkpoint_target['position']
        quaternion = checkpoint_target['quaternion']
        rospy.loginfo(
            'moving to checkpoint_id=%d: x=%.3f y=%.3f z=%.3f qx=%.4f qy=%.4f qz=%.4f qw=%.4f',
            checkpoint_target['checkpoint_id'],
            position[0], position[1], position[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )

        target = PoseStamped()
        target.pose.position.x = position[0]
        target.pose.position.y = position[1]
        target.pose.position.z = position[2]
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]

        command_type = CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET)
        goal = CtlCommandGoal(target=target, type=command_type)
        self.__action_active = True
        try:
            self.__command.send_goal(goal)
            while not rospy.is_shutdown():
                if self.__command.wait_for_result(rospy.Duration(0.2)):
                    result = self.__command.get_result()
                    rospy.loginfo('move command result: %s', result)
                    return result.type == CtlCommandResult.TERMINATE_SUCCESS
            if rospy.is_shutdown():
                self.__command.cancel_goal()
                rospy.logwarn('move command cancelled by node shutdown')
            return False
        finally:
            self.__action_active = False

    def __run_user_flow(self, msg):
        try:
            if not self.__notify_mission_start():
                return

            # USER EDIT AREA: define route planning and game-specific flow here.
            # ***********************************************
            # Works at any simulation speed. The 5 min mission limit and the time bonus are in SIM
            # time (organizers; the web server runs the sim at ~0.37x real time, local GPU ~0.92x),
            # so the whole budget is in sim seconds. The real-time factor (rtf) is measured live and
            # only used to turn sim durations into wall-clock timeouts. Plan = best expected score.
            # Uses only the official interfaces: /mission/get_checkpoints, /ctl/command,
            # /event/checkpoint_passed, /event/mission_finished and /sensor_fusion/navigation.
            import math
            import time
            from ib2_msgs.msg import Navigation

            PLAY_TIME_SEC = 300.0      # mission limit, SIM seconds from the start signal
            FINISH_MARGIN_SEC = 5.0    # sim seconds always kept for Mission Finish (the call takes < 0.1 s)
            MOVE_TIMEOUT_MIN_SIM = 60.0  # sim s: a move's timeout is max(this, 2x its estimate)
            MAX_ATTEMPTS = 3           # move commands per checkpoint (1 + 2 corrections)
            SETTLE_MAX_SIM = 3.0       # sim s: max wait for the robot to come to rest after a move
            STILL_LIN = 0.01           # m/s: "at rest" linear speed
            STILL_ANG = 0.02           # rad/s: "at rest" angular speed
            SERVICE_WAIT_SEC = 10.0    # max wait for a service / action server to appear
            NOTIFY_TIMEOUT_SEC = 10.0  # max wait for a checkpoint_passed result (~10 ms locally)
            NAV_WAIT_SEC = 10.0        # wall s: max wait for a fresh navigation message
            RTF_DEFAULT = 0.5          # used until enough time has passed to measure it
            CP_POINTS = 10.0           # web result page: 10 pt per Lv3 checkpoint
            GOAL_BONUS = 30.0          # web result page: goal score 30 (all cps passed + at the goal at finish)
            MISSION_REF_SIM = 300.0    # time bonus = 2 - mission time (sim s) / 300 (fits the 1.66 at 101.8 s)
            FINISH_SIM = 1.0           # sim s from the last checkpoint to Mission Finish
            # move-time model (sim s), fitted on 26 local moves (rmse 2.2 s):
            #   MOVE_A + MOVE_B * distance_m + MOVE_C * rotation_deg, times a live correction factor
            MOVE_A, MOVE_B, MOVE_C = 11.1, 6.25, 0.078
            VISIT_OVERHEAD_SIM = 2.0   # pose check + notify per checkpoint (no settle wait when the move ended on target)
            FAST_POS_M, FAST_ANG_DEG = 0.07, 7.0  # notify at once if this far inside the Lv3 bands (10 cm / 10 deg)
            time_ratios = [1.0]        # actual / predicted move time, learned live
            t_start = time.monotonic()
            s_start = rospy.get_time()

            def elapsed():
                return time.monotonic() - t_start

            def sim_elapsed():
                return rospy.get_time() - s_start

            def rtf():
                w = elapsed()
                if w < 1.5:
                    return RTF_DEFAULT
                return max(0.05, min(1.5, sim_elapsed() / w))

            def remaining():
                # wall seconds left before we must send Mission Finish
                # sim seconds left before we must send Mission Finish
                return PLAY_TIME_SEC - FINISH_MARGIN_SEC - sim_elapsed()

            def q_normalize(q):
                n = math.sqrt(sum(c * c for c in q)) or 1.0
                return [c / n for c in q]

            def q_rotate(q, v):
                x, y, z, w = q_normalize(q)
                tx = 2.0 * (y * v[2] - z * v[1])
                ty = 2.0 * (z * v[0] - x * v[2])
                tz = 2.0 * (x * v[1] - y * v[0])
                return [v[0] + w * tx + (y * tz - z * ty),
                        v[1] + w * ty + (z * tx - x * tz),
                        v[2] + w * tz + (x * ty - y * tx)]

            def unit(v):
                n = math.sqrt(sum(c * c for c in v))
                return [c / n for c in v] if n > 1e-9 else [0.0, 0.0, 0.0]

            def angle_deg(a, b):
                d = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
                return math.degrees(math.acos(d))

            def twist_deg(q, axis):
                x, y, z, w = q_normalize(q)
                if w < 0.0:
                    x, y, z, w = -x, -y, -z, -w
                return math.degrees(2.0 * math.atan2(x * axis[0] + y * axis[1] + z * axis[2], w))

            def dist(a, b):
                return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

            def parse_bands(bands, key):
                # [(threshold, level), ...] sorted tightest first; [] if absent or malformed
                out = []
                for b in bands or []:
                    try:
                        out.append((float(b[key]), int(b['level'])))
                    except Exception:
                        pass
                return sorted(out)

            def classify(value, bands):
                for threshold, level in bands:
                    if value <= threshold:
                        return level
                return 0

            def notify_passed(cid):
                # bounded version of __notify_checkpoint_passed: finite waits, cancel on timeout
                rospy.loginfo('notify checkpoint_id=%d passed', cid)
                if not self.__checkpoint_client.wait_for_server(rospy.Duration(SERVICE_WAIT_SEC)):
                    rospy.logwarn('checkpoint_passed server not available after %.0f s', SERVICE_WAIT_SEC)
                    return False
                self.__checkpoint_client.send_goal(CheckpointPassedGoal(checkpoint_id=cid))
                if not self.__checkpoint_client.wait_for_result(rospy.Duration(NOTIFY_TIMEOUT_SEC)):
                    rospy.logwarn('checkpoint_passed for %d timed out after %.0f s; cancelling', cid, NOTIFY_TIMEOUT_SEC)
                    self.__checkpoint_client.cancel_goal()
                    return False
                result = self.__checkpoint_client.get_result()
                rospy.loginfo('checkpoint_passed result: %s', result)
                return result is not None and result.success

            def parse_checkpoints():
                rospy.wait_for_service('/mission/get_checkpoints', timeout=SERVICE_WAIT_SEC)
                res = self.__checkpoints()
                if not res.success:
                    raise RuntimeError('get_checkpoints failed: %s' % res.message)
                payload = json.loads(res.message)
                cps = []
                for item in payload.get('checkpoints', []):
                    cid = self.__get_checkpoint_id(item)
                    pos = item.get('position')
                    if cid is None or not (isinstance(pos, list) and len(pos) >= 3):
                        continue
                    att = item.get('attitude') or {}
                    quat = att.get('quaternion') or [0.0, 0.0, 0.0, 1.0]
                    marker = (item.get('marker_pose') or {}).get('position')
                    cps.append({
                        'id': cid,
                        'pos': [float(c) for c in pos[:3]],
                        'quat': q_normalize([float(c) for c in quat[:4]]),
                        'marker': [float(c) for c in marker[:3]] if marker else None,
                        'pos_bands': parse_bands(item.get('score_bands'), 'radius') or [(0.1, 3), (0.2, 2), (0.3, 1)],
                        'cen_bands': parse_bands(att.get('centering_bands'), 'angle'),
                        'roll_bands': parse_bands(att.get('roll_bands'), 'angle'),
                    })
                goal_id = payload.get('goalpoint_id')
                return cps, (int(goal_id) if goal_id is not None else None)

            # navigation: keep the latest message from one subscriber instead of wait_for_message,
            # so a slow or briefly silent topic never aborts the mission
            nav = {'msg': None, 'seq': 0}

            def on_nav(m):
                nav['msg'] = m
                nav['seq'] += 1

            nav_sub = rospy.Subscriber('/sensor_fusion/navigation', Navigation, on_nav, queue_size=1)

            def read_nav(timeout=NAV_WAIT_SEC):
                # a message newer than this call if one comes within timeout, else the last known one
                seq0 = nav['seq']
                deadline = time.monotonic() + timeout
                while not rospy.is_shutdown() and time.monotonic() < deadline and nav['seq'] == seq0:
                    time.sleep(0.02)
                m = nav['msg']
                if m is None:
                    raise RuntimeError('no navigation data received')
                if nav['seq'] == seq0:
                    rospy.logwarn('no fresh navigation data in %.0f s; using last known pose', timeout)
                p, o = m.pose.pose.position, m.pose.pose.orientation
                t = m.twist
                lin = math.sqrt(t.linear.x ** 2 + t.linear.y ** 2 + t.linear.z ** 2)
                ang = math.sqrt(t.angular.x ** 2 + t.angular.y ** 2 + t.angular.z ** 2)
                return [p.x, p.y, p.z], [o.x, o.y, o.z, o.w], lin, ang

            def settle():
                # wait (sim time) until at rest - the scorer samples the pose at <= 5 Hz - then read the pose
                s_deadline = rospy.get_time() + SETTLE_MAX_SIM
                w_deadline = time.monotonic() + min(2.0 * SETTLE_MAX_SIM, max(remaining(), 1.0)) / rtf()
                still_since = None
                pos, quat = None, None
                while (not rospy.is_shutdown() and rospy.get_time() < s_deadline
                       and time.monotonic() < w_deadline):
                    pos, quat, lin, ang = read_nav()
                    if lin < STILL_LIN and ang < STILL_ANG:
                        still_since = still_since or rospy.get_time()
                        if rospy.get_time() - still_since >= 0.5:
                            break
                    else:
                        still_since = None
                    time.sleep(0.1)
                if pos is None:
                    pos, quat = read_nav()[:2]
                return pos, quat

            def estimate_level(cp, pos, quat):
                # same rule as the published scoring: lowest of position / centering / roll levels
                d = dist(pos, cp['pos'])
                levels = [classify(d, cp['pos_bands'])]
                c = r = 0.0
                if cp['marker'] is not None and (cp['cen_bands'] or cp['roll_bands']):
                    view = unit([cp['marker'][i] - pos[i] for i in range(3)])
                    c = angle_deg(q_rotate(quat, [1.0, 0.0, 0.0]), view)
                    r = abs((twist_deg(quat, view) - twist_deg(cp['quat'], view) + 180.0) % 360.0 - 180.0)
                    if cp['cen_bands']:
                        levels.append(classify(c, cp['cen_bands']))
                    if cp['roll_bands']:
                        levels.append(classify(r, cp['roll_bands']))
                return min(levels), d, c, r

            def rot_deg(qa, qb):
                d = abs(sum(x * y for x, y in zip(q_normalize(qa), q_normalize(qb))))
                return math.degrees(2.0 * math.acos(min(1.0, d)))

            def time_scale():
                return max(0.7, min(1.5, sorted(time_ratios)[len(time_ratios) // 2]))

            def est_move_raw(pa, qa, pb, qb):
                if use_wrench[0]:
                    return wrench_time(pa, qa, pb, qb)
                return MOVE_A + MOVE_B * dist(pa, pb) + MOVE_C * rot_deg(qa, qb)

            def est_move_sim(pa, qa, pb, qb):
                return time_scale() * est_move_raw(pa, qa, pb, qb)

            def est_visit_sim(pa, qa, pb, qb):
                # sim seconds to move a -> b, settle and notify
                return est_move_sim(pa, qa, pb, qb) + VISIT_OVERHEAD_SIM

            def move(cp, timeout):
                # timeout in wall seconds
                target = PoseStamped()
                target.header.frame_id = 'iss_body'
                target.pose.position.x, target.pose.position.y, target.pose.position.z = cp['pos']
                (target.pose.orientation.x, target.pose.orientation.y,
                 target.pose.orientation.z, target.pose.orientation.w) = cp['quat']
                goal = CtlCommandGoal(target=target, type=CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET))
                if not self.__command.wait_for_server(rospy.Duration(SERVICE_WAIT_SEC)):
                    rospy.logwarn('/ctl/command server not available after %.0f s', SERVICE_WAIT_SEC)
                    return False
                self.__action_active = True
                try:
                    self.__command.send_goal(goal)
                    deadline = time.monotonic() + timeout
                    while not rospy.is_shutdown() and time.monotonic() < deadline:
                        if self.__command.wait_for_result(rospy.Duration(0.2)):
                            res = self.__command.get_result()
                            return res is not None and res.type == CtlCommandResult.TERMINATE_SUCCESS
                    rospy.logwarn('move to checkpoint %d timed out after %.0f wall s; cancelling', cp['id'], timeout)
                    self.__command.cancel_goal()
                    self.__command.wait_for_result(rospy.Duration(5.0))
                    return False
                finally:
                    self.__action_active = False

            # ---- parallel fly + turn: STAND_BY + our own controller on /ctl/wrench ----------------
            # The built-in MOVE flies first and then turns in place (~12 s per checkpoint). After a
            # STAND_BY command the built-in controller stops publishing /ctl/wrench, so we steer with
            # the same control laws and gains as the built-in one (ctl.yaml), but along a reference
            # that moves and turns at the same time. Safety net: built-in MOVE if a leg goes wrong.
            from geometry_msgs.msg import WrenchStamped
            import threading
            MASS, INERTIA = 3.216, 0.0136        # ctl.yaml ctl_body (isotropic inertia)
            KP_POS, KD_POS = 0.6219, 1.1152      # ctl.yaml pos_ctl
            KP_ATT, KD_ATT = 2.6181, 3.2361      # ctl.yaml att_ctl
            A_MAX, V_MAX = 0.02, 0.18            # reference accel m/s^2 / speed m/s
            AL_MAX, W_MAX = 0.20, 0.40           # reference angular accel rad/s^2 / rate rad/s
            F_LIM, T_LIM = 0.15, 0.006           # caps on commanded |force| N / |torque| Nm (fans ~0.18 N / 8.2 mNm)
            LAG_START, LAG_STOP = 0.05, 0.20     # m: tracking error where the reference slows / stops
            ALAG_START, ALAG_STOP = 8.0, 25.0    # deg: same for attitude
            ABORT_ERR = 0.40                     # m: tracking error that aborts the leg -> built-in MOVE
            LEG_GRACE = 8.0                      # sim s after the reference ends before falling back
            WRENCH_TAIL = 2.0                    # sim s: typical convergence after the reference ends
            CONV_POS, CONV_ANG = 0.03, 3.0       # m / deg: "on target" (Lv3 needs 0.10 m / 10 deg)
            use_wrench = [True]
            wrench_pub = rospy.Publisher('/ctl/wrench', WrenchStamped, queue_size=1)
            wstate = {'ref': None, 'run': True, 'active': False, 'tref': 0.0, 'tlast': 0.0, 'err': 0.0, 'aerr': 0.0}

            def qmul(a, b):
                ax, ay, az, aw = a
                bx, by, bz, bw = b
                return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                        aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]

            def qconj(q):
                return [-q[0], -q[1], -q[2], q[3]]

            def qexp(axis, ang):
                s = math.sin(ang / 2.0)
                return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(ang / 2.0)]

            def trap_T(total, a, vmax):
                ta = vmax / a
                if a * ta * ta >= total:
                    return 2.0 * math.sqrt(max(total, 0.0) / a)
                return 2.0 * ta + (total - a * ta * ta) / vmax

            def trap(total, a, vmax):
                # trapezoid (or triangle) profile: (duration, f(t) -> (x, v, acc))
                if total < 1e-9:
                    return 0.0, lambda t: (0.0, 0.0, 0.0)
                ta = vmax / a
                if a * ta * ta >= total:
                    ta = math.sqrt(total / a)
                    vp, tc = a * ta, 0.0
                else:
                    vp, tc = vmax, (total - a * ta * ta) / vmax
                T = 2 * ta + tc

                def f(t):
                    if t <= 0:
                        return 0.0, 0.0, 0.0
                    if t < ta:
                        return 0.5 * a * t * t, a * t, a
                    if t < ta + tc:
                        return 0.5 * a * ta * ta + vp * (t - ta), vp, 0.0
                    if t < T:
                        td = T - t
                        return total - 0.5 * a * td * td, a * td, -a
                    return total, 0.0, 0.0
                return T, f

            def wrench_time(pa, qa, pb, qb):
                return max(trap_T(dist(pa, pb), A_MAX, V_MAX),
                           trap_T(math.radians(rot_deg(qa, qb)), AL_MAX, W_MAX)) + WRENCH_TAIL

            def set_leg(p0, q0, p1, q1):
                d = [p1[i] - p0[i] for i in range(3)]
                L = math.sqrt(sum(c * c for c in d))
                u = [c / L for c in d] if L > 1e-9 else [0.0, 0.0, 0.0]
                dq = qmul(qconj(q_normalize(q0)), q_normalize(q1))
                if dq[3] < 0:
                    dq = [-c for c in dq]
                ang = 2.0 * math.atan2(math.sqrt(sum(c * c for c in dq[:3])), dq[3])
                ax = unit(dq[:3]) if ang > 1e-6 else [1.0, 0.0, 0.0]
                Tp, fp = trap(L, A_MAX, V_MAX)
                Ta, fa = trap(ang, AL_MAX, W_MAX)
                wstate.update(ref=dict(p0=p0, q0=q_normalize(q0), u=u, ax=ax, fp=fp, fa=fa),
                              tref=0.0, tlast=rospy.get_time(), err=0.0, aerr=0.0)
                return max(Tp, Ta)

            def control_loop():
                while wstate['run'] and not rospy.is_shutdown():
                    m, ref = nav['msg'], wstate['ref']
                    if wstate['active'] and m is not None and ref is not None:
                        # reference governor: the reference only advances while the robot keeps up
                        now = rospy.get_time()
                        dt = max(0.0, now - wstate['tlast'])
                        wstate['tlast'] = now
                        k = min(1.0, max(0.0, 1.0 - (wstate['err'] - LAG_START) / (LAG_STOP - LAG_START)))
                        k = min(k, max(0.0, 1.0 - (wstate['aerr'] - ALAG_START) / (ALAG_STOP - ALAG_START)))
                        wstate['tref'] += dt * k
                        x, v, a = ref['fp'](wstate['tref'])
                        th, w, al = ref['fa'](wstate['tref'])
                        if k < 1.0:
                            v, w, a = v * k, w * k, 0.0
                        rr = [ref['p0'][i] + ref['u'][i] * x for i in range(3)]
                        vr = [ref['u'][i] * v for i in range(3)]
                        ar = [ref['u'][i] * a for i in range(3)]
                        qr = qmul(ref['q0'], qexp(ref['ax'], th))
                        wr = [ref['ax'][i] * w for i in range(3)]    # reference body frame, fixed axis
                        p, o = m.pose.pose.position, m.pose.pose.orientation
                        r = [p.x, p.y, p.z]
                        q = q_normalize([o.x, o.y, o.z, o.w])
                        vn = [m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]     # iss frame
                        wn = [m.twist.angular.x, m.twist.angular.y, m.twist.angular.z]  # body frame
                        wstate['err'] = math.sqrt(sum((r[i] - rr[i]) ** 2 for i in range(3)))
                        acc = [ar[i] - KP_POS * (r[i] - rr[i]) - KD_POS * (vn[i] - vr[i]) for i in range(3)]
                        f_body = q_rotate(qconj(q), [c * MASS for c in acc])
                        fn = math.sqrt(sum(c * c for c in f_body))
                        if fn > F_LIM:
                            f_body = [c * F_LIM / fn for c in f_body]
                        qe = qmul(qconj(qr), q)
                        wstate['aerr'] = math.degrees(2.0 * math.acos(min(1.0, abs(qe[3]))))
                        sgn = 1.0 if qe[3] >= 0 else -1.0
                        wr_b = q_rotate(qconj(qe), wr)
                        wc = [-2.0 * KP_ATT / KD_ATT * sgn * qe[i] + wr_b[i] for i in range(3)]
                        tq = [KD_ATT * INERTIA * (wc[i] - wn[i]) for i in range(3)]
                        tn = math.sqrt(sum(c * c for c in tq))
                        if tn > T_LIM:
                            tq = [c * T_LIM / tn for c in tq]
                        msg = WrenchStamped()
                        msg.header.stamp = m.pose.header.stamp
                        msg.header.frame_id = 'body'
                        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = f_body
                        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = tq
                        wrench_pub.publish(msg)
                    time.sleep(0.01)

            def standby():
                # built-in controller -> STAND_BY (stops publishing /ctl/wrench); False if refused
                try:
                    g = CtlCommandGoal(target=PoseStamped(), type=CtlStatusType(type=CtlStatusType.STAND_BY))
                    self.__command.send_goal(g)
                    if not self.__command.wait_for_result(rospy.Duration(5.0)):
                        return False
                    res = self.__command.get_result()
                    return res is not None and res.type == CtlCommandResult.TERMINATE_SUCCESS
                except Exception as exc:
                    rospy.logwarn('standby failed: %s', exc)
                    return False

            def wrench_leg(cp, limit_sim):
                # fly to cp with our controller; True when on target (robot keeps being held there)
                pos, quat = read_nav()[:2]
                T = set_leg(pos, quat, cp['pos'], cp['quat'])
                wstate['active'] = True
                s0 = rospy.get_time()
                limit = min(T + LEG_GRACE, limit_sim)
                why = 'not on target %.0f s after the reference ended' % LEG_GRACE
                while not rospy.is_shutdown() and rospy.get_time() - s0 < limit:
                    if wstate['err'] > ABORT_ERR:
                        why = 'tracking error %.2f m' % wstate['err']
                        break
                    p, q, lin, ang = read_nav()
                    lvl, d, c, r = estimate_level(cp, p, q)
                    if (rospy.get_time() - s0 >= T and d < CONV_POS and c < CONV_ANG and r < CONV_ANG
                            and lin < STILL_LIN and ang < STILL_ANG):
                        return True, T
                    time.sleep(0.05)
                rospy.logwarn('cp %d: wrench leg aborted (%s)', cp['id'], why)
                return False, T

            def fly(cp, timeout_sim):
                # one leg to cp; True if the robot ended on target. Falls back to the built-in MOVE.
                if use_wrench[0]:
                    ok, T = wrench_leg(cp, timeout_sim)
                    if ok:
                        return True
                    wstate['active'] = False
                    ok = move(cp, max(5.0, timeout_sim) / rtf())
                    if standby():
                        p, q = read_nav()[:2]
                        set_leg(p, q, p, q)          # hold the pose the built-in MOVE reached
                        wstate['active'] = True
                    else:
                        use_wrench[0] = False
                        rospy.logwarn('STAND_BY refused; built-in MOVE from now on')
                    return ok
                return move(cp, timeout_sim / rtf())

            def score(n_cps, mission_sim, all_done):
                # web scoring (sim v1.2, confirmed 116.80 = (80 + 30) x 1.0618):
                # (checkpoint points + goal score) x time bonus (2 - mission time / 300)
                bonus = max(0.0, 2.0 - mission_sim / MISSION_REF_SIM)
                return (CP_POINTS * n_cps + (GOAL_BONUS if all_done else 0.0)) * bonus

            def plan(here, left, n_done, all_ok, budget):
                # order of remaining checkpoints with the best expected final score, finishing within
                # `budget` sim s. The goal cp scores like any other; the goal bonus needs ALL cps AND the
                # robot at the goal at Mission Finish (web judge v1.2), so only goal-LAST earns it.
                # Exhaustive DFS, pruned by budget, capped in size. [] = finish now.
                s_now = sim_elapsed()
                best = {'v': score(n_done, s_now + FINISH_SIM, False), 't': 0.0, 'order': []}
                nodes = [0]

                def dfs(pos, quat, rest, t_sim, order):
                    nodes[0] += 1
                    if nodes[0] > 150000:
                        return
                    for i, c in enumerate(rest):
                        t2 = t_sim + est_visit_sim(pos, quat, c['pos'], c['quat'])
                        if t2 > budget:
                            continue
                        order.append(c)
                        # web judge: goal = all cps passed + robot within the goal margin at Mission Finish
                        all_done = all_ok and len(rest) == 1 and c['id'] == goal_id
                        v = score(n_done + len(order), s_now + t2 + FINISH_SIM, all_done)
                        if v > best['v'] + 1e-6:
                            best.update(v=v, t=t2, order=list(order))
                        dfs(c['pos'], c['quat'], rest[:i] + rest[i + 1:], t2, order)
                        order.pop()

                dfs(here[0], here[1], list(left), 0.0, [])
                return best['order'], best['t'], best['v']

            def visit(cp, reserve):
                # move (with corrections) until level 3 is confirmed; notify if worth it.
                # reserve: sim seconds that must stay available afterwards
                best = None
                for attempt in range(1, MAX_ATTEMPTS + 1):
                    here, here_q = read_nav()[:2]
                    raw = est_move_raw(here, here_q, cp['pos'], cp['quat'])
                    est_sim = time_scale() * raw
                    budget = remaining() - reserve
                    if budget <= 3.0 or (attempt > 1 and budget < est_sim + VISIT_OVERHEAD_SIM):
                        rospy.logwarn('cp %d: no time left for attempt %d (%.0f s left)', cp['id'], attempt, budget)
                        break
                    timeout = min(max(MOVE_TIMEOUT_MIN_SIM, 2.0 * est_sim), budget)  # sim s
                    s0 = rospy.get_time()
                    ok = fly(cp, timeout)
                    took = rospy.get_time() - s0
                    if ok and attempt == 1:
                        time_ratios.append(took / raw)
                    # the move ends ~1 cm from target and nearly still, so check the pose right away
                    # and only wait for the robot to settle if it is not clearly inside the Lv3 bands
                    pos, quat = read_nav()[:2]
                    level, d, c, r = estimate_level(cp, pos, quat)
                    if not (ok and level >= 3 and d <= FAST_POS_M and c <= FAST_ANG_DEG and r <= FAST_ANG_DEG):
                        pos, quat = settle()
                        level, d, c, r = estimate_level(cp, pos, quat)
                    rospy.loginfo('cp %d attempt %d: move_ok=%s took=%.1f sim s (rtf %.2f) dist=%.3fm '
                                  'centering=%.1fdeg roll=%.1fdeg -> est level %d',
                                  cp['id'], attempt, ok, took, rtf(), d, c, r, level)
                    best = level if best is None else max(best, level)
                    if level >= max(lv for _, lv in cp['pos_bands']):
                        break
                if best and best > 0:
                    return notify_passed(cp['id'])
                rospy.logwarn('cp %d: not notified (estimated level %s)', cp['id'], best)
                return False

            try:
                checkpoints, goal_id = parse_checkpoints()
                left = list(checkpoints)   # the goal checkpoint scores like any other one
                n_done, all_ok = 0, True
                here = read_nav()[:2]
                ctl_thread = threading.Thread(target=control_loop)
                ctl_thread.daemon = True
                ctl_thread.start()
                if standby():
                    set_leg(here[0], here[1], here[0], here[1])   # hold the start pose
                    wstate['active'] = True
                else:
                    use_wrench[0] = False
                    rospy.logwarn('STAND_BY refused at start; using the built-in MOVE')
                while elapsed() < 1.0 and not rospy.is_shutdown():
                    time.sleep(0.1)    # measure rtf before the first plan
                while left and not rospy.is_shutdown():
                    # re-plan before every checkpoint with the latest rtf and speed estimates
                    order, need, value = plan(here, left, n_done, all_ok, remaining())
                    rospy.loginfo('plan (rtf %.2f, %.0f sim s left, ~%.0f sim s needed, est score %.1f): %s (goal %s)',
                                  rtf(), remaining(), need, value, [c['id'] for c in order], goal_id)
                    if not order:
                        break
                    cp = order[0]
                    left.remove(cp)
                    ok = False
                    try:
                        ok = visit(cp, 0.0)
                    except Exception as exc:
                        rospy.logerr('cp %d failed, continuing: %s', cp['id'], exc)
                    if ok:
                        n_done += 1
                    else:
                        all_ok = False
                    here = read_nav()[:2]
            except Exception as exc:
                rospy.logerr('mission flow error, finishing anyway: %s', exc)
            finally:
                if not wstate['active']:
                    wstate['run'] = False
                    nav_sub.unregister()
                # else: keep holding the final pose (the goal) through Mission Finish
            rospy.loginfo('route done at %.1f wall s / %.1f sim s (rtf %.2f)', elapsed(), sim_elapsed(), rtf())
            # the template's mission_finished call below waits for its service with no limit;
            # check it here with a limit so a missing service is logged instead of hanging silently
            try:
                rospy.wait_for_service('/event/mission_finished', timeout=SERVICE_WAIT_SEC)
            except rospy.ROSException as exc:
                rospy.logerr('mission_finished service not available: %s', exc)
            # ***********************************************

            if not rospy.is_shutdown():
                self.__notify_mission_finished()
        except Exception as exc:
            rospy.logerr('user flow failed: %s', exc)

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    # Optional subscriber callback examples.
    # def __callback_nav(self, data):
    #     position = data.pose.pose.position
    #     orientation = data.pose.pose.orientation
    #     rospy.loginfo('current pose: x=%.3f y=%.3f z=%.3f qx=%.4f qy=%.4f qz=%.4f qw=%.4f',
    #                   position.x, position.y, position.z,
    #                   orientation.x, orientation.y, orientation.z, orientation.w)
    #
    # def __callback_camera_image(self, data):
    #     rospy.loginfo('camera image: width=%d height=%d encoding=%s',
    #                   data.width, data.height, data.encoding)

    def __stop_processing(self, request):
        try:
            if self.__action_active:
                self.__command.cancel_goal()
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()
