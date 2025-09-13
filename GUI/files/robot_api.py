from flask import Flask, request, jsonify
from Commander import Commander, Mode
from action import TimedJointJogAction, TimedCartesianJogAction, Move2JointsAction
from tools.shared_struct import RobotInputData, RobotOutputData
import tools.PAROL6_ROBOT as PAROL6_ROBOT
from multiprocessing import Array
import logging
import numpy as np

app = Flask(__name__)

commander: Commander = None
shared_string: Array = None


@app.route('/api/status', methods=['GET'])
def get_status():
    return jsonify(commander.to_dict())


@app.route('/api/actions', methods=['GET'])
def get_actions():
    return jsonify(commander.get_action_queue())


@app.route('/api/actions', methods=['DELETE'])
def clear_actions():
    commander.clear_queue()
    return jsonify({"message": "Action queue cleared"})


@app.route('/api/enable/<int:flag>', methods=['POST'])
def toggle_enable(flag):
    commander.set_enable(bool(flag))
    return jsonify({"enabled": commander.is_enabled()})


from action import SingleJointJogAction, TimedJointJogAction
import tools.PAROL6_ROBOT as PAROL6

@app.before_request
def _guard_init():
    global commander
    if commander is None:
        return jsonify({"error": "Commander not initialized"}), 503

@app.route('/api/inject_joint_jog', methods=['POST'])
def inject_joint_jog():
    if commander is None:
        return jsonify({"error": "Commander not initialized"}), 503

    if not request.is_json:
        return jsonify({"error": "Expected application/json"}), 400
    data = request.get_json()

    # parse joint_id
    try:
        joint_id = int(data.get("joint_id"))
    except Exception:
        return jsonify({"error": "joint_id must be integer"}), 400
    if joint_id < 0 or joint_id > 5:
        return jsonify({"error": "joint_id must be in [0..5]"}), 400

    # parse duration
    try:
        duration_s = float(data.get("time", 0.0))
    except Exception:
        return jsonify({"error": "time must be number"}), 400
    if duration_s <= 0:
        return jsonify({"error": "time must be > 0"}), 400

    # Prefer percent input (signed allowed). Support backward compat: 'speed' (step/s).
    percent_in = data.get("percent", None)
    speed_in = data.get("speed", None)

    direction = None
    percent = None

    if percent_in is not None:
        try:
            percent = float(percent_in)
        except Exception:
            return jsonify({"error": "percent must be numeric"}), 400
        # allow signed percent: negative => direction -1
        if percent < 0:
            direction = -1
            percent = abs(percent)
        else:
            direction = 1
        # explicit direction override
        if "direction" in data:
            try:
                d = int(data.get("direction"))
                if d not in (1, -1):
                    raise ValueError()
                direction = d
            except Exception:
                return jsonify({"error": "direction must be 1 or -1"}), 400

        if percent < 0 or percent > 100:
            return jsonify({"error": "percent must be in [0..100]"}), 400

    elif speed_in is not None:
        # backward compatibility: convert step/s -> percent
        try:
            speed_in = float(speed_in)
        except Exception:
            return jsonify({"error": "speed must be numeric"}), 400
        if speed_in == 0:
            return jsonify({"error": "speed cannot be zero; use percent instead"}), 400
        direction = 1 if speed_in > 0 else -1
        max_step = float(PAROL6.Joint_max_jog_speed[joint_id])
        if max_step <= 0:
            return jsonify({"error": "Invalid joint max speed config"}), 500
        percent = min(abs(speed_in) / max_step * 100.0, 100.0)
    else:
        return jsonify({"error": "provide 'percent' (preferred) or 'speed'"}), 400

    # 构造并注入动作
    action = TimedJointJogAction(
        joint_id=joint_id,
        percent=percent,
        direction=direction,
        duration_s=duration_s,
        shared_string=shared_string
    )

    try:
        ok = commander.inject_action(action)
    except Exception as e:
        return jsonify({"injected": False, "error": repr(e)}), 500

    return jsonify({
        "injected": bool(ok),
        "joint_id": joint_id,
        "percent": percent,
        "direction": direction,
        "duration_s": duration_s
    })

# 在文件顶部（或现有 imports 区）添加：
# from Commander import commander        # <- 如果你的 commander 实例在 Commander.py，取消注释并按需修改
# from action import TimedCartesianJogAction
# from flask import request, jsonify

@app.route('/api/inject_cartesian_jog', methods=['POST'])
def inject_cartesian_jog():
    """
    POST JSON:
    {
      "dx": float (m, per-loop displacement, default 0),
      "dy": float (m, default 0),
      "dz": float (m, default 0),
      "rx": float (deg, default 0),
      "ry": float (deg, default 0),
      "rz": float (deg, default 0),
      "frame": "WRF"|"TRF" (default "WRF"),
      "speed_pct": float 0..100 (default 50),
      "interval_s": float >0 (control loop interval, default 0.01),
      "duration_s": float >0 (total duration in seconds, required)
    }
    """
    if not request.is_json:
        return jsonify({"error": "Expected application/json"}), 400
    data = request.get_json()

    # parse & validate numeric params
    try:
        dx = float(data.get("dx", 0.0))
        dy = float(data.get("dy", 0.0))
        dz = float(data.get("dz", 0.0))
        rx = float(data.get("rx", 0.0))
        ry = float(data.get("ry", 0.0))
        rz = float(data.get("rz", 0.0))
        frame = str(data.get("frame", "WRF")).upper()
        speed_pct = float(data.get("speed_pct", 50.0))
        interval_s = float(data.get("interval_s", 0.01))
        duration_s = float(data.get("duration_s", 0.0))
    except Exception:
        return jsonify({"error": "dx/dy/dz/rx/ry/rz/speed_pct/interval_s/duration_s must be numbers"}), 400

    if frame not in ("WRF", "TRF"):
        return jsonify({"error": "frame must be 'WRF' or 'TRF'"}), 400
    if duration_s <= 0.0:
        return jsonify({"error": "duration_s must be > 0"}), 400
    if interval_s <= 0.0:
        return jsonify({"error": "interval_s must be > 0"}), 400

    # clamp speed_pct
    speed_pct = max(0.0, min(100.0, speed_pct))

    # create action
    try:
        action = TimedCartesianJogAction(
            dx=dx, dy=dy, dz=dz,
            rx=rx, ry=ry, rz=rz,
            frame=frame,
            speed_pct=speed_pct,
            interval_s=interval_s,
            duration_s=duration_s,
            shared_string=None  # 如果你有共享显示 buffer，可把它传进来
        )
    except Exception as e:
        return jsonify({"error": "failed to create action", "detail": str(e)}), 500

    # inject into commander
    try:
        # 注意：确保当前作用域可以访问到 commander 实例
        commander.inject_action(action)
    except Exception as e:
        return jsonify({"error": "failed to inject action", "detail": str(e)}), 500

    return jsonify({
        "ok": True,
        "msg": "TimedCartesianJogAction injected",
        "params": {
            "dx": dx, "dy": dy, "dz": dz,
            "rx": rx, "ry": ry, "rz": rz,
            "frame": frame, "speed_pct": speed_pct,
            "interval_s": interval_s, "duration_s": duration_s
        }
    }), 200


@app.route('/api/mode/<int:mode_val>', methods=['POST'])
def set_mode(mode_val):
    try:
        mode_enum = Mode(mode_val)
    except ValueError:
        return jsonify({"error": f"Invalid mode value: {mode_val}"}), 400

    commander.current_mode.value = mode_enum.value
    return jsonify({"message": f"Mode set to {mode_enum.name}", "mode_value": mode_enum.value})


@app.route('/api/mode', methods=['GET'])
def get_mode():
    mode_val = commander.current_mode.value
    mode_name = Mode(mode_val).name
    return jsonify({"current_mode": mode_name, "mode_value": mode_val})


@app.route('/api/exit', methods=['POST'])
def exit_api():
    print("Try to shutdown")
    shutdown = request.environ.get('werkzeug.server.shutdown')
    if shutdown:
        shutdown()
        return 'Server shutting down...'
    else:
        return 'Server not running under Werkzeug', 500


def start_flask_api(commander_ref: Commander, shared_string_ref: Array):
    """
    Starts the Flask API server with access to commander and shared_string.
    Must be called in a separate thread or process.
    """
    global commander
    global shared_string
    commander = commander_ref
    shared_string = shared_string_ref

    print("[FlaskAPI] Starting server at http://localhost:8000")
    app.run(host="0.0.0.0", port=8000, debug=False, use_reloader=False, threaded=True)


@app.route('/api/inject_move_joint', methods=['POST'])
def inject_move_joint():
    if commander is None:
        return jsonify({"error": "Commander not initialized"}), 503
    if not request.is_json:
        return jsonify({"error": "Expected application/json"}), 400

    data = request.get_json()

    # ------- 1) 解析 6 轴角度(°) -------
    q_deg = None
    if isinstance(data.get("q"), list) and len(data["q"]) == 6:
        q_deg = data["q"]
    elif isinstance(data.get("q_deg"), list) and len(data["q_deg"]) == 6:
        q_deg = data["q_deg"]
    else:
        # 尝试 j1..j6
        keys = ["j1","j2","j3","j4","j5","j6"]
        if all(k in data for k in keys):
            q_deg = [data[k] for k in keys]

    if q_deg is None:
        return jsonify({"error": "provide 6 joint angles by 'q' (list of 6), 'q_deg' (list of 6), or fields j1..j6"}), 400

    # 转成 float 并校验
    try:
        q_deg = [float(x) for x in q_deg]
    except Exception:
        return jsonify({"error": "joint angles must be numeric"}), 400

    # 限位检查（把度转弧度，与 Joint_limits_radian 比较）
    q_rad = [PAROL6_ROBOT.DEG2RAD(x) for x in q_deg]
    for i in range(6):
        lo, hi = PAROL6_ROBOT.Joint_limits_radian[i]
        if not (lo <= q_rad[i] <= hi):
            return jsonify({
                "error": "target out of joint limits",
                "joint": i,
                "target_deg": q_deg[i],
                "limits_rad": [lo, hi]
            }), 400

    # ------- 2) 解析轨迹参数 -------
    # t>0 优先生效；否则使用 v/a；都没有则走保守值
    t = data.get("t", data.get("time", 0))
    try:
        t = float(t)
    except Exception:
        t = 0.0
    if t < 0:
        return jsonify({"error": "t must be >= 0"}), 400

    v_pct = data.get("v", data.get("velocity"))
    a_pct = data.get("a", data.get("acceleration"))
    if v_pct is not None:
        try: v_pct = float(v_pct)
        except Exception: return jsonify({"error": "v must be numeric [0..100]"}), 400
        if not (0 <= v_pct <= 100): return jsonify({"error": "v must be in [0..100]"}), 400
    if a_pct is not None:
        try: a_pct = float(a_pct)
        except Exception: return jsonify({"error": "a must be numeric [0..100]"}), 400
        if not (0 <= a_pct <= 100): return jsonify({"error": "a must be in [0..100]"}), 400

    profile = (data.get("profile") or data.get("func") or "poly").lower()
    if profile not in ("poly", "trap"):
        return jsonify({"error": "profile must be 'poly' or 'trap'"}), 400

    tracking = (data.get("tracking") or ("speed" if data.get("speed_mode") else "pos")).lower()
    if tracking not in ("pos", "speed"):
        return jsonify({"error": "tracking must be 'pos' or 'speed'"}), 400

    interval_s = data.get("interval_s", data.get("interval", 0.01))
    try:
        interval_s = float(interval_s)
    except Exception:
        return jsonify({"error": "interval_s must be numeric"}), 400
    if interval_s < 1e-3:
        return jsonify({"error": "interval_s must be >= 0.001"}), 400

    tolerance_steps = data.get("tolerance_steps", 6)
    try:
        tolerance_steps = int(tolerance_steps)
        if tolerance_steps < 1:
            raise ValueError()
    except Exception:
        return jsonify({"error": "tolerance_steps must be integer >= 1"}), 400

    # ------- 3) 构造并注入 Action -------
    try:
        action = Move2JointsAction(
            q_deg=q_deg,
            t=t if t > 0 else 0.0,
            v_pct=v_pct,
            a_pct=a_pct,
            profile=profile,
            tracking="speed" if tracking == "speed" else "pos",
            interval_s=interval_s,
            tolerance_steps=tolerance_steps,
            shared_string=shared_string
        )
    except Exception as e:
        return jsonify({"error": f"failed to build action: {repr(e)}"}), 500

    try:
        ok = commander.inject_action(action)
    except Exception as e:
        return jsonify({"injected": False, "error": repr(e)}), 500

    return jsonify({
        "injected": bool(ok),
        "q_deg": q_deg,
        "mode": ("t" if t > 0 else ("v/a" if (v_pct is not None and a_pct is not None) else "conservative")),
        "t": t,
        "v_pct": v_pct,
        "a_pct": a_pct,
        "profile": profile,
        "tracking": tracking,
        "interval_s": interval_s,
        "tolerance_steps": tolerance_steps
    }), 200
