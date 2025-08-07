from flask import Flask, request, jsonify
from Commander import Commander, Mode
from action import SingleJointJogAction
from tools.shared_struct import RobotInputData, RobotOutputData
import tools.PAROL6_ROBOT
from multiprocessing import Array
import logging

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


@app.route('/api/inject_joint_jog', methods=['POST'])
def inject_joint_jog():
    data = request.json
    joint_id = data.get("joint_id")
    speed = data.get("speed")
    time = data.get("time")
    global shared_string

    if joint_id is None or speed is None:
        return jsonify({"error": "Missing joint_id or speed"}), 400
    
    action = SingleJointJogAction(joint_id=joint_id, speed=speed, shared_string=shared_string)

    for i in range(round(time/0.01)):
        success = commander.inject_action(action)

    return jsonify({"injected": success, "joint_id": joint_id, "speed": speed})


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
