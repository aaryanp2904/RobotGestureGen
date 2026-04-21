# nao_server.py - RUN IN PYTHON 2.7
from SimpleXMLRPCServer import SimpleXMLRPCServer
from naoqi import ALProxy
import sys
import time
import threading

class NaoBridge:
    def __init__(self, robot_ip, robot_port):
        print("Connecting to NAO at {}:{}...".format(robot_ip, robot_port))
        self.motion = ALProxy("ALMotion", robot_ip, robot_port)
        self.posture = ALProxy("ALRobotPosture", robot_ip, robot_port)
        self.life = ALProxy("ALAutonomousLife", robot_ip, robot_port)
        self.tts = ALProxy("ALTextToSpeech", robot_ip, robot_port)
        # Disable autonomous life to prevent it from interfering with our animation
        if self.life.getState() != "disabled":
            self.life.setState("disabled")

        self.motion.wakeUp()
        self.posture.goToPosture("StandInit", 0.5)
        print("NAO is initialized and listening for commands on port 8000.")

    def play_trajectory(self, names, angles, times):
        """Receives trajectory arrays from Python 3 and executes them with BLOCKING call to simulator."""
        print("Received trajectory payload for {} joints. Executing...".format(len(names)))
        start_time = time.time()
        self.motion.angleInterpolation(names, angles, times, True)
        elapsed_time = time.time() - start_time
        print("Simulation execution time: {:.2f} seconds".format(elapsed_time))
        return True

    def play_motion_with_speech(self, names, all_angles, all_times, frame_texts):
        """
        Receive the FULL trajectory + per-frame speech text in one call.
        Uses setAngles() in a timed loop instead of angleInterpolation to
        avoid cubic interpolation velocity overshoots.
        Speech is fired at the correct times inline.
        """
        num_joints = len(names)
        num_frames = len(all_times[0]) if num_joints > 0 else 0
        print("Received full payload: {} joints, {} frames.".format(num_joints, num_frames))

        if num_frames == 0:
            print("No frames to play.")
            return True

        # --- Pre-process speech events from frame_texts ---
        speech_events = []  # list of (time, text)
        prev_text = ""
        for i in range(num_frames):
            txt = frame_texts[i]
            if txt and txt != prev_text:
                speech_events.append((all_times[0][i], txt))
            prev_text = txt

        print("Speech events: {}".format(len(speech_events)))

        # Compute frame interval from data
        if num_frames > 1:
            frame_dt = all_times[0][1] - all_times[0][0]
        else:
            frame_dt = 0.033

        print("Playing trajectory: {} frames, {:.1f}s, frame_dt={:.4f}s".format(
            num_frames, all_times[0][-1], frame_dt))

        # --- Timed loop: setAngles per frame + speech ---
        speech_idx = 0
        overall_start = time.time()
        speed_fraction = 0.5  # fraction of max speed for setAngles (0.0-1.0)

        for frame_i in range(num_frames):
            frame_abs_time = all_times[0][frame_i]
            target_wall = overall_start + frame_abs_time

            # Fire any speech events due at this frame's time
            while speech_idx < len(speech_events) and speech_events[speech_idx][0] <= frame_abs_time:
                evt_text = speech_events[speech_idx][1]
                self._say_threaded(evt_text)
                speech_idx += 1

            # Build angle list for this frame (one value per joint)
            frame_angles = [all_angles[j][frame_i] for j in range(num_joints)]

            # Set target angles - robot moves toward them at safe speed
            try:
                self.motion.setAngles(names, frame_angles, speed_fraction)
            except Exception as e:
                print("Error in setAngles at frame {}: {}".format(frame_i, str(e)))

            # Sleep until next frame time
            now = time.time()
            sleep_duration = target_wall - now + frame_dt
            if sleep_duration > 0:
                time.sleep(sleep_duration)

            # Progress logging every 10 seconds
            if frame_i > 0 and frame_i % int(10.0 / frame_dt) == 0:
                elapsed = time.time() - overall_start
                print("  [{:.0f}s / {:.0f}s] frame {}/{}".format(
                    elapsed, all_times[0][-1], frame_i, num_frames))

        elapsed = time.time() - overall_start
        print("Full playback complete in {:.2f}s.".format(elapsed))
        return True

    def _say_threaded(self, text):
        """Fire TTS in a daemon thread (non-blocking)."""
        def _run():
            try:
                print("  [TTS] \"{}\"".format(text))
                self.tts.say(str(text))
            except Exception as e:
                print("  [TTS ERROR] {}".format(str(e)))
        t = threading.Thread(target=_run)
        t.daemon = True
        t.start()

    def say_async(self, text):
        """Trigger text-to-speech asynchronously (kept for backward compat)."""
        self._say_threaded(text)
        return True

    def stop(self):
        """Emergency stop command triggered by Ctrl+C"""
        print("\nEMERGENCY STOP RECEIVED!")
        self.motion.killAll()
        self.posture.goToPosture("StandInit", 0.5)
        return True

    def rest(self):
        """Returns the robot to a resting state."""
        print("Returning to rest state.")
        self.posture.goToPosture("StandInit", 0.5)
        self.motion.rest()
        return True

if __name__ == "__main__":
    robot_port = int(sys.argv[1]) if len(sys.argv) > 1 else 31559
    robot_ip = "127.0.0.1"

    server = SimpleXMLRPCServer(("localhost", 8000), allow_none=True)
    server.register_introspection_functions()
    server.register_instance(NaoBridge(robot_ip, robot_port))

    print("Python 2 Bridge Server running. Waiting for Python 3 client...")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server.")