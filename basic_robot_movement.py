from naoqi import ALProxy
import time

IP = "127.0.0.1"
PORT = 31559

def main():
    # Services
    motion = ALProxy("ALMotion", IP, PORT)
    posture = ALProxy("ALRobotPosture", IP, PORT)
    life = ALProxy("ALAutonomousLife", IP, PORT)
    tts = ALProxy("ALTextToSpeech", IP, PORT)

    # Disable autonomous behavior
    life.setState("disabled")

    # Wake up and stand
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.5)

    # Head movement
    motion.setAngles("HeadYaw", 0.8, 0.2)
    time.sleep(1.0)
    motion.setAngles("HeadYaw", -0.8, 0.2)
    time.sleep(1.0)
    motion.setAngles("HeadYaw", 0.0, 0.2)

    # Right arm wave
    names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"]
    angles = [0.5, -0.3, 1.2, 0.5]
    motion.setAngles(names, angles, 0.2)

    time.sleep(1.5)

    # Speech (silent in simulator, works on real NAO)
    tts.say("Hello. I am running from Visual Studio Code.")

    print("Demo complete.")

if __name__ == "__main__":
    main()
