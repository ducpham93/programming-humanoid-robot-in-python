'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.started = False
        self.startTime = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        if(self.keyframes == ([],[],[])):
            self.started = False
            return target_joints
        
        names, times, keys = keyframes

        if (not self.started):
            self.started = True
            self.startTime = perception.time
        currentTime = perception.time - self.startTime
        
        finishedJoints = 0
        for (i, name) in enumerate(names):
            
            if (currentTime > times[i][-1]):
                finishedJoints += 1
                if (finishedJoints == len(names)):
                    self.started = False
                    self.keyframes = ([], [], [])
                    break
                continue
            
            timeslot = -1
            
            for j in range(len(times[i])):
                if (j == len(times[i]) - 1):
                    break
                if (times[i][j] <= currentTime and currentTime < times[i][j + 1]):
                    timeslot = j
                    break
                
            t = (currentTime - times[i][timeslot]) / (times[i][timeslot + 1] - times[i][timeslot]) 

            if (timeslot == -1):
                break
            elif (timeslot == 0):
                p2 = keys[i][timeslot][0]
                p3 = p2 + keys[i][timeslot][1][2]
                p0 = 0
                p1 = 0
            elif (timeslot == 1):
                p2 = keys[i][timeslot][0]
                p3 = p2 + keys[i][timeslot][1][2]
                p0 = 0
                p1 = keys[i][timeslot - 1][0]
            elif (timeslot == len(times[i]) - 2):
                p2 = 0
                p3 = 0
                p0 = keys[i][timeslot][0]
                p1 = p0 + keys[i][timeslot][1][2]
            else:
                p2 = keys[i][timeslot][0]
                p3 = p2 + keys[i][timeslot][1][2]
                p0 = keys[i][timeslot - 1][0]
                p1 = p0 + keys[i][timeslot - 1][1][2]


            angle = ((1 - t) ** 3) * p0 + 3 * t * ((1 - t) ** 2) * p1 + 3 * (t ** 2) * (1 - t) * p2 + (t ** 3) * p3
            target_joints[name] = angle
            
            if(name == "LHipYawPitch"):
                target_joints["RHipYawPitch"] = angle
        
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
