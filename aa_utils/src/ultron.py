#!/usr/bin/env python

"""
Created on Tue Mar 15 14:06:29 2016

@author: ahmad
"""

import random
import roslaunch
import yaml
import os, sys, signal
import threading

if os.name == 'posix' and sys.version_info[0] < 3:
    import subprocess32 as subprocess
else:
    import subprocess


class Command(object):
    def __init__(self, cmd):
        self.cmd = cmd
        self.process = None

    def run(self, timeout):
        def target():
            print 'Thread started'
            self.process = subprocess.Popen(self.cmd, shell=True)
            self.process.communicate()
            print 'Thread finished'

        thread = threading.Thread(target=target)
        thread.start()

        thread.join(timeout)
        if thread.is_alive():
            print 'Terminating process'

            subprocess.Popen("pkill -9 sptam_node",shell=True)
            self.process.kill()
            # thread.join()
        print self.process.returncode


def slamTrial(yamldoc, slam='sptam', package='aa_utils', dataset='MH1', launchFile='SPTAM-MH1-Automated.launch', bagFile='/home/ahmad/Downloads/MH_01_easy.bag', pwdir='/home/ahmad/catkin_ws/src/aa_utils/config/Generated/'):
    while True:
        salt =  str(random.randint(1,10000))
        try:
            os.makedirs(pwdir)
        except:
            pass
        slamOutfile = pwdir+ dataset+ '-'+ slam +'-' + salt +'-slam.yaml'
        slamReaderOutfile = pwdir+ dataset+ '-'+ slam +'-' + salt +'-reader.yaml'
        gtReaderOutfile = pwdir+ dataset+ '-'+ slam +'-' + salt +'-gtreader.yaml'
        publisherOutfile = pwdir+ dataset+ '-'+ slam +'-' + salt +'-publisher.yaml'
        if not os.path.isfile(slamOutfile):
            break

    opened = 0
    with open(slamOutfile, 'w') as outfile:
        print slamOutfile
        outfile.write( yaml.dump(yamldoc[slam], default_flow_style=False) )
        outfile.flush()
        opened = opened + 1

    with open(slamReaderOutfile, 'w') as outfile:
        print slamReaderOutfile
        outfile.write( yaml.dump(yamldoc["SLAMReader"], default_flow_style=False) )
        outfile.flush()
        opened = opened + 1


    with open(gtReaderOutfile, 'w') as outfile:
        print gtReaderOutfile
        outfile.write( yaml.dump(yamldoc["GTReader"], default_flow_style=False) )
        outfile.flush()
        opened = opened + 1

    with open(publisherOutfile, 'w') as outfile:
        print publisherOutfile
        outfile.write( yaml.dump(yamldoc["CamInfoPublisher"], default_flow_style=False) )
        outfile.flush()
        opened = opened + 1


    if opened == 4:
        slamconfigFileArg = 'slamconfigFile:="'+ slamOutfile +'"'
        slamReaderconfigFileArg = 'slamReaderconfigFile:="'+ slamReaderOutfile +'"'
        gtReaderconfigFileArg = 'gtReaderconfigFile:="'+ gtReaderOutfile +'"'
        publisherconfigFileArg = 'publisherconfigFile:="'+ publisherOutfile +'"'

        bagFileArg = 'bagFile:="' +bagFile +'"'

        duration = subprocess.check_output("rosbag info -y -k duration "+bagFile, shell=True, stdin=None, stderr=None)
        tolerance = 5 #seconds
        duration = float(duration.strip()) + tolerance
        cmdline = '/opt/ros/indigo/bin/roslaunch '+ package +' '+ launchFile +' '+ bagFileArg +' '+ slamconfigFileArg+' '+ slamReaderconfigFileArg+' '+ gtReaderconfigFileArg+' '+ publisherconfigFileArg
        print cmdline
        command = Command(cmdline)
        command.run(timeout=duration)
    else:
        print "Failed to write to files!"

def evaluate_loss_ate(gtFile, outFile):
    return subprocess.check_output("/home/ahmad/Downloads/ate_new/evaluate_ate.py --estimate_scale --plot v1.png " + gtFile + " "+ outFile, shell=True).strip()

if __name__=="__main__":
    doc = {
  "SLAMReader": {
    "topic": "/sptam/robot/pose",
    "outfile": "/home/ahmad/catkin_ws/outputs/SPTAM-MH1.csv"
  },
  "sptam": {'DescriptorExtractor': {'Name': 'BRIEF', 'bytes': 32},
 'DescriptorMatcher': {'Name': 'BruteForce-Hamming', 'crossCheck': False},
 'EpipolarDistance': 0,
 'FeatureDetector': {'Name': 'GFTT',
                     'minDistance': 15.0,
                     'nfeatures': 2000,
                     'qualityLevel': 0.01,
                     'useHarrisDetector': False},
 'FramesBetweenKeyFrames': 0,
 'FrustumFarPlaneDist': 10000.0,
 'FrustumNearPlaneDist': 0.10000000000000001,
 'KeyFrameDistance': 0,
 'MatchingCellSize': 15,
 'MatchingDistance': 25,
 'MatchingNeighborhood': 1},
  "GTReader": {
    "topic": "/leica/position",
    "outfile": "/home/ahmad/catkin_ws/outputs/SPTAM-MH1_gt.csv"
  },
  "rosBag": {
    "args": " --clock /home/ahmad/Downloads/MH_01_easy.bag"
  },
  "CamInfoPublisher": {
    "infile": "/home/ahmad/Downloads/MH_01_easy/mav0/sensors.yaml"
  }
}

    # slamTrial(doc)
    slamTrial(doc)
    print evaluate_loss_ate(doc["GTReader"]["outfile"],doc["SLAMReader"]["outfile"])
    exit(0)
