#!/usr/bin/env python

"""
Created on Tue Mar 15 14:06:29 2016

@author: ahmad, alex
"""

# run with:   rosrun python_scrips_ros hyper1.py _outfile:=best_spptam.yaml


import pickle
import time
from hyperopt import fmin, tpe, hp, STATUS_OK, Trials
import hyperopt.pyll.stochastic
import rospy
import random
import roslaunch
import rospkg
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

class Full_data:
    data = {
  "SLAMReader": {
    "topic": "/sptam/robot/pose",
    "outfile": os.environ['HOME']+"/catkin_ws/outputs/SPTAM-MH1.csv"
  },
  "sptam": {
    "MatchingNeighborhood": 1,
    "KeyFrameDistance": 0,
    "FrustumFarPlaneDist": 10000.0,
    "DescriptorMatcher": {
      "crossCheck": False,
      "Name": "BruteForce-Hamming"
    },
    "FramesBetweenKeyFrames": 0,
    "DescriptorExtractor": {
      "bytes": 32,
      "Name": "BRIEF"
    },
    "FrustumNearPlaneDist": 0.10000000000000001,
    "MatchingDistance": 25,
    "EpipolarDistance": 0,
    "MatchingCellSize": 15,
    "FeatureDetector": {
      "qualityLevel": 0.01,
      "nfeatures": 2000,
      "Name": "GFTT",
      "minDistance": 15.0,
      "useHarrisDetector": False
    }
  },
  "GTReader": {
    "topic": "/leica/position",
    "outfile": os.environ['HOME']+"/catkin_ws/outputs/SPTAM-MH1_gt.csv"
  },
  "rosBag": {
    "args": " --clock "+rospkg.RosPack().get_path('aa_utils')+"/datasets/MH_01_easy.bag"
  },
  "CamInfoPublisher": {
    "infile": rospkg.RosPack().get_path('aa_utils')+"/config/MH1-sensors.yaml"
  }
}
    def apply_params (self,params):
        for key in sorted(params.iterkeys()):
          self.data["sptam"][key]=params[key]
    def write_to_file(self,file_path):
      #file = open(file_path, 'w+')
      with open(file_path, 'w+') as yaml_file:
          yaml_file.write( yaml.dump(self.data, default_flow_style=False))


def f(params):
    full_data= Full_data()

    loss=0
    #full_data.apply_params(params)
    slamTrial(full_data.data)
    loss=evaluate_loss_ate(full_data.data["GTReader"]["outfile"],full_data.data["SLAMReader"]["outfile"])
    #loss=hyper_test.test()
    print loss

    return {
        'loss': loss,
        'status': STATUS_OK,
        # -- store other results like this
        'eval_time': time.time(),
        'other_stuff': {'type': None, 'value': [0, 1, 2]},
        # -- attachments are handled differently
        'attachments':
            {'time_module': pickle.dumps(time.time)}
        }

def evaluate_loss_ate(gtFile, outFile):
    return subprocess.check_output(rospkg.RosPack().get_path('aa_utils')+"/src/evaluate_ate.py --estimate_scale --plot v1.png " + gtFile + " "+ outFile, shell=True).strip()


def slamTrial(yamldoc, slam='sptam', package='aa_utils', dataset='MH1', launchFile='SPTAM-MH1-Automated.launch', bagFile=rospkg.RosPack().get_path('aa_utils')+'/datasets/MH_01_easy.bag', pwdir=rospkg.RosPack().get_path('aa_utils')+'/config/Generated/'):
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



if __name__=="__main__":
 #Params for SPTAM
  space ={'DescriptorExtractor': hp.choice('DescriptorExtractor', [
                              {
                                  'Name': 'BRIEF',
                                  'bytes': 32
                              },
                              {
                                  'Name': 'ORB',
                                  'bytes': 32
                              },
                          ]),
         'MatchingCellSize': hp.quniform('MatchingCellSize', 10, 40, 1),   # default is 15, so we do something between 10 and 40
         'MatchingNeighborhood': hp.quniform('MatchingNeighborhood', 1, 3, 1), #default is 1 so we make it between 1 and 3
         'MatchingDistance': hp.uniform ('MatchingDistance', 15, 35), #default is 25
         'EpipolarDistance' : hp.quniform('EpipolarDistance', 0, 3, 1), #default is 0 and a double, doesn't make sense to be double so I make it int between 0 and 3
         'KeyFrameDistance' : hp.quniform('KeyFrameDistance', 0, 3, 1), #unkown description but the default was 0
         'FramesBetweenKeyFrames' : hp.quniform('FramesBetweenKeyFrames', 0, 3, 1), #unknown description but the default was 0
         'FrustumNearPlaneDist' : hp.uniform ('FrustumNearPlaneDist', 0.0, 10.0), #default is 0.1
         'FrustumFarPlaneDist' : hp.uniform ('FrustumFarPlaneDist' , 500, 2000) #default is 1000
  }



  trials = Trials()
  best = fmin(f, space, algo=tpe.suggest, max_evals=10, trials=trials)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     algo=tpe.suggest, max_evals=10, trials=trials)

  full_data= Full_data()
  full_data.apply_params(best)
  rospy.init_node('hyperpySPPTAM')
  file_path = rospy.get_param('~outfile', 'std_msgs/String')
  full_data.write_to_file(file_path)

  print 'best: \n', best
