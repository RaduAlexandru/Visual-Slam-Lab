#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements: 
# sudo apt-get install python-argparse

"""
This script computes the absolute trajectory error from the ground truth
trajectory and the estimated trajectory.
"""

import sys
import numpy
import argparse
import associate
import trajectory_format_changer
import quat_lib
   
def align_func(data, rot, trans, scale):

  return rot * (scale * (data - data[:, 0]) + data[:, 0]) + trans

def align(model,data,estimate_scale=False, estimate_shift_rotation=False):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    scale -- scaling factor to align both point sets (1x1)
    
    Estimated model: data = scale * rot * model  + trans
    """
    
    numpy.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    model_firstcentered = model - model[:,0]
    data_firstcentered = data - data[:,0]
    
    
    
    
    W = numpy.zeros( (3,3) )
    SP = 0
    N = 0
    
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
        SP += model_firstcentered[:,column].transpose() * data_firstcentered[:,column]
        N += model_firstcentered[:,column].transpose() * model_firstcentered[:,column]
    
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    S = numpy.matrix(numpy.identity( 3 ))
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    
    if estimate_scale:
      scale = float(SP / N);
    else:
      scale = 1.
      
    if estimate_shift_rotation: # estimate_scale benoetigt die SVD der Kovarianzmatrix auch
      rot = U*S*Vh
      trans = data.mean(1) - scale * rot * model.mean(1)
    else:
      rot = numpy.identity(3)
      trans = numpy.zeros((3, 1))

    model_aligned = align_func(model, rot, trans, scale)
    print(trans)
    print(model_aligned.mean(1))
    print(data.mean(1))

    alignment_error = model_aligned - data
    
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
    
    print(trans_error)
        
    return rot,trans,trans_error, scale

def plot_traj(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib. 
    
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    
    """
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    z = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
            z.append(traj[i][2])
        elif len(x)>0:
            ax.plot(x,y,z,style,color=color,label=label)
            label=""
            x=[]
            y=[]
            z=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,z,style,color=color,label=label)
            

if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: [timestamp] tx ty tz qx qy qz qw) (alternative format [R, t]: [timestamp] r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz), whether or not a timestamp must be present is controlled by the argument --no_timestamps')
    parser.add_argument('second_file', help='estimated trajectory (format: [timestamp] tx ty tz qx qy qz qw) (alternative format [R, t]: [timestamp] r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz), whether or not a timestamp must be present is controlled by the argument --no_timestamps')
    parser.add_argument('--no_timestamps', help='indicates that the input file does not have a timestamp in the first column, associating is done via line numbers',\
      action='store_const', const=True, default=False)
    parser.add_argument('--ignore_timestamps', help='indicates that the input file does have a timestamp in the first column but that line numbers should be used for association',\
      action='store_const', const=True, default=False)
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0, type=float)
    parser.add_argument('--assume_scale', help='scaling factor for the second trajectory (default: 1.0), the first trajectory point is assumed as origin',default=1.0, type=float)
    parser.add_argument('--estimate_scale', \
      help='estimate appropriate scaling factor for second trajectory to optimally align to the first trajectory (cannot be combined with --scale or --estimate_scale_shift_rotation or --estimate_shift_rotation), the first trajectory point is assumed as origin', \
      action='store_const', const=True, default=False)
    parser.add_argument('--estimate_shift_rotation', help='estimate appropriate scaling factor for second trajectory to optimally align to the first trajectory (cannot be combined with --estimate_scale_shift_rotation or --estimate_scale)', \
      action='store_const', const=True, default=False)
    parser.add_argument('--align_frame_start', help='index of point of the trajectory that is the first to be used to estimate the rigid or similarity transform between the two trajectories',default=0, type=int)
    parser.add_argument('--align_frame_end', help='index of point of the trajectory that is the last to be used to estimate the rigid or similarity transform between the two trajectories',default=-1, type=int)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02, type=float)
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    parser.add_argument('--show_plot', help='plot the first and the aligned second trajectory to a window', \
      action='store_const', const=True, default=False)
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()
    
    if (args.show_plot and args.plot == None):
      sys.exit("You cannot use --show_plot without --plot")
    
    if (args.estimate_scale and args.assume_scale != 1.):
      sys.exit("You cannot combine the options --estimate_scale and --scale")
      
    first_list = associate.read_file_list(args.first_file, bool(args.no_timestamps), args.ignore_timestamps)
    second_list = associate.read_file_list(args.second_file, bool(args.no_timestamps), args.ignore_timestamps)
    
    if args.align_frame_end == -1: # keine Angabe, dann wird davon ausgegangen, dass die ganze Trajektorie alignt werden soll
      args.align_frame_end = len(first_list)

    
    
    matches = associate.associate(first_list, second_list, float(args.offset),float(args.max_difference))    

    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence? Do you have timestamps (see argument --no_timestamps)?")

    for a,b in matches:
      if len(first_list[a]) > 8:
        tmp = trajectory_format_changer.convert_rotmat_format_to_quat_format([0] + first_list[a])
        first_list[a] = tmp[1:];
      if len(second_list[b]) > 8:
        tmp = trajectory_format_changer.convert_rotmat_format_to_quat_format([0] + second_list[b])
        second_list[b] = tmp[1:];
        
    first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
    second_xyz = numpy.matrix([[float(value)*float(args.assume_scale) for value in second_list[b][0:3]] for a,b in matches]).transpose()

    rot,trans, _, scale = align(second_xyz[:, args.align_frame_start:args.align_frame_end], \
      first_xyz[:, args.align_frame_start:args.align_frame_end], args.estimate_scale, args.estimate_shift_rotation)
     
    quat = quat_lib.rotmatrix_to_quat(rot)
    
    print(args.first_file + " = (s * R(q) * " + args.second_file + " + t)")
    print("estimated rigid transform: tx ty tz qx qy qz qw:\nt = %f %f %f\nq = %f %f %f %f" % (trans[0], trans[1], trans[2], quat[0], quat[1], quat[2], quat[3]) )
    print("given / estimated scale:%f" % scale)
    
    second_xyz_aligned = align_func(second_xyz, rot, trans, scale)
    #traj_len = 0.
    #for (prev_point, point) in zip( first_xyz.transpose()[0::10], first_xyz.transpose()[10::10] ):
    #  traj_len += numpy.linalg.norm( prev_point - point )
      
    trans_error = numpy.linalg.norm(first_xyz - second_xyz_aligned, axis=0)
    
    first_stamps = first_list.keys()
    first_stamps.sort()
    first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()
    
    second_stamps = second_list.keys()
    second_stamps.sort()
    second_xyz_full = numpy.matrix([[float(value) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
    
    second_xyz_full_aligned = align_func(second_xyz_full, rot, trans, scale)
    
    if args.verbose:
        print "compared_pose_pairs %d pairs"%(len(trans_error))

        print "len of trajectory %f m" % traj_len
        print "absolute_translational_error.rmse %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        print "absolute_translational_error.mean %f m"%numpy.mean(trans_error)
        print "absolute_translational_error.median %f m"%numpy.median(trans_error)
        print "absolute_translational_error.std %f m"%numpy.std(trans_error)
        print "absolute_translational_error.min %f m"%numpy.min(trans_error)
        print "absolute_translational_error.max %f m"%numpy.max(trans_error)
        print "absolute_translational_error.rmse / len of trajectory %f" % (numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)) / traj_len)
    else:
        print "absolute_translational_error.rmse %f m" % (numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)))
        print "absolute_translational_error.mean %f m" % numpy.mean(trans_error)
        
    if args.save_associations:
        file = open(args.save_associations,"w")
        file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A)]))
        file.close()
        
    if args.save:
        file = open(args.save,"w")
        file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps,second_xyz_full_aligned.transpose().A)]))
        file.close()
        
    if args.plot:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        import matplotlib.pylab as pylab
        from mpl_toolkits.mplot3d import Axes3D
        from matplotlib.patches import Ellipse
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plot_traj(ax,first_stamps,first_xyz_full.transpose().A,'-',"darkgreen","ground truth")
        plot_traj(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"blue","estimated")
        
        # label="difference"
        #for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
            # print( "Time diff: %f" % numpy.abs(b - a) )
            #ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
            # label="" 
            
        ax.legend()
            
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.savefig(args.plot,dpi=90)
        
    if args.show_plot:
        fig.show();
        input("Press Enter to continue...")
        

        