#!/usr/bin/env python 
import rospy
import numpy as np
import math
import os
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion

system_path = rospy.get_param('/system_name','/home/alex/pit-navigator-utah/Simulation')
sun_direction = np.array((0,0,0))

def newMotionPrimitives(outfilename):
    UNICYCLE_MPRIM_16DEGS = 1
    if (UNICYCLE_MPRIM_16DEGS == 1):
        resolution = rospy.get_param('/move_base/global_costmap/resolution', default=0.1)
        numberofangles = 16  #preferably a power of 2, definitely multiple of 8
        numberofprimsperangle = 11 

        #multipliers (multiplier is used as costmult*cost)
        forwardcostmult = 2 
        backwardcostmult = 2 
        forwardandturncostmult = 4 
        stopinplacecostmult = 1 
        turninplacecostmult = 8 
        
        #note, what is shown x,y,theta *changes* (that is, dx,dy,dtheta and not absolute numbers)
        
        #0 degreees
        basemprimendpts0_c = np.zeros((numberofprimsperangle, 4))  #x,y,theta,costmult 
        #angles are positive counterclockwise
        #0 theta change
        basemprimendpts0_c[0,:] =  [ 1, 0, 0, forwardcostmult]
        basemprimendpts0_c[1,:] =  [ 8, 0, 0, forwardcostmult] 
        basemprimendpts0_c[2,:] =  [-1, 0, 0, backwardcostmult]
        basemprimendpts0_c[7,:] =  [-8, 0, 0, backwardcostmult]      
        #1/16 theta change
        basemprimendpts0_c[3,:] =  [ 8, 1, 1, forwardandturncostmult] 
        basemprimendpts0_c[4,:] =  [ 8,-1,-1, forwardandturncostmult] 
        #turn in place
        basemprimendpts0_c[5,:] =  [ 0, 0, 1, turninplacecostmult] 
        basemprimendpts0_c[6,:] =  [ 0, 0,-1, turninplacecostmult] 
        #stop maintaining the same heading
        basemprimendpts0_c[8,:] =  [ 0, 0, 0, stopinplacecostmult] 
        #1/16 theta change going backward
        basemprimendpts0_c[9,:] = [-8,-1, 1, backwardcostmult] 
        basemprimendpts0_c[10,:] = [-8, 1,-1, backwardcostmult] 
        
        #45 degrees
        basemprimendpts45_c = np.zeros((numberofprimsperangle, 4))  #x,y,theta,costmult (multiplier is used as costmult*cost)
        #angles are positive counterclockwise
        #0 theta change 
        basemprimendpts45_c[0,:] =  [ 1, 1, 0, forwardcostmult] 
        basemprimendpts45_c[1,:] =  [ 6, 6, 0, forwardcostmult] 
        basemprimendpts45_c[2,:] =  [-1,-1, 0, backwardcostmult]
        basemprimendpts45_c[7,:] =  [-6,-6, 0, backwardcostmult]      
        #1/16 theta change
        basemprimendpts45_c[3,:] =  [ 5, 7, 1, forwardandturncostmult] 
        basemprimendpts45_c[4,:] =  [ 7, 5,-1, forwardandturncostmult]     
        #turn in place
        basemprimendpts45_c[5,:] =  [ 0, 0, 1, turninplacecostmult] 
        basemprimendpts45_c[6,:] =  [ 0, 0,-1, turninplacecostmult] 
        #stop maintaining the same heading
        basemprimendpts45_c[8,:] =  [ 0, 0, 0, stopinplacecostmult] 
        #1/16 theta change going back
        basemprimendpts45_c[9,:] = [-5,-7, 1, backwardcostmult] 
        basemprimendpts45_c[10,:] = [-7,-5,-1, backwardcostmult]     
        
        #22.5 degrees
        basemprimendpts22p5_c = np.zeros((numberofprimsperangle, 4))  #x,y,theta,costmult (multiplier is used as costmult*cost)
        #angles are positive counterclockwise
        #0 theta change     
        basemprimendpts22p5_c[0,:] =  [ 2, 1, 0, forwardcostmult] 
        basemprimendpts22p5_c[1,:] =  [ 6, 3, 0, forwardcostmult]     
        basemprimendpts22p5_c[2,:] =  [-2,-1, 0, backwardcostmult]
        basemprimendpts22p5_c[7,:] =  [-6,-3, 0, backwardcostmult]       
        #1/16 theta change
        basemprimendpts22p5_c[3,:] =  [ 5, 4, 1, forwardandturncostmult] 
        basemprimendpts22p5_c[4,:] =  [ 7, 2,-1, forwardandturncostmult]     
        #turn in place
        basemprimendpts22p5_c[5,:] =  [ 0, 0, 1, turninplacecostmult] 
        basemprimendpts22p5_c[6,:] =  [ 0, 0,-1, turninplacecostmult] 
        #stops maintaining the same heading
        basemprimendpts22p5_c[8,:] =  [ 0, 0, 0, stopinplacecostmult] 
        #1/16 theta change going back
        basemprimendpts22p5_c[9,:] = [-5,-4, 1, backwardcostmult] 
        basemprimendpts22p5_c[10,:] = [-7,-2,-1, backwardcostmult]     

        
    else:
        print( 'ERROR: undefined mprims type\n') 
        return     
     
    path = outfilename
    if os.path.exists(path):
        os.remove(path)
    fout = open(path, 'a') 


    #write the header
    fout.write('resolution_m: %f\n'% resolution) 
    fout.write('numberofangles: %d\n'% numberofangles) 
    fout.write('totalnumberofprimitives: %d\n'% (numberofprimsperangle*numberofangles)) 

    #iterate over angles
    for angleind in range(numberofangles):
        
        #iterate over primitives    
        for primind in range(numberofprimsperangle):
            fout.write('primID: %d\n'% (primind)) 
            fout.write('startangle_c: %d\n'% (angleind)) 

            #current angle
            currentangle = (angleind)*2*math.pi/numberofangles 
            currentangle_36000int = round((angleind)*36000/numberofangles) 
            
            #compute which template to use
            if (currentangle_36000int%9000 == 0):
                basemprimendpts_c = basemprimendpts0_c[primind,:]     
                angle = currentangle 
                print( '90\n') 
            elif ((currentangle_36000int)% 4500 == 0):
                basemprimendpts_c = basemprimendpts45_c[primind,:] 
                angle = currentangle - 45*math.pi/180 
                print( '45\n') 
            # elif ((currentangle_36000int-7875)% 9000 == 0):
            #     basemprimendpts_c = basemprimendpts33p75_c[primind,:] 
            #     basemprimendpts_c[1] = basemprimendpts33p75_c[primind, 2]  #reverse x and y
            #     basemprimendpts_c[2] = basemprimendpts33p75_c[primind, 1] 
            #     basemprimendpts_c[3] = -basemprimendpts33p75_c[primind, 3]  #reverse the angle as well
            #     angle = currentangle - 78.75*math.pi/180 
            #     print( '78p75\n') 
            elif ((currentangle_36000int-6750)% 9000 == 0):
                basemprimendpts_c = basemprimendpts22p5_c[primind,:].copy() 
                basemprimendpts_c[0] = basemprimendpts22p5_c[primind, 1]  #reverse x and y
                basemprimendpts_c[1] = basemprimendpts22p5_c[primind, 0] 
                basemprimendpts_c[2] = -basemprimendpts22p5_c[primind, 2]  #reverse the angle as well
                #print( '%d %d %d onto %d %d %d\n'% basemprimendpts22p5_c(1), basemprimendpts22p5_c(2), basemprimendpts22p5_c(3), /
                #    basemprimendpts_c(1), basemprimendpts_c(2), basemprimendpts_c(3)) 
                angle = currentangle - 67.5*math.pi/180 
                print( '67p5\n')             
            # elif ((currentangle_36000int-5625)% 9000 == 0):
            #     basemprimendpts_c = basemprimendpts11p25_c[primind,:] 
            #     basemprimendpts_c[1] = basemprimendpts11p25_c[primind, 2]  #reverse x and y
            #     basemprimendpts_c[2] = basemprimendpts11p25_c[primind, 1] 
            #     basemprimendpts_c[3] = -basemprimendpts11p25_c[primind, 3]  #reverse the angle as well
            #     angle = currentangle - 56.25*math.pi/180 
            #     print( '56p25\n') 
            # elif ((currentangle_36000int-3375)% 9000 == 0):
            #     basemprimendpts_c = basemprimendpts33p75_c[primind,:] 
            #     angle = currentangle - 33.75*math.pi/180 
            #     print( '33p75\n') 
            elif ((currentangle_36000int-2250)% 9000 == 0):
                basemprimendpts_c = basemprimendpts22p5_c[primind,:] 
                angle = currentangle - 22.5*math.pi/180 
                print( '22p5\n') 
            # elif ((currentangle_36000int-1125)% 9000 == 0):
            #     basemprimendpts_c = basemprimendpts11p25_c[primind,:] 
            #     angle = currentangle - 11.25*math.pi/180 
            #     print( '11p25\n') 
            else:
                print( 'ERROR: invalid angular resolution. angle = %d\n'% currentangle_36000int) 
                return 
             
            
            #now figure out what action will be        
            baseendpose_c = basemprimendpts_c[0:3] 
            additionalactioncostmult = basemprimendpts_c[3]         
            endx_c = round(baseendpose_c[0]*math.cos(angle) - baseendpose_c[1]*math.sin(angle))         
            endy_c = round(baseendpose_c[0]*math.sin(angle) + baseendpose_c[1]*math.cos(angle)) 
            endtheta_c = (angleind + baseendpose_c[2])% numberofangles 
            endpose_c = [endx_c, endy_c, endtheta_c] 
            
            print( 'rotation angle=%f\n'% (angle*180/math.pi)) 
            
            #if (baseendpose_c[2] == 0 and baseendpose_c[3] == 0):
                #print( 'endpose=%d %d %d\n'% endpose_c(1), endpose_c(2), endpose_c(3)) 
             
            
            #generate intermediate poses (remember they are w.r.t 0,0 (and not
            #centers of the cells)
            numofsamples = 10 
            intermcells_m = np.zeros((numofsamples,3)) 
            if (UNICYCLE_MPRIM_16DEGS == 1):
                startpt = [0, 0, currentangle] 
                endpt = [endpose_c[0]*resolution, endpose_c[1]*resolution, ((angleind + baseendpose_c[2])% numberofangles )*2*math.pi/numberofangles]
                intermcells_m = np.zeros((numofsamples,3)) 
                if ((endx_c == 0 and endy_c == 0) or baseendpose_c[2] == 0): #turn in place or move forward            
                    for iind in range(numofsamples):
                        intermcells_m[iind,:] = [startpt[0] + (endpt[0] - startpt[0])*(iind)/(numofsamples-1), 
                                                startpt[1] + (endpt[1] - startpt[1])*(iind)/(numofsamples-1), 
                                                0] 
                        rotation_angle = (baseendpose_c[2] ) * (2*math.pi/numberofangles) 
                        intermcells_m[iind,2] = (startpt[2] + rotation_angle*(iind/(numofsamples-1.0)))% (2*math.pi) 
                                 
                else: #unicycle-based move forward or backward
                    R = np.array([[math.cos(startpt[2]),  math.sin(endpt[2]) - math.sin(startpt[2])], 
                                  [math.sin(startpt[2]), -math.cos(endpt[2]) + math.cos(startpt[2])]])
                    S = np.dot(np.linalg.pinv(R),np.array([endpt[0] - startpt[0] , endpt[1] - startpt[1]]))
                    l = S[0]  
                    tvoverrv = S[1] 
                    rv = (baseendpose_c[2]*2*math.pi/numberofangles + l/tvoverrv) 
                    tv = tvoverrv*rv 
                            
                    if ((np.all(l < 0) and np.all(tv > 0)) or (np.all(l > 0) and np.all(tv < 0))):
                        print( 'WARNING: l = %d < 0 -> bad action start/ points\n'% l) 
                        l = 0 
                     
                    #compute rv
                    #rv = baseendpose_c(3)*2*math.pi/numberofangles 
                    #compute tv
                    #tvx = (endpt(1) - startpt(1))*rv/(math.sin(endpt(3)) - math.sin(startpt(3)))
                    #tvy = -(endpt(2) - startpt(2))*rv/(math.cos(endpt(3)) - math.cos(startpt(3)))
                    #tv = (tvx + tvy)/2.0               
                    #generate samples
                    for iind in range(numofsamples):                                       
                        dt = (iind)/(numofsamples-1.0) 
                                            
                        #dtheta = rv*dt + startpt(3) 
                        #intermcells_m(iind,:) = [startpt(1) + tv/rv*(math.sin(dtheta) - math.sin(startpt(3))) /
                        #                        startpt(2) - tv/rv*(math.cos(dtheta) - math.cos(startpt(3))) /
                        #                        dtheta] 
                        
                        if(np.all(abs(dt*tv) < abs(l))):
                            intermcells_m[iind,:] = np.array([startpt[0] + dt*tv*math.cos(startpt[2]), startpt[1] + dt*tv*math.sin(startpt[2]), startpt[2]])
                        else:
                            dtheta = rv*(dt - l/tv) + startpt[2] 
                            intermcells_m[iind,:] = np.array([startpt[0] + l*math.cos(startpt[2]) + tvoverrv*(math.sin(dtheta) - math.sin(startpt[2])), startpt[1] + l*math.sin(startpt[2]) - tvoverrv*(math.cos(dtheta) - math.cos(startpt[2])), dtheta])

                    #correct
                    errorxy = [endpt[0] - intermcells_m[numofsamples-1,0] ,
                               endpt[1] - intermcells_m[numofsamples-1,1]] 
                    print( 'l=%f errx=%f erry=%f\n'% (l, errorxy[0], errorxy[1])) 
                    interpfactor = np.linspace(0,1,numofsamples)
                    intermcells_m[:,0] = intermcells_m[:,0] + errorxy[0]*interpfactor.transpose() 
                    intermcells_m[:,1] = intermcells_m[:,1] + errorxy[1]*interpfactor.transpose() 
                                                         
            #write out
            fout.write('endpose_c: %d %d %d\n'% (endpose_c[0], endpose_c[1], endpose_c[2])) 
            fout.write('additionalactioncostmult: %d\n'% additionalactioncostmult) 
            fout.write('intermediateposes: %d\n'% len(intermcells_m[:,0])) 
            for interind in range(len(intermcells_m[:,0])):
                fout.write('%.4f %.4f %.4f\n'% (intermcells_m[interind,0], intermcells_m[interind,1], intermcells_m[interind,2])) 
    fout.close()


def callback(data):
    global sun_direction
    tempx=data.x
    tempy=data.y
    tempz=data.z
    tempw=data.w
    direction = tf.transformations.euler_from_quaternion([tempx,tempy,tempz,tempw])
    error = np.sum(np.sqrt(np.square(sun_direction-np.array(direction))))
    if (error>np.math.pi/8):
        sun_direction = direction
        path = system_path+'/catkin_ws/src/global_planner/sbpl_lattice_planner/sbpl/matlab/mprim/pit_nav.mprim'
        newMotionPrimitives(path)
    
def listener():
    rospy.init_node('MPrimGeneratorNode', anonymous=True)
    rospy.Subscriber("sun_direction", Quaternion, callback)
    rospy.spin()

if __name__ == '__main__':
    newMotionPrimitives("/home/alex/pit-navigator-utah/Simulation/outfilename.mprim")
    listener()