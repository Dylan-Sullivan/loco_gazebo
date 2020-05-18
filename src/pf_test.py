#!/usr/bin/env python

import rospy
import numpy as np
from numpy.random import uniform

from loco_pilot.msg import Command
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Quaternion, Transform, Vector3
from gazebo_msgs.srv import GetWorldProperties, GetLinkState
from tf.transformations import *
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt

x_store=[]
mu_store=[]
cov_store=[]
error_sq_store=[]

initialize=True
est_pos=None
thruster_cmds=None
particles=None
weights=None
rospy_rate=1
ax2=None
old_time=0

# Compute the distance from bottom each particle should read
# based on bathymetry data
def comp_particle_depth(x):
    bath_data=0.25*np.array([0,1,2,3,4,5,6,7,8,7,6,5,4,3,2,1,0,0,0,0,0,0])
    x=int(math.floor(x))
    if x<0:
        dist_from_bot=3
    else:
        dist_from_bot=3-bath_data[x]
    return dist_from_bot

def predict(particles, u, std, dt):
    N = len(particles)
    particles += u * dt + (np.random.randn(N) * std)

def update(particles, weights, z, R):
    #print(z)
    for i in range(len(particles)):
        particle_range=comp_particle_depth(particles[i])
        likelihood=1/abs(z-particle_range)
        weights[i]*=likelihood
    weights += 1.e-100 # avoid round-off to zero
    weights /= sum(weights) # normalize

def estimate(particles, weights):
    mean=0
    var=0
    for i in range(len(particles)):
        mean+=particles[i]*weights[i]
    for i in range(len(weights)):
        var+=(particles[i]-mean)**2*weights[i]

    return mean, var

def simple_resample(particles, weights):
    #print "Particles being resampled."
    N = len(particles)
    cumulative_sum = np.cumsum(weights)
    indexes = np.searchsorted(cumulative_sum, np.random.random_sample((N,)))

    # resample according to indexes
    particles[:] = particles[indexes]
    weights.fill(1.0 / N)

def neff(weights):
    return 1. / np.sum(np.square(weights))

def estimation(data):

    global est_pos
    global thruster_cmds
    global initialize

    global weights
    global particles
    global x_store
    global mu_store
    global cov_store
    global old_time

    # Get actual state of LoCO
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        link_state = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState )
        loco_base=link_state("robot::loco_base_frame","")
        #actual_pos=Vector3(loco_base.link_state.pose.position.x,loco_base.link_state.pose.position.y,loco_base.link_state.pose.position.z)
        actual_posx=loco_base.link_state.pose.position.x
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # Get time property of world
    rospy.wait_for_service('/gazebo/get_world_properties')
    try:
        world_props = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        time=world_props().sim_time
        dt=time-old_time
        old_time=time
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # Thruster command
    thrusters = Command()
    thrusters.pitch=0
    thrusters.yaw=0
    thrusters.throttle=0.5
    equiv_veloc=math.sqrt(46.3*thrusters.throttle/20.578)
    # throttle 0.5 approximate velocity of 1.06 m/s

    # Sonar data
    bath_range=data.ranges[0]

    # Initialize particles if necessary
    N=200 # Number of particles
    if initialize:
        weights = np.ones(N)/N
        particles = np.random.normal(0, 1, N) # mean, std, num
        initialize= False

    # Propagate
    predict(particles, equiv_veloc, .25, dt)
    # Update
    update(particles, weights, bath_range, 0.2)

    # Weights Plot
    # plt.figure(1)
    # plt.clf()
    # ax1=plt.axes()
    # plt.title('Weights')
    # plt.xlabel('Position (m)')
    # plt.ylabel('Weight')
    # for i in range(len(particles)):
    #     ax1.plot(particles[i],weights[i],'k.')


    # Resample if necessary
    if neff(weights) < N/2:
        #print "resampling"
        simple_resample(particles, weights)

    # Estimate position
    mu, var = estimate(particles, weights)
    error_sq_store.append((mu-actual_posx)**2)
    current_rmse=math.sqrt(sum(error_sq_store)/len(error_sq_store))

    # Position Plot
    plt.figure(2)
    ax2=plt.axes()
    plt.title('Estimated vs Actual X Position over Time')
    plt.xlim(0,20)
    plt.ylim(0,20)
    plt.xlabel('Time (s)')
    plt.ylabel('X Position (m)')
    ax2.plot(time,actual_posx,'bx',label='Actual Position')
    ax2.plot(time,mu,'r.',label='Estimated Position')
    plt.legend(loc=2)
    textvar1=plt.text(0.5,16,'Estimation Mean: %f m'%mu)
    textvar2=plt.text(0.5,15,'Estimation Variance: %f m^2'%var)
    textvar3=plt.text(0.5,14,'Total RMSE: %f m'%current_rmse)

    plt.draw()
    plt.pause(0.001)
    textvar1.remove()
    textvar2.remove()
    textvar3.remove()

    # Update data storage
    x_store.append(actual_posx)
    mu_store.append(mu)
    cov_store.append(var)

    # Update position vector to publish
    pos_vector=Vector3(mu,3,0)
    est_pos.publish(pos_vector)

    # Publish thruster command
    thruster_cmds.publish(thrusters)


def filter_func():

    global est_pos
    global thruster_cmds
    global rospy_rate

    rospy.init_node('project_node', anonymous=False)
    rate=rospy.Rate(rospy_rate) # 1 Hz

    est_pos=rospy.Publisher("/est_pos",Vector3, queue_size=1)
    thruster_cmds=rospy.Publisher("/loco/command",Command, queue_size=1)

    rospy.Subscriber("/sonar_data", LaserScan, estimation)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        filter_func()
    except rospy.ROSInterruptException:
        pass

# def create_gaussian_particles(mean, std, N):
#     particles = np.empty((N, 1))
#     particles = mean + (np.random.randn(N) * std)
#     return particles
