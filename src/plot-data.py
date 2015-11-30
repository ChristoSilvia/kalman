#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

def smallest_positive(test_list):
  smallest = float('inf')
  smallest_index = None

  for i, x in enumerate(test_list):
    if x < smallest and x > 0:
      smallest = x
      smallest_index = i
  return smallest, smallest_index

depth = 0.7
width = 1.4

state = np.array([0.3, 0.4, 0.03, 0.02])

def next_event(state):
  t_l = state[0]/(-state[2])
  t_r = (width - state[0])/state[2]
  t_u = (depth - state[1])/state[3]
  t_d = state[1]/(-state[3])
  
  return smallest_positive([t_l, t_r, t_u, t_d])

def advance_time(state, t):
  t_next_event, wall_number = next_event(state)

  if t < t_next_event:
    print("No bounce")
    new_state = state
    new_state[:2] += t * state[2:]
    return new_state
  else:
    t_advanced_so_far = 0.0
    bounce_counter = 1
    print("{0} > {1}".format(t-t_advanced_so_far, t_next_event))
    while t - t_advanced_so_far > t_next_event:
      print("bounce {0}".format(bounce_counter))
      t_advanced_so_far += t_next_event

      new_state = state
      new_state[:2] += t_next_event * state[2:]

      if wall_number is 0 or wall_number is 1:
        new_velocity = state[2:]
        new_velocity[0] *= -1
      else:
        new_velocity = state[2:]
        new_velocity[1] *= -1

      new_state[2:] = new_velocity
      
      print("New state: {0}".format(new_state))     
 
      t_next_event, wall_number = next_event(new_state)
      bounce_counter += 1

    print(t - t_advanced_so_far)
    new_new_state = new_state
    new_new_state[:2] += (t - t_advanced_so_far) * new_state[2:]
    print("Resultant state: {0}".format(new_new_state))     
    return new_new_state 

n = 100
dt = 0.4
positions = np.empty((n+1,2))

initial_state = np.array([0.3, 0.4, 0.1, 0.09])
state = initial_state
positions[0,:] = initial_state[:2]

plt.axis([0,width,0,depth])
plt.ion()
plt.show()

for i in xrange(1,n+1):
  state = advance_time(state, dt)
  positions[i,:] = state[:2]
  plt.plot(positions[i-1:i,0],positions[i-1:i,1])
  plt.scatter(positions[i-1:i,0],positions[i-1:i,1])
  plt.draw()

