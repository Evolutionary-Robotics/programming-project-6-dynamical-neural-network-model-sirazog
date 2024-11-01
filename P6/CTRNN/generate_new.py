# -*- coding: utf-8 -*-
"""
Created on Tue Oct 22 22:28:26 2024

@author: herzogag
"""

import pyrosim.pyrosim as ps

# Global parameters
l = 1 # length
w = 1 # width 
h = 1 # height 

new_l = 2
new_w = 1
new_h = 1

leg_l = 0.5
leg_w = 0.5
leg_h = 1

x = 0
y = 0
z = 1

new_z = 3

    
def Create_Robot():
    ps.Start_URDF("body.urdf")
    ps.Send_Cube(name="Torso",pos=[x,y,z],size=[l,w,h]) # Parent
    ps.Send_Joint(name="Right_Torso", parent="Torso", child="Right_Arm", type="revolute", position = [0.5,0,0.5])
    ps.Send_Cube(name="Right_Arm", pos = [1, 0.5, 0.25], size = [2, 0.5, 0.5])
    ps.Send_Joint(name = "Left_Torso", parent ="Torso", child="Left_Arm", type ="revolute", position = [-0.5, 0, 0.5])  
    ps.Send_Cube(name="Left_Arm", pos = [-1, 0.5, 0.25], size = [2, 0.5, 0.5])
    ps.End()
# =============================================================================
#     ps.Start_URDF("body.urdf")
#     ps.Send_Cube(name = "Torso", pos = [x,y,new_z], size = [new_l, new_w, new_h])
#     ps.Send_Joint(name = "Front_Left", parent = "Torso", child="Front_Left_Leg", type = "revolute", position = [0.1, 0.05, 1])
#     ps.Send_Cube(name = "Front_Left_Leg", pos = [0.9, 0.4, 1], size = [leg_l, leg_w, leg_h])
#     ps.Send_Joint(name = "Front_Right", parent = "Torso", child="Front_Right_Leg", type = "revolute", position = [0.1, -0.05, 1])
#     ps.Send_Cube(name = "Front_Right_Leg", pos = [0.9, -0.4, 1], size = [leg_l, leg_w, leg_h])
#     ps.Send_Joint(name = "Rear_Left", parent = "Torso", child="Rear_Left_Leg", type = "revolute", position = [-0.1, 0.05, 1])
#     ps.Send_Cube(name = "Rear_Left_Leg", pos = [-0.9, 0.4, 1], size = [leg_l, leg_w, leg_h])
#     ps.Send_Joint(name = "Rear_Right", parent = "Torso", child="Rear_Right_Leg", type = "revolute", position = [-0.1, -0.05, 1])
#     ps.Send_Cube(name = "Rear_Right_Leg", pos = [-0.9, -0.4, 1], size = [leg_l, leg_w, leg_h])
#     ps.End()
# =============================================================================
    
Create_Robot()