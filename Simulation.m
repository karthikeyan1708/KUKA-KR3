clear all
close all
clc

T = [0 0 120.32];
 
kukakr3Initiation(T)

global kr3
Q = [10 -100 111 166 100 333]
matFK = kukakr3ForwardKinematics(Q)
Qn = kukakr3InverseKinematics(matFK)
kr3Teach(Q)           