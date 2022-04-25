clc;
clear all;
close all;

[q_Robot, q_camera, t_Robot, t_camera]  = data_quaternion();

[Rx, px, Tx] = handeye_transform(q_Robot, t_Robot, q_camera, t_camera);
