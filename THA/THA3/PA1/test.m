clc
close all
clear all

% Extract debug file
A = readmatrix('debug/pa1-debug-a-output1.txt');
A(1,:) = [];
A = A';
B = readmatrix('debug/pa1-debug-b-output1.txt');
B(1,:) = [];
B = B';
C = readmatrix('debug/pa1-debug-c-output1.txt');
C(1,:) = [];
C = C';
D = readmatrix('debug/pa1-debug-d-output1.txt');
D(1,:) = [];
D = D';
E = readmatrix('debug/pa1-debug-e-output1.txt');
E(1,:) = [];
E = E';
F = readmatrix('debug/pa1-debug-f-output1.txt');
F(1,:) = [];
F = F';
G = readmatrix('debug/pa1-debug-g-output1.txt');
G(1,:) = [];
G = G';

T = point_registration(A,B);
R = T(1:3,1:3);
p = T(1:3,4);
b = R*A+p;
error = norm(mean(B-b,2));

