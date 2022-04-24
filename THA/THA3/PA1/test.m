clc
close all
clear all

%% Goal 1
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

%% Goal 2
% Extract debug file
A_cell = readcell('debug/pa1-debug-a-optpivot.txt');
num_D = cell2mat(A_cell(1,1));
num_H = cell2mat(A_cell(1,2));
num_frame = cell2mat(A_cell(1,3));
A = readmatrix('debug/pa1-debug-a-optpivot.txt');

%% Goal 3
calBody = readcell('debug/pa1-debug-a-calbody.txt');
num_d = cell2mat(calBody(1,1));
num_a = cell2mat(calBody(1,2));
num_c = cell2mat(calBody(1,3));
calBody = readmatrix('debug/pa1-debug-a-calbody.txt')';
calReading = readcell('debug/pa1-debug-a-calreadings.txt');
num_frames = cell2mat(calReading(1,4));
calReading = readmatrix('debug/pa1-debug-a-calreadings.txt')';
idx = 0;
d = zeros(4,num_d);
a = zeros(4,num_a);
d(1:3,:) = calBody(:,1:num_d);
d(4,:) = ones(1,num_d);
a(1:3,:) = calBody(:,num_d+1:num_d+num_a);
a(4,:) = ones(1,num_a);
c = calBody(:,num_d+num_a+1:end);
C_exp = zeros(3,num_frames*num_c);
for i = 1:num_frames
    D = calReading(:,(i-1)*(num_d+num_a+num_c)+1:(i-1)*(num_d+num_a+num_c)+num_d);
    F_D = D*pinv(d);
    F_D = [F_D;0 0 0 1];
    A = calReading(:,num_d+(i-1)*(num_d+num_a+num_c)+1:(i-1)*(num_d+num_a+num_c)+num_d+num_a);
    C = calReading(:,num_d+num_a+(i-1)*(num_d+num_a+num_c)+1:(i-1)*(num_d+num_a+num_c)+num_d+num_a+num_c);
    F_A = A*pinv(a);
    F_A = [F_A;0 0 0 1];
    F = pinv(F_D)*F_A;
    C_exp(:,(i-1)*num_frames+1:(i-1)*num_frames+num_c) = F(1:3,1:3)*c+F(1:3,4);
    C_error = abs(C_exp(:,(i-1)*num_frames+1:(i-1)*num_frames+num_c)-C);
end








