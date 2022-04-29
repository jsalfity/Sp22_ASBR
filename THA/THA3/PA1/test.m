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

T = point_registration(D,G);
R = T(1:3,1:3);
p = T(1:3,4);
b = R*D+p;
error = norm(mean(G-b,2));

disp(['Point cloud registration is completed. The norm of the error is ',...
    num2str(error)])

%% Goal 2

%% Goal 3
calBody_cell = readcell('debug/pa1-debug-g-calbody.txt');
num_d = cell2mat(calBody_cell(1,1));
num_a = cell2mat(calBody_cell(1,2));
num_c = cell2mat(calBody_cell(1,3));
calBody = readmatrix('debug/pa1-debug-g-calbody.txt')';
calReading = readcell('debug/pa1-debug-g-calreadings.txt');
num_frames = cell2mat(calReading(1,4));
calReading = readmatrix('debug/pa1-debug-g-calreadings.txt')';
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

disp(['Expected value of C is calculated, and the difference from the distorted location ',...
    'of C is ', num2str(norm(C_error))])

%% Goal 4
emPivot_cell = readcell('debug/pa1-debug-g-empivot.txt');
num_G = cell2mat(emPivot_cell(1,1));
num_frame = cell2mat(emPivot_cell(1,2));
emPivot = readmatrix('debug/pa1-debug-g-empivot.txt')';
emPivot(:,1) = [];

G0 = 1/num_G*sum(emPivot(:,1:num_G),2);
g_j = emPivot-G0;

R = zeros(3*num_frame,3);
P = zeros(3*num_frame,1);

for i = 1:num_frame
    G_j = emPivot(:,(i-1)*num_G+1:(i-1)*num_G+num_G);
    g = g_j(:,(i-1)*num_G+1:(i-1)*num_G+num_G);
    g = [g;ones(1,6)];
    F_G = G_j*pinv(g);
    R((i-1)*3+1:(i-1)*3+3,:) = F_G(1:3,1:3);
    P((i-1)*3+1:(i-1)*3+3,:) = F_G(1:3,4);
end

F = [R P];

[b_post, ~] = pivot_calibration(F);

disp('Pivot calibration of the EM tracking probe. Dimple location is:')
b_post

%% Goal 5
optPivot_cell = readcell('debug/pa1-debug-g-optpivot.txt');
num_D = cell2mat(optPivot_cell(1,1));
num_H = cell2mat(optPivot_cell(1,2));
num_frame = cell2mat(optPivot_cell(1,3));
optPivot = readmatrix('debug/pa1-debug-g-optpivot.txt')';

calBody_cell = readcell('debug/pa1-debug-g-calbody.txt');
num_d = cell2mat(calBody_cell(1,1));
num_a = cell2mat(calBody_cell(1,2));
num_c = cell2mat(calBody_cell(1,3));
calBody = readmatrix('debug/pa1-debug-g-calbody.txt')';

% calculate F_D
D_all = zeros(3,8);
d_all = zeros(4,8);
for i = 1:num_frame
    D = optPivot(:,(i-1)*(num_D+num_H)+1:(i-1)*(num_D+num_H)+num_D);
    d = [calBody(:,1:num_d);ones(1,num_d)];
    D_all = [D_all D];  
    d_all = [d_all d];
end
D_all(:,1:8) = [];
d_all(:,1:8) = [];
F_D = D_all*pinv(d_all);
F_D = [F_D;0 0 0 1];
F_D = inv(F_D);

% Transfer from opt frame to em frame
optPivot_em = F_D(1:3,1:3)*optPivot+F_D(1:3,4);

% Calibration
H0 = 1/num_G*sum(optPivot_em(:,1:num_H),2);
h_j = optPivot_em-H0;

R = zeros(3*num_frame,3);
P = zeros(3*num_frame,1);

for i = 1:num_frame
    H_j = optPivot_em(:,(i-1)*num_H+1:(i-1)*num_H+num_H);
    h = h_j(:,(i-1)*num_H+1:(i-1)*num_H+num_H);
    h = [h;ones(1,6)];
    F_H = H_j*pinv(h);
    R((i-1)*3+1:(i-1)*3+3,:) = F_H(1:3,1:3);
    P((i-1)*3+1:(i-1)*3+3,:) = F_H(1:3,4);
end

F = [R P];

[b_post, ~] = pivot_calibration(F);

disp('Pivot calibration of the optical tracking probe. Dimple location is:')
b_post






