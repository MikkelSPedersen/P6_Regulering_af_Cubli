function  y=siminertia(u,t,par)
% y=simktau(u,t,par) simulates K/(1+s*tau) using lsim
% par=[K tau]
% Example: par=[2 3] inputt='inputsq' f1=.02 maintests
%
% 31/10-02,MK

J_R=par(1); B_R=par(2);

s = tf('s');
l = 0.092;
m_R = 0.412;
%J_R = 5.4*10^-3;
%B_R = 1*10^-3;
m_H = 0.220;
J_H = 0.699*10^-3;
B_H = 1.83*10^-6;
g = 9.82
phi = -44.1;
G = s*phi / (s^2 + ((m_R * l + m_H * l)*g+B_R*s)/(J_R - (l^2 * m_H)))

%t=[0 t(1:length(t)-1)];
%y=lsim(G,u,t);
y=impulse(G,t);
