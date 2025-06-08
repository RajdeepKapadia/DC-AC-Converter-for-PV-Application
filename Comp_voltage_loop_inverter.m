clc;
clear all;
close all;

%% Given 

% Plant Parameters 
L = 1.15e-3; 
C = 1.5e-6;
Rl = 57.6;

% PI Parameters
kp = 80e-6; 
ti = 0.5; 

%% Defining frequency vector in Hertz up to 50 kHz 
freq_Hz = logspace(0, log10(500000), 1000); % Frequency in Hertz

%% Convert frequency from Hertz to rad/s
freq_rad_s = 2 * pi * freq_Hz; % Conversion to radians per second

%% Transfer function - current loop Gid approximation 

s = tf('s'); 

% Numerator 
 N = 1; 

% Denominator 
D = C*s; 

Gp = (N/D); 

%% Transfer function of PI controller 

num = [kp*ti kp];
den = [ti 0]; 

Gc = tf(num, den); 

%% Compensated System
G = Gp*Gc; 

%% Bode Plot

% Retrieve Bode plot data
[mag, phase, wout] = bode(G, freq_rad_s);

% Convert frequencies from rad/s to Hz
freq_Hz_from_bode = wout / (2 * pi);

% Plot magnitude and phase on separate plots
figure;
subplot(2,1,1);
semilogx(freq_Hz_from_bode, 20*log10(squeeze(mag)));
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Gain Plot')
grid on;

subplot(2,1,2);
semilogx(freq_Hz_from_bode, squeeze(phase));
xlabel('Frequency (Hz)');
ylabel('Phase (degrees)');
title ('Phase Plot')
grid on;
