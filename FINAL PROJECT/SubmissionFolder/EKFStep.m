function [Q_estimate,z,P] = EKFStep(Q_odo,Fq,Fv,Hw,Hq,QNewSym,subsVecSym,subsVecNum,z,P,H,disturbanceCov,sensorCov) 

%Q_odo = double(subs(QNewSym, subsVecSym, subsVecNum)); 
Q_estimate = Q_odo;
P = Fq * P * Fq' + Fv * disturbanceCov * Fv';

nu = z - H * Q_estimate; 
%Hq = eye(3);
S = Hq * P * Hq' + Hw * sensorCov * Hw';  
K = P * Hq' / S; 
%K=eye(3);
Q_estimate = Q_estimate + K * nu;  
Q_process = diag([1e-2, 1e-2, 1e-2]);  % Small artificial noise to prevent P from going to 0
P = (eye(3) - K*Hq) * P * (eye(3) - K*Hq)' + K * sensorCov * K'+ Q_process;  


end