function [BodyqSensor, W1qW2] = ...
            S2SCalibUA(qSUA, interv1, interv2, axisMov, qST, TqS1, WBqW1)
% Estimation of the misalignment BodyqSensor using the angular velocity of
% two rotations around two different axis of that body. 
% 
% Note: data should be collected during two or more intercalated static 
% postures that represent the rotation around that specific axis. 
% For example, N-pose by 5 seconds followed by a static flexion for 5 
% seconds and repeat the secuence for at least 2 times.
% 
% This function assumes that the initial posture is the reference posture
% in order to calculate.
%
% The sensor orientation data qS and the intervals of time where the
% static postures were performed are used to calculate the rotation axes.
%
% Sergio Salinas
% Feb 2025

% 1. BodyqSensor
%Rotation axes using angular velocities
interv = {interv1, interv2};

for k = 1: width(qSUA) % number of sensors
    ax = zeros(3,length(interv));  %variable to collect the axis vectors

    for i = 1:length(interv)  %rotation axes, should be 2
        
        % Do I have several static postures or a functional movement?
        nInterv = length(interv{i}); %number of time intervals

        switch nInterv 
            case 2 %=> functional motion, should be in the second interval
                qData = qSUA{ interv{i}{2}, k};
                angv = angvel(qData,1,"point"); %,qData(1)
                pcs = pca(angv(2:end,:));
                axis = pcs(:,1);

            otherwise  %=> static postures
                 error("At least 2 time intervals are needed. " + ...
                    "The first one should be a static posture.");
        end
       
        ax(:,i) = (axis/norm(axis))';
        if (axisMov{k}{i} == "-x") || (axisMov{k}{i} == "-y")
            ax(:,i) = -ax(:,i);
        end
    end
    
    if (axisMov{k}{1} == "y") || (axisMov{k}{1} == "-y") %first is the axis Y
        ax = [ax(:,2), ax(:,1)];  %reorganized ax
    end

    % LOOKING for all options
    
    %Third axis (Z): perpendicular to the gyro axes
    axisZ = cross(ax(:,1),ax(:,2));

    %First option SqB
    axisX1 = cross(ax(:,2),axisZ);
    SrB = [axisX1/norm(axisX1) ax(:,2)/norm(ax(:,2)) axisZ/norm(axisZ)];
    SqB1 = normalize(quaternion(SrB,'rotmat','point'));

    %Second option SqB
    axisY2 = cross(axisZ,ax(:,1));
    SrB = [ax(:,1)/norm(ax(:,1)) axisY2/norm(axisY2) axisZ/norm(axisZ)];
    SqB2 = normalize(quaternion(SrB,'rotmat','point'));

    %USING N pose
    W2qS2_Ref = meanrot([qSUA{ interv{1}{1}, k}; qSUA{ interv{2}{1}, k}], 'omitnan'); %using both static postures
    
    %transfering the original axes to Npose
    axisX3 = (rotatepoint(W2qS2_Ref,ax(:,1)'))';  %XBody observed from WS
    axisY4 = (rotatepoint(W2qS2_Ref,ax(:,2)'))';  %YBody observed from WS
  
    %gravity from the WS perspective
    ZaxisWSqWS = [0; 0; 1];
    
    %Third option SqB 
    axisY3 = cross(ZaxisWSqWS,axisX3);
    axisZ3 = cross(axisX3,axisY3);
    WSrB = [axisX3/norm(axisX3) axisY3/norm(axisY3) axisZ3/norm(axisZ3)];
    WSqB3 = normalize(quaternion(WSrB,'rotmat','point'));

    %Fourth option SqB 
    axisX4 = cross(axisY4,ZaxisWSqWS);
    axisZ4 = cross(axisX4,axisY4);
    WSrB = [axisX4/norm(axisX4) axisY4/norm(axisY4) axisZ4/norm(axisZ4)];
    WSqB4 = normalize(quaternion(WSrB,'rotmat','point'));

    %Using initial pose to calculate the relation between references
    W1qS1_Ref = meanrot([qST{ interv{1}{1}, k}; qST{ interv{2}{1}, k}], 'omitnan'); %using both static postures
    
    %Using initial pose to calculate the relation between references    
    % assuming the body segments are vertical TqUA ~= I
    UAqWS2option = meanrot([SqB2.conj * W2qS2_Ref.conj, SqB1.conj * W2qS2_Ref.conj, WSqB3.conj, WSqB4.conj]);
                   

    W1qW2option = W1qS1_Ref * conj(TqS1(k)) * UAqWS2option;
    
    [W1qW2ideal,~] = ClosestRotz(W1qW2option);
    
    W1qW2(k) = W1qW2ideal; 
    BodyqSensor(k) = TqS1(k) * W1qS1_Ref.conj * W1qW2(k) * W2qS2_Ref;
end
end