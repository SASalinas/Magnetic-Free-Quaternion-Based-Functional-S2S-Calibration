function [BodyqSensor, WBqWS] = ...
            S2SCalibT(qSUA, interv1, interv2, axisMov)
% Sergio Salinas
% Feb 2025

% 1. BodyqSensor
%Rotation axes 
interv = {interv1, interv2};

for k = 1: width(qSUA) % number of sensors
    ax = zeros(3,length(interv));  %variable to collect the axis vectors

    for i = 1:length(interv)  %rotation axes, should be 2
        
        nInterv = length(interv{i}); %number of time intervals

        switch nInterv 
            case 2 %=> functional motion, should be in the second interval
                qData = qSUA{ interv{i}{2}, k};
                angv = angvel(qData,1,"point");                                 
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
    WSqS_Ref = meanrot([qSUA{ interv{1}{1}, k}; qSUA{ interv{2}{1}, k}], 'omitnan'); %using both static postures

    %gravity from the WS perspective
    ZaxisWSqWS = [0; 0; 1];
    
    %transfering the gravity vector to Sensor frame
    Sgv = (rotatepoint(WSqS_Ref.conj,ZaxisWSqWS'))';  
    
    %Third option SqB
    axisY3 = cross(Sgv,ax(:,1));
    axisZ3 = cross(ax(:,1),axisY3);
    SrB = [ax(:,1)/norm(ax(:,1)) axisY3/norm(axisY3) axisZ3/norm(axisZ3)];
    SqB3 = normalize(quaternion(SrB,'rotmat','point'));

    %Fourth option SqB
    axisX4 = cross(ax(:,2),Sgv);
    axisZ4 = cross(axisX4,ax(:,2));
    SrB = [axisX4/norm(axisX4) ax(:,2)/norm(ax(:,2)) axisZ4/norm(axisZ4)];
    SqB4 = normalize(quaternion(SrB,'rotmat','point'));
   
    %Using initial pose to calculate the relation between references    
    % assuming the body close to the WB so WBqB ~= I
    WBqWSoption = meanrot([SqB1,SqB2,SqB3,SqB4]).conj * WSqS_Ref.conj;
    
    %Estimation of rotation between WB and WS
    [WBqWSideal,~] = ClosestRotz(WBqWSoption);
        
    WBqWS(k) = WBqWSideal;
    BodyqSensor(k) = WBqWS(k) * WSqS_Ref;
end
end