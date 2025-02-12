function [q_min,error,Theta_sol] = ClosestRotz(q)
% Based on a section of the repository https://github.com/pslade2/RealTimeKin
% P. Slade, A. Habib, et al., “An open-source and wearable system for
% measuring 3D human motion in real-time,” IEEE Transactions on
% Biomedical Engineering, vol. 69, no. 2, pp. 678–688, 2021.

Theta = -360:0.5:360;
error=inf;
q_min = quaternion.zeros;
for k = 1:length(Theta)
    Rotz = quaternion(cosd(Theta(k)/2),0,0,sind(Theta(k)/2));
    M = min(dist(Rotz,q),[],"all");
    if error>M
        error = M;
        q_min = Rotz;        
        Theta_sol = Theta(k);
    end
end

end