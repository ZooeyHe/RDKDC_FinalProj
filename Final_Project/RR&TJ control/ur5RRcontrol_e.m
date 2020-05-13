function finalerr = ur5RRcontrol_e(gdesired,k,ur5)
% Implement a discrete-time resolved rate control system.
% The output is the error of orientation and position

[rows, cols] = size(gdesired);
if ((rows ~= 4) | (cols ~= 4))
  error('ur5RRcontrol requires a 4x4 matrix. Check your dimensions.');
end

q_offset=[0 -pi/2 0 -pi/2 0 0]';
q_read=ur5.get_current_joints();
q=q_read-q_offset;
t=0.2;

% Setup the threshold
v_threshold=0.05;
omega_threshold=pi/180;

% Initialization
gt_star_t0 = inv(gdesired)*ur5FwdKin(q);
xi0=getXi(gt_star_t0);
v_err=xi0(1:3,1);
omega_err=xi0(4:6,1);

while norm(v_err)>=v_threshold || norm(omega_err)>=omega_threshold
    gst = ur5FwdKin(q);
    gt_star_t = inv(gdesired)*gst;
    xi=getXi(gt_star_t);
    v_err=xi(1:3,1);
    omega_err=xi(4:6,1);
    J=ur5BodyJacobian(q);
    
% check singularity
    if det(J)<=1e-10
        finalerr=-1;
        return
    end
    
% calculate join angles of next step
    q_next=q-k*t*inv(J)*xi;
    
% restrict the angle from -pi to pi
    for i=1:6
        while q_next(i,1)>pi
            q_next(i,1)=q_next(i,1)-2*pi;
        end
        while q_next(i,1)<-pi
            q_next(i,1)=q_next(i,1)+2*pi;
        end
    end
    
    ur5.move_joints(q_next+q_offset,1);
    pause(1.05)
    q=q_next;
end

g_final=ur5FwdKin(q);
R_final=g_final(1:3,1:3);
R_d=gdesired(1:3,1:3);
finalerr(1,1)=sqrt(trace((R_final-R_d)*(R_final-R_d)'));
finalerr(2,1)=norm(v_err);


end

