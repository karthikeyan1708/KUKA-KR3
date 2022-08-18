
function angleDH = kukakr3InverseKinematics(mat)

global kr3


if ~exist('mat','var')
    error('Parameters not informed');
end

fprintf('calculating inverse kinematics\n');

L = kr3.links;
F = kr3.tool.T;


d1 = L(1).d;
a1 = L(1).a;
a2 = abs(L(2).a);
a3 = abs(L(3).a);
d4 = abs(L(4).d);
d6 = abs(L(6).d);
f  = F(3,4);

o1 = L(1).offset;


tra = mat(1:3,4);
rot = mat(1:3,1:3);


p04 = tra-(d6+f)*rot(1:3,3);


t_1aux_1 = atan2(p04(1,1), -p04(2,1));

t_1aux = [t_1aux_1, t_1aux_1+pi, t_1aux_1-pi];


p = 0;
for i1=1:3
    t_1 = t_1aux(i1);
    
    if (t_1 > L(1).qlim(1)) && (t_1 < L(1).qlim(2))
        
        
        p01 = [a1*cos(t_1+o1) a1*sin(t_1+o1) d1]'; 
        p14 = p04-p01;
        
        r01 = [ cos(t_1 + pi/2),  0, -sin(t_1 + pi/2);
            sin(t_1 + pi/2),  0,  cos(t_1 + pi/2);
            0, -1,                   0];  
        r10 = r01';
        p14r = r10*p14;
        
       
        alpha = atan2(a3,d4);
      
        l4 = sqrt(d4^2+a3^2);
        
        beta = acos((-a2^2-l4^2+norm(p14)^2)/(-2*a2*l4));
       
        t_3aux_1 = pi-(beta-alpha);
        t_3aux_2 = pi-(beta+alpha);
        t_3aux = [t_3aux_1, t_3aux_2, -t_3aux_1, -t_3aux_2];
        
        
        gamma = atan2(-p14r(2), -p14r(1));
        omega = acos((-a2^2-norm(p14)^2+l4^2)/(-2*a2*norm(p14)));
       
        t_2aux_1 = (gamma-omega);
        t_2aux_2 = (gamma+omega);
        t_2aux = [t_2aux_1, t_2aux_2, -t_2aux_1, -t_2aux_2];
        
        for i3=1:4
            t_3 = t_3aux(i3);
            
            if (t_3 > L(3).qlim(1)) && (t_3 < L(3).qlim(2))
                for i2=1:4
                    t_2 = t_2aux(i2);
                    
                    if (t_2 > L(2).qlim(1)) && (t_2 < L(2).qlim(2))
                        
                       
                        T1 = rotz(t_1+L(1).offset)*tran([L(1).a,0,L(1).d])*rotx(L(1).alpha);
                        T2 = rotz(t_2+L(2).offset)*tran([L(2).a,0,L(2).d])*rotx(L(2).alpha);
                        T3 = rotz(t_3+L(3).offset)*tran([L(3).a,0,L(3).d])*rotx(L(3).alpha);
                        T4 = tran([L(4).a,0,L(4).d]);
                        T03 = T1*T2*T3*T4;
                        vet = T03(1:3,4);
                        
                        
                        if round(vet,2) == round(p04,2)
                           
                            p = p+1;
                            if p==1
                                Qp = [t_1, t_2, t_3];
                            else
                                Qp = [Qp; t_1, t_2, t_3];
                            end
                        end
                    end
                end
            end
        end
    end
end

if p==0
    error('No solution found');
end


r = 0;
for ip=1:p
    t_1 = Qp(ip,1);
    t_2 = Qp(ip,2);
    t_3 = Qp(ip,3);
    
    T1 = rotz(t_1+L(1).offset)*tran([L(1).a,0,L(1).d])*rotx(L(1).alpha);
    T2 = rotz(t_2+L(2).offset)*tran([L(2).a,0,L(2).d])*rotx(L(2).alpha);
    T3 = rotz(t_3+L(3).offset)*tran([L(3).a,0,L(3).d])*rotx(L(3).alpha);
    T4 = rotz(L(4).offset); 
    T03 = T1*T2*T3*T4;
    
    
    R03 = T03(1:3,1:3);
    R3t = R03\rot;
    
    
    t_4 = atan2(-R3t(2,3),-R3t(1,3));
    t_5 = atan2(sqrt(R3t(2,3)^2+R3t(1,3)^2),-R3t(3,3));
    t_6 = atan2(R3t(3,2),R3t(3,1));
    
   
    if (t_4 > L(4).qlim(1)) || (t_4 < L(4).qlim(2)) || ...
            (t_5 > L(5).qlim(1)) || (t_5 < L(5).qlim(2)) || ...
            (t_6 > L(6).qlim(1)) || (t_6 < L(6).qlim(2))
   
        if ip==1 && r==0
            r = r+1;
            Qr = [t_4 t_5 t_6];
            Qn = [Qp(ip,:) Qr];
        else
            r = r+1;
            Qr = [t_4 t_5 t_6];
            Qn = [Qn; Qp(ip,:) Qr];
        end
        
        
        t_4 = atan2(R3t(2,3),R3t(1,3));
        t_5 = atan2(-sqrt(R3t(2,3)^2+R3t(1,3)^2),-R3t(3,3));
        t_6 = atan2(-R3t(3,2),-R3t(3,1));
        
        
        if (t_4 > L(4).qlim(1)) || (t_4 < L(4).qlim(2)) || ...
                (t_5 > L(5).qlim(1)) || (t_5 < L(5).qlim(2)) || ...
                (t_6 > L(6).qlim(1)) || (t_6 < L(6).qlim(2))
            
            if ip==1 && r==0
                r = r+1;
                Qr = [t_4 t_5 t_6];
                Qn = [Qp(ip,:) Qr];
            else
                r = r+1;
                Qr = [t_4 t_5 t_6];
                Qn = [Qn; Qp(ip,:) Qr];
            end
        end
    else
        
       
        t_4 = atan2(R3t(2,3),R3t(1,3));
        t_5 = atan2(-sqrt(R3t(2,3)^2+R3t(1,3)^2),-R3t(3,3));
        t_6 = atan2(-R3t(3,2),-R3t(3,1));
        
        
        if (t_4 > L(4).qlim(1)) || (t_4 < L(4).qlim(2)) || ...
                (t_5 > L(5).qlim(1)) || (t_5 < L(5).qlim(2)) || ...
                (t_6 > L(6).qlim(1)) || (t_6 < L(6).qlim(2))
           
            if ip==1 && r==0
                r = r+1;
                Qr = [t_4 t_5 t_6];
                Qn = [Qp(ip,:) Qr];
            else
                r = r+1;
                Qr = [t_4 t_5 t_6];
                Qn = [Qn; Qp(ip,:) Qr];
            end
        end
    end
end

if r==0
    error('No solution found');
end


fprintf('%2i solutions found!\n', r);
angleDH = round(Qn*180/pi,4);


    function H = rotz(alpha)
        
        H = [cos(alpha), -sin(alpha), 0, 0;
            sin(alpha),  cos(alpha), 0, 0;
            0,           0, 1, 0;
            0,           0, 0, 1];
        function H = rotx(gama)
            
            H = [1,         0,          0, 0;
                0, cos(gama), -sin(gama), 0;
                0, sin(gama),  cos(gama), 0;
                0,         0,          0, 1];
            function H = tran(vet)
                
                H = [1, 0,  0, vet(1,1);
                    0, 1,  0, vet(1,2);
                    0, 0,  1, vet(1,3);
                    0, 0,  0,        1];