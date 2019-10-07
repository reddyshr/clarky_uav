% IMPORTANT NOTATION
    %rw = right wing
    %lw = left wing
    %rt = right tail
    %lt = left tail

classdef UAV
    properties 
        % STATE, [x y z phi theta psi] [xdot ydot zdot phidot thetadot psidot]
        q = zeros(6,1); 
        qdot = zeros(6,1);
        
        % CONTROL INPUTS
        thrust = 0;
        flapAngles = zeros(4,1); %[rw, lw, rt, lt] degrees
        
        aoa = 0; %degrees
        
        % GEOMETRY
        csArea = [10.05; 10.05; 1.86; 1.86]; %[rwArea, lwArea, rtArea, ltArea]
        
        % 4 x 4 x 4 matrix
        % the first 4 x 4 dimensions are the homogenous transformation
        % matrix. the third dimension consists of each surface frame... rw,
        % lw, rt, lt
        bodyToZero = zeros(4);
         
        ZeroToBody = zeros(4);
        
        RightWingToZero = zeros(4)
        
        RightWingToBody = zeros(4);
             
        LeftWingToZero = zeros(4);
        
        LeftWingToBody = zeros(4);
       
        RightTailToZero = zeros(4);
        
        RightTailToBody = zeros(4);
        
        LeftTailToZero = zeros(4);
        
        LeftTailToBody = zeros(4);
        
        surfaceToBodyTrans = zeros(4);
%         
%         phi = 0;
%         theta = 0;
%         psi = 0;
%         Rx = zeros(3);
%         Ry = zeros(3);
%         Rz = zeros(3);
%         wRb = zeros(3);         
%         
        bodyToWorldTrans = zeros(4);
        
        % INERTIAL CHARACTERISTICS
        %mass -> kg, inertia tensors -> kg*m^2
        mass = 727.06; 
        inertiaMatrix = [3841.14       0         0;
                         0       6954.64         0;
                         0             0   3180.70];
                     
        % AERODYNAMIC CHAACTERISTICS
        
        rho = 1.225; % air density (kg/m^3)
        %Coefficient of Lift and Drag
        %MAYBE EXTRAPOLATE FOR SMALLER ANGLE OF ATTACKS
        minAOA = -5;
        maxAOA = 55;
        CL = zeros(1, 66);
        CD = zeros(1, 66);
         
        flapCorrection = (0.1 / 10); %0.1 change in CL and CD per 10 degrees of flap movement
    end
        
    methods
        
        function ncForces = calculateNCForces(obj)
            %ncForces are the generalized forces along the generalized
            %coordinates, x, y, z, phi, theta, psi
            ncForces = zeros(6,1);
            
            % CALCULATE LIFT AND DRAG FORCES ACTING ON EACH COMPONENT
            %liftForces = [rwLift, lwLift, rtLift, ltLift];
            %dragForces = [rwDrag, lwDrag, rtDrag, ltDrag];
            %csCL = [rwCL, lwCL, rtCL, ltCL];
            %csCD = [rwCD, lwCD, rtCD, ltCD];
            obj.aoa = obj.updateAOA;
%             disp(obj.aoa)
%             disp(obj.q)
            csCL = obj.getCL;
            csCD = obj.getCD;
            v = obj.getFlightSpeed;
            
            liftForces = zeros(4,1);
            dragForces = zeros(4,1);
            
            for i = 1:4
                liftForces(i) = csCL(i)*(obj.rho / 2)*obj.csArea(i)*v*v;
                dragForces(i) = -csCD(i)*(obj.rho / 2)*obj.csArea(i)*v*v;
            end
            
            % FIND TOTAL NC FORCE ACTING ON BODY wrt INERTIAL FRAME
            totalForce = [0; 0; 0];
            for i = 1:4
                tempLiftForce = [0; 0; liftForces(i)];
                tempDragForce = [dragForces(i); 0; 0];
                
                tempWorldLiftForce = obj.bodyToWorldTrans(1:3, 1:3) * (obj.surfaceToBodyTrans(1:3,1:3,i) * tempLiftForce);
                tempWorldDragForce = obj.bodyToWorldTrans(1:3, 1:3) * (obj.surfaceToBodyTrans(1:3,1:3,i) * tempDragForce);
                
                totalForce = totalForce + tempWorldLiftForce + tempWorldDragForce;
            end
                thrustForce = [obj.thrust; 0; 0];
                totalForce = totalForce + obj.bodyToWorldTrans(1:3, 1:3) * thrustForce;
            
            ncForces(1) = totalForce(1);
            ncForces(2) = totalForce(2);
            ncForces(3) = totalForce(3);
            
            %% FIND TOTAL NC MOMENT ACTING On BODY wrt INERTIAL FRAME
            totalBodyMoment = [0; 0; 0];
            for i = 1:4
                surfForce = [dragForces(i); 0; liftForces(i)];
                surfForceBody = (obj.surfaceToBodyTrans(1:3,1:3,i) * surfForce);
                momentArmVect = obj.surfaceToBodyTrans(1:3, 4, i);
                totalBodyMoment = totalBodyMoment + cross(momentArmVect, surfForceBody);
            end
            
            totalWorldMoment = obj.bodyToWorldTrans(1:3, 1:3) * totalBodyMoment;
            ncForces(4) = totalWorldMoment(1);
            ncForces(5) = totalWorldMoment(2);
            ncForces(6) = totalWorldMoment(3);
            
        end
        
        function state = getState(obj)
            state = zeros(12,1);
            state(1:6) = obj.q;
            state(7:12) = obj.qdot;
        end
        
        function compCL = getCL(obj)
            compCL = zeros(4,1);
            a = obj.aoa;
            if (a < obj.minAOA)
                a = obj.minAOA;
            elseif (a > obj.maxAOA)
                a = obj.maxAOA;
            end
            clLower = obj.CL(floor(a) + abs(obj.minAOA) + 1);
            clUpper = obj.CL(ceil(a) + abs(obj.minAOA) + 1);
            cl = ((clUpper - clLower) * (a - floor(a))) + clLower;
            for i = 1:4
                compCL(i) = cl + (obj.flapCorrection * obj.flapAngles(i));
            end            
        end
        
        function compCD = getCD(obj)
            compCD = zeros(4,1);
            a = obj.aoa;
            if (a < obj.minAOA)
                a = obj.minAOA;
                % should we update the obj.aoa here as well? 
            elseif (a > obj.maxAOA)
                a = obj.maxAOA;
            end
            cdLower = obj.CD(floor(a) + abs(obj.minAOA) + 1);
            cdUpper = obj.CD(ceil(a) + abs(obj.minAOA) + 1);
            cd = ((cdUpper - cdLower) * (a - floor(a))) + cdLower;
            for i = 1:4
                %abs of flap angles because drag only increases with
                %nonzero angle regardless of sign
                compCD(i) = cd + (obj.flapCorrection * abs(obj.flapAngles(i)));
            end
        end
        
        function flightSpeed = getFlightSpeed(obj)
            % determine flight speed
            xdot = obj.qdot(1);
            ydot = obj.qdot(2);
            zdot = obj.qdot(3);
            flightSpeed = sqrt(xdot^2 + ydot^2 + zdot^2);
        end
        
        function wRb = updateBodyToWorld(obj)
            phi = obj.q(4);
            theta = obj.q(5);
            psi = obj.q(6);
            Rx = [1        0         0;
                  0 cos(phi) -sin(phi);
                  0 sin(phi)  cos(phi)];
            Ry = [ cos(theta) 0 sin(theta);
                            0 1          0;
                  -sin(theta) 0 cos(theta)];
            Rz = [cos(psi) -sin(psi) 0;
                  sin(psi)  cos(psi) 0;
                         0         0 1];
            wRb = [Rz*Ry*Rx obj.q(1:3); 0 0 0 1];         
        
        end
        
        function a = updateAOA(obj)            
            % update aoa property
            obj.bodyToWorldTrans = obj.updateBodyToWorld;
            wRb = obj.bodyToWorldTrans(1:3,1:3)';
            w_vel = obj.qdot(1:3);
            b_vel = wRb' * w_vel;
%             disp(b_vel)
%             disp(w_vel)
%             disp(wRb')
            % clip the aoa values
            if rad2deg(atan((b_vel(3) / b_vel(1)))) < obj.minAOA
                a = obj.minAOA;
            elseif rad2deg(atan((b_vel(3) / b_vel(1)))) > obj.maxAOA
                a = obj.maxAOA;
            else
                a = rad2deg(atan((b_vel(3) / b_vel(1))));
            end
        end
        
        function obj = UAV(startQ, startQDOT, startFlap, startThrust)
            %initialize UAV state
            obj.q = startQ;
            obj.qdot = startQDOT;
            obj.flapAngles = startFlap;
            obj.thrust = startThrust;
            
            obj.bodyToZero = [0 -1  0      0;
                              0  0 -1   5.43;
                              1  0  0 -61.56;
                              0  0  0      1];
         
            obj.ZeroToBody = [obj.bodyToZero(1:3,1:3)' -obj.bodyToZero(1:3,1:3)' * obj.bodyToZero(1:3,4); 0 0 0 1];
        
            obj.RightWingToZero = [0 1 0   -75;
                               0 0 1 11.65;
                               1 0 0   -25;
                               0 0 0     1];
            obj.RightWingToBody = obj.ZeroToBody * obj.RightWingToZero;

            obj.LeftWingToZero = [0 1 0    75;
                              0 0 1 11.65;
                              1 0 0   -25;
                              0 0 0     1];
            obj.LeftWingToBody = obj.ZeroToBody * obj.LeftWingToZero;

            obj.RightTailToZero = [0 sqrt(3)/2       0.5 -41.22;
                               0      -0.5 sqrt(3)/2  28.53;
                               1         0         0 -277.5;
                               0         0         0      1];
            obj.RightTailToBody = obj.ZeroToBody * obj.RightTailToZero;

            obj.LeftTailToZero = [0 sqrt(3)/2      -0.5  41.22;
                              0       0.5 sqrt(3)/2  28.53;
                              1         0         0 -277.5;
                              0         0         0      1];
            obj.LeftTailToBody = obj.ZeroToBody * obj.LeftTailToZero;

            obj.surfaceToBodyTrans = cat(3, obj.RightWingToBody,  obj.LeftWingToBody, obj.RightTailToBody, obj.LeftTailToBody);

%             phi = obj.q(4);
%             theta = obj.q(5);
%             psi = obj.q(6);
%             Rx = [1        0         0;
%                   0 cos(phi) -sin(phi);
%                   0 sin(phi)  cos(phi)];
%             Ry = [ cos(theta) 0 sin(theta);
%                             0 1          0;
%                   -sin(theta) 0 cos(theta)];
%             Rz = [cos(psi) -sin(psi) 0;
%                   sin(psi)  cos(psi) 0;
%                          0         0 1];
%             wRb = Rz*Ry*Rx;         

%             obj.bodyToWorldTrans = [wRb obj.q(1:3); 0 0 0 1];
            obj.bodyToWorldTrans = obj.updateBodyToWorld;
                
            obj.aoa = obj.updateAOA;
            %initilize CL and CD matrix
            i = 1;
            j = 1;
            for a = -5:60
                if (a >= -5 && a <= 16)
                    obj.CL(i) = 0.0567*a + 0.28;              
                elseif (a > 16 && a <= 25)
                    obj.CL(i) = -.0222*(a-16) + 1.2;                   
                elseif (a > 25 && a <= 26)
                    obj.CL(i) = -0.2*(a - 25) + 1.0;                    
                elseif (a > 26 && a <= 60)
                    obj.CL(i) = -0.0071*(a - 26) + 0.8;                    
                end
                i = i + 1;
            end
                      
            for a = -5:60
                if (a >= -5 && a <= 5)
                    obj.CD(j) = 0.0035*a + 0.025;              
                elseif (a > 5 && a <= 10)
                    obj.CD(j) = .009*(a-5) + 0.045;                   
                elseif (a > 10 && a <= 23)
                    obj.CD(j) = 0.0162*(a - 10) + 0.09;                    
                elseif (a > 23 && a <= 27)
                    obj.CD(j) = 0.0325*(a - 23) + 0.3;  
                elseif (a > 27 && a <= 60)
                    obj.CD(j) = 0.017*(a - 27) + 0.43; 
                end
                j = j + 1;
            end
        end
    end
end