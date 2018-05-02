classdef ProbingData < handle
    %%  By Long Wang, 2015/3/19
    %   This class is used to store ProbingData for experiments for CMU
    properties
        Fail;
        RefPoint; %  the reference point above the surface
        NumPointsInDepth = 10;
        %   The contact information
        ContactBallCenter;  % This will be only used in robot control.
        % This is the center of the EE ball at
        % low force contact.
        ContactSurfNorm;    % This is the contact surface norm at a high force contact
        ContactPoint;    % Apply the stable contact surf norm that was obtained at high force contact on
        % an accurate low force Contact ball center, we
        % have this Contact point in robot world frame
        ContactForce;   % The low force when detecting the contact
        DesiredPos; % The desired positions of EE ball center during probing
        CurrentPos; % The current/actual positon of EE ball center during probing.
        DeformedPoints; % The deformed points is obtained by adding the actual ball center and the actual surface norm measurement
        DeformedForce;  % The force measurement during the probing motion
        ContactFlags;
        % This flag is determined by the controller weather it thinks the robot is in contact
        % If true, that indicates the corresponding DeformedPoints had been
        % applied an offset obtained from the force sensed surface norm
        % If not true, that indicates the corresponding DeformedPoints should be the same as CurrentPos
        % Note that ContactFlag=0 does not mean not in contact, it just
        % meas that the interaction force is smaller than threshold.
        %% all the followings are written in tracker frame
        T_robot2TK = eye(4);
        T_FT2robot = eye(4);
        RefPoint_TK = zeros(3,1); %  the reference point above the surface
        %   The contact information
        ContactBallCenter_TK_tracked = zeros(3,1);
        ContactBallCenter_TK_robkin = zeros(3,1);
        % This will be only used in robot control.
        % This is the center of the EE ball at
        % low force contact.
        ContactSurfNorm_TK_robkin = zeros(3,1); % This is the contact surface norm at a high force contact
        ContactPoint_TK_robkin = zeros(3,1);
        ContactPoint_TK_tracked = zeros(3,1);
        % Apply the stable contact surf norm that was obtained at high force contact on
        % an accurate low force Contact ball center, we
        % have this Contact point in robot world frame
        ContactForce_TK_robkin;   % The low force when detecting the contact
        DesiredPos_TK_robkin; % The desired positions of EE ball center during probing
        CurrentPos_TK_tracked;
        CurrentPos_TK_robkin; % The current/actual positon of EE ball center during probing.
        DeformedPoints_TK_tracked;
        DeformedPoints_TK_robkin;
        % The deformed points is obtained by adding the actual ball center and the actual surface norm measurement
        DeformedForce_TK_robkin;  % The force measurement during the probing motion
        %%  the followings are the target locations associated with each probing point
        Target_1 = zeros(6,1); % 6 x 1, the first 3 are position, the second 3 are unit dir vec
        Target_2 = zeros(6,1);
        CompletionTime;
        FORCE_LIMIT_FLAG = 0;
    end
    methods
        function obj = Apply_Transformation_robot2TK(obj)
            %  Load Transformation
            T_robot2TK = obj.T_robot2TK;
            %%   Apply transformations
            obj.RefPoint_TK = ...
                T_robot2TK(1:3,1:3)*obj.RefPoint+T_robot2TK(1:3,4);
            obj.ContactBallCenter_TK_robkin = ...
                T_robot2TK(1:3,1:3)*obj.ContactBallCenter+T_robot2TK(1:3,4);
            obj.ContactSurfNorm_TK_robkin= ...
                T_robot2TK(1:3,1:3)*obj.ContactSurfNorm;
            obj.ContactPoint_TK_robkin= ...
                T_robot2TK(1:3,1:3)*obj.ContactPoint+T_robot2TK(1:3,4);
            obj.ContactForce_TK_robkin= ...
                T_robot2TK(1:3,1:3)*obj.ContactForce;
            %   for multiple depth points
            obj.DesiredPos_TK_robkin = zeros(obj.NumPointsInDepth,3);
            obj.CurrentPos_TK_robkin = zeros(obj.NumPointsInDepth,3);
            obj.DeformedPoints_TK_robkin = zeros(obj.NumPointsInDepth,3);
            obj.DeformedForce_TK_robkin = zeros(obj.NumPointsInDepth,3);
            for i=1:obj.NumPointsInDepth
                obj.DesiredPos_TK_robkin(i,:)= ...
                    (T_robot2TK(1:3,1:3)*obj.DesiredPos(i,:)' + T_robot2TK(1:3,4))';
                obj.CurrentPos_TK_robkin(i,:) = ...
                    (T_robot2TK(1:3,1:3)*obj.CurrentPos(i,:)'+ T_robot2TK(1:3,4))';
                obj.DeformedPoints_TK_robkin(i,:) = ...
                    (T_robot2TK(1:3,1:3)*obj.DeformedPoints(i,:)'+ T_robot2TK(1:3,4))';
                obj.DeformedForce_TK_robkin(i,:) = ...
                    (T_robot2TK(1:3,1:3)*obj.DeformedForce(i,:)')';
            end
        end
    end
    
end

