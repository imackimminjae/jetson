clear;
close all;
clc;

%% ========================= ROS domain =========================
rosDomainId = "1";
setenv("ROS_DOMAIN_ID", rosDomainId);

%% ========================= Settings =========================
% One-click experiment selection. Change only scaleMode for model/full-size.
scaleMode = "fullscale";             % "model" | "fullscale"
mapName = "knu";         % customroad3, shiftedfork, threeway,
                                  % strictfork, splitroundabout, hole,
                                  % roundabout3x3, knu
routeName = "scenario1";             % KNU: scenario1 ... scenario21; section1 ... section17

% ROS batch runner owns path selection and outcome decisions.
enableKnuBatch = true;
activeBatchToken = "";
setappdata(0,"knu_batch_request",struct());

% Vehicle state source.
% "px4_map" uses the map-aligned nav_msgs/Odometry from the Jetson bridge.
% "px4_sih_raw" retains the old MATLAB first-sample alignment as fallback.
poseSource = "px4_map";           % "px4_map" | "px4_sih_raw" | "motive"

Ts = 0.1;                       % MATLAB loop rate [s]
gridRepublishSec = 0.10;        % camera_module() is heavy. Start with 0.5 s.
fullPathRepublishSec = 2.0;     % global path heartbeat period [s]
% Keep false until the Jetson node ignores identical-path heartbeats without
% resetting waypoint progress. Initial path is still published three times.
enablePathHeartbeat = false;

% Published topics.
gridMapTopic = "/grid_map";
globalPathTopic = "/debug/global_path";

% Map-aligned Motive odometry. Custom convention: orientation.z is yaw [rad].
px4MapOdomTopic = "/motive/vehicle/odom_map";       % nav_msgs/Odometry

% Legacy raw SIH fallback. Do not enable MATLAB publication while the
% Jetson bridge is publishing px4MapOdomTopic, or two publishers will race.
px4SihRawPoseTopic = "/px4/sih/odom";        % PoseStamped, orientation.z=yaw
publishMatlabAlignedOdom = false;

% Motive pose input.
motivePoseTopic = "/motive/vehicle/pose";
poseTimeoutSec = 0.50;

% Optional MPC trace input.
% Runtime 주행 중 렉을 줄이려면 false 유지.
% 조향 원인 분석할 때만 true.
mpcTraceTopic = "/debug/lower_mpc_trace";   % current lower node, 30 fields
enableMpcTraceSub = false;

% Upper-planner dense reference path consumed by the current lower controller.
% The lower controller uses the full path; this lightweight log stores only
% poses(1:2) as a quick visualization of the path head.
lowerReferencePathTopic = "/planner/lower_reference_path";  % nav_msgs/Path
enableLowerReferencePathSub = true;

% Body-frame OccupancyGrid output.
% tracking_control uses body_to_grid(p_body), so /grid_map must be BODY frame.
gridFrameID = "base_link";      % change to "Vehicle" if tracking_control expects Vehicle

% Bring-up helper:
% true이면 Motive pose가 아직 없어도 초기 ego pose 기준 grid를 1회 publish한다.
% 실제 주행에서는 tracking node의 pose guard를 유지하는 것이 안전하다.
publishInitialGridBeforePose = true;

% Pose correction from Motive frame to MATLAB map frame.
% Keep zero/false if Motive coordinates already match map_gen coordinates.
posePositionScale = 1.0;
poseXOffset = 0.0;
poseYOffset = 0.0;
poseZOffset = 0.0;
poseYawOffsetRad = 0.0;

poseSwapXY = false;
poseInvertX = false;
poseInvertY = false;

% Velocity estimate from Motive pose difference.
poseSpeedLpfAlpha = 0.4;
poseMaxDtForSpeed = 0.50;

%% ========================= Runtime load reduction =========================
% 실시간성 우선이면 아래처럼 사용:
%   enablePoseLog = false;
%   enableLoopLog = false;
%   enableFigure = false;
%   enableMpcTraceSub = false;
%
% 분석용이면 필요한 항목만 true.
enableRosLog = true;
enablePoseLog = true;
enableMpcTraceLog = false;      % enableMpcTraceSub=true일 때만 의미 있음
enableLoopLog = false;
enableLowerReferencePathLog = true;  % enableLowerReferencePathSub=true일 때만 의미 있음

enableFigure = true;            % PX4 SIH 실시간 차량 위치 표시
enableVehicleWheels = true;     % 네 바퀴 표시 (앞바퀴 조향은 pose로 추정)
enableGridDebugFigure = true;   % Figure 2: generated body-frame grid
plotEveryN = 3;                 % enableFigure=true일 때만 적용
enableAnimationGif = true;      % Figure 1 주행 장면을 GIF로 즉시 저장
animationGifPlaybackSpeed = 4.0; % 4.0 = 실시간 대비 4배속 재생
animationGifCaptureEveryN = plotEveryN; % main-loop N회마다 한 프레임 캡처
printEveryN = 10;
printPathPublish = true;
printGridPublishEveryMessage = true;

%% ========================= Logging settings =========================
logRootDir = fullfile(pwd, "matlab_ros_logs");
logSessionName = char(datetime("now", "Format", "yyyyMMdd_HHmmss_SSS"));
logDir = fullfile(logRootDir, logSessionName);
logFileDir = logDir;
if enableKnuBatch
    % Header templates only; actual samples go into per-route/segment folders.
    logFileDir = fullfile(logDir,"_headers");
end

poseLogFile = "";
mpcTraceLogFile = "";
loopLogFile = "";
lowerReferencePathLogFile = "";
animationGifFile = "";
animationGifInitialized = false;
animationGifFrameCount = 0;

if enableAnimationGif && ~enableFigure
    warning("MATLAB:animationGifDisabled", ...
        "enableAnimationGif requires enableFigure=true. GIF recording is disabled.");
    enableAnimationGif = false;
end
assert(animationGifPlaybackSpeed>0 && isfinite(animationGifPlaybackSpeed), ...
    "animationGifPlaybackSpeed must be positive and finite.");
assert(animationGifCaptureEveryN>=1 && ...
    animationGifCaptureEveryN==floor(animationGifCaptureEveryN), ...
    "animationGifCaptureEveryN must be a positive integer.");
assert(mod(animationGifCaptureEveryN,plotEveryN)==0, ...
    "animationGifCaptureEveryN must be a multiple of plotEveryN.");

if enableRosLog || enableAnimationGif
    if ~exist(logFileDir, "dir")
        mkdir(logFileDir);
    end
end

if enableAnimationGif
    animationGifFile = fullfile(logDir,"drive_animation.gif");
end

if enableRosLog
    if enablePoseLog
        poseLogFile = fullfile(logFileDir, "pose_log.csv");
        createCsvWithHeader(poseLogFile, ...
            "t_unix,topic,raw_x,raw_y,raw_z,x,y,z,yaw_rad,yaw_deg,v_est,vx,vy,vz,yaw_rate_deg,valid");
    end

    if enableMpcTraceSub && enableMpcTraceLog
        mpcTraceLogFile = fullfile(logFileDir, "mpc_trace_log.csv");
        createCsvWithHeader(mpcTraceLogFile, ...
            "t_matlab_rx,t_node,cur_x,cur_y,cur_yaw,cur_speed,nearest_dense_point," + ...
            "lateral_error,heading_error,first_steering_rad,applied_steering_norm," + ...
            "valid_solve,solve_time_ms,fallback_mode,virtual_steer,virtual_throttle," + ...
            "virtual_brake,mavlink_throttle,mavlink_steering,objective," + ...
            "estimated_effective_steering_rad,previous_applied_steering_rad,actuator_enabled," + ...
            "actuator_delay_sec,actuator_tau_sec,solver_only_time_ms," + ...
            "selected_steering_rad,output_steering_rad,fallback_available,kappa_ref0,delta_prev_rad");
    end

    if enableLoopLog
        loopLogFile = fullfile(logFileDir, "loop_log.csv");
        createCsvWithHeader(loopLogFile, ...
            "k,t_unix,pose_age_sec,loop_elapsed_sec,x,y,z,yaw_deg,v_est," + ...
            "mpc_age_sec,mpc_valid_solve,mpc_lateral_error,mpc_heading_error," + ...
            "mpc_first_steering_rad,mpc_applied_steering_norm,mpc_fallback_mode," + ...
            "mpc_mavlink_throttle,mpc_mavlink_steering,mpc_objective");
    end

    if enableLowerReferencePathSub && enableLowerReferencePathLog
        lowerReferencePathLogFile = fullfile(logFileDir, "lower_reference_first2_log.csv");
        createCsvWithHeader(lowerReferencePathLogFile, ...
            "t_matlab_rx,t_msg,topic,frame_id,message_pose_count," + ...
            "p0_x,p0_y,p0_z,p1_x,p1_y,p1_z," + ...
            "segment_yaw_rad,segment_yaw_deg,segment_length_m");
    end

    sessionInfoFile = fullfile(logDir, "session_info.txt");
    fid = fopen(sessionInfoFile, "w");
    fprintf(fid, "ROS_DOMAIN_ID=%s\n", getenv("ROS_DOMAIN_ID"));
    fprintf(fid, "scaleMode=%s\n", char(scaleMode));
    fprintf(fid, "mapName=%s\n", char(mapName));
    fprintf(fid, "routeName=%s\n", char(routeName));
    fprintf(fid, "poseSource=%s\n", char(poseSource));
    fprintf(fid, "gridMapTopic=%s\n", char(gridMapTopic));
    fprintf(fid, "globalPathTopic=%s\n", char(globalPathTopic));
    fprintf(fid, "motivePoseTopic=%s\n", char(motivePoseTopic));
    fprintf(fid, "px4MapOdomTopic=%s\n", char(px4MapOdomTopic));
    fprintf(fid, "px4SihRawPoseTopic=%s\n", char(px4SihRawPoseTopic));
    fprintf(fid, "publishMatlabAlignedOdom=%d\n", publishMatlabAlignedOdom);
    fprintf(fid, "mpcTraceTopic=%s\n", char(mpcTraceTopic));
    fprintf(fid, "enableMpcTraceSub=%d\n", enableMpcTraceSub);
    fprintf(fid, "lowerReferencePathTopic=%s\n", char(lowerReferencePathTopic));
    fprintf(fid, "enableLowerReferencePathSub=%d\n", enableLowerReferencePathSub);
    fprintf(fid, "enablePoseLog=%d\n", enablePoseLog);
    fprintf(fid, "enableMpcTraceLog=%d\n", enableMpcTraceLog);
    fprintf(fid, "enableLoopLog=%d\n", enableLoopLog);
    fprintf(fid, "enableLowerReferencePathLog=%d\n", enableLowerReferencePathLog);
    fprintf(fid, "enableFigure=%d\n", enableFigure);
    fprintf(fid, "enableVehicleWheels=%d\n", enableVehicleWheels);
    fprintf(fid, "enableGridDebugFigure=%d\n", enableGridDebugFigure);
    fprintf(fid, "enableAnimationGif=%d\n", enableAnimationGif);
    fprintf(fid, "animationGifPlaybackSpeed=%.3f\n", animationGifPlaybackSpeed);
    fprintf(fid, "animationGifCaptureEveryN=%d\n", animationGifCaptureEveryN);
    fprintf(fid, "gridFrameID=%s\n", char(gridFrameID));
    fprintf(fid, "publishInitialGridBeforePose=%d\n", publishInitialGridBeforePose);
    fprintf(fid, "Ts=%.6f\n", Ts);
    fprintf(fid, "gridRepublishSec=%.6f\n", gridRepublishSec);
    fprintf(fid, "fullPathRepublishSec=%.6f\n", fullPathRepublishSec);
    fprintf(fid, "enablePathHeartbeat=%d\n", enablePathHeartbeat);
    fprintf(fid, "enableKnuBatch=%d\n", enableKnuBatch);
    fclose(fid);

    fprintf("[MATLAB LOG] logDir=%s\n", logDir);
end

if enableAnimationGif
    fprintf("[MATLAB GIF] recording %.1fx animation: %s\n", ...
        animationGifPlaybackSpeed,animationGifFile);
end

%% ========================= Scenario / Ego =========================
[scenario,egoVehicle,waypoints,scaleProfile,mapInfo] = ...
    create_navigation_scenario(mapName, ...
        "ScaleMode",scaleMode,"RouteName",routeName);
if size(waypoints, 2) < 2 || size(waypoints, 1) < 2
    error("Selected map must return at least two waypoints with x,y columns.");
end

fprintf("[MATLAB] map=%s, route=%s, scale=%s, roadWidth=%.3f m\n", ...
    mapInfo.MapName,mapInfo.RouteName,scaleProfile.Mode,mapInfo.RoadWidth);
fprintf("[MATLAB] egoVehicle.Length = %.3f m\n", egoVehicle.Length);
fprintf("[MATLAB] global path waypoints = %d\n", size(waypoints, 1));

mapInitialPosition = reshape(double(egoVehicle.Position), 1, []);
mapInitialYawRad = deg2rad(double(egoVehicle.Yaw));

%% ========================= Ego state =========================
pk = egoVehicle.Position;
yawkDeg = egoVehicle.Yaw;
vk = 0.0;
velk = [0 0 0];
angvk = [0 0 0];

initialPoseState = struct( ...
    "position", pk, ...
    "yawRad", deg2rad(yawkDeg), ...
    "yawDeg", yawkDeg, ...
    "vel", velk, ...
    "speed", vk, ...
    "yawRateDeg", 0.0, ...
    "valid", false, ...
    "rx_time", 0.0);

setappdata(0, "vehicle_pose_state", initialPoseState);
setappdata(0, "px4_sih_alignment", struct( ...
    "initialized", false, ...
    "sourcePosition", [0.0; 0.0], ...
    "sourceZ", 0.0, ...
    "sourceYaw", 0.0, ...
    "targetPosition", mapInitialPosition(1:2).', ...
    "targetZ", mapInitialPosition(3), ...
    "targetYaw", mapInitialYawRad, ...
    "rotation", eye(2)));
setappdata(0, "mpc_trace_state", emptyMpcTraceState());

%% ========================= Figure =========================
mainFigure = [];
wheelPlot = [];
gridDebugFigure = [];
gridDebugAx = [];

if enableFigure
    mainFigure = figure(1);
    clf(mainFigure);
    ax = axes("Parent",mainFigure);
    chasePlot(egoVehicle, ...
        "Centerline", "off", ...
        "ViewLocation", [egoVehicle.Length * 1.5, 0], ...
        "ViewHeight", egoVehicle.Length * 8, ...
        "ViewPitch", 90, ...
        "Parent", ax);
    hold(ax, "on");
    if enableVehicleWheels
        wheelPlot = update_vehicle_wheels(ax,egoVehicle,wheelPlot);
        text(ax,0.02,0.02,"Front wheels: pose-based steering estimate", ...
            "Units","normalized","Color",[0.15 0.15 0.15], ...
            "BackgroundColor","w","Margin",3,"FontSize",9, ...
            "HandleVisibility","off","HitTest","off");
    end
end

if enableGridDebugFigure
    gridDebugFigure = figure(2);
    clf(gridDebugFigure);
    set(gridDebugFigure,"Name","MATLAB /grid_map debug", ...
        "NumberTitle","off","Color","w","Visible","on");
    gridDebugAx = axes("Parent",gridDebugFigure);
    title(gridDebugAx,"Waiting for first camera grid ...");
    xlabel(gridDebugAx,"x body [m]");
    ylabel(gridDebugAx,"y body [m]");
    axis(gridDebugAx,"equal");
    grid(gridDebugAx,"on");
    drawnow;
end

%% ========================= ROS 2 setup =========================
poseLogTarget = poseLogFile;
mpcLogTarget = mpcTraceLogFile;
referenceLogTarget = lowerReferencePathLogFile;
batchLog = [];
visualGeneration = -1;
if enableKnuBatch
    batchLog = KnuBatchLogSession(logDir,struct("pose",poseLogFile, ...
        "mpc",mpcTraceLogFile,"loop",loopLogFile,"reference",lowerReferencePathLogFile));
    poseLogTarget = batchLog;
    mpcLogTarget = batchLog;
    referenceLogTarget = batchLog;
end
node = ros2node("/matlab_grid_bridge");
if enableKnuBatch
    assert(poseSource=="px4_map", "KNU batch requires map-aligned PX4 odometry.");
    batchRouteSub = ros2subscriber(node,"/knu_batch/active_route","std_msgs/String", ...
        @knuBatchRouteCb,"Reliability","reliable","Durability","transientlocal","Depth",1);
    batchReadyPub = ros2publisher(node,"/knu_batch/grid_ready","std_msgs/String");
    batchPausePub = ros2publisher(node,"/knu_batch/logging_paused","std_msgs/String");
    batchStatusSub = ros2subscriber(node,"/knu_batch/status","std_msgs/String", ...
        @(msg)knuBatchStatusCb(msg,batchLog,batchPausePub), ...
        "Reliability","reliable","Durability","transientlocal","Depth",1);
    batchRouteLog = fullfile(logDir,"batch_routes.csv");
    if ~exist(logDir,"dir"), mkdir(logDir); end
    createCsvWithHeader(batchRouteLog,"t_unix,route,token");
end

gridPub = ros2publisher(node, gridMapTopic, "nav_msgs/OccupancyGrid");
pathPub = ros2publisher(node, globalPathTopic, "nav_msgs/Path", ...
    "Reliability", "reliable", "Durability", "transientlocal", "Depth", 1);
alignedOdomPub = [];

if poseSource == "motive"
    % Existing custom convention: orientation.z contains yaw [rad].
    motiveSub = ros2subscriber(node, motivePoseTopic, ...
        "geometry_msgs/PoseStamped", ...
        @(msg) motivePoseCb(msg, motivePoseTopic, poseLogFile, ...
                            posePositionScale, poseXOffset, poseYOffset, poseZOffset, ...
                            poseYawOffsetRad, poseSwapXY, poseInvertX, poseInvertY, ...
                            poseSpeedLpfAlpha, poseMaxDtForSpeed)); %#ok<NASGU>

elseif poseSource == "px4_map"
    px4MapSub = ros2subscriber(node, px4MapOdomTopic, ...
        "nav_msgs/Odometry", ...
        @(msg) px4MapOdomCb(msg, px4MapOdomTopic, poseLogTarget)); %#ok<NASGU>

elseif poseSource == "px4_sih_raw"
    px4SihSub = ros2subscriber(node, px4SihRawPoseTopic, ...
        "geometry_msgs/PoseStamped", ...
        @(msg) px4SihPoseCb(msg, px4SihRawPoseTopic, poseLogFile, ...
                            mapInitialPosition, mapInitialYawRad, ...
                            posePositionScale, poseXOffset, poseYOffset, poseZOffset, ...
                            poseYawOffsetRad, poseSwapXY, poseInvertX, poseInvertY, ...
                            poseSpeedLpfAlpha, poseMaxDtForSpeed)); %#ok<NASGU>

    if publishMatlabAlignedOdom
        alignedOdomPub = ros2publisher(node, px4MapOdomTopic, "nav_msgs/Odometry");
    end

else
    error("poseSource must be 'px4_map', 'px4_sih_raw', or 'motive'.");
end

if enableMpcTraceSub
    mpcTraceSub = ros2subscriber(node, mpcTraceTopic, ...
        "std_msgs/Float64MultiArray", ...
        @(msg) mpcTraceCb(msg, mpcTraceTopic, "mpc_trace_state", mpcLogTarget)); %#ok<NASGU>
    fprintf("[MATLAB LOG] subscribed mpc trace: %s\n", char(mpcTraceTopic));
else
    fprintf("[MATLAB LOG] mpc trace subscriber disabled\n");
end

if enableLowerReferencePathSub
    lowerReferencePathSub = ros2subscriber(node, lowerReferencePathTopic, ...
        "nav_msgs/Path", ...
        @(msg) lowerReferenceFirst2Cb(msg,lowerReferencePathTopic,referenceLogTarget), ...
        "Reliability","reliable","Durability","transientlocal","Depth",1); %#ok<NASGU>
    fprintf("[MATLAB LOG] subscribed lower reference path (saving first two): %s\n", ...
        char(lowerReferencePathTopic));
else
    fprintf("[MATLAB LOG] lower reference path subscriber disabled\n");
end

fprintf("ROS2 MATLAB grid bridge ready:\n");
fprintf("  pose source: %s\n", char(poseSource));
if poseSource == "motive"
    fprintf("  sub %-28s geometry_msgs/PoseStamped\n", char(motivePoseTopic));
    fprintf("      position.x/y = pose, orientation.z = yaw [rad], not quaternion\n");
elseif poseSource == "px4_map"
    fprintf("  sub %-28s nav_msgs/Odometry (orientation.z=yaw [rad], not quaternion)\n", ...
        char(px4MapOdomTopic));
else
    fprintf("  sub %-28s geometry_msgs/PoseStamped (orientation.z=yaw [rad])\n", ...
        char(px4SihRawPoseTopic));
    if publishMatlabAlignedOdom
        fprintf("  pub %-28s nav_msgs/Odometry (MATLAB map aligned)\n", ...
            char(px4MapOdomTopic));
    end
end
if enableMpcTraceSub
    fprintf("  sub %-28s std_msgs/Float64MultiArray\n", char(mpcTraceTopic));
end
if enableLowerReferencePathSub
    fprintf("  sub %-28s nav_msgs/Path (save first two points)\n", ...
        char(lowerReferencePathTopic));
end
fprintf("  pub %-28s nav_msgs/OccupancyGrid (BODY frame)\n", char(gridMapTopic));
fprintf("  pub %-28s nav_msgs/Path\n", char(globalPathTopic));
fprintf("  lower MPC consumes the aligned odometry and transient-local path.\n");
fprintf("  grid frame_id=%s, convention: drivable=100, non-drivable=0\n", char(gridFrameID));
fprintf("  gridRepublishSec=%.2f sec\n", gridRepublishSec);
fprintf("  global path heartbeat=%d\n", enablePathHeartbeat);
if enableRosLog
    fprintf("  logDir=%s\n", logDir);
end

pause(1.0);  % DDS discovery grace period

%% ========================= Initial global path publish =========================
pathMsg = makePathMsg(waypoints(:, 1:2), "map");

if ~enableKnuBatch
for i = 1:3
    send(pathPub, pathMsg);
    if printPathPublish
        fprintf("[MATLAB PATH] initial publish %d: topic=%s poses=%d\n", ...
            i, char(globalPathTopic), numel(pathMsg.poses));
    end
    pause(0.20);
end

end % single-route initial publication

lastFullPathPublishT = tic;

%% ========================= Main loop =========================
r = rateControl(1 / Ts);
k = 1;

lastGridPublishT = tic;
hasPublishedGrid = false;
gridPublishCount = 0;

while true
    loopTic = tic;
    if enableKnuBatch
        request = getappdata(0,"knu_batch_request");
        if isfield(request,"done") && request.done
            batchLog.closeSegment("batch_done");
            fprintf("[KNU BATCH] All routes finished. ROS holds neutral control.\n");
            break;
        end
        if isfield(request,"token") && string(request.token)~=activeBatchToken
            routeName = string(request.route);
            [waypoints,routeInfo] = get_navigation_waypoints("knu",routeName);
            activeBatchToken = string(request.token);
            mapInfo.RouteName = routeName;
            hasPublishedGrid = false;
            fid = fopen(batchRouteLog,"a");
            if fid>=0
                fprintf(fid,"%.6f,%s,%s\n",nowSecUTC(),routeName,activeBatchToken);
                fclose(fid);
            end
            fprintf("[KNU BATCH] route=%s; waiting for aligned pose/grid\n",routeName);
        end
    end

    %% ----- Read latest selected vehicle pose -----
    st = getappdata(0, "vehicle_pose_state");
    nowT = nowSecUTC();
    poseAgeSec = nowT - st.rx_time;
    poseFresh = st.valid && poseAgeSec <= poseTimeoutSec;

    if poseFresh
        pk = st.position;
        yawkDeg = st.yawDeg;
        vk = st.speed;
        velk = st.vel;
        angvk = [0 0 st.yawRateDeg];
    else
        if mod(k, max(1, printEveryN * 2)) == 0
            fprintf("[MATLAB] waiting/stale %s pose: valid=%d age=%.3f sec\n", ...
                char(poseSource), st.valid, poseAgeSec);
        end
    end

    %% ----- Apply selected state to scenario vehicle -----
    egoVehicle.Position = pk;
    egoVehicle.Yaw = yawkDeg;
    egoVehicle.Velocity = velk;
    egoVehicle.AngularVelocity = angvk;

    % Legacy fallback only. In normal px4_map mode the Jetson bridge is the
    % sole publisher of /px4/sih/odom_map.
    if poseFresh && poseSource == "px4_sih_raw" && publishMatlabAlignedOdom
        alignedMsg = makeAlignedOdomMsg(st, "map", "base_link");
        send(alignedOdomPub, alignedMsg);
    end

    batchVisualActive = ~enableKnuBatch;
    if enableKnuBatch
        batchVisualActive = batchLog.isRecording() && isfield(st,"batchGeneration") && ...
            st.batchGeneration==batchLog.Generation && poseFresh;
        if batchVisualActive && visualGeneration~=batchLog.Generation
            visualGeneration = batchLog.Generation;
            animationGifFile = batchLog.gifPath();
            animationGifInitialized = false;
            animationGifFrameCount = 0;
            wheelPlot = [];
            setappdata(0,"mpc_trace_state",emptyMpcTraceState());
            if enableFigure && isgraphics(mainFigure)
                % Dispose pose history held by wheelPlot and the previous plot axes.
                clf(mainFigure);
                ax = axes("Parent",mainFigure);
                chasePlot(egoVehicle,"Centerline","off", ...
                    "ViewLocation",[egoVehicle.Length*1.5,0], ...
                    "ViewHeight",egoVehicle.Length*8,"ViewPitch",90,"Parent",ax);
                hold(ax,"on");
                title(ax,sprintf('%s | segment %d',batchLog.Route,visualGeneration));
            end
            fprintf("[KNU LOG] new recording: %s\n",batchLog.SegmentDir);
        end
    end

    if enableFigure && batchVisualActive && mod(k, plotEveryN) == 0
        updatePlots(scenario);
        if enableVehicleWheels
            wheelPlot = update_vehicle_wheels(ax,egoVehicle,wheelPlot);
        end
        if enableAnimationGif && mod(k,animationGifCaptureEveryN)==0
            drawnow;
            try
                gifDelaySec = max(0.02, ...
                    Ts*animationGifCaptureEveryN/animationGifPlaybackSpeed);
                % drawnow may deliver a transition callback: check the gate again.
                canWriteGif = ~enableKnuBatch || (batchLog.isRecording() && ...
                    visualGeneration==batchLog.Generation);
                if canWriteGif
                    appendFigureToGif(mainFigure,animationGifFile, ...
                        gifDelaySec,~animationGifInitialized);
                    animationGifInitialized = true;
                    animationGifFrameCount = animationGifFrameCount+1;
                end
                if animationGifFrameCount==1
                    fprintf("[MATLAB GIF] first frame saved: %s\n",animationGifFile);
                end
            catch ME
                warning("MATLAB:animationGifWrite", ...
                    "GIF recording disabled after a write failure: %s",ME.message);
                enableAnimationGif = false;
            end
        else
            drawnow limitrate;
        end
    end

    % %% ----- Republish global path heartbeat -----
    % if enablePathHeartbeat && toc(lastFullPathPublishT) >= fullPathRepublishSec
    %     send(pathPub, pathMsg);
    %     if printPathPublish
    %         fprintf("[MATLAB PATH] republish: topic=%s poses=%d\n", ...
    %             char(globalPathTopic), numel(pathMsg.poses));
    %     end
    %     lastFullPathPublishT = tic;
    % end

    %% ----- Selected pose -> camera_module -> /grid_map -----
    shouldPublishGrid = poseFresh && ...
        (toc(lastGridPublishT) >= gridRepublishSec || ~hasPublishedGrid);

    if ~hasPublishedGrid && publishInitialGridBeforePose
        shouldPublishGrid = true;
    end

    if shouldPublishGrid
        try
            % scaleProfile selects the full-scale legacy calibration or the
            % existing reduced-scale calibration. Passing scenario lets the
            % cached chase plot follow the latest ego pose without rebuilding
            % all KNU road graphics on every grid publication.
            binaryMap = camera_module(egoVehicle,scaleProfile,scenario);
            if enableGridDebugFigure
                updateGridDebugFigure(gridDebugAx,binaryMap, ...
                    scaleProfile.Mode,gridPublishCount+1,"GENERATED");
            end

            gridMsg = makeOccGridMsgBody(binaryMap, gridFrameID);
            send(gridPub, gridMsg);
            if enableKnuBatch && poseFresh && strlength(activeBatchToken)>0
                readyMsg = ros2message(batchReadyPub);
                readyMsg.data = char(jsonencode(struct("route",routeName,"token",activeBatchToken)));
                send(batchReadyPub,readyMsg);
            end

            lastGridPublishT = tic;
            hasPublishedGrid = true;
            gridPublishCount = gridPublishCount + 1;
            if enableGridDebugFigure
                updateGridDebugFigure(gridDebugAx,binaryMap, ...
                    scaleProfile.Mode,gridPublishCount,"PUBLISHED /grid_map");
            end
            if printGridPublishEveryMessage || gridPublishCount == 1 || ...
                    mod(gridPublishCount,10) == 0
                fprintf(['[MATLAB GRID] publish #%d: topic=%s frame=%s ' ...
                    'size=%dx%d resolution=%.4f road_ratio=%.3f\n'], ...
                    gridPublishCount,char(gridMapTopic),char(gridFrameID), ...
                    gridMsg.info.width,gridMsg.info.height, ...
                    gridMsg.info.resolution, ...
                    nnz(binaryMap.B)/numel(binaryMap.B));
            end
        catch ME
            warning("MATLAB:gridPublish", ...
                "[MATLAB GRID] grid publish failed, but path publisher stays alive: %s", ...
                ME.message);
            lastGridPublishT = tic;   % prevent warning spam
        end
    end

    waitfor(r);
    loopElapsed = toc(loopTic);

    %% ----- Read latest optional MPC trace -----
    mpcTrace = getappdata(0, "mpc_trace_state");
    if enableMpcTraceSub && mpcTrace.valid
        mpcAge = nowSecUTC() - mpcTrace.rx_time;
    else
        mpcAge = NaN;
    end
    mpcSummary = getMpcTraceSummary(mpcTrace);

    if mod(k, printEveryN) == 0
        fprintf("[MATLAB] k=%04d pos=(%.3f,%.3f,%.3f) yaw=%.2fdeg v_est=%.3f pose_age=%.3f loop=%.3f\n", ...
            k, pk(1), pk(2), pk(3), yawkDeg, vk, poseAgeSec, loopElapsed);

        if enableMpcTraceSub && mpcTrace.valid
            fprintf("[MPC] age=%.3f valid=%d e_y=%.3f e_psi=%.3f steer_rad=%.4f steer_norm=%.3f fallback=%d MAV=(%.3f,%.3f) solve=%.2fms\n", ...
                mpcAge, ...
                mpcSummary.valid_solve, ...
                mpcSummary.lateral_error, ...
                mpcSummary.heading_error, ...
                mpcSummary.first_steering_rad, ...
                mpcSummary.applied_steering_norm, ...
                mpcSummary.fallback_mode, ...
                mpcSummary.mavlink_throttle, ...
                mpcSummary.mavlink_steering, ...
                mpcSummary.solve_time_ms);
        end
    end

    currentLoopLog = loopLogFile;
    if enableKnuBatch
        currentLoopLog = "";
        if batchVisualActive && batchLog.isRecording() && ...
                isfield(st,"batchGeneration") && st.batchGeneration==batchLog.Generation
            currentLoopLog = batchLog.logPath("loop",st.source_stamp);
        end
    end
    if strlength(currentLoopLog) > 0
        appendLoopLog(currentLoopLog, k, nowT, poseAgeSec, loopElapsed, ...
                      pk, yawkDeg, vk, mpcAge, mpcSummary);
    end

    if hasPublishedGrid && exist("gridMsg", "var") && mod(k, max(1, printEveryN * 2)) == 0
        fprintf("[MATLAB grid BODY] frame=%s origin=(%.3f, %.3f) res=%.4f size=%dx%d\n", ...
            char(gridFrameID), ...
            gridMsg.info.origin.position.x, ...
            gridMsg.info.origin.position.y, ...
            gridMsg.info.resolution, ...
            gridMsg.info.width, ...
            gridMsg.info.height);
    end

    k = k + 1;
end

if enableKnuBatch
    batchLog.closeSegment("main_loop_exit");
    % Stop callbacks after the batch; leave figures/results available for inspection.
    clear batchStatusSub batchRouteSub px4MapSub mpcTraceSub lowerReferencePathSub;
end

%% ========================= Local functions =========================
function knuBatchStatusCb(msg,logSession,pausePub)
    try
        status = jsondecode(msg.data);
        if logSession.onStatus(status) && logSession.Phase~="running"
            % No reanchor until this acknowledgement. onStatus closes files first.
            response = ros2message(pausePub);
            response.data = char(jsonencode(struct("token",logSession.Token,"paused",true)));
            send(pausePub,response);
        end
    catch ME
        logSession.closeSegment("status_error");
        warning("KnuBatch:Status","%s",ME.message);
    end
end

function knuBatchRouteCb(msg)
    try
        request = jsondecode(msg.data);
        if isfield(request,"done") && request.done
            setappdata(0,"knu_batch_request",request);
        elseif isfield(request,"route") && isfield(request,"token") && ...
                any(string(request.route)=="scenario"+string(1:21))
            setappdata(0,"knu_batch_request",request);
        end
    catch ME
        warning("KnuBatch:InvalidRequest","%s",ME.message);
    end
end

function updateGridDebugFigure(ax,binaryMap,scaleMode,sequence,statusText)
    if isempty(ax) || ~isgraphics(ax)
        return;
    end

    B = double(extractGridFromBinaryMap(binaryMap) > 0);
    cla(ax);
    h = pcolor(ax,double(binaryMap.X),double(binaryMap.Y),B);
    set(h,"EdgeColor","none");
    colormap(ax,gray(2));
    clim(ax,[0 1]);
    axis(ax,"equal");
    axis(ax,"tight");
    hold(ax,"on");
    plot(ax,0,0,"ro","MarkerFaceColor","r","MarkerSize",7);
    hold(ax,"off");
    xlabel(ax,"x body [m]");
    ylabel(ax,"y body [m]");
    roadRatio = nnz(B)/numel(B);
    title(ax,sprintf('%s | %s #%d | road ratio %.3f', ...
        string(scaleMode),string(statusText),sequence,roadRatio));
    drawnow;
end

function motivePoseCb(msg, topicName, logFile, ...
                      posePositionScale, poseXOffset, poseYOffset, poseZOffset, ...
                      poseYawOffsetRad, poseSwapXY, poseInvertX, poseInvertY, ...
                      poseSpeedLpfAlpha, poseMaxDtForSpeed)

    nowT = nowSecUTC();

    % PoseStamped custom convention:
    %   position.x/y = vehicle position
    %   orientation.z = yaw [rad]
    rawX = double(msg.pose.position.x);
    rawY = double(msg.pose.position.y);
    rawZ = double(msg.pose.position.z);

    x = rawX;
    y = rawY;
    z = rawZ;

    if poseSwapXY
        tmp = x;
        x = y;
        y = tmp;
    end
    if poseInvertX
        x = -x;
    end
    if poseInvertY
        y = -y;
    end

    x = posePositionScale * x + poseXOffset;
    y = posePositionScale * y + poseYOffset;
    z = posePositionScale * z + poseZOffset;

    yawRad = double(msg.pose.orientation.z) + poseYawOffsetRad;
    yawRad = wrapToPiRad(yawRad);
    yawDeg = rad2deg(yawRad);

    old = getappdata(0, "vehicle_pose_state");

    speed = old.speed;
    vel = old.vel;
    yawRateDeg = old.yawRateDeg;

    if old.valid
        dt = nowT - old.rx_time;

        if dt > 1e-4 && dt <= poseMaxDtForSpeed
            dp = [x y z] - old.position;

            vMeas = norm(dp(1:2)) / dt;
            alpha = min(max(poseSpeedLpfAlpha, 0.0), 1.0);
            speed = alpha * vMeas + (1.0 - alpha) * old.speed;

            vel = [dp(1) / dt, dp(2) / dt, dp(3) / dt];

            yawDiffDeg = wrapTo180Deg(yawDeg - old.yawDeg);
            yawRateDeg = yawDiffDeg / dt;
        else
            % If Motive update gap is too large, do not trust finite-difference velocity.
            speed = 0.0;
            vel = [0 0 0];
            yawRateDeg = 0.0;
        end
    else
        speed = 0.0;
        vel = [0 0 0];
        yawRateDeg = 0.0;
    end

    latest = struct();
    latest.position = [x y z];
    latest.yawRad = yawRad;
    latest.yawDeg = yawDeg;
    latest.vel = vel;
    latest.speed = speed;
    latest.yawRateDeg = yawRateDeg;
    latest.valid = true;
    latest.rx_time = nowT;

    setappdata(0, "vehicle_pose_state", latest);

    if strlength(string(logFile)) > 0
        appendPoseLog(logFile, nowT, topicName, rawX, rawY, rawZ, latest);
    end
end

function px4MapOdomCb(msg, topicName, logFile)
    % The map-aligned Motive source uses the project-specific convention
    % orientation.z=yaw [rad], not a quaternion.
    nowT = nowSecUTC();

    x = double(msg.pose.pose.position.x);
    y = double(msg.pose.pose.position.y);
    z = double(msg.pose.pose.position.z);

    yawRad = wrapToPiRad(double(msg.pose.pose.orientation.z));

    vel = [double(msg.twist.twist.linear.x), ...
           double(msg.twist.twist.linear.y), ...
           double(msg.twist.twist.linear.z)];
    yawRateRad = double(msg.twist.twist.angular.z);

    latest = struct();
    latest.position = [x y z];
    latest.yawRad = yawRad;
    latest.yawDeg = rad2deg(yawRad);
    latest.vel = vel;
    latest.speed = hypot(vel(1),vel(2));
    latest.yawRateDeg = rad2deg(yawRateRad);
    latest.valid = all(isfinite([latest.position,latest.yawRad, ...
                                 latest.vel,latest.yawRateDeg]));
    latest.rx_time = nowT;

    latest.source_stamp = double(msg.header.stamp.sec)+1e-9*double(msg.header.stamp.nanosec);
    latest.batchGeneration = -1;
    outputFile = string.empty;
    if isa(logFile,"KnuBatchLogSession")
        [accepted,latest] = logFile.acceptPose(latest,latest.source_stamp);
        if accepted
            latest.batchGeneration = logFile.Generation;
            outputFile = logFile.logPath("pose",latest.source_stamp);
        end
    else
        outputFile = string(logFile);
    end
    % Map generation still follows live pose while recording is paused.
    setappdata(0,"vehicle_pose_state",latest);
    if ~isempty(outputFile) && strlength(outputFile)>0
        appendPoseLog(outputFile,nowT,topicName,x,y,z,latest);
    end
end

function px4SihPoseCb(msg, topicName, logFile, ...
                      mapInitialPosition, mapInitialYawRad, ...
                      posePositionScale, poseXOffset, poseYOffset, poseZOffset, ...
                      poseYawOffsetRad, poseSwapXY, poseInvertX, poseInvertY, ...
                      poseSpeedLpfAlpha, poseMaxDtForSpeed)

    nowT = nowSecUTC();

    rawX = double(msg.pose.position.x);
    rawY = double(msg.pose.position.y);
    rawZ = double(msg.pose.position.z);
    rawYaw = double(msg.pose.orientation.z);
    rawYaw = wrapToPiRad(rawYaw);

    % Apply the same optional axis convention to position and yaw.
    sourcePosition = [rawX; rawY];
    sourceHeading = [cos(rawYaw); sin(rawYaw)];

    if poseSwapXY
        sourcePosition = sourcePosition([2 1]);
        sourceHeading = sourceHeading([2 1]);
    end
    if poseInvertX
        sourcePosition(1) = -sourcePosition(1);
        sourceHeading(1) = -sourceHeading(1);
    end
    if poseInvertY
        sourcePosition(2) = -sourcePosition(2);
        sourceHeading(2) = -sourceHeading(2);
    end

    sourcePosition = posePositionScale * sourcePosition;
    sourceHeading = signOrOne(posePositionScale) * sourceHeading;
    sourceYaw = atan2(sourceHeading(2), sourceHeading(1));

    alignment = getappdata(0, "px4_sih_alignment");

    if ~alignment.initialized
        alignment.initialized = true;
        alignment.sourcePosition = sourcePosition;
        alignment.sourceZ = posePositionScale * rawZ;
        alignment.sourceYaw = sourceYaw;
        alignment.targetPosition = mapInitialPosition(1:2).' + [poseXOffset; poseYOffset];
        alignment.targetZ = mapInitialPosition(3) + poseZOffset;
        alignment.targetYaw = wrapToPiRad(mapInitialYawRad + poseYawOffsetRad);

        yawDelta = wrapToPiRad(alignment.targetYaw - alignment.sourceYaw);
        alignment.rotation = [cos(yawDelta), -sin(yawDelta); ...
                              sin(yawDelta),  cos(yawDelta)];
        setappdata(0, "px4_sih_alignment", alignment);

        fprintf("[MATLAB PX4] initial alignment: raw=(%.3f,%.3f,%.1fdeg) -> map=(%.3f,%.3f,%.1fdeg)\n", ...
            rawX, rawY, rad2deg(rawYaw), ...
            alignment.targetPosition(1), alignment.targetPosition(2), ...
            rad2deg(alignment.targetYaw));
    end

    relativePosition = sourcePosition - alignment.sourcePosition;
    mapPositionXY = alignment.targetPosition + alignment.rotation * relativePosition;
    z = alignment.targetZ + (posePositionScale * rawZ - alignment.sourceZ);
    yawRad = wrapToPiRad(alignment.targetYaw + sourceYaw - alignment.sourceYaw);

    position = [mapPositionXY(1), mapPositionXY(2), z];
    old = getappdata(0, "vehicle_pose_state");
    speed = 0.0;
    vel = [0.0 0.0 0.0];
    yawRateDeg = 0.0;

    if old.valid
        dt = nowT - old.rx_time;
        if dt > 1e-4 && dt <= poseMaxDtForSpeed
            dp = position - old.position;
            measuredSpeed = norm(dp(1:2)) / dt;
            alpha = min(max(poseSpeedLpfAlpha,0.0),1.0);
            speed = alpha * measuredSpeed + (1.0-alpha) * old.speed;
            vel = dp / dt;
            yawRateDeg = rad2deg(wrapToPiRad(yawRad-old.yawRad)) / dt;
        end
    end

    latest = struct();
    latest.position = position;
    latest.yawRad = yawRad;
    latest.yawDeg = rad2deg(yawRad);
    latest.vel = vel;
    latest.speed = speed;
    latest.yawRateDeg = yawRateDeg;
    latest.valid = all(isfinite([latest.position, latest.yawRad, latest.vel]));
    latest.rx_time = nowT;

    setappdata(0, "vehicle_pose_state", latest);

    if strlength(string(logFile)) > 0
        appendPoseLog(logFile, nowT, topicName, rawX, rawY, rawZ, latest);
    end
end

function msg = makeAlignedOdomMsg(st, frameID, childFrameID)
    msg = ros2message("nav_msgs/Odometry");
    msg.header.frame_id = char(frameID);
    msg.child_frame_id = char(childFrameID);

    msg.pose.pose.position.x = double(st.position(1));
    msg.pose.pose.position.y = double(st.position(2));
    msg.pose.pose.position.z = double(st.position(3));
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = wrapToPiRad(double(st.yawRad));
    msg.pose.pose.orientation.w = 0.0;

    msg.twist.twist.linear.x = double(st.vel(1));
    msg.twist.twist.linear.y = double(st.vel(2));
    msg.twist.twist.linear.z = double(st.vel(3));
    msg.twist.twist.angular.z = deg2rad(double(st.yawRateDeg));
end

function value = signOrOne(value)
    value = sign(value);
    if value == 0
        value = 1.0;
    end
end

function s = emptyMpcTraceState()
    s = struct( ...
        "rx_time", 0.0, ...
        "topic", "", ...
        "valid", false, ...
        "data", zeros(1, 30));
end

function mpcTraceCb(msg, topicName, appKey, logFile)
    nowT = nowSecUTC();
    vals = double(msg.data(:)).';

    st = struct();
    st.rx_time = nowT;
    st.topic = string(topicName);
    st.valid = ~isempty(vals);
    st.data = vals;

    setappdata(0, appKey, st);

    if isa(logFile,"KnuBatchLogSession")
        outputFile = "";
        if ~isempty(vals), outputFile = logFile.logPath("mpc",vals(1)); end
    else
        outputFile = string(logFile);
    end
    if strlength(outputFile)>0
        appendMpcTraceLog(outputFile, nowT, vals);
    end
end

function lowerReferenceFirst2Cb(msg,topicName,logFile)
    % Save only the first two poses. The current lower controller itself
    % retains and samples the full /planner/lower_reference_path message.
    if numel(msg.poses) < 2
        warning("MATLAB:lowerReferencePath", ...
            "Lower reference path has fewer than two poses: topic=%s count=%d", ...
            char(topicName),numel(msg.poses));
        return;
    end

    if isa(logFile,"KnuBatchLogSession")
        stamp = double(msg.header.stamp.sec)+1e-9*double(msg.header.stamp.nanosec);
        outputFile = logFile.logPath("reference",stamp);
    else
        outputFile = string(logFile);
    end
    if strlength(outputFile)==0, return; end

    try
        appendLowerReferenceFirst2Log(outputFile,nowSecUTC(),topicName,msg);
    catch ME
        warning("MATLAB:lowerReferencePathLog", ...
            "Failed to append lower reference path log: %s",ME.message);
    end
end

function msg = makeOccGridMsgBody(binaryMap, frameID)
    % Body-frame OccupancyGrid for tracking_control body_to_grid lookup.
    %
    % Convention:
    %   frame_id = base_link / Vehicle
    %   origin   = lower-left outer corner in BODY frame
    %   yaw      = 0
    %   data     = drivable 100, non-drivable 0

    frameID = char(frameID);

    Braw = extractGridFromBinaryMap(binaryMap) > 0;
    Xraw = double(binaryMap.X);
    Yraw = double(binaryMap.Y);

    xMin = min(Xraw(:));
    xMax = max(Xraw(:));
    yMin = min(Yraw(:));
    yMax = max(Yraw(:));

    xVals = unique(sort(Xraw(:)));
    yVals = unique(sort(Yraw(:)));

    dxVals = diff(xVals);
    dyVals = diff(yVals);
    dxVals = dxVals(abs(dxVals) > 1e-9);
    dyVals = dyVals(abs(dyVals) > 1e-9);

    if isempty(dxVals)
        dx = 0.1;
    else
        dx = median(abs(dxVals));
    end

    if isempty(dyVals)
        dy = 0.1;
    else
        dy = median(abs(dyVals));
    end

    res = min(dx, dy);

    xq = xMin:res:xMax;
    yq = yMin:res:yMax;
    [Xq, Yq] = meshgrid(xq, yq);

    F = scatteredInterpolant(Xraw(:), Yraw(:), double(Braw(:)), ...
                             "nearest", "nearest");
    Vq = F(Xq, Yq);
    Vq(isnan(Vq)) = 0.0;

    Bsq = Vq > 0.5;

    % MATLAB grid convention:
    %   drivable = 100
    %   non-drivable = 0
    %
    % tracking_control launch:
    %   grid_positive_is_drivable:=true
    %   grid_value_threshold:=1
    occ = int8(zeros(size(Bsq)));
    occ(Bsq) = 100;

    dataVec = reshape(occ.', [], 1);

    originBodyCorner = [xMin - 0.5 * res; ...
                        yMin - 0.5 * res];

    msg = ros2message("nav_msgs/OccupancyGrid");
    msg.header.frame_id = frameID;

    msg.info.resolution = single(res);
    msg.info.width = uint32(size(occ, 2));
    msg.info.height = uint32(size(occ, 1));

    msg.info.origin.position.x = double(originBodyCorner(1));
    msg.info.origin.position.y = double(originBodyCorner(2));
    msg.info.origin.position.z = 0.0;

    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data = int8(dataVec);
end

function msg = makePathMsg(xy, frameID)
    frameID = char(frameID);

    msg = ros2message("nav_msgs/Path");
    msg.header.frame_id = frameID;

    n = size(xy, 1);
    msg.poses = repmat(ros2message("geometry_msgs/PoseStamped"), n, 1);

    for i = 1:n
        msg.poses(i).header.frame_id = frameID;
        msg.poses(i).pose.position.x = xy(i, 1);
        msg.poses(i).pose.position.y = xy(i, 2);
        msg.poses(i).pose.position.z = 0.0;
        msg.poses(i).pose.orientation.w = 1.0;
    end
end

function summary = getMpcTraceSummary(mpcTrace)
    summary = struct( ...
        "valid_solve", false, ...
        "lateral_error", NaN, ...
        "heading_error", NaN, ...
        "first_steering_rad", NaN, ...
        "applied_steering_norm", NaN, ...
        "solve_time_ms", NaN, ...
        "fallback_mode", NaN, ...
        "mavlink_throttle", NaN, ...
        "mavlink_steering", NaN, ...
        "objective", NaN);

    if ~isfield(mpcTrace, "valid") || ~mpcTrace.valid
        return;
    end

    d = mpcTrace.data;
    if numel(d) < 19
        return;
    end

    % C++ Float64MultiArray index is 0-based.
    % MATLAB index is 1-based.
    summary.lateral_error = d(7);           % C++ index 6
    summary.heading_error = d(8);           % C++ index 7
    summary.first_steering_rad = d(9);      % C++ index 8
    summary.applied_steering_norm = d(10);  % C++ index 9
    summary.valid_solve = d(11) ~= 0.0;      % C++ index 10
    summary.solve_time_ms = d(12);          % C++ index 11
    summary.fallback_mode = d(13);          % 0=QP, 1=legacy sequence, 2=hold, 3=neutral, 4=stop, 5=curvature fallback
    summary.mavlink_throttle = d(17);       % C++ index 16
    summary.mavlink_steering = d(18);       % C++ index 17
    summary.objective = d(19);              % C++ index 18
    if numel(d) >= 30
        summary.selected_steering_rad = d(26);
        summary.output_steering_rad = d(27);
        summary.fallback_available = d(28) ~= 0;
        summary.kappa_ref0 = d(29);
        summary.delta_prev_rad = d(30);
    end
end

function createCsvWithHeader(filePath, headerLine)
    fid = fopen(filePath, "w");
    if fid < 0
        error("Failed to create CSV log file: %s", filePath);
    end
    fprintf(fid, "%s\n", char(headerLine));
    fclose(fid);
end

function appendPoseLog(filePath, t, topicName, rawX, rawY, rawZ, st)
    fid = fopen(filePath, "a");
    if fid < 0
        warning("Failed to open pose log file: %s", filePath);
        return;
    end

    fprintf(fid, "%.9f,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d\n", ...
        t, char(topicName), ...
        rawX, rawY, rawZ, ...
        st.position(1), st.position(2), st.position(3), ...
        st.yawRad, st.yawDeg, st.speed, ...
        st.vel(1), st.vel(2), st.vel(3), ...
        st.yawRateDeg, st.valid);

    fclose(fid);
end

function appendMpcTraceLog(filePath, tRx, vals)
    fid = fopen(filePath, "a");
    if fid < 0
        warning("Failed to open mpc trace log file: %s", filePath);
        return;
    end

    fprintf(fid, "%.9f", tRx);
    vals(end+1:30) = NaN;  % Old traces retain a fixed-width CSV schema.
    for i = 1:30
        fprintf(fid, ",%.9g", vals(i));
    end
    fprintf(fid, "\n");

    fclose(fid);
end

function appendLowerReferenceFirst2Log(filePath,tRx,topicName,msg)
    poseCount = numel(msg.poses);
    p0 = msg.poses(1).pose.position;
    p1 = msg.poses(2).pose.position;

    x0 = double(p0.x); y0 = double(p0.y); z0 = double(p0.z);
    x1 = double(p1.x); y1 = double(p1.y); z1 = double(p1.z);
    dx = x1-x0;
    dy = y1-y0;
    segmentYawRad = atan2(dy,dx);
    segmentLength = hypot(dx,dy);

    frameID = strrep(char(msg.header.frame_id),",","_");
    topicText = strrep(char(topicName),",","_");
    tMsg = double(msg.header.stamp.sec) + ...
        1e-9*double(msg.header.stamp.nanosec);

    fid = fopen(filePath,"a");
    if fid < 0
        warning("Failed to open lower reference path log file: %s",filePath);
        return;
    end

    fprintf(fid,["%.9f,%.9f,%s,%s,%d," + ...
        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g," + ...
        "%.9g,%.9g,%.9g\n"], ...
        tRx,tMsg,topicText,frameID,poseCount, ...
        x0,y0,z0,x1,y1,z1, ...
        segmentYawRad,rad2deg(segmentYawRad),segmentLength);

    fclose(fid);
end

function appendLoopLog(filePath, k, t, poseAgeSec, loopElapsed, ...
                       pk, yawkDeg, vk, mpcAge, mpcSummary)
    fid = fopen(filePath, "a");
    if fid < 0
        warning("Failed to open loop log file: %s", filePath);
        return;
    end

    fprintf(fid, "%d,%.9f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", ...
        k, t, poseAgeSec, loopElapsed, ...
        pk(1), pk(2), pk(3), yawkDeg, vk, ...
        mpcAge, ...
        mpcSummary.valid_solve, ...
        mpcSummary.lateral_error, ...
        mpcSummary.heading_error, ...
        mpcSummary.first_steering_rad, ...
        mpcSummary.applied_steering_norm, ...
        mpcSummary.fallback_mode, ...
        mpcSummary.mavlink_throttle, ...
        mpcSummary.mavlink_steering, ...
        mpcSummary.objective);

    fclose(fid);
end

function appendFigureToGif(figureHandle,filePath,delaySec,isFirstFrame)
    assert(isgraphics(figureHandle,"figure"), ...
        "Figure 1 was closed before the GIF frame could be captured.");

    capturedFrame = getframe(figureHandle);
    rgbFrame = frame2im(capturedFrame);
    [indexedFrame,colorMap] = rgb2ind(rgbFrame,256);

    if isFirstFrame
        imwrite(indexedFrame,colorMap,filePath,"gif", ...
            "LoopCount",Inf,"DelayTime",delaySec);
    else
        imwrite(indexedFrame,colorMap,filePath,"gif", ...
            "WriteMode","append","DelayTime",delaySec);
    end
end

function yawRad = wrapToPiRad(yawRad)
    yawRad = atan2(sin(yawRad), cos(yawRad));
end

function yawDeg = wrapTo180Deg(yawDeg)
    yawDeg = mod(yawDeg + 180.0, 360.0) - 180.0;
end

function t = nowSecUTC()
    t = posixtime(datetime("now", "TimeZone", "UTC"));
end

function gridRaw = extractGridFromBinaryMap(binaryMap)
    candidates = {"B", "C", "map", "occupancy", "grid", "BW", "Binary", "img"};

    for i = 1:numel(candidates)
        fieldName = candidates{i};

        if isfield(binaryMap, fieldName)
            gridRaw = double(binaryMap.(fieldName));

            if islogical(gridRaw)
                gridRaw = double(gridRaw);
            end

            return;
        end
    end

    error("Binary_map does not contain a supported occupancy field. Update extractGridFromBinaryMap().");
end
