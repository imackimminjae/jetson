classdef KnuBatchLogSession < handle
    % Per-route recording gate. Coordinate resets are never travel samples.
    % ROS-free so lifecycle and discontinuity handling can be tested directly.
    properties (SetAccess = private)
        Root
        Templates
        Token = ""
        Route = ""
        Phase = "startup"
        Sequence = -1
        Session = ""
        PhaseStamp = Inf
        Generation = 0
        SegmentDir = ""
        PoseReady = false
        LastPoseStamp = -Inf
        LastPose = []
        Distance = 0
        SampleCount = 0
    end
    properties (Access = private)
        StatusClock = []
        SegmentNumber = 0
        FirstPoseStamp = NaN
        LastResultToken = ""
    end
    methods
        function obj = KnuBatchLogSession(root,templates)
            obj.Root = string(root);
            obj.Templates = templates;
            if ~isfolder(root), mkdir(root); end
        end

        function accepted = onStatus(obj,status)
            accepted = false;
            required = {'session','token','route','phase','sequence','phase_stamp_sec'};
            if ~isstruct(status) || ~all(isfield(status,required)), return; end
            if ~isscalar(status.sequence) || ~isfinite(status.sequence) || ...
                    ~isscalar(status.phase_stamp_sec) || ~isfinite(status.phase_stamp_sec)
                return;
            end
            sameSession = string(status.session)==obj.Session;
            if sameSession && status.sequence < obj.Sequence, return; end
            changed = ~sameSession || status.sequence~=obj.Sequence;
            if changed
                obj.closeSegment("phase_"+string(status.phase));
                if string(status.token)~=obj.Token
                    obj.SegmentNumber = 0;
                end
                obj.Session = string(status.session);
                obj.Token = string(status.token);
                obj.Route = string(status.route);
                obj.Phase = string(status.phase);
                obj.Sequence = status.sequence;
                obj.PhaseStamp = status.phase_stamp_sec;
                obj.event(struct('event','phase','token',obj.Token, ...
                    'route',obj.Route,'phase',obj.Phase,'stamp',obj.PhaseStamp));
            end
            obj.StatusClock = tic;
            if isfield(status,'last_result') && isstruct(status.last_result) && ...
                    isfield(status.last_result,'token')
                result = status.last_result;
                if string(result.token)~=obj.LastResultToken
                    directory = obj.routeDirectory(string(result.token));
                    if ~isfolder(directory), mkdir(directory); end
                    obj.writeJson(fullfile(directory,"route_result.json"),result);
                    obj.LastResultToken = string(result.token);
                end
            end
            accepted = true;
        end

        function yes = isRecording(obj)
            if ~isempty(obj.StatusClock) && toc(obj.StatusClock)>2.0
                obj.closeSegment("status_timeout");
            end
            yes = obj.Phase=="running" && obj.PoseReady && ...
                ~isempty(obj.StatusClock) && toc(obj.StatusClock)<=2.0;
        end

        function [accepted,st] = acceptPose(obj,st,stamp)
            accepted = false;
            obj.isRecording(); % Expire an abandoned runner before accepting data.
            if obj.Phase~="running" || isempty(obj.StatusClock) || ...
                    toc(obj.StatusClock)>2.0 || ~st.valid || ...
                    ~isfinite(stamp) || stamp<obj.PhaseStamp || stamp<=obj.LastPoseStamp
                return;
            end
            if obj.PoseReady
                dt = stamp-obj.LastPoseStamp;
                jump = norm(st.position(1:2)-obj.LastPose(1:2));
                if dt>0.6
                    obj.closeSegment("pose_gap");
                elseif jump>max(10.0,10.0*dt)
                    obj.closeSegment("pose_jump");
                end
            end
            if ~obj.PoseReady
                obj.openSegment(stamp);
                % A new coordinate origin cannot supply a finite-difference velocity.
                st.speed = 0;
                st.vel = [0 0 0];
                st.yawRateDeg = 0;
            else
                obj.Distance = obj.Distance+norm(st.position(1:2)-obj.LastPose(1:2));
            end
            obj.LastPoseStamp = stamp;
            obj.LastPose = st.position;
            obj.SampleCount = obj.SampleCount+1;
            accepted = true;
        end

        function path = logPath(obj,kind,stamp)
            path = "";
            if nargin<3, stamp = obj.LastPoseStamp; end
            if ~obj.isRecording() || ~isfinite(stamp) || stamp<obj.FirstPoseStamp
                return;
            end
            if isfield(obj.Templates,kind) && strlength(obj.Templates.(kind))>0
                [~,name,extension] = fileparts(obj.Templates.(kind));
                path = fullfile(obj.SegmentDir,name+extension);
            end
        end

        function path = gifPath(obj)
            path = "";
            if obj.isRecording()
                path = fullfile(obj.SegmentDir,"drive_animation.gif");
            end
        end

        function closeSegment(obj,reason)
            if ~obj.PoseReady, return; end
            % Close the gate first; a subsequent pause acknowledgement is then safe.
            obj.PoseReady = false;
            info = struct('route',obj.Route,'token',obj.Token, ...
                'segment',obj.SegmentNumber,'first_stamp_sec',obj.FirstPoseStamp, ...
                'last_stamp_sec',obj.LastPoseStamp,'sample_count',obj.SampleCount, ...
                'distance_m',obj.Distance,'end_reason',string(reason));
            obj.writeJson(fullfile(obj.SegmentDir,"segment_info.json"),info);
            obj.event(struct('event','segment_end','info',info));
            obj.LastPose = [];
            obj.LastPoseStamp = -Inf;
        end
    end
    methods (Access = private)
        function openSegment(obj,stamp)
            obj.SegmentNumber = obj.SegmentNumber+1;
            directory = obj.routeDirectory(obj.Token);
            obj.SegmentDir = fullfile(directory,sprintf('segment_%03d',obj.SegmentNumber));
            % Never overwrite an earlier run, including a late MATLAB restart.
            while isfolder(obj.SegmentDir)
                obj.SegmentNumber = obj.SegmentNumber+1;
                obj.SegmentDir = fullfile(directory,sprintf('segment_%03d',obj.SegmentNumber));
            end
            mkdir(obj.SegmentDir);
            kinds = fieldnames(obj.Templates);
            for i=1:numel(kinds)
                source = string(obj.Templates.(kinds{i}));
                if strlength(source)>0
                    [~,name,extension] = fileparts(source);
                    copyfile(source,fullfile(obj.SegmentDir,name+extension));
                end
            end
            obj.FirstPoseStamp = stamp;
            obj.LastPoseStamp = -Inf;
            obj.LastPose = [];
            obj.Distance = 0;
            obj.SampleCount = 0;
            obj.Generation = obj.Generation+1;
            obj.PoseReady = true;
            obj.event(struct('event','segment_start','route',obj.Route,'token',obj.Token, ...
                'directory',obj.SegmentDir,'stamp',stamp));
        end

        function directory = routeDirectory(obj,token)
            directory = fullfile(obj.Root,regexprep(token,'[^a-zA-Z0-9_-]','_'));
        end

        function event(obj,value)
            fid = fopen(fullfile(obj.Root,"batch_events.jsonl"),"a");
            assert(fid>=0,"Cannot write MATLAB batch events.");
            cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
            fprintf(fid,"%s\n",jsonencode(value));
        end
    end
    methods (Static, Access = private)
        function writeJson(path,value)
            temporary = string(path)+".tmp";
            fid = fopen(temporary,"w");
            assert(fid>=0,"Cannot write batch metadata: %s",temporary);
            fprintf(fid,"%s\n",jsonencode(value));
            fclose(fid);
            movefile(temporary,path,'f');
        end
    end
end
