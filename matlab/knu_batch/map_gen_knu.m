function [waypoints,mapData] = map_gen_knu(scenario,egoVehicle,options)

arguments
    scenario (1,1) drivingScenario
    egoVehicle = []
    options.DataFile (1,1) string = fullfile( ...
        fileparts(mfilename("fullpath")),"processed_road_map.mat")
    options.WidthSamplesFile (1,1) string = fullfile( ...
        fileparts(mfilename("fullpath")),"width_samples.csv")
    options.WidthMode (1,1) string {mustBeMember(options.WidthMode, ...
        ["boundary-profile","constant"])} = "boundary-profile"
    options.WidthSegmentLength (1,1) double {mustBePositive} = 12.0
    options.MaxWidthSegments (1,1) double {mustBeInteger,mustBePositive} = 20
    options.MinLocalInliers (1,1) double {mustBeInteger,mustBePositive} = 6
    options.MinimumRoadWidth (1,1) double {mustBePositive} = 3.0
    options.MaximumRoadWidth (1,1) double {mustBePositive} = 12.0
    options.RouteRoadIDs (1,:) double {mustBeInteger,mustBePositive} = 28
    options.ReverseRoute (1,1) logical = false
    options.WaypointScenario (1,1) string = "scenario1"
    options.ConnectRoad27 (1,1) logical = true
    options.JunctionOverlap (1,1) double {mustBeNonnegative} = 1.0
    options.JunctionConnectionTolerance (1,1) double ...
        {mustBePositive} = 1.25
    options.PlaceEgo (1,1) logical = false
    options.StartOffset (1,1) double {mustBeNonnegative} = 8.0
    options.EndOffset (1,1) double {mustBeNonnegative} = 8.0
    options.WaypointSpacing (1,1) double {mustBePositive} = 15.0
end

assert(options.MaximumRoadWidth>options.MinimumRoadWidth, ...
    "MaximumRoadWidth must exceed MinimumRoadWidth.");
assert(isfile(options.DataFile), ...
    "Processed road MAT file not found: %s",options.DataFile);

data = load(options.DataFile);
required = ["geoReference","roadCenterlines","roadWidths","roadIDs", ...
    "roadQuality","referenceBoundaries","boundarySegments"];

for k = 1:numel(required)
    assert(isfield(data,required(k)),"Missing MAT variable: %s",required(k));
end

roadCount = numel(data.roadCenterlines);
roadWidths = double(data.roadWidths(:));
roadIDs = double(data.roadIDs(:));
roadQuality = normalizeQuality(data.roadQuality);

assert(numel(roadWidths)==roadCount,"Road width/count mismatch.");
assert(numel(roadIDs)==roadCount,"Road ID/count mismatch.");
assert(numel(roadQuality)==roadCount,"Road quality/count mismatch.");
assert(all(options.RouteRoadIDs<=roadCount), ...
    "RouteRoadIDs must be between 1 and %d.",roadCount);

if ~isempty(scenario.GeoReference) && ...
        any(abs(double(scenario.GeoReference(:))- ...
        double(data.geoReference(:)))>1e-7)
    warning("map_gen_ortho:GeoReferenceMismatch", ...
        "Scenario GeoReference differs from processed map GeoReference.");
end

useBoundaryProfiles = options.WidthMode=="boundary-profile";
widthSamples = table();

if useBoundaryProfiles
    if isfile(options.WidthSamplesFile)
        widthSamples = readtable(options.WidthSamplesFile);
        validateWidthSampleTable(widthSamples);
    else
        warning("map_gen_ortho:MissingWidthSamples", ...
            ["Width sample CSV was not found. Falling back to one " ...
             "boundary-estimated width per road: %s"], ...
            options.WidthSamplesFile);
        useBoundaryProfiles = false;
    end
end

%% Add one-lane roads using center paths and boundary-derived width data.
widthProfiles = cell(roadCount,1);
renderedRoadCenterlines = cell(roadCount,1);
boundaryProfileRoadCount = 0;

for k = 1:roadCount
    roadCenters = double(data.roadCenterlines{k});
    roadCenters = extendConnectedRoadEnds(roadCenters,k, ...
        data.roadCenterlines,options.JunctionOverlap, ...
        options.JunctionConnectionTolerance);
    renderedRoadCenterlines{k} = roadCenters;
    baseWidth = roadWidths(k);

    validateattributes(roadCenters,{'double'}, ...
        {'2d','ncols',3,'finite'},mfilename, ...
        sprintf("roadCenterlines{%d}",k));
    validateattributes(baseWidth,{'double'}, ...
        {'scalar','positive','finite'},mfilename, ...
        sprintf("roadWidths(%d)",k));

    if useBoundaryProfiles
        [laneSpecification,widthProfiles{k}] = ...
            makeBoundaryInformedLaneSpec(roadCenters,baseWidth, ...
            roadIDs(k),roadQuality(k),widthSamples,options);
    else
        laneSpecification = lanespec(1,"Width",baseWidth);
        widthProfiles{k} = makeConstantProfile(roadIDs(k), ...
            roadCenters,baseWidth,"per-road-boundary-estimate");
    end

    if widthProfiles{k}.UsesLongitudinalProfile
        boundaryProfileRoadCount = boundaryProfileRoadCount+1;
    end

    road(scenario,roadCenters,"Lanes",laneSpecification, ...
        "Name",sprintf("ORTHO_%02d",roadIDs(k)));
end

%% Bridge the isolated curved road 27 to its neighboring junctions.
road27Connectors = cell(0,1);
road27ConnectorWidth = NaN;

if options.ConnectRoad27
    [road27Connectors,road27ConnectorWidth] = ...
        makeRoad27Connectors(data.roadCenterlines,roadIDs,roadWidths);

    connectorNames = ["ORTHO_CONNECT_27_LEFT","ORTHO_CONNECT_27_RIGHT"];
    connectorLane = lanespec(1,"Width",road27ConnectorWidth);

    for k = 1:numel(road27Connectors)
        road27Connectors{k} = extendPolylineEnds( ...
            road27Connectors{k},options.JunctionOverlap);
        road(scenario,road27Connectors{k},"Lanes",connectorLane, ...
            "Name",connectorNames(k));
    end
end

%% Assemble the selected centerline route for the ego vehicle.
route = double(data.roadCenterlines{options.RouteRoadIDs(1)});

for k = 2:numel(options.RouteRoadIDs)
    nextRoute = double(data.roadCenterlines{options.RouteRoadIDs(k)});

    distanceToStart = norm(route(end,1:2)-nextRoute(1,1:2));
    distanceToEnd = norm(route(end,1:2)-nextRoute(end,1:2));

    if distanceToEnd<distanceToStart
        nextRoute = flipud(nextRoute);
        connectionGap = distanceToEnd;
    else
        connectionGap = distanceToStart;
    end

    if connectionGap>3.0
        warning("map_gen_ortho:RouteGap", ...
            "Road %d to road %d has a %.2f m route gap.", ...
            options.RouteRoadIDs(k-1),options.RouteRoadIDs(k),connectionGap);
    end

    if connectionGap<0.20
        route = [route;nextRoute(2:end,:)]; %#ok<AGROW>
    else
        route = [route;nextRoute]; %#ok<AGROW>
    end
end


if options.ReverseRoute
    route = flipud(route);
end

route = trimPolyline(route,options.StartOffset,options.EndOffset);
% waypoints = resamplePolyline(route,options.WaypointSpacing);
[waypoints,routeInfo] = get_navigation_waypoints("knu",options.WaypointScenario);

assert(size(waypoints,1)>=2,"The selected route is too short.");

if options.PlaceEgo
    assert(~isempty(egoVehicle), ...
        "PlaceEgo=true requires an egoVehicle input.");

    egoStartPosition = waypoints(1,:);
    if lower(strtrim(options.WaypointScenario))=="scenario2"
        % Keep the experimental route unchanged, but start the ego on the
        % nearest road centerline instead of the off-road first waypoint.
        egoStartPosition(1:2) = [-252.789 93.548];
    end

    egoVehicle.Position = egoStartPosition;
    headingVector = waypoints(2,1:2)-waypoints(1,1:2);
    egoVehicle.Yaw = atan2d(headingVector(2),headingVector(1));
    egoVehicle.Velocity = [0 0 0];
    egoVehicle.AngularVelocity = [0 0 0];
end

if nargout>1
    mapData = data;
    mapData.roadQuality = roadQuality;
    mapData.routeRoadIDs = options.RouteRoadIDs(:).';
    mapData.routeWaypoints = waypoints;
    mapData.routeQuality = roadQuality(options.RouteRoadIDs);
    mapData.routeInfo = routeInfo;
    if isfield(routeInfo,"Section")
        mapData.routeRoadIDs = routeInfo.Section.RoadIDs;
        [~,routeIndices] = ismember(mapData.routeRoadIDs,roadIDs);
        mapData.routeQuality = roadQuality(routeIndices);
    end
    mapData.scenarioRoadCount = roadCount+numel(road27Connectors);
    mapData.referenceBoundaryCount = numel(data.referenceBoundaries);
    mapData.widthMode = options.WidthMode;
    mapData.widthProfiles = widthProfiles;
    mapData.renderedRoadCenterlines = renderedRoadCenterlines;
    mapData.boundaryProfileRoadCount = boundaryProfileRoadCount;
    mapData.road27Connectors = road27Connectors;
    mapData.road27ConnectorWidth = road27ConnectorWidth;
    mapData.cameraModuleCompatible = true;
end
end


function output = extendConnectedRoadEnds( ...
    input,currentRoadIndex,roadCenterlines,overlap,tolerance)
output = input;
if overlap<=0 || size(input,1)<2
    return;
end

if endpointTouchesAnotherRoad( ...
        input(1,:),currentRoadIndex,roadCenterlines,tolerance)
    startTangent = unitVector(input(2,:)-input(1,:));
    output = [input(1,:)-overlap*startTangent;output];
end

if endpointTouchesAnotherRoad( ...
        input(end,:),currentRoadIndex,roadCenterlines,tolerance)
    endTangent = unitVector(input(end,:)-input(end-1,:));
    output = [output;input(end,:)+overlap*endTangent];
end
end


function connected = endpointTouchesAnotherRoad( ...
    endpoint,currentRoadIndex,roadCenterlines,tolerance)
connected = false;

for roadIndex = 1:numel(roadCenterlines)
    if roadIndex==currentRoadIndex
        continue;
    end

    otherCenters = double(roadCenterlines{roadIndex});
    distance = min(vecnorm( ...
        otherCenters(:,1:2)-endpoint(1:2),2,2));
    if distance<=tolerance
        connected = true;
        return;
    end
end
end


function output = extendPolylineEnds(input,overlap)
output = input;
if overlap<=0 || size(input,1)<2
    return;
end

startTangent = unitVector(input(2,:)-input(1,:));
endTangent = unitVector(input(end,:)-input(end-1,:));
output = [input(1,:)-overlap*startTangent; ...
    input;input(end,:)+overlap*endTangent];
end


function [connectors,connectorWidth] = ...
    makeRoad27Connectors(roadCenterlines,roadIDs,roadWidths)
targetIndex = find(roadIDs==27,1);
assert(~isempty(targetIndex), ...
    "map_gen_knu:MissingRoad27","Road ID 27 was not found.");

targetRoad = double(roadCenterlines{targetIndex});
assert(size(targetRoad,1)>=2, ...
    "map_gen_knu:ShortRoad27","Road ID 27 needs at least two points.");

leftJunction = meanNearestEndpoints( ...
    roadCenterlines,roadIDs,[22 23 26],targetRoad(1,:));
rightJunction = meanNearestEndpoints( ...
    roadCenterlines,roadIDs,[28 29],targetRoad(end,:));

leftArrivalTangent = unitVector(targetRoad(2,:)-targetRoad(1,:));
rightDepartureTangent = unitVector( ...
    targetRoad(end,:)-targetRoad(end-1,:));
rightArrivalTangent = endpointTangentAwayFromJunction( ...
    roadCenterlines,roadIDs,28,rightJunction);

leftDepartureTangent = unitVector(targetRoad(1,:)-leftJunction);
connectors = {
    cubicConnector(leftJunction,targetRoad(1,:), ...
        leftDepartureTangent,leftArrivalTangent)
    cubicConnector(targetRoad(end,:),rightJunction, ...
        rightDepartureTangent,rightArrivalTangent)
    };
connectorWidth = double(roadWidths(targetIndex));
end


function junction = meanNearestEndpoints( ...
    roadCenterlines,roadIDs,neighborIDs,targetPoint)
junctionPoints = zeros(numel(neighborIDs),3);

for k = 1:numel(neighborIDs)
    roadIndex = find(roadIDs==neighborIDs(k),1);
    assert(~isempty(roadIndex), ...
        "map_gen_knu:MissingNeighborRoad", ...
        "Neighbor road ID %d was not found.",neighborIDs(k));
    centers = double(roadCenterlines{roadIndex});
    endpointDistances = [ ...
        norm(centers(1,1:2)-targetPoint(1:2)), ...
        norm(centers(end,1:2)-targetPoint(1:2))];

    if endpointDistances(1)<=endpointDistances(2)
        junctionPoints(k,:) = centers(1,:);
    else
        junctionPoints(k,:) = centers(end,:);
    end
end

junction = mean(junctionPoints,1);
end


function tangent = endpointTangentAwayFromJunction( ...
    roadCenterlines,roadIDs,roadID,junction)
roadIndex = find(roadIDs==roadID,1);
assert(~isempty(roadIndex), ...
    "map_gen_knu:MissingTangentRoad", ...
    "Road ID %d was not found.",roadID);
centers = double(roadCenterlines{roadIndex});

if norm(centers(1,1:2)-junction(1:2)) <= ...
        norm(centers(end,1:2)-junction(1:2))
    tangent = centers(2,:)-centers(1,:);
else
    tangent = centers(end-1,:)-centers(end,:);
end

tangent = unitVector(tangent);
end


function centers = cubicConnector(p0,p3,startTangent,endTangent)
chordLength = norm(p3(1:2)-p0(1:2));
controlLength = min(2.0,chordLength/3);
p1 = p0+controlLength*startTangent;
p2 = p3-controlLength*endTangent;
sampleCount = max(8,ceil(chordLength/0.25)+1);
t = linspace(0,1,sampleCount).';
centers = (1-t).^3.*p0 + ...
    3*(1-t).^2.*t.*p1 + ...
    3*(1-t).*t.^2.*p2 + ...
    t.^3.*p3;
end


function output = unitVector(input)
inputNorm = norm(input(1:2));
assert(inputNorm>1e-9, ...
    "map_gen_knu:ZeroTangent","Cannot form a connector from a zero tangent.");
output = input/inputNorm;
end


function [laneSpecification,profile] = makeBoundaryInformedLaneSpec( ...
    roadCenters,baseWidth,roadID,quality,widthSamples,options)

totalLength = cumulativeDistance(roadCenters);
totalLength = totalLength(end);
roadRows = widthSamples.road_id==roadID;
trustedRows = roadRows & widthSamples.valid~=0 & ...
    widthSamples.inlier~=0 & isfinite(widthSamples.station_m) & ...
    isfinite(widthSamples.width_m);

trustedStations = double(widthSamples.station_m(trustedRows));
trustedWidths = double(widthSamples.width_m(trustedRows));
qualitySupportsProfile = any(quality==["Reliable","Moderate"]);

if ~qualitySupportsProfile || ...
        numel(trustedWidths)<options.MinLocalInliers || totalLength<2.0
    laneSpecification = lanespec(1,"Width",baseWidth);
    profile = makeConstantProfile(roadID,roadCenters,baseWidth, ...
        "per-road-boundary-estimate");
    return;
end

[trustedStations,uniqueIndex] = unique(trustedStations,"stable");
trustedWidths = trustedWidths(uniqueIndex);

segmentCount = min(options.MaxWidthSegments, ...
    max(2,ceil(totalLength/options.WidthSegmentLength)));
segmentEdges = linspace(0,totalLength,segmentCount+1);
segmentCenters = (segmentEdges(1:end-1)+segmentEdges(2:end))/2;
localWidths = zeros(1,segmentCount);

for segmentIndex = 1:segmentCount
    insideSegment = trustedStations>=segmentEdges(segmentIndex) & ...
        trustedStations<=segmentEdges(segmentIndex+1);

    if any(insideSegment)
        localWidths(segmentIndex) = median(trustedWidths(insideSegment));
    else
        [~,nearestSample] = min(abs( ...
            trustedStations-segmentCenters(segmentIndex)));
        localWidths(segmentIndex) = trustedWidths(nearestSample);
    end
end

localWidths = movmedian(localWidths,3,"Endpoints","shrink");
localWidths = min(max(localWidths,options.MinimumRoadWidth), ...
    options.MaximumRoadWidth);

if max(localWidths)-min(localWidths)<0.10
    width = median(localWidths);
    laneSpecification = lanespec(1,"Width",width);
    profile = makeConstantProfile(roadID,roadCenters,width, ...
        "local-boundary-profile-nearly-constant");
    return;
end

laneSpecificationCells = cell(1,segmentCount);
for segmentIndex = 1:segmentCount
    laneSpecificationCells{segmentIndex} = lanespec(1, ...
        "Width",localWidths(segmentIndex));
end

laneSpecificationArray = [laneSpecificationCells{:}];
segmentRange = diff(segmentEdges)/totalLength;
segmentRange(end) = 1-sum(segmentRange(1:end-1));
taperLength = min(4.0,max(0.5,0.5*totalLength/segmentCount));
connector = laneSpecConnector("TaperShape","Linear", ...
    "TaperLength",taperLength);
laneSpecification = compositeLaneSpec(laneSpecificationArray, ...
    "SegmentRange",segmentRange,"Connector",connector);

profile = struct();
profile.RoadID = roadID;
profile.Source = "local-original-boundary-distances";
profile.UsesLongitudinalProfile = true;
profile.Quality = quality;
profile.SegmentEdges = segmentEdges;
profile.SegmentWidths = localWidths;
profile.TrustedSampleCount = numel(trustedWidths);
end


function profile = makeConstantProfile(roadID,roadCenters,width,source)
stations = cumulativeDistance(roadCenters);
profile = struct();
profile.RoadID = roadID;
profile.Source = source;
profile.UsesLongitudinalProfile = false;
profile.Quality = "";
profile.SegmentEdges = [0 stations(end)];
profile.SegmentWidths = width;
profile.TrustedSampleCount = 0;
end


function validateWidthSampleTable(widthSamples)
requiredVariables = ["road_id","station_m","valid","inlier","width_m"];
availableVariables = string(widthSamples.Properties.VariableNames);

for k = 1:numel(requiredVariables)
    assert(any(availableVariables==requiredVariables(k)), ...
        "Width sample CSV is missing column: %s",requiredVariables(k));
end
end


function quality = normalizeQuality(input)
if ischar(input)
    quality = string(cellstr(input));
else
    quality = string(input);
end
quality = quality(:);
end


function output = trimPolyline(input,startOffset,endOffset)
input = removeDuplicatePoints(input);
s = cumulativeDistance(input);
totalLength = s(end);

assert(startOffset+endOffset<totalLength, ...
    "StartOffset + EndOffset must be shorter than the selected route.");

sampleStations = unique([startOffset;s(s>startOffset & ...
    s<totalLength-endOffset);totalLength-endOffset]);
output = interpolatePolyline(input,s,sampleStations);
end


function output = resamplePolyline(input,spacing)
input = removeDuplicatePoints(input);
s = cumulativeDistance(input);

sampleStations = (0:spacing:s(end)).';
if isempty(sampleStations) || sampleStations(end)<s(end)
    sampleStations(end+1,1) = s(end);
end

output = interpolatePolyline(input,s,sampleStations);
end


function output = interpolatePolyline(input,s,sampleStations)
output = zeros(numel(sampleStations),size(input,2));
for column = 1:size(input,2)
    output(:,column) = interp1(s,input(:,column), ...
        sampleStations,"linear");
end
end


function output = removeDuplicatePoints(input)
distance = vecnorm(diff(input(:,1:2),1,1),2,2);
output = input([true;distance>1e-6],:);
end


function s = cumulativeDistance(input)
s = [0;cumsum(vecnorm(diff(input(:,1:2),1,1),2,2))];
end
