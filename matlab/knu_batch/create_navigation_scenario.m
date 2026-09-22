function [scenario,egoVehicle,waypoints,scaleProfile,mapInfo] = ...
    create_navigation_scenario(mapName,varargin)
%CREATE_NAVIGATION_SCENARIO Create a matched road, ego, and waypoint mission.

ip=inputParser;
addParameter(ip,"ScaleMode","model",@(x)ischar(x)||isstring(x));
addParameter(ip,"RouteName","target",@(x)ischar(x)||isstring(x));
addParameter(ip,"GeoReference",[], ...
    @(x)isempty(x)||(isnumeric(x)&&numel(x)==3));
parse(ip,varargin{:});

mapKey=lower(regexprep(strtrim(string(mapName)),'[ _-]',''));
isKnu=any(mapKey==["knu","mapknu","mapgenknu","campus"]);

if isKnu
    scaleProfile=experiment_scale_profile(string(ip.Results.ScaleMode),"fullscale");
    if scaleProfile.Mode~="fullscale"
        error("create_navigation_scenario:KnuRequiresFullscale", ...
            "map_gen_knu uses real KNU coordinates and requires ScaleMode='fullscale'.");
    end
    dataFile=fullfile(fileparts(mfilename("fullpath")),"processed_road_map.mat");
    assert(isfile(dataFile), ...
        "KNU processed map file not found: %s",dataFile);
    geoData=load(dataFile,"geoReference");
    assert(isfield(geoData,"geoReference"), ...
        "processed_road_map.mat is missing geoReference.");
    defaultGeoReference=double(geoData.geoReference(:)).';
else
    scaleProfile=experiment_scale_profile(string(ip.Results.ScaleMode),"model");
    defaultGeoReference=[35.89663 128.61677 0];
end

if isempty(ip.Results.GeoReference)
    geoReference=defaultGeoReference;
else
    geoReference=double(ip.Results.GeoReference(:)).';
end

scenario=drivingScenario("GeoReference",geoReference);
v=scaleProfile.Vehicle;
egoVehicle=vehicle(scenario,"ClassID",1,"Position",[0 0 0],"Yaw",90, ...
    "Length",v.Length,"Width",v.Width,"Height",v.Height, ...
    "Wheelbase",v.Wheelbase,"RearOverhang",v.RearOverhang, ...
    "Mesh",driving.scenario.carMesh,"Name","Car");

if isKnu
    routeName=normalizeKnuRouteName(ip.Results.RouteName);
    [~,~,sectionRoutes]=knu_section_routes();
    [waypoints,mapData]=map_gen_knu(scenario,egoVehicle, ...
        "WaypointScenario",routeName,"PlaceEgo",true);
    mapInfo=struct( ...
        "MapName","knu", ...
        "RouteName",routeName, ...
        "ScaleProfile",scaleProfile, ...
        "RoadWidth",median(double(mapData.roadWidths(:)),"omitnan"), ...
        "AvailableRoutes",["scenario"+string(1:4),sectionRoutes], ...
        "MapData",mapData);
else
    [waypoints,mapInfo]=map_gen_waypoints(scenario,egoVehicle,mapName, ...
        "ScaleMode",ip.Results.ScaleMode,"RouteName",ip.Results.RouteName);
end
scaleProfile=mapInfo.ScaleProfile;
end

function routeName=normalizeKnuRouteName(input)
routeName=lower(regexprep(strtrim(string(input)),'[ _-]',''));
if any(routeName==["target","default","upper","route1"])
    routeName="scenario1";
elseif any(routeName==["decoy","lower","route2"])
    routeName="scenario2";
end
routeNumber=regexp(routeName,'^(?:route|scenario)(\d+)$','tokens','once');
sectionNumber=regexp(routeName,'^section(\d+)$','tokens','once');
if ~isempty(routeNumber)
    routeName="scenario"+string(str2double(routeNumber{1}));
elseif ~isempty(sectionNumber) && str2double(sectionNumber{1})>=1
    routeName="scenario"+string(str2double(sectionNumber{1})+4);
end
[~,~,sectionRoutes]=knu_section_routes();
availableRoutes=["scenario"+string(1:4),sectionRoutes];
if ~any(routeName==availableRoutes)
    error("create_navigation_scenario:UnknownKnuRoute", ...
        "Unknown KNU route '%s'. Choose %s (or section1 ... section%d).", ...
        input,strjoin(availableRoutes,", "),numel(sectionRoutes));
end
end
