function [waypoints,routeInfo] = ...
    get_navigation_waypoints(mapName,scenarioName)
%GET_NAVIGATION_WAYPOINTS Return a route scenario within a selected map.
%
% The map selects the road geometry. The scenario selects only one route
% (a waypoint set) that belongs to that map.

arguments
    mapName = "yintersection"
    scenarioName = "scenario1"
end

mapKey = normalizeMapName(mapName);
scenarioKey = normalizeScenarioName(scenarioName);
sectionInfo = struct();
if mapKey=="knu"
    sectionNumber = regexp(scenarioKey,"^section(\d+)$","tokens","once");
    if ~isempty(sectionNumber)
        [~,~,sectionRoutes] = knu_section_routes();
        number = str2double(sectionNumber{1});
        assert(number>=1 && number<=numel(sectionRoutes), ...
            "get_navigation_waypoints:UnknownSection", ...
            "Choose section1 ... section%d.",numel(sectionRoutes));
        scenarioKey = "scenario"+string(number+4);
    end
end

switch mapKey
    case "yintersection"
        mapDisplayName = "Y intersection";
        switch scenarioKey
            case "scenario1"
                routeDisplayName = "Upper branch";
                waypoints = [ ...
                     0   0.0  0
                    10   0.0  0
                    20   0.0  0
                    35   5.0  0
                    40   6.5  0
                    45   8.0  0
                ];
            case "scenario2"
                routeDisplayName = "Lower branch";
                waypoints = [ ...
                     0   0.0  0
                    10   0.0  0
                    20   0.0  0
                    35  -5.0  0
                    40  -6.5  0
                    45  -8.0  0
                ];
            otherwise
                throwUnknownScenario(mapKey,scenarioKey,"scenario1, scenario2");
        end

    case "roundabout"
        mapDisplayName = "Roundabout";
        switch scenarioKey
            case "scenario1"
                routeDisplayName = "Upper left";
                waypoints = [ ...
                     0   0  0
                    20   2  0
                    30  12  0
                    50   2  0
                    60   0  0
                ];
            case "scenario2"
                routeDisplayName = "Upper right";
                waypoints = [ ...
                    60    0  0
                    50   2  0
                    38   7  0
                    30  10  0
                    20    0  0
                    0   0   0
                ];
            otherwise
                throwUnknownScenario(mapKey,scenarioKey,"scenario1, scenario2");
        end

    case "branch"
        mapDisplayName = "Two-branch road";
        switch scenarioKey
            case "scenario1"
                routeDisplayName = "Upper branch";
                waypoints = [ ...
                     0   0.0  0
                    18   0.0  0
                    32   5.8  0
                    40   6.8  0
                    48   3.0  0
                    54   0.0  0
                ];
            case "scenario2"
                routeDisplayName = "Lower branch";
                waypoints = [ ...
                     0   0.0  0
                    18   0.0  0
                    24  -2.5  0
                    34  -4.5  0
                    48  -8.0  0
                ];
            otherwise
                throwUnknownScenario(mapKey,scenarioKey,"scenario1, scenario2");
        end
    case {"knu","osm"}
        if mapKey=="knu"
            mapDisplayName = "Processed KNU road map";
        else
            mapDisplayName = "Generated OSM road map";
        end

        switch scenarioKey
            case "scenario1"
                routeDisplayName = "Southeast to northeast";
                waypoints = [ ...
                    150  -140   0
                    130  -103   0
                    108   -59   0
                    110   -29   0
                    118   -15   0
                    173    -2   0
                ];
            case "scenario2"
                routeDisplayName = "West to northeast";
                waypoints = [ ...
                   -250   101   0
                   -209    86   0
                   -201    74   0
                   -133    56   0
                    -76    67   0
                    -26    84   0
                     -4    95   0
                ];
            case "scenario3"
                routeDisplayName = "North to northeast via roundabout";
                waypoints = [ ...
                        187 220 0
                        187 192 0
                        195 168 0
                        215 157 0
                        235 142 0
                        255 135 0
                        270 158 0
                        278 172 0
                ];
            case "scenario4"
                routeDisplayName = "West to east via southeast junction";
                waypoints = [ ...
                     0     7    0
                     50    5   0
                     85    -5   0
                     104   -20   0
                     120    -14  0
                     130   -10   0
                    150   -4   0
                ];

            otherwise
                if mapKey=="knu"
                    [waypoints,sectionInfo] = knu_section_routes(scenarioKey);
                    routeDisplayName = sectionInfo.ScenarioName;
                else
                    throwUnknownScenario(mapKey,scenarioKey, ...
                        "scenario1, scenario2, scenario3, scenario4");
                end
        end

    case "knulegacy"
        mapDisplayName = "Legacy generated KNU road map";
        switch scenarioKey
            case "scenario1"
                routeDisplayName = "Legacy route";
                waypoints = [ ...
                    -100    40.0   0
                    -110    49.0   0
                    -130    65.0   0
                    -153    86.5   0
                    -160   116.0   0
                    -216   152.0   0
                    -170    77.0   0
                     629  -217.5   0
                     652.9 -187.4  0
                ];
            otherwise
                throwUnknownScenario(mapKey,scenarioKey,"scenario1");
        end
end

waypoints = double(waypoints);
routeInfo = struct( ...
    "MapId",mapKey, ...
    "MapName",mapDisplayName, ...
    "ScenarioId",scenarioKey, ...
    "ScenarioName",routeDisplayName);
if ~isempty(fieldnames(sectionInfo))
    routeInfo.Section = sectionInfo;
end
end


function mapKey = normalizeMapName(mapName)
if isnumeric(mapName)
    validateattributes(mapName,{'numeric'}, ...
        {'scalar','integer','positive','finite'},mfilename,'mapName');
    mapKey = "map"+string(mapName);
else
    assert((ischar(mapName) && isrow(mapName)) || ...
        (isstring(mapName) && isscalar(mapName)), ...
        "get_navigation_waypoints:InvalidMapName", ...
        "mapName must be a text scalar or a positive integer.");
    mapKey = lower(strtrim(string(mapName)));
    mapKey = replace(mapKey,[" ","_","-"],"");
end

switch mapKey
    case {"map1","yintersection","y","mapgenyintersection"}
        mapKey = "yintersection";
    case {"map2","roundabout","mapgenroundabout"}
        mapKey = "roundabout";
    case {"map3","branch","twobranch","mapgen6"}
        mapKey = "branch";
    case {"map4","fiveway","mapgen8"}
        mapKey = "fiveway";
    case {"map5","knu","mapgenknu"}
        mapKey = "knu";
    case {"map6","osm","mapgenosm"}
        mapKey = "osm";
    case {"map7","knulegacy","mapgen3"}
        mapKey = "knulegacy";
    otherwise
        error("get_navigation_waypoints:UnknownMap", ...
            ["Unknown map '%s'. Choose yintersection, roundabout, branch, " ...
             "fiveway, knu, osm, or knulegacy."],mapKey);
end
end


function scenarioKey = normalizeScenarioName(scenarioName)
if isnumeric(scenarioName)
    validateattributes(scenarioName,{'numeric'}, ...
        {'scalar','integer','positive','finite'},mfilename,'scenarioName');
    scenarioKey = "scenario"+string(scenarioName);
else
    assert((ischar(scenarioName) && isrow(scenarioName)) || ...
        (isstring(scenarioName) && isscalar(scenarioName)), ...
        "get_navigation_waypoints:InvalidScenarioName", ...
        "scenarioName must be a text scalar or a positive integer.");
    scenarioKey = lower(strtrim(string(scenarioName)));
    scenarioKey = replace(scenarioKey,[" ","_","-"],"");
end

scenarioNumber = regexp(scenarioKey,"^(?:scenario|route)(\d+)$", ...
    "tokens","once");
if ~isempty(scenarioNumber)
    scenarioKey = "scenario"+scenarioNumber{1};
end
end


function throwUnknownScenario(mapKey,scenarioKey,availableScenarios)
error("get_navigation_waypoints:UnknownScenario", ...
    "Map '%s' has no '%s'. Available scenarios: %s.", ...
    mapKey,scenarioKey,availableScenarios);
end
