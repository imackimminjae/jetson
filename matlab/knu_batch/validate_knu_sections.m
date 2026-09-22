function validate_knu_sections()
%VALIDATE_KNU_SECTIONS Check gentle OSM geometry and scenario integration.
[~,~,routes]=knu_section_routes();
assert(numel(routes)==17);
covered=[];
allStarts=zeros(0,2); allEnds=zeros(0,2);
for k=1:numel(routes)
    [wp,info]=get_navigation_waypoints('knu',routes(k));
    assert(size(wp,2)==3 && size(wp,1)>=2 && all(isfinite(wp),'all'));
    assert(all(wp(:,3)==0));
    spacing=vecnorm(diff(wp(:,1:2)),2,2);
    assert(all(spacing>1e-4 & spacing<=5.01));
    delta=diff(wp(:,1:2)); unit=delta./spacing;
    turn=acosd(max(-1,min(1,sum(unit(1:end-1,:).*unit(2:end,:),2))));
    a=delta(1:end-1,:); b=delta(2:end,:);
    crossMagnitude=abs(a(:,1).*b(:,2)-a(:,2).*b(:,1));
    radius=spacing(1:end-1).*spacing(2:end).*vecnorm(a+b,2,2) ./ ...
        max(2*crossMagnitude,1e-12);
    assert(max(turn)<=25.1 && min(radius)>=11.9, ...
        'Abrupt turn in %s.',routes(k));
    along=[0;cumsum(spacing)]; early=wp(along<along(end)-25,1:2);
    assert(distanceToSegments(wp(end,1:2),early(1:end-1,:),early(2:end,:))>5, ...
        'The final point is passed earlier in %s.',routes(k));
    allStarts=[allStarts;wp(1:end-1,1:2)]; %#ok<AGROW>
    allEnds=[allEnds;wp(2:end,1:2)]; %#ok<AGROW>
    assert(isequal(wp,get_navigation_waypoints('knu','section'+string(k))));
    assert(isequal(wp,get_navigation_waypoints('knu',k+4)));
    assert(isequal(wp,get_navigation_waypoints('knu','route'+string(k+4))));
    assert(info.Section.SectionNumber==k);
    assert(info.Section.CoverageToleranceMeters==20);
    covered=[covered,info.Section.RoadIDs]; %#ok<AGROW>
end
assert(isequal(unique(covered),1:45),'Not all 45 road areas are represented.');
data=load(fullfile(fileparts(mfilename('fullpath')),'processed_road_map.mat'), ...
    'roadCenterlines');
for k=1:numel(data.roadCenterlines)
    center=data.roadCenterlines{k};
    assert(all(distanceToSegments(center(:,1:2),allStarts,allEnds)<=20), ...
        'OSM route does not represent processed road %d within 20 m.',k);
end
for invalid=["section0","section18","scenario22"]
    rejected=false;
    try
        get_navigation_waypoints('knu',invalid);
    catch
        rejected=true;
    end
    assert(rejected,'Invalid route was accepted.');
end
for k=[5 9 13]
    [~,ego,wp,~,info]=create_navigation_scenario('knu', ...
        'ScaleMode','fullscale','RouteName','section'+string(k));
    assert(isequal(wp,get_navigation_waypoints('knu',k+4)));
    assert(isequal(ego.Position,wp(1,:)));
    heading=atan2d(wp(2,2)-wp(1,2),wp(2,1)-wp(1,1));
    assert(abs(ego.Yaw-heading)<1e-9);
    assert(numel(info.AvailableRoutes)==21);
    assert(info.MapData.scenarioRoadCount==47);
    assert(isequal(info.MapData.routeRoadIDs,info.MapData.routeInfo.Section.RoadIDs));
    assert(all(ismember(info.MapData.routeRoadIDs,1:45)));
end
fprintf(['PASS: 17 gentle OSM sections, <=25 deg waypoint turns, >=12 m sampled radius, ' ...
    '<=5 m spacing, 45 road areas within 20 m, and scenario9/13/17 builds.\n']);
end

function distance=distanceToSegments(points,starts,ends)
distance=inf(size(points,1),1);
for k=1:size(starts,1)
    delta=ends(k,:)-starts(k,:);
    t=max(0,min(1,sum((points-starts(k,:)).*delta,2)/sum(delta.^2)));
    nearest=starts(k,:)+t.*delta;
    distance=min(distance,vecnorm(points-nearest,2,2));
end
end
