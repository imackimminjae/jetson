function summary = preview_knu_sections(sectionNumber)
%PREVIEW_KNU_SECTIONS Plot all sections, or one section (1 ... 17), without ROS.
if nargin<1, sectionNumber=[]; end
[~,~,routes]=knu_section_routes();
if ~isempty(sectionNumber)
    validateattributes(sectionNumber,{'numeric'}, ...
        {'scalar','integer','>=',1,'<=',numel(routes)});
    routes=routes(sectionNumber);
end
dataFile=fullfile(fileparts(mfilename('fullpath')),'processed_road_map.mat');
data=load(dataFile,'roadCenterlines');
figure('Name','KNU road sections','Color','w');
if numel(routes)>1
    tiledlayout(ceil(numel(routes)/4),4,'TileSpacing','compact');
else
    tiledlayout(1,1);
end
names=strings(numel(routes),1); lengths=zeros(numel(routes),1);
for k=1:numel(routes)
    ax=nexttile; hold(ax,'on');
    for r=1:numel(data.roadCenterlines)
        p=data.roadCenterlines{r};
        plot(ax,p(:,1),p(:,2),'Color',[.8 .82 .85]);
    end
    [wp,info]=knu_section_routes(routes(k));
    plot(ax,wp(:,1),wp(:,2),'-','LineWidth',1.6,'Color',[.1 .35 .75]);
    scatter(ax,wp(1,1),wp(1,2),36,[.1 .6 .3],'filled');
    scatter(ax,wp(end,1),wp(end,2),36,[.8 .15 .15],'s','filled');
    index=unique(round(linspace(1,size(wp,1)-1,5)));
    direction=diff(wp(:,1:2));
    quiver(ax,wp(index,1),wp(index,2),direction(index,1),direction(index,2), ...
        0,'Color',[.1 .35 .75],'MaxHeadSize',2);
    title(ax,sprintf('section%d / %s / %.0f m', ...
        info.SectionNumber,routes(k),info.LengthMeters),'Interpreter','none');
    axis(ax,'equal'); grid(ax,'on');
    if numel(routes)>1, xlim(ax,[-285 325]); ylim(ax,[-175 265]); end
    xlabel(ax,'East [m]'); ylabel(ax,'North [m]');
    names(k)=info.ScenarioName; lengths(k)=info.LengthMeters;
end
summary=table(routes(:),names,lengths, ...
    'VariableNames',{'Route','Description','LengthMeters'});
end
