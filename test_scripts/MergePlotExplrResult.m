function MergePlotExplrResult(ExplrName)
%%  This func merges the robot exploration results and plot them
%   By Long Wang, 2016/11
Setup_Directories_PSM;
root_path = fileparts(getenv('PSMCMD'));
data_path = [root_path,'/Data'];
%%  The Exploration Data Sets
if nargin<1
    ExplrName = 'JMR';
end
switch ExplrName
    case 'JMR'
        ZoneNames = cell(4,1);
        ZoneNames{1} = 'zone1_adp_wrist_cdp';
        ZoneNames{2} = 'zone2_adp_wrist_cdp';
        ZoneNames{3} = 'zone3_adp_wrist_cdp';
        ZoneNames{4} = 'zone4_adp_wrist_cdp';
        ZoneColors = {'g','m','c','b'};
        pointCloud_All_Zones = cell(4,1);
    case 'Hamlyn'
        ZoneNames = cell(4,1);
        ZoneNames{1} = 'Hamlyn_V2_Z1';
        ZoneNames{2} = 'Hamlyn_V2_Z2';
        ZoneNames{3} = 'Hamlyn_V2_Z3';
        ZoneNames{4} = 'Hamlyn_V2_Z4';
        ZoneColors = {'g','m','c','b'};
        pointCloud_All_Zones = cell(4,1);
    case 'KidneyVU'
        ZoneNames = cell(7,1);
        ZoneNames{1} = 'KidneyZ1-21Oct';
        ZoneNames{2} = 'KidneyZ2-21Oct';
        ZoneNames{3} = 'KidneyZ3-21Oct';
        ZoneNames{4} = 'KidneyZ4-21Oct';
        ZoneNames{5} = 'KidneyZ5-21Oct';
        ZoneNames{6} = 'KidneyZ6-21Oct';
        ZoneNames{7} = 'KidneyZ7-21Oct';
        ZoneColors = {'g','m','c','b','r','y','k'};
%         ZoneColors = {'g','m','c'};
        pointCloud_All_Zones = cell(7,1);
end
N_zones = length(ZoneNames);
for i = 1:N_zones
    %   load exploration data
    load([data_path,'/',ZoneNames{i}]);
    %   compute contact and plot
    logger.compute_contact;
    if i==1
        logger.plot_explr_map('new figure','on',...
            'MarkerSize',1,'MarkerColor',ZoneColors{i});
    else
        logger.plot_explr_map('new figure','off',...
            'MarkerSize',1,'MarkerColor',ZoneColors{i});
    end
    fprintf('%0.0f zones of %0.0f finished ... \n',i,N_zones);
    %   pass to point cloud data structure
    contact_positions = logger.plotData.contact_pos;
    contact_flags = logger.plotData.contact_flags;
    pointCloud_All_Zones{i} = pointCloud(contact_positions(:,contact_flags==1)');
end
fprintf(' [ok].')
%%  merge point clouds
MergeGridStep = 0.5;
for i=1:N_zones
    if i==1
        pointCloudMerge = pointCloud_All_Zones{i};
    else
        pointCloudMerge = pcmerge(pointCloudMerge,pointCloud_All_Zones{i},MergeGridStep);
    end
end
RobotExplrPtCloud = pcdownsample(pointCloudMerge,'gridAverage',MergeGridStep);
save([data_path,'/PSMExplrPtCloud','_',ExplrName],'RobotExplrPtCloud');
pcwrite(RobotExplrPtCloud,[data_path,'/PSMExplrPtCloud','_',ExplrName],'PLYFormat','binary');
end


