function Setup_Directories_data()
Data_path = getenv('NRIDATASHARE');
fprintf('Setting up all the directories ..')
restoredefaultpath;
addpath(...
    genpath(Data_path));
rmpath(...
    genpath([Data_path,filesep,'.git']));
fprintf('..[ok]\n');
end