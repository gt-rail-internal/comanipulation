function[] = computeDTW()

baseDir = "./";

testDir = baseDir + "csvFiles/Test/";
predDir = baseDir + "csvFiles/Predictions/";

predTrajCsvFile = dir(fullfile(predDir,'*.csv'));

for index = 1:length(predTrajCsvFile)
    if contains(predTrajCsvFile(index).name, "predtraj_")
        currPredPath = predTrajCsvFile(index).folder+"/"+predTrajCsvFile(index).name;
        predNameSplit = split(predTrajCsvFile(index).name,"_");
        currTrainPath = testDir+"traj_"+predNameSplit(2)+".csv"

        currTrain = Read_Traj(currTrainPath);
        currPred = Read_Traj(currPredPath);

        obsTimesteps = 100;
        currTrain = currTrain(obsTimesteps:end,:);

        if length(currPred) > length(currTrain)
            currPred = currPred(1:length(currTrain),:);
        else
            currTrain = currTrain(1:length(currPred),:);
        end
        currPred(:,1) = currPred(:,1) + (obsTimesteps-1);
        distance = DTW_dis(currTrain, currPred,0) 
    end
end
