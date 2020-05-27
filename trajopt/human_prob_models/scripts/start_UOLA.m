function[] = start_UOLA()
numTraj = 0;
numPrediction = 0;
baseDir = "./";
modelDir = baseDir + "TRO_model.mat";
UOLA_init(modelDir);

myDir = baseDir + "csvFiles/Train/";
myFiles = dir(fullfile(myDir,'*.csv'));
myFiles(randperm(length(myFiles)));
for k = 1:length(myFiles)
  currTrainFile = myFiles(k).folder + "/" + myFiles(k).name
  UOLA_learn(modelDir, currTrainFile)
end

myDir = baseDir + "csvFiles/Test/";
testTrajCsvFile = dir(fullfile(myDir,'*.csv'));
for i = 1:length(testTrajCsvFile)
    if contains(testTrajCsvFile(i).name, "trimmed")
        observation = testTrajCsvFile(i).folder+"/"+testTrajCsvFile(i).name;
        predictionMeans = testTrajCsvFile(i).folder+"/../Predictions/"+"pred"+testTrajCsvFile(i).name;
        predictionVar = testTrajCsvFile(i).folder+"/../Predictions/"+"varPred"+testTrajCsvFile(i).name;
        sampledMeans = testTrajCsvFile(i).folder+"/../Predictions/"+"predSampled"+testTrajCsvFile(i).name;
        sampledVar = testTrajCsvFile(i).folder+"/../Predictions/"+"varPredSampled"+testTrajCsvFile(i).name;
        UOLA_predict(modelDir,observation,predictionMeans,predictionVar,sampledMeans, sampledVar)
    end
end
"Done"