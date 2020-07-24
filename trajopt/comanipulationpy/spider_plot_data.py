import os
import csv
import matlab.engine

def test_to_format(test_number, lines):
    """
    Takes the test number and will output the formatted lists for that test

    test_number: line number in csv minus 2 (ignore column header and start index at 0)
    lines: all of the csv rows formatted as a list of strings
    """
    eng = matlab.engine.start_matlab()
    eng.cd('spiderPlot', nargout=0)
    l = lines[test_number]
    matlab_inputs = []
    for i, t in zip(range(5), [0,2,3,4,1]):
        avgs = []
        sds = []
        min_sd = 0.0001
        for j in range(4):
            data = l[(t * 4) + (j + 1)]
            avg_sd = data.split(' +/- ')
            avg = float('%.4f'%(float(avg_sd[0]))) * 100
            sd = float('%.4f'%(float(avg_sd[1]))) * 100
            if j == 3:
                avg /= -100
                sd /= 100
            if sd < min_sd:
                sd = min_sd
            avgs.append(avg)
            sds.append(sd)
        matlab_inputs.append([e for e in avgs])
        matlab_inputs.append([e for e in sds])
        # print("D" + str(i + 1) + " = [" + ' '.join([str(elem) for elem in avgs]) + "];")
        # print("E" + str(i + 1) + " = [" + ' '.join([str(elem)+"," for elem in sds[:-1]]) + str(sds[-1]) + "];")
        avgs = []
        sds = []
    eng.spiderPlot(matlab.double(matlab_inputs), nargout=0)


def make_spider_plot():
    """
    Takes the data from ExperimentResults and outputs the format of the data to create the graphs.
    Will be able to copy and paste the output to radar_test.m for graph creation.
    """
    file_name = '../human_prob_models/scripts/csvFiles/ExperimentResults.csv'
    file = csv.reader(open(file_name), delimiter=',')
    lines = []
    for line in file:
        lines.append(line)
    lines = lines[1:]
    test_to_format(-1, lines)