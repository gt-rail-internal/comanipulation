import os
import csv

def test_to_format(test_number, lines):
    """
    Takes the test number and will output the formatted lists for that test

    test_number: line number in csv minus 2 (ignore column header and start index at 0)
    lines: all of the csv rows formatted as a list of strings
    """
    l = lines[test_number]
    test = l[0]
    print("Test:", test)
    for i, t in zip(range(5), [0,2,3,4,1]):
        avgs = []
        sds = []
        for j in range(4):
            data = l[(t * 4) + (j + 1)]
            avg_sd = data.split(' +/- ')
            avg = float('%.4f'%(float(avg_sd[0]))) * 100
            sd = float('%.4f'%(float(avg_sd[1]))) * 100
            if j == 3:
                avg /= 100
                avg *= -1
                sd /= 100
            avgs.append(avg)
            sds.append(sd)
        print("D" + str(i + 1) + " = [" + ' '.join([str(elem) for elem in avgs]) + "];")
        print("E" + str(i + 1) + " = [" + ' '.join([str(elem)+"," for elem in sds[:-1]]) + str(sds[-1]) + "];")
        avgs = []
        sds = []


def to_spider_plot_format():
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
    test_to_format(1, lines)

to_spider_plot_format()