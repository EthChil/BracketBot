import time

mappings = {}


mappingFile = open("4671RegMap.txt", 'r')

#UNCOMMENT TO GENERATE LEFT SIDE
#setupFile = open("TrinamicConfig/test_4_L.c", 'r')
#output = open("TrinamicConfig/parsedConfigOpenloopL.txt", 'w')

#UNCOMMENT TO GENERATE RIGHT SIDE
#setupFile = open("TrinamicConfig/test_4_R.c", 'r')
#output = open("TrinamicConfig/parsedConfigOpenloopR.txt", 'w')

#UNCOMMENT TO GENERATE TEST FILE
# setupFile = open("TrinamicConfig/parse_test.c", 'r')
# output = open("TrinamicConfig/parse_test_output.txt", 'w')

for line in mappingFile:
    values = line.split()
    print(values)

    mappings[values[0]] = int(values[1], 16)

for line in setupFile:
    if(("//" in line and ";" not in line ) or len(line) < 5):
        print("skipped", line)
        continue

    if("wait" in line):
        output.write("wait\n")
        continue

    splitForm = line.replace(",", "").replace(");", "").split()[1:]
    print(splitForm)

    data = []

    for i in range(1, 5):
        data.append(int(splitForm[1][2*i:(2*i)+2], 16))

    output.write(str(mappings[splitForm[0]]) + ',' + str(data[0]) + ',' + str(data[1]) + ',' + str(data[2]) + ',' + str(data[3]) + '\n')

output.close()
setupFile.close()
mappingFile.close()

