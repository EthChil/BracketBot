import time

mappings = {}

mappingFile = open("4671RegMap.txt", 'r')
setupFile = open("TrinamicConfig/test_4_L_openloop.c", 'r')
output = open("TrinamicConfig/parsedConfigOpenloop.txt", 'w')

for line in mappingFile:
    values = line.split()
    print(values)

    mappings[values[0]] = int(values[1], 16)

for line in setupFile:
    if("//" in line or len(line) < 5):
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


