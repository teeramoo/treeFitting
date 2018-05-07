pkg load signal
filename = "testRead.csv";
data = dlmread (filename, ',',1,0)
typeinfo(data)
rows(data)
columns(data)
number1 = data(1)
pointCloud = data(:,2)
[pks idx] = findpeaks(pointCloud);
pks
idx
