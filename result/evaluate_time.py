import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import sys

argc=len(sys.argv)
print("argc=",argc)
filenames = []
index_vec,modified_lba_vec = [],[]
master_lba_vec = []
modified_lba_vec.append([])
for i in range(argc):
    if i != 0:
        file = sys.argv[i]
        filenames.append(file)
min_index=sys.maxsize
file_index=0
for file in filenames:
    index,gtsam_t1,gtsam_t2 = [],[],[]
    print(file)
    with open(file, 'r') as f:
        lines = f.readlines()
        data_index=0
        for line in lines:
            value = [float(s) for s in line.split(' ')]
            index.append(data_index)
            if value[1] > 500.0:
                value[1] = 0
            if file_index==0:
                master_lba_vec.append(value[1])
            else:
                modified_lba_vec[file_index-1].append(value[1])
            print(value[1])
            data_index+=1
    index_vec.append(index)
    min_index=min(min_index, len(index))
    print(len(modified_lba_vec[0]))
    modified_lba_vec.append(gtsam_t2)
    file_index+=1

arr_mean = np.mean(master_lba_vec)
arr_var = np.var(master_lba_vec)
arr_std = np.std(master_lba_vec, ddof=1)
arr_mean2 = np.mean(modified_lba_vec[0])
arr_var2 = np.var(modified_lba_vec[0])
arr_std2 = np.std(modified_lba_vec[0], ddof=1)
print("Master:")
print(arr_mean)
print(arr_std)
print("Modified:")
print(arr_mean2)
print(arr_std2)
arr_mean=str(arr_mean)
arr_std=str(arr_std)
arr_mean2=str(arr_mean2)
arr_std2=str(arr_std2)
print("--------------")

# # PLOT
file_index=0
for i in range(len(filenames)):
    names = filenames[i].split('/')
    name=(names[len(names)-1].split('.'))[0]
    tmp_index=index_vec[i][0:min_index-1]
    if file_index <=4 :
        data=master_lba_vec[0:min_index-1]
        plt.plot(tmp_index, data, label="Master :" + str(file_index) +name+"(mean:"+arr_mean[0:5]+"; std:"+ arr_std[0:5] + ")")
    else :
        data=modified_lba_vec[file_index-1][0:min_index-1]
        plt.plot(tmp_index, data, label="Modified :" + str(file_index)+name+"(mean:"+arr_mean2[0:5]+"; std:"+ arr_std2[0:5] + ")")
    file_index+=1
plt.legend(loc='best')
plt.show()
