import matplotlib.pyplot as plt
import csv

# 初始化列表存储数据
s_P_0_values = []
determinant0_values = []
s_P_1_values = []
determinant1_values = []

# 读取文件中的数据
with open('determiant_error_correlation.csv', mode='r') as file:
    reader = csv.reader(file)
    next(reader)  # 跳过表头

    for row in reader:
        s_P_0_values.append(float(row[0]))
        determinant0_values.append(abs(float(row[1])))
        s_P_1_values.append(float(row[2]))
        determinant1_values.append(abs(float(row[3])))

# 绘制 s_P_0 vs determinant0 的关系曲线
plt.plot(determinant0_values, s_P_0_values, marker='o', linestyle='None', label='ANRS error vs determinant(abs)')

# 绘制 s_P_1 vs determinant1 的关系曲线
plt.plot(determinant1_values, s_P_1_values, marker='x', linestyle='None', label='GTRS error vs determinant(abs)')

# 设置图例和标签
plt.xlabel('determinant')
plt.ylabel('s_P')
plt.title('s_P vs determinant Relationship')
plt.legend()
plt.grid(True)

# 显示图形
plt.show()
print(min(determinant0_values))
print(min(determinant1_values))