import matplotlib.pyplot as plt

# 原始数据
respo = {"1":{"id":1,"position":{"x":17.21447492588015,"y":-21.431638488170023},"digits":[8],"last_update":572.445,"in_region":True},"2":{"id":2,"position":{"x":16.54307687165997,"y":-19.781428951962283},"digits":[2],"last_update":624.011,"in_region":True},"3":{"id":3,"position":{"x":13.876325920983106,"y":-12.492756278120206},"digits":[4],"last_update":645.782,"in_region":True},"4":{"id":4,"position":{"x":18.679685105955922,"y":-11.533955852910054},"digits":[2],"last_update":610.264,"in_region":True},"5":{"id":5,"position":{"x":18.95372564264234,"y":-4.542890300636703},"digits":[4],"last_update":591.308,"in_region":True},"6":{"id":6,"position":{"x":15.581707115725763,"y":-1.8800577542007488},"digits":[4],"last_update":670.507,"in_region":True},"7":{"id":7,"position":{"x":16.66126263212825,"y":-5.568857312919789},"digits":[4],"last_update":667.776,"in_region":True},"8":{"id":8,"position":{"x":12.788967117086328,"y":-4.180733684420524},"digits":[2],"last_update":674.757,"in_region":True},"9":{"id":9,"position":{"x":14.953856900775065,"y":-9.280340050178019},"digits":[3],"last_update":605.734,"in_region":True},"10":{"id":10,"position":{"x":12.535701915202356,"y":-11.569983339906845},"digits":[3],"last_update":692.762,"in_region":True},"11":{"id":11,"position":{"x":20.222873939485556,"y":-1.4095249548093594},"digits":[4],"last_update":662.151,"in_region":False}}



# 坐标、标签及颜色处理
x_coords = []   # 对应 x1 -> 竖直轴（向上为正）
y_coords = []   # 对应 -y1 -> 水平轴（向左为正）
labels = []
colors = []

for obj in respo.values():
    x1 = obj["position"]["x"]
    y1 = obj["position"]["y"]
    
    x_coords.append(x1)         # 竖直轴，x1 向上为正
    y_coords.append(-y1)        # 水平轴，-y1 向左为正
    labels.append("".join(str(d) for d in obj["digits"]))
    
    # 根据 in_region 判断颜色：True 为蓝色，False 为红色
    colors.append("blue" if obj["in_region"] else "red")

# 绘图
plt.figure(figsize=(8, 6))
plt.scatter(y_coords, x_coords, color=colors)

# 添加点的标签
for x, y, label in zip(y_coords, x_coords, labels):
    plt.text(x + 0.3, y + 0.3, label, fontsize=9)

# 绘制两条水平黑色虚线：x1=8.8 和 x1=4.8
plt.axhline(y=8.8, color='black', linestyle='--')
plt.axhline(y=4.8, color='black', linestyle='--')

# 设置坐标轴和样式
plt.axhline(0, color='gray', linestyle='--')
plt.axvline(0, color='gray', linestyle='--')
plt.xlabel("y")
plt.ylabel("x")
plt.title("Visiualization of Cube Position")
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
