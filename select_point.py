import cv2
import yaml
import os

# 创建字典来保存标点位置和颜色
points = {}
colors = {}

# 当前标点的编号
point_count = 1

# YAML 文件路径
yaml_file = "new_points_data.yaml"

# 如果 YAML 文件存在，加载其内容
if os.path.exists(yaml_file):
    with open(yaml_file, 'r',encoding='utf-8') as f:
        data = yaml.safe_load(f)
else:
    data = {'nodes': []}

# 用于保存每次标点的坐标
current_corners = []

# 标记是否正在进行一次标点
marking = False


# 回调函数：鼠标点击事件
def click_event(event, x, y, flags, param):
    global point_count, marking, current_corners

    # 当左键点击时，保存点击位置
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"点击位置：({x}, {y})")

        if marking:
            # 保存点击位置
            current_corners.append([x, y])
            # 在图像上标记点击的点
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)  # 绘制红色小圆点
            cv2.imshow("Map", img)


# 加载地图图像
img = cv2.imread("C:\Files\E\Radar\img.png")  # 替换为你的地图文件路径

# 显示地图图像
cv2.imshow("Map", img)

# 设置鼠标回调函数
cv2.setMouseCallback("Map", click_event)

while True:
    # 等待用户按下键盘
    key = cv2.waitKey(1) & 0xFF

    if key == ord('e'):  # 按下 'E' 键结束标点
        if marking and current_corners:
            # 提示用户输入颜色
            color = input("请输入颜色（如 'red', 'blue' 等）：")
            description = input("请输入区域描述：")
            colors[str(point_count)] = color  # 保存颜色

            # 创建一个新的节点字典并将其加入数据
            new_node = {
                'id': str(point_count),
                'description': description,
                'flag': color,
                'corners': current_corners
            }
            data['nodes'].append(new_node)

            # 增加点编号
            point_count += 1

            # 重置标点数据
            marking = False
            current_corners = []
            print("标点结束")
        else:
            print("没有标点数据")

    elif key == ord('q'):  # 按下 'Q' 键退出程序
        print("程序结束")
        break

    elif key == ord('s'):  # 按下 'S' 键开始标点
        if not marking:
            print("开始标点，请点击地图上的位置")
            marking = True
        else:
            print("已经在标点模式中")

# 将新数据保存到 YAML 文件中（追加模式）
with open(yaml_file, 'a') as f:
    yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

# 打印所有保存的标点
print("保存的标点：")
for point, coord in points.items():
    print(f"{point}: {coord}")

# 关闭窗口
cv2.destroyAllWindows()
