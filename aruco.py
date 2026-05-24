import cv2

# 1. 选择预定义的 ArUco 字典 (例如: 6x6位, 共250个标记)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# 2. 指定要生成的标记ID和图片尺寸(像素)
marker_id = 25
marker_size = 200 

# 3. 生成标记图像
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# 4. 保存为本地图片
cv2.imwrite(f"aruco_marker_id_{marker_id}.png", marker_image)
print(f"成功生成并保存了 ID 为 {marker_id} 的 ArUco 标记！")