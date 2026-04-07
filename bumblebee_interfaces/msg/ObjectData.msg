# ObjectData.msg
string name         # 객체 이름 (예: "object1")
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
int32 id
int32 count
float32[] obb       # OBB 좌표 (8개의 x, y 값으로 구성됨)
