
Sorting_area_base = [
    ([53.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-22.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-97.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-172.0, 427.0, 185.0, -180.0, 0.00, 90.00]),  # A row
    ([53.0, 486.0, 235.0, -180.0, 0.00, 90.00],[-22.0, 486.0, 235.0, -180.0, 0.00, 90.00],[-97.0, 486.0, 235.0, -180.0, 0.00, 90.00],[-172.0, 486.0, 235.0, -180.0, 0.00, 90.00]), # B row
    ([-186.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-261.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-336.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-411.0, 427.0, 185.0, -180.0, 0.00, 90.00]) , # C row
    ([-186.0, 489.0, 235.0, -180.0, 0.00, 90.00],[-261.0, 489.0, 235.0, -180.0, 0.00, 90.00],[-336.0, 489.0, 235.0, -180.0, 0.00, 90.00],[-411.0, 489.0, 235.0, -180.0, 0.00, 90.00]) ,  # D row
]

Sorting_place = [[]]
if len(Sorting_place) == 0:
    print("列表為空")
else:
    print("列表有元素")
object = [[2,1,1,0],[1,1,0,2]]
count_map = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
order_map = {'A': 0, 'B': 1, 'C': 2, 'D': 3}
items = []
iteam = items[:3]
matrix = [[None]*3 for _ in range(4)]
col_counter = 1
for row,col in enumerate(object, start=0):
    tt = []
    for i,counter in enumerate(col, start=0):
        for j in range(counter):
            if i == 0:
                tt.append('A')
            elif i == 1:
                tt.append('B')
            elif i == 2:    
                tt.append('C')
            elif i == 3:
                tt.append('D')
            if (col_counter)%3 == 0:
               items.append(tt)
               tt = []
               col_counter = 1
            else:
                col_counter+=1
    col_counter = 1
    items.append(tt)
print(f"🔶 items: {items}")
while items:
    iteam = items[0]
    del items[0]
    tt = []
    for j, item in enumerate(iteam):
        row = order_map[item]
        col = count_map[item]


        # 計算位置（加上列的基礎座標 + 欄位間隔）
        # x,y,z,rx,ry,rz = Sorting_area_base[row]
        # x = x - col * 75.0

        x,y,z,rx,ry,rz = Sorting_area_base[row][col]

        if j == 0:
            x -= 75.0
            print("👉 第一個物體：夾具偏移 (x - 50)")
        elif j == 2:
            x += 75.0
            print("🔁 第三個物體：夾具偏移 (x + 50)")
        matrix[row][col] = item
        count_map[item] += 1
        tt.append([x,y,z,rx,ry,rz])
    Sorting_place.append(tt)
    print(f"🔷 {Sorting_place}")
