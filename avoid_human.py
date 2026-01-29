# ===============================
# SLAM型配送 人回避 検証コード
# ===============================
import random
import heapq
import time
import copy

# ===== マップ =====
RAW_MAP = [
"■■■Ｂ・・◆◆◆◆・・Ａ■■■",
"■■■■・・◆◆◆◆・・■■■■",
"■■■■・・◆◆◆◆・・■■■■",
"■■■■・・②＃＃＃・・■■■■",
"■■■■・・＃＃＃②・・■■■■",
"■■■■・・◆◆◆◆・・■■■■",
"・・・・・・◆◆◆◆・・・・・・",
"・・・・・・◆◆◆◆・・・・・・",
"◆＃①◆◆◆◆◆◆◆◆◆◆＃①◆",
"◆＃＃◆◆◆◆◆◆◆◆◆◆＃＃◆",
"◆＃＃◆◆◆◆◆◆◆◆◆◆＃＃◆",
"◆①＃◆◆◆◆◆◆◆◆◆◆①＃◆",
"・・・・・・◆◆◆◆・・・・・・",
"・・・・・・◆◆◆◆・・・・・・",
"■Ｃ■■・・②＃＃＃・・受■■■",
"■■■■・・＃＃＃②・・■■■■",
]

H, W = 16, 16

# ===== 方向 =====
DIRS = {
    0: (-1, 0),  # 北
    1: (0, -1),  # 西
    2: (1, 0),   # 南
    3: (0, 1),   # 東
}
DIR_CHAR = {0:"▲",1:"◀",2:"▼",3:"▶"}

CENTER = 2
CNN = 5

# ===== ユーティリティ =====
def manhattan(a,b):
    return abs(a[0]-b[0])+abs(a[1]-b[1])

def find(ch):
    for r in range(H):
        for c in range(W):
            if grid[r][c]==ch:
                return (r,c)

# ===== 人配置（仕様どおり）=====
def place_people(n=6):
    cells=[]
    for r in range(H):
        for c in range(W):
            if grid[r][c] in "・＃":
                cells.append((r,c))

    banned = set()
    fixed = "ＡＢＣ受◯◎"
    for r in range(H):
        for c in range(W):
            if grid[r][c] in fixed:
                banned.add((r,c))

    people=[]
    random.shuffle(cells)
    for r,c in cells:
        if any(manhattan((r,c),b)<=2 for b in banned):
            continue
        people.append((r,c))
        banned.add((r,c))
        if len(people)>=n:
            break

    for r,c in people:
        grid[r][c]="◯"

# ===== CNN =====
def get_cnn(pos,dir):
    cnn=[[" "]*CNN for _ in range(CNN)]
    for y in range(CNN):
        for x in range(CNN):
            wy = pos[0]+(y-CENTER)
            wx = pos[1]+(x-CENTER)
            if 0<=wy<H and 0<=wx<W:
                cnn[y][x]=grid[wy][wx]
    return cnn

def front_index(dir):
    dy,dx=DIRS[dir]
    return CENTER+dy, CENTER+dx

# ===== A* =====
def astar(start,goal,map_):
    pq=[(0,start)]
    prev={}
    cost={start:0}
    while pq:
        _,cur=heapq.heappop(pq)
        if cur==goal: break
        for d in DIRS.values():
            nr,nc=cur[0]+d[0],cur[1]+d[1]
            if not(0<=nr<H and 0<=nc<W): continue
            if map_[nr][nc] in "■◆◎": continue
            ncst=cost[cur]+1
            if (nr,nc) not in cost or ncst<cost[(nr,nc)]:
                cost[(nr,nc)]=ncst
                pr=ncst+manhattan((nr,nc),goal)
                heapq.heappush(pq,(pr,(nr,nc)))
                prev[(nr,nc)]=cur
    if goal not in prev: return []
    path=[goal]
    while path[-1]!=start:
        path.append(prev[path[-1]])
    return path[::-1]

# ===== 初期化 =====
grid=[list(r) for r in RAW_MAP]
place_people()

agent=find("受")
dir=0
goal=find("Ａ")

# ===== stepループ =====
for step in range(60):
    vis=copy.deepcopy(grid)
    vis[agent[0]][agent[1]]=DIR_CHAR[dir]
    print("\nSTEP",step)
    for r in vis: print("".join(r))

    cnn=get_cnn(agent,dir)
    fy,fx=front_index(dir)

    # 人検知
    if cnn[fy][fx]=="◯":
        wy=agent[0]+(fy-CENTER)
        wx=agent[1]+(fx-CENTER)
        grid[wy][wx]="◎"
        print("👀 人検知 → 停止 & A*再計算")
        time.sleep(0.5)
        continue

    path=astar(agent,goal,grid)
    if len(path)<2:
        print("❌ 進路なし")
        break

    nr,nc=path[1]
    dr,dc=nr-agent[0],nc-agent[1]
    for k,v in DIRS.items():
        if v==(dr,dc):
            dir=k
    agent=(nr,nc)
    time.sleep(0.3)