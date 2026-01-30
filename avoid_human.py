# ==========================================
# SLAM型配送：CNN回転＋人回避【最終安定版】
# ==========================================
import random, heapq, time, copy

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
    0:(-1,0),  # 北
    1:(0,-1),  # 西
    2:(1,0),   # 南
    3:(0,1),   # 東
}
ARROW={0:"▲",1:"◀",2:"▼",3:"▶"}

# ===== CNN =====
CNN=5
CENTER=2
FRONT=(CENTER-1, CENTER)

# ===== util =====
def manhattan(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])

def find(ch):
    for r in range(H):
        for c in range(W):
            if grid[r][c]==ch:
                return (r,c)

# ===== 人配置（マンハッタン距離2）=====
def place_people(n=6):
    cand=[(r,c) for r in range(H) for c in range(W) if grid[r][c] in "・＃"]
    banned=set(find(x) for x in "ＡＢＣ受" if find(x))
    random.shuffle(cand)
    for r,c in cand:
        if any(manhattan((r,c),b)<=2 for b in banned):
            continue
        grid[r][c]="◯"
        banned.add((r,c))
        if sum(1 for r in range(H) for c in range(W) if grid[r][c]=="◯")>=n:
            break

# ===== CNN（回転対応）=====
def get_cnn(pos,dir):
    cnn=[[" "]*CNN for _ in range(CNN)]
    for y in range(CNN):
        for x in range(CNN):
            dy,dx=y-CENTER,x-CENTER
            if dir==0: ry,rx=dy,dx
            elif dir==1: ry,rx=dx,-dy
            elif dir==2: ry,rx=-dy,-dx
            else: ry,rx=-dx,dy
            r,c=pos[0]+ry,pos[1]+rx
            if 0<=r<H and 0<=c<W:
                cnn[y][x]=grid[r][c]
    return cnn

# ===== A* =====
def astar(start,goal,map_):
    pq=[(0,start)]
    cost={start:0}
    prev={}
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
                heapq.heappush(pq,(ncst+manhattan((nr,nc),goal),(nr,nc)))
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
goal=find("Ａ")
dir=0
mode="ROTATE"

# ===== step loop =====
for step in range(80):
    vis=copy.deepcopy(grid)
    vis[agent[0]][agent[1]]=ARROW[dir]
    print(f"\nSTEP {step} MODE:{mode}")
    for r in vis: print("".join(r))

    path=astar(agent,goal,grid)
    if len(path)<2:
        print("❌ 経路なし")
        break

    nr,nc=path[1]
    dr,dc=nr-agent[0],nc-agent[1]
    next_dir=[k for k,v in DIRS.items() if v==(dr,dc)][0]

    # ===== 回転 =====
    if mode=="ROTATE":
        if dir!=next_dir:
            dir=next_dir
            print("🔄 回転")
            mode="MOVE"
            time.sleep(0.3)
            continue
        mode="MOVE"

    # ===== 前進 step（唯一の衝突判定）=====
    if mode=="MOVE":
        # ★ 実際に進むマスだけを見る（唯一正しい）
        if grid[nr][nc] == "◯":
            grid[nr][nc] = "◎"
            print("👀 次マスに人 → 停止 & 再探索")
            time.sleep(0.4)
            continue

        agent = (nr, nc)
        print("➡ 前進")
        mode = "ROTATE"
        time.sleep(0.3)