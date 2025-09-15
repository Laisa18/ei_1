from heapq import heappush, heappop

#cáluclo da heurística, quantas casas anda na vertical e na horizontal de um ponto a outro
def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(grid, start, goal):
    #se estamos no objetivo retorna a posição atual e o custo que seria 0
    if start == goal:
        return [start], 0

    R = len(grid) #guardamos a quantidade de linhas
    
    C = len(grid[0]) #guardamos a quantidade de colunas

    openh = []

    heappush(openh, (manhattan(start, goal), 0, start))
    came = {start: None}
    g = {start: 0}
    while openh:
        f, gc, cur = heappop(openh)
        if cur == goal:
            # reconstrói caminho
            path = []
            p = cur
            while p is not None:
                path.append(p)
                p = came[p]
            return path[::-1], g[cur]
        x, y = cur
        for nx, ny in [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]:
            if 0 <= nx < R and 0 <= ny < C and grid[nx][ny] == 0:
                ng = gc + 1
                if ng < g.get((nx,ny), 10**9):
                    g[(nx,ny)] = ng
                    came[(nx,ny)] = cur
                    heappush(openh, (ng + manhattan((nx,ny), goal), ng, (nx,ny)))
    return None, float("inf")

def nearest_by_astar(grid, start, targets):
    """Escolhe o alvo alcançável com menor custo A* a partir de start."""
    best = (None, None, float("inf"))
    for t in targets:
        p, d = astar(grid, start, t)
        if p is not None and d < best[2]:
            best = (t, p, d)
    return best  # (alvo, caminho, distancia)

# ---------- Planejamento super simples ----------
def route_collect(grid, start, bins, bays, capacity):
    """
    bins: dict {pos: quantidade}
    bays: lista de pos
    Estratégia:
      - enquanto houver bins:
          - se couber, vá ao coletor mais próximo e colete
          - senão, vá à baia mais próxima e esvazie
      - no fim, volta à baia
    """
    pos = start
    carga = 0
    caminho_total = [start]
    restante = bins.copy()

    def goto(pos_atual, alvo):
        p, d = astar(grid, pos_atual, alvo)
        if p is None:
            raise RuntimeError(f"Sem caminho de {pos_atual} até {alvo}.")
        return p[1:], d  # sem repetir célula inicial

    while restante:
        # escolher coletor mais próximo
        alvo, pbin, _ = nearest_by_astar(grid, pos, list(restante.keys()))
        qtd = restante[alvo]
        if carga + qtd <= capacity:
            step, _ = goto(pos, alvo)
            caminho_total += step
            pos = alvo
            carga += qtd
            del restante[alvo]
        else:
            # esvaziar antes
            bay, pbay, _ = nearest_by_astar(grid, pos, bays)
            step, _ = goto(pos, bay)
            caminho_total += step
            pos = bay
            carga = 0

    # volta à baia mais próxima no final
    bay, pbay, _ = nearest_by_astar(grid, pos, bays)
    step, _ = goto(pos, bay)
    caminho_total += step
    return caminho_total

# ---------- Visualização simples ----------
def draw(grid, path, start, bins, bays):
    R, C = len(grid), len(grid[0])
    canvas = [[" " for _ in range(C)] for __ in range(R)]
    for r in range(R):
        for c in range(C):
            if grid[r][c] == 1:
                canvas[r][c] = "#"
    for (r,c) in path:
        if grid[r][c] == 0:
            canvas[r][c] = "."
    for (r,c) in bins.keys():
        canvas[r][c] = "X"
    for (r,c) in bays:
        canvas[r][c] = "D"
    sx, sy = start
    canvas[sx][sy] = "S"
    print("\nMapa (S início, X coletor, D baia, # obstáculo, . rota)\n")
    for r in range(R):
        print("".join(canvas[r]))

# ---------- Exemplo mínimo ----------
if __name__ == "__main__":
    grid = [
        [0,0,0,0,0,0],
        [0,1,1,0,0,0],
        [0,0,0,0,1,0],
        [0,1,0,0,1,0],
        [0,0,0,1,0,0],
        [0,0,0,0,0,0],
    ]
    start = (0,0)
    bays  = [(0,5), (5,0)]           # multi-baias
    bins_ = {(1,3): 2, (2,5): 3, (4,0): 2}  # posição -> quantidade
    capacity = 4

    path = route_collect(grid, start, bins_, bays, capacity)
    print(f"Passos totais: {len(path)-1}")
    draw(grid, path, start, bins_, bays)
