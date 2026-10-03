/*
 * maze.c
 *
 *  Created on: 2026/06/17
 *      Author: Karasawa
 */
#include "global.h"
#include "maze.h"
#include "flash.h"
MazeData maze = { 0 }; //迷路情報を格納する構造体

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：MazeInit
// 内容：迷路情報の初期化を行う
// 引数：MazeData* maze - 迷路情報構造体へのポインタ
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void MazeInit(MazeData* p_maze) {
	int i, j;
	for (i = 0; i < MAZE_SIZE; i++) {
		for (j = 0; j < MAZE_SIZE; j++) {
			p_maze->map[i][j].north = false;
			p_maze->map[i][j].east = false;
			p_maze->map[i][j].south = false;
			p_maze->map[i][j].west = false;
			p_maze->map[i][j].visited = false;
			p_maze->step_map[i][j] = 0;
		}
	}
	p_maze->goal_x = GOAL_X; //ゴール座標の初期化
	p_maze->goal_y = GOAL_Y; //ゴール座標の初期化
	p_maze->r_cnt = 0; //経路カウンタの初期化
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：MazeMakeStepMap
// 内容：幅優先探索を使用し、ゴールセルから各マスへの歩数マップを作成する
// 引数：MazeData* maze - 迷路情報構造体へのポインタ
//       uint8_t start_x - 現在の自機座標X
//       uint8_t start_y - 現在の自機座標Y
//       bool is_second - 2回目の探索かどうか
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void MazeMakeStepMap(MazeData* p_maze, uint8_t start_x, uint8_t start_y, bool is_second) {
	// 幅優先探索を使用して、ゴールから各マスへの歩数を作成する。
	// 一次探索では未知マスも通行候補とし、二次走行では訪問済みマスだけを使う。
	const uint8_t UNREACHED = MAX_ROUTE - 1;
	uint8_t queue[MAZE_SIZE * MAZE_SIZE];
	uint16_t head = 0;
	uint16_t tail = 0;

	//--全てのセルの歩数を初期化--
	for (int i = 0; i < MAZE_SIZE; i++) {
		for (int j = 0; j < MAZE_SIZE; j++) {
			p_maze->step_map[i][j] = UNREACHED;
		}
	}

	// 入力座標またはゴール座標が範囲外なら、未到達のまま終了する。
	if (start_x >= MAZE_SIZE || start_y >= MAZE_SIZE ||
			p_maze->goal_x >= MAZE_SIZE || p_maze->goal_y >= MAZE_SIZE) {
		return;
	}

	//--ゴールセルの歩数を0に設定し、キューへ入れる--
	p_maze->step_map[p_maze->goal_x][p_maze->goal_y] = 0;
	queue[tail++] = p_maze->goal_y * MAZE_SIZE + p_maze->goal_x;

	//--ゴールから四方向へ、距離の小さい順に広げる--
	while (head < tail) {
		uint8_t cell = queue[head++];
		uint8_t x = cell % MAZE_SIZE;
		uint8_t y = cell / MAZE_SIZE;
		uint8_t next_step = p_maze->step_map[x][y] + 1;

		// 北
		if (y < MAZE_SIZE - 1 && !p_maze->map[x][y].north &&
				p_maze->step_map[x][y + 1] == UNREACHED &&
				(!is_second || p_maze->map[x][y + 1].visited)) {
			p_maze->step_map[x][y + 1] = next_step;
			queue[tail++] = (y + 1) * MAZE_SIZE + x;
		}
		// 東
		if (x < MAZE_SIZE - 1 && !p_maze->map[x][y].east &&
				p_maze->step_map[x + 1][y] == UNREACHED &&
				(!is_second || p_maze->map[x + 1][y].visited)) {
			p_maze->step_map[x + 1][y] = next_step;
			queue[tail++] = y * MAZE_SIZE + (x + 1);
		}
		// 南
		if (y > 0 && !p_maze->map[x][y].south &&
				p_maze->step_map[x][y - 1] == UNREACHED &&
				(!is_second || p_maze->map[x][y - 1].visited)) {
			p_maze->step_map[x][y - 1] = next_step;
			queue[tail++] = (y - 1) * MAZE_SIZE + x;
		}
		// 西
		if (x > 0 && !p_maze->map[x][y].west &&
				p_maze->step_map[x - 1][y] == UNREACHED &&
				(!is_second || p_maze->map[x - 1][y].visited)) {
			p_maze->step_map[x - 1][y] = next_step;
			queue[tail++] = y * MAZE_SIZE + (x - 1);
		}
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：MazeMakeRoute
// 内容：歩数マップを基に、現在の自機座標からゴールまでの最短経路を作成する
// 引数：MazeData* maze - 迷路情報構造体へのポインタ
//       uint8_t start_x - 現在の自機座標X
//       uint8_t start_y - 現在の自機座標Y
//       SearchDirection start_dir - 現在の自機の向き
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void MazeMakeRoute(MazeData* p_maze, uint8_t start_x, uint8_t start_y, SearchDirection start_dir) {
	//経路を作成する処理
	//--経路カウンタを初期化--
	p_maze->r_cnt = 0;
	//--自機座標からゴールまでの経路を作成--
	int x = start_x;
	int y = start_y;
	SearchDirection dir = start_dir; // 現在向いている方向

	// ゴール(歩数0)に到達するまで繰り返す
	while (p_maze->step_map[x][y] > 0) {
		uint8_t current_step = p_maze->step_map[x][y];
		uint8_t next_step = current_step - 1;
		SearchDirection next_dir = dir; // 次に行く方向

		// 周囲4マスを北・東・南・西の優先順で調べ、
		// 壁がなく歩数が1小さい方向を次の進行方向にする。
		if (y < MAZE_SIZE - 1 && !p_maze->map[x][y].north &&
				p_maze->step_map[x][y + 1] == next_step) {
			next_dir = KNorth;
		}
		else if (x < MAZE_SIZE - 1 && !p_maze->map[x][y].east &&
				 p_maze->step_map[x + 1][y] == next_step) {
			next_dir = KEast;
		}
		else if (y > 0 && !p_maze->map[x][y].south &&
				 p_maze->step_map[x][y - 1] == next_step) {
			next_dir = KSouth;
		}
		else if (x > 0 && !p_maze->map[x][y].west &&
				 p_maze->step_map[x - 1][y] == next_step) {
			next_dir = KWest;
		}
		else {
			// 経路が見つからない場合(エラー)は終了
			break;
		}

		// 現在の進行方向(dir)と次の方向(next_dir)の相対角度を求める
		// KNorth(0), KEast(1), KSouth(2), KWest(3)
		int diff = (next_dir - dir + 4) % 4; //enumはint型なので、diffは0～3の範囲になる

		// r_cnt は uint8_t なので、255を超える書き込みは行わない。
		if (p_maze->r_cnt == MAX_ROUTE - 1) {
			break;
		}

		if (diff == 0) {
			p_maze->route[p_maze->r_cnt] = KForward;
		} else if (diff == 1) {
			p_maze->route[p_maze->r_cnt] = KTurnR90;
		} else if (diff == 2) {
			p_maze->route[p_maze->r_cnt] = KTurn180;
		} else if (diff == 3) {
			p_maze->route[p_maze->r_cnt] = KTurnL90;
		}
		
		p_maze->r_cnt++;

		// 機体の座標と向きを更新
		dir = next_dir;
		if (dir == KNorth) y += 1;
		else if (dir == KEast) x += 1;
		else if (dir == KSouth) y -= 1;
		else if (dir == KWest) x -= 1;
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：MazeMakeRouteLeftHand
// 内容：左手法により次の1手のみを決定する
// 引数：MazeData* maze - 迷路情報構造体へのポインタ
//       uint8_t start_x - 現在の自機座標X
//       uint8_t start_y - 現在の自機座標Y
//       SearchDirection start_dir - 現在の自機の向き
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void MazeMakeRouteLeftHand(MazeData* p_maze, uint8_t start_x, uint8_t start_y, SearchDirection start_dir) {
	// 左手法で次の1手だけを決定する処理
	
	// -- 経路カウンタを初期化 --
	p_maze->r_cnt = 0;

	// -- 現在の座標と向きを取得 --
	int x = start_x;
	int y = start_y;
	SearchDirection dir = start_dir; // 現在向いている方向

	bool wall_left  = false; // 左の壁
	bool wall_front = false; // 前の壁
	bool wall_right = false; // 右の壁

	// 自分が向いている方向(dir)を基準にして、前後左右の壁の有無を調べる
	if (dir == KNorth) {
		wall_left  = p_maze->map[x][y].west;
		wall_front = p_maze->map[x][y].north;
		wall_right = p_maze->map[x][y].east;
	} else if (dir == KEast) {
		wall_left  = p_maze->map[x][y].north;
		wall_front = p_maze->map[x][y].east;
		wall_right = p_maze->map[x][y].south;
	} else if (dir == KSouth) {
		wall_left  = p_maze->map[x][y].east;
		wall_front = p_maze->map[x][y].south;
		wall_right = p_maze->map[x][y].west;
	} else if (dir == KWest) {
		wall_left  = p_maze->map[x][y].south;
		wall_front = p_maze->map[x][y].west;
		wall_right = p_maze->map[x][y].north;
	}

	//左手法により次の手を決定する
	// 優先順位：左 ・ 前 ・ 右 ・ 180度ターン
	if (!wall_left) {
		// 左に壁がない場合は、左に曲がる
		p_maze->route[p_maze->r_cnt] = KTurnL90;
	} 
	else if (!wall_front) {
		// 左に壁があり、前方には壁がない場合は、直進する
		p_maze->route[p_maze->r_cnt] = KForward;
	} 
	else if (!wall_right) {
		// 左も前も壁があり、右には壁がない場合は、右に曲がる
		p_maze->route[p_maze->r_cnt] = KTurnR90;
	} 
	else {
		// 左も前も右も壁（行き止まり）の場合は、180度ターンする
		p_maze->route[p_maze->r_cnt] = KTurn180;
	}
	
	// 1手分のみ記録したのでカウンタを1にする
	p_maze->r_cnt = 1;
}
//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：MazeWriteWallMap
// 内容：現在のマウスの座標に、取得済みの絶対方向の壁情報を書き込み、訪問済みにする
// 引数：MazeData* maze - 迷路情報構造体へのポインタ
//       uint8_t x - 書き込むX座標
//       uint8_t y - 書き込むY座標
//       Section section - 書き込む壁情報
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void MazeWriteWallMap(MazeData* p_maze, uint8_t x, uint8_t y, Section section) {

	if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) {
		// 座標が迷路の範囲外の場合は何もしない
		return;
	}

	p_maze->map[x][y] = section; //現在のマウスの座標に壁情報を書き込む
	//周辺のマスの壁情報も更新する
	if (y < MAZE_SIZE - 1) {
		p_maze->map[x][y + 1].south = section.north;
	}
	if (x < MAZE_SIZE - 1) {
		p_maze->map[x + 1][y].west = section.east;
	}
	if (y > 0) {
		p_maze->map[x][y - 1].north = section.south;
	}
	if (x > 0) {
		p_maze->map[x - 1][y].east = section.west;
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：MazeWriteFlsh
// 内容：現在の迷路情報を圧縮し、フラッシュメモリにバックアップとして書き込む
// 引数：MazeData* maze - 迷路情報構造体へのポインタ
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void MazeWriteFlsh(MazeData* p_maze) {
	// 迷路情報をフラッシュメモリに書き込む処理
	FlashEnableWrite();
	
	uint32_t address = 0;
	// 1マスあたり5bitなので、16bit(1 Halfword)に2マス分のデータを圧縮して書き込む
	for (int i = 0; i < MAZE_SIZE; i++) {
		for (int j = 0; j < MAZE_SIZE; j += 2) {
			uint16_t data = 0;
			
			// 1つめのマス (下位8ビットに格納)
			if (p_maze->map[i][j].north)   data |= (1 << 0);
			if (p_maze->map[i][j].east)    data |= (1 << 1);
			if (p_maze->map[i][j].south)   data |= (1 << 2);
			if (p_maze->map[i][j].west)    data |= (1 << 3);
			if (p_maze->map[i][j].visited) data |= (1 << 4);

			// 2つめのマス (上位8ビットに格納)
			if (p_maze->map[i][j + 1].north)   data |= (1 << 8);
			if (p_maze->map[i][j + 1].east)    data |= (1 << 9);
			if (p_maze->map[i][j + 1].south)   data |= (1 << 10);
			if (p_maze->map[i][j + 1].west)    data |= (1 << 11);
			if (p_maze->map[i][j + 1].visited) data |= (1 << 12);

			FlashWriteHalfword(address, data);
			address++;
		}
	}
	
	FlashDisableWrite();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：MazeReadFlsh
// 内容：フラッシュメモリから保存されたデータを読み込み、迷路情報として復元する
// 引数：MazeData* maze - 迷路情報構造体へのポインタ
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void MazeReadFlsh(MazeData* p_maze) {
	// フラッシュメモリから迷路情報を読み込む処理
	uint32_t address = 0;
	
	for (int i = 0; i < MAZE_SIZE; i++) {
		for (int j = 0; j < MAZE_SIZE; j += 2) {
			uint16_t data = FlashReadHalfword(address);
			
			// 1つめのマスの復元
			p_maze->map[i][j].north   = (data & (1 << 0)) != 0;
			p_maze->map[i][j].east    = (data & (1 << 1)) != 0;
			p_maze->map[i][j].south   = (data & (1 << 2)) != 0;
			p_maze->map[i][j].west    = (data & (1 << 3)) != 0;
			p_maze->map[i][j].visited = (data & (1 << 4)) != 0;

			// 2つめのマスの復元
			p_maze->map[i][j + 1].north   = (data & (1 << 8)) != 0;
			p_maze->map[i][j + 1].east    = (data & (1 << 9)) != 0;
			p_maze->map[i][j + 1].south   = (data & (1 << 10)) != 0;
			p_maze->map[i][j + 1].west    = (data & (1 << 11)) != 0;
			p_maze->map[i][j + 1].visited = (data & (1 << 12)) != 0;
			
			address++;
		}
	}
}

// Certification planner. Static scratch space keeps BFS arrays off the MCU stack.
// A visited cell has all four sides resolved: three sensors plus the entry edge.
// Wall observations are mirrored by MazeWriteWallMap, including open edges.
#define EXPLORE_INF UINT16_MAX
static uint16_t explore_start[MAZE_SIZE * MAZE_SIZE];
static uint16_t explore_goal[MAZE_SIZE * MAZE_SIZE];
static uint16_t explore_current[MAZE_SIZE * MAZE_SIZE];
static uint16_t explore_queue[MAZE_SIZE * MAZE_SIZE];

static bool ExploreNeighbor(MazeData* m, uint16_t cell, unsigned dir,
        bool known_only, uint16_t* next) {
    int x = cell % MAZE_SIZE, y = cell / MAZE_SIZE;
    int nx = x, ny = y;
    bool wall;
    switch (dir) {
    case KNorth: ny++; wall = m->map[x][y].north; break;
    case KEast: nx++; wall = m->map[x][y].east; break;
    case KSouth: ny--; wall = m->map[x][y].south; break;
    default: nx--; wall = m->map[x][y].west; break;
    }
    if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE || wall)
        return false;
    if (known_only && !m->map[x][y].visited && !m->map[nx][ny].visited)
        return false;
    *next = ny * MAZE_SIZE + nx;
    return true;
}

static void ExploreDistances(MazeData* m, uint16_t origin, bool known_only,
        uint16_t* distance) {
    uint16_t head = 0, tail = 0;
    for (unsigned i = 0; i < MAZE_SIZE * MAZE_SIZE; i++) distance[i] = EXPLORE_INF;
    distance[origin] = 0;
    explore_queue[tail++] = origin;
    while (head < tail) {
        uint16_t cell = explore_queue[head++];
        for (unsigned d = 0; d < 4; d++) {
            uint16_t next;
            if (ExploreNeighbor(m, cell, d, known_only, &next) &&
                    distance[next] == EXPLORE_INF) {
                distance[next] = distance[cell] + 1;
                explore_queue[tail++] = next;
            }
        }
    }
}

ExploreResult MazePlanExplore(MazeData* m, uint8_t x, uint8_t y,
        SearchDirection dir, bool reached_goal, uint16_t* known, uint16_t* possible) {
    m->r_cnt = 0;
    *known = *possible = EXPLORE_INF;
    if (x >= MAZE_SIZE || y >= MAZE_SIZE || m->goal_x >= MAZE_SIZE ||
            m->goal_y >= MAZE_SIZE || (unsigned)dir >= 4) return KExploreError;
    uint16_t current = y * MAZE_SIZE + x;
    uint16_t goal = m->goal_y * MAZE_SIZE + m->goal_x;
    ExploreDistances(m, goal, true, explore_goal);
    *known = explore_goal[0];
    ExploreDistances(m, goal, false, explore_goal);
    *possible = explore_goal[0];
    if (*possible == EXPLORE_INF) return KExploreError;

    bool known_only = reached_goal;
    if (reached_goal) {
        if (*known == EXPLORE_INF) return KExploreError;
        uint16_t target = 0;
        if (*known != *possible) {
            ExploreDistances(m, 0, false, explore_start);
            ExploreDistances(m, current, true, explore_current);
            uint16_t best = EXPLORE_INF;
            target = EXPLORE_INF;
            // Only frontier cells that might lie on a shorter start-goal route.
            // Among them prefer the nearest observable cell; ties favor start.
            for (uint16_t c = 0; c < MAZE_SIZE * MAZE_SIZE; c++) {
                if (m->map[c % MAZE_SIZE][c / MAZE_SIZE].visited ||
                        explore_current[c] == EXPLORE_INF ||
                        explore_start[c] == EXPLORE_INF || explore_goal[c] == EXPLORE_INF)
                    continue;
                if ((uint32_t)explore_start[c] + explore_goal[c] >= *known) continue;
                if (explore_current[c] < best ||
                        (explore_current[c] == best && target != EXPLORE_INF &&
                         explore_start[c] < explore_start[target])) {
                    target = c;
                    best = explore_current[c];
                }
            }
            if (target == EXPLORE_INF) return KExploreError;
        } else if (current == 0) {
            return KExploreDone;
        }
        ExploreDistances(m, target, true, explore_goal);
    }
    if (explore_goal[current] == EXPLORE_INF || explore_goal[current] == 0)
        return KExploreError;
    // Prefer continuing straight when equal-length choices exist.
    for (unsigned i = 0; i < 4; i++) {
        unsigned d = (dir + i) % 4;
        uint16_t next;
        if (ExploreNeighbor(m, current, d, known_only, &next) &&
                explore_goal[next] != EXPLORE_INF &&
                explore_goal[next] + 1 == explore_goal[current]) {
            unsigned turn = (d + 4 - dir) % 4;
            m->route[0] = turn == 0 ? KForward : turn == 1 ? KTurnR90 :
                    turn == 2 ? KTurn180 : KTurnL90;
            m->r_cnt = 1;
            return KExploreMove;
        }
    }
    return KExploreError;
}
