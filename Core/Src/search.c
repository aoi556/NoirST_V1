/*
 * search.c
 *
 *  Created on: 2026/06/17
 *      Author: Karasawa
 */
#include "global.h"
#include "search.h"
#include "sensor.h"
//#include "Core/Inc/search.h"
#include "drive.h"
#include "motor.h"
#include <math.h>

MouseData mouse = { 0 }; //マウス情報を格納する構造体

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：SearchInit
// 内容：マウスの初期状態（座標、向き、探索フラグ、壁情報、経路カウンタ）の初期化を行う
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void SearchInit(void) {
	mouse.second = false; //2回目の探索かどうかを初期化
	mouse.x = 0; //マウスのX座標を初期化
	mouse.y = 0; //マウスのY座標を初期化
	mouse.dir = KNorth; //マウスの方向を初期化
	mouse.section.north = false; //北方向の壁の有無を初期化
	mouse.section.east = false;  //東方向の壁の有無を初期化
	mouse.section.south = false; //南方向の壁の有無を初期化
	mouse.section.west = false;  //西方向の壁の有無を初期化
	mouse.section.visited = false; //訪問済みかどうかを初期化
	mouse.r_cnt = 0; //経路カウンタを初期化
	MazeInit(&maze); //迷路情報を初期化
}

//前方宣言
void GetSectionInfo(void);
void AdvancePosition(void);
void ConfirmRoute(void);
void OutputDebugMap(MazeData* p_maze);
void OutputDebugStepMap(MazeData* p_maze);
void OutputDebugRoute(MazeData* p_maze);
void SetupWallDetourTest(void);
// Mode 0: certify the shortest route and return home, like SearchC,
// with separate straight and turn speeds.
// Capture walls at each cell entrance; compensate distance travelled while
// computing the next action. Stop if planning has consumed the entry margin.
// Plan through visited cells only. Stop before a turn, frontier, or home.
// Lookahead changes planner scratch/route data, so restore the current action.
static unsigned KnownStraightCells(bool reached_goal, bool already_known) {
    if (!reached_goal || !already_known) return 0;
    int x = mouse.x, y = mouse.y;
    unsigned cells = 0;
    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {1, 0, -1, 0};
    while (cells < MAZE_SIZE) {
        Section s = maze.map[x][y];
        bool wall = mouse.dir == KNorth ? s.north : mouse.dir == KEast ? s.east :
                mouse.dir == KSouth ? s.south : s.west;
        int nx = x + dx[mouse.dir], ny = y + dy[mouse.dir];
        if (wall || nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE ||
                !maze.map[nx][ny].visited) break;
        cells++;
        x = nx;
        y = ny;
        uint16_t known, possible;
        ExploreResult result = MazePlanExplore(&maze, x, y, mouse.dir, true, &known, &possible);
        if (result != KExploreMove || maze.route[0] != KForward) break;
    }
    maze.route[0] = KForward;
    maze.r_cnt = 1;
    return cells;
}
void SearchExploreSlalom(void) {
    DriveSlalomStop();
    mouse.second = false;
    GetSectionInfo();
    mouse.section.north = false;
    MazeWriteWallMap(&maze, mouse.x, mouse.y, mouse.section);
    if (!DriveSlalomStraight(HALF_CELL_MM, EXPLORE_SLALOM_SPEED_MM_S, true)) return;
    AdvancePosition();
    bool reached_goal = false;
    while (true) {
        float mark = DriveSlalomDistance();
        bool already_known = maze.map[mouse.x][mouse.y].visited;
        GetSectionInfo();
        MazeWriteWallMap(&maze, mouse.x, mouse.y, mouse.section);
        if (mouse.x == maze.goal_x && mouse.y == maze.goal_y) reached_goal = true;
        uint16_t known, possible;
        ExploreResult result = MazePlanExplore(&maze, mouse.x, mouse.y,
                mouse.dir, reached_goal, &known, &possible);
        // Do not print while moving: blocking UART output consumes turn entry distance.
        if (result == KExploreDone) {
            float remaining = HALF_CELL_MM - (DriveSlalomDistance() - mark);
            if (!DriveSlalomStraight(remaining, 0.0f, true)) return;
            DriveRotateR180();
            mouse.dir = (mouse.dir + 2) % 4;
            MazeWriteFlsh(&maze);
            printf("Mode 0: shortest route confirmed, returned to start, map saved\n");
            return;
        }
        if (result != KExploreMove || maze.r_cnt == 0) {
            DriveSlalomStop();
            printf("Mode 0: route not found at (%d, %d)\n", mouse.x, mouse.y);
            return;
        }
        float travelled = DriveSlalomDistance() - mark;
        bool ok = false;
        switch (maze.route[0]) {
        case KForward:
            {
                unsigned cells = KnownStraightCells(reached_goal, already_known);
                // Lookahead runs while moving; include its elapsed distance.
                travelled = DriveSlalomDistance() - mark;
                float remaining = FULL_CELL_MM * (cells ? cells : 1U) - travelled;
                if (cells) {
                    float ramp = fminf(HALF_CELL_MM, remaining * 0.5f);
                    ok = DriveSlalomStraight(ramp, EXPLORE_SLALOM_KNOWN_STRAIGHT_SPEED_MM_S, true);
                    float cruise = remaining - 2.0f * ramp;
                    if (ok && cruise > 0.0f)
                        ok = DriveSlalomStraight(cruise, EXPLORE_SLALOM_KNOWN_STRAIGHT_SPEED_MM_S, true);
                    if (ok) ok = DriveSlalomStraight(ramp, EXPLORE_SLALOM_SPEED_MM_S, true);
                    if (ok) for (unsigned i = 1; i < cells; i++) AdvancePosition();
                } else {
                    ok = DriveSlalomStraight(remaining, EXPLORE_SLALOM_SPEED_MM_S, true);
                }
            }
            break;
        case KTurnL90:
            ok = DriveExploreSlalom90(false, travelled);
            if (ok) mouse.dir = (mouse.dir + 3) % 4;
            break;
        case KTurnR90:
            ok = DriveExploreSlalom90(true, travelled);
            if (ok) mouse.dir = (mouse.dir + 1) % 4;
            break;
        case KTurn180:
            ok = DriveSlalomStraight(HALF_CELL_MM - travelled, 0.0f, true);
            if (ok) {
                // Use the entrance snapshot, not side sensors during rotation.
                if (sensor.wall_f && sensor.wall_l) {
                    DriveRotateR90();
                    DriveSetPosition();
                    DriveRotateR90();
                    DriveSetPosition();
                } else if (sensor.wall_f && sensor.wall_r) {
                    DriveRotateL90();
                    DriveSetPosition();
                    DriveRotateL90();
                    DriveSetPosition();
                } else {
                    DriveRotateR180();
                }
                ok = DriveSlalomStraight(HALF_CELL_MM, EXPLORE_SLALOM_SPEED_MM_S, true);
            }
            if (ok) mouse.dir = (mouse.dir + 2) % 4;
            break;
        default:
            break;
        }
        if (!ok) {
            DriveSlalomStop();
            printf("Mode 0: motion failed or planning overrun at (%d, %d)\n", mouse.x, mouse.y);
            return;
        }
        AdvancePosition();
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：SearchB
// 内容：幅優先探索（足立法）を用いた迷路の探索・最短走行を行う
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void SearchB(void) {
	//幅優先探索を用いた探索アルゴリズムBの実装
	if (mouse.second) {//二次走行の場合は
		MazeReadFlsh(&maze);//フラッシュメモリから迷路情報を読み込む
		// 保存地図を検証してから位置合わせ・発進する。
		MazeMakeStepMap(&maze, mouse.x, mouse.y, true);
		if (!maze.map[0][0].visited || maze.map[0][0].north ||
				!maze.map[maze.goal_x][maze.goal_y].visited ||
				maze.step_map[0][0] == MAX_ROUTE - 1 ||
				!maze.map[0][1].visited ||
				maze.step_map[0][1] == MAX_ROUTE - 1) {
			printf("SearchB: saved map has no usable start-goal route. Run exploration first.\n");
			return;
		}
		DriveSetStartPosition();
	}
	//====スタート位置壁情報取得====
	GetSectionInfo(); //スタート位置の壁情報を取得
	mouse.section.north = false; //スタート位置の北側は壁がない
	MazeWriteWallMap(&maze, mouse.x, mouse.y, mouse.section); //スタート位置の壁情報を迷路マップに書き込む
	//====前に壁が無い想定で問答無用で前進====
	/*半区画前進関数*/
	DriveHalfSectionAccel(); //半区画加速走行
	AdvancePosition(); //マウスの座標を進行方向に1区画進める
	GetSectionInfo(); //進んだ先の壁情報を取得
	MazeWriteWallMap(&maze, mouse.x, mouse.y, mouse.section); //進んだ先の壁情報を迷路マップに書き込む
	//====探索走行====
	while (mouse.x != maze.goal_x || mouse.y != maze.goal_y) {
		// 現在マスで観測した壁を使い、距離マップと経路を毎回更新する。
		MazeMakeStepMap(&maze, mouse.x, mouse.y, mouse.second);
		MazeMakeRoute(&maze, mouse.x, mouse.y, mouse.dir);
		if (maze.r_cnt == 0) {
			// ゴールへ向かう経路が見つからない場合は走行を中止する。
			DriveSetSpeed(0.0f);
			DriveSetAccel(0.0f);
			MotorStop();
			printf("SearchB: route not found at (%d, %d)\n", mouse.x, mouse.y);
			return;
		}

		//----進行----
		switch (maze.route[0]) {
			//----前進----
		case KForward:
			if (mouse.second) {
                unsigned cells = 1;
                while (cells < maze.r_cnt && maze.route[cells] == KForward) cells++;
                DriveKnownStraight(cells, SECOND_RUN_STRAIGHT_SPEED_MM_S);
                for (unsigned j = 1; j < cells; j++) AdvancePosition();
            } else DriveOneSection(); //半区画前進関数
			break;
			//----左90度回転----
		case KTurnL90:
			DriveTurnL90(); //左90度回転関数
			mouse.dir = (mouse.dir + 3) % 4; //左90度回転
			break;
			//----右90度回転----
		case KTurnR90:
			DriveTurnR90(); //右90度回転関数
			mouse.dir = (mouse.dir + 1) % 4; //右90度回転
			break;
			//----180度回転----
		case KTurn180:
			GetWallSensorInfo(); //センサーの壁情報を取得
			/*半区画減速走行をここに書く*/
			DriveHalfSectionBreak();
			if (sensor.wall_f && sensor.wall_l) {
				/*右旋回*/
				DriveRotateR90();
				/*位置合わせ*/
				DriveSetPosition();
				/*右旋回*/
				DriveRotateR90();
				/*位置合わせ*/
				DriveSetPosition();
			}
			else if (sensor.wall_f && sensor.wall_r) {
				/*左旋回*/
				DriveRotateL90();
				/*位置合わせ*/
				DriveSetPosition();
				/*左旋回*/
				DriveRotateL90();
				/*位置合わせ*/
				DriveSetPosition();
			}
			else {
				DriveRotateR180();
				/*180度旋回*/
			}
			/*半区画加速走行をここに書く*/
			DriveHalfSectionAccel();
			mouse.dir = (mouse.dir + 2) % 4; //180度回転
			break;
		default:
			break;
		}
		AdvancePosition(); //マウスの座標を進行方向に1区画進める
		GetSectionInfo(); //進んだ先の壁情報を取得
		MazeWriteWallMap(&maze, mouse.x, mouse.y, mouse.section); //進んだ先の壁情報を迷路マップに書き込む
	}
	////====ゴール到達====
	//ゴール到達時の処理
	DriveHalfSectionBreak();
	/*180度旋回をする*/
	DriveRotateR180();
	mouse.dir = (mouse.dir + 2) % 4; //180度回転
	if (!mouse.second) {
		//フラッシュメモリに迷路情報を書き込む
		MazeWriteFlsh(&maze);
	}
}

// Mode 4 uses a fixed saved-map route: no flash writes or map rebuilding
// between motions. Every action starts/ends at a cell entrance/exit.
void SearchSlalom(void) {
    MazeReadFlsh(&maze);
    if (maze.goal_x >= MAZE_SIZE || maze.goal_y >= MAZE_SIZE) {
        printf("Slalom: invalid saved goal\n");
        return;
    }
    MazeMakeStepMap(&maze, 0, 0, true);
    if (!maze.map[0][0].visited || maze.map[0][0].north ||
            !maze.map[maze.goal_x][maze.goal_y].visited ||
            !maze.map[0][1].visited || maze.step_map[0][0] == MAX_ROUTE - 1 ||
            maze.step_map[0][1] == MAX_ROUTE - 1) {
        printf("Slalom: no saved route; run exploration first\n");
        return;
    }
    MazeMakeRoute(&maze, 0, 1, KNorth);
    if (maze.r_cnt != maze.step_map[0][1]) {
        printf("Slalom: incomplete saved route\n");
        return;
    }
    DriveSlalomStop();
    DriveSetStartPosition();
    if (!DriveSlalomStraight(HALF_CELL_MM, SLALOM_SPEED_MM_S, true)) return;
    AdvancePosition();
    for (uint16_t i = 0; i < maze.r_cnt; ++i) {
        bool ok = false;
        switch (maze.route[i]) {
        case KForward:
            {
                uint16_t cells = 1;
                while (i + cells < maze.r_cnt && maze.route[i + cells] == KForward) cells++;
                ok = DriveSlalomStraight(HALF_CELL_MM, SECOND_RUN_STRAIGHT_SPEED_MM_S, true);
                float cruise = FULL_CELL_MM * (cells - 1U);
                if (ok && cruise > 0.0f)
                    ok = DriveSlalomStraight(cruise, SECOND_RUN_STRAIGHT_SPEED_MM_S, true);
                if (ok) ok = DriveSlalomStraight(HALF_CELL_MM, SLALOM_SPEED_MM_S, true);
                if (ok) {
                    for (uint16_t j = 1; j < cells; j++) AdvancePosition();
                    i += cells - 1U;
                }
            }
            break;
        case KTurnL90:
            ok = DriveSlalom90(false);
            if (ok) mouse.dir = (mouse.dir + 3) % 4;
            break;
        case KTurnR90:
            ok = DriveSlalom90(true);
            if (ok) mouse.dir = (mouse.dir + 1) % 4;
            break;
        case KTurn180:
            // Retain the calibrated stationary U-turn, at the cell centre.
            ok = DriveSlalomStraight(HALF_CELL_MM, 0.0f, true);
            if (ok) {
                DriveRotateR180();
                ok = DriveSlalomStraight(HALF_CELL_MM, SLALOM_SPEED_MM_S, true);
            }
            if (ok) mouse.dir = (mouse.dir + 2) % 4;
            break;
        default:
            break;
        }
        if (!ok) {
            DriveSlalomStop();
            printf("Slalom: stopped at route index %u\n", (unsigned)i);
            return;
        }
        AdvancePosition();
    }
    if (!DriveSlalomStraight(HALF_CELL_MM, 0.0f, true)) return;
    DriveRotateR180();
    mouse.dir = (mouse.dir + 2) % 4;
    printf("Slalom complete: (%d, %d)\n", mouse.x, mouse.y);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：SearchC
// 内容：ゴール到達後も短縮候補を探索し、最短距離確定後にスタートへ帰還する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void SearchC(void) {
	// モード2専用。SearchInit後、スタートから往路・追加探索・帰還を行う。
	//====スタート位置壁情報取得====
	GetSectionInfo(); //スタート位置の壁情報を取得
	mouse.section.north = false; //スタート位置の北側は壁がない
	MazeWriteWallMap(&maze, mouse.x, mouse.y, mouse.section); //スタート位置の壁情報を迷路マップに書き込む
	//====前に壁が無い想定で問答無用で前進====
	/*半区画前進関数*/
	DriveHalfSectionAccel(); //半区画加速走行
	AdvancePosition(); //マウスの座標を進行方向に1区画進める
	GetSectionInfo(); //進んだ先の壁情報を取得
	MazeWriteWallMap(&maze, mouse.x, mouse.y, mouse.section); //進んだ先の壁情報を迷路マップに書き込む
	//====探索走行====
    bool reached_goal = false;
    bool certified = false;
    bool already_known = false;
    while (true) {
        if (mouse.x == maze.goal_x && mouse.y == maze.goal_y) reached_goal = true;
        uint16_t known, possible;
        ExploreResult result = MazePlanExplore(&maze, mouse.x, mouse.y,
                mouse.dir, reached_goal, &known, &possible);
        if (reached_goal && known != UINT16_MAX && known == possible && !certified) {
            certified = true;
            printf("Shortest route certified: %u cells. Returning home.\n", (unsigned)known);
        }
        if (result == KExploreDone) break;
        if (result == KExploreError) {
            DriveSetSpeed(0.0f);
            DriveSetAccel(0.0f);
            MotorStop();
            printf("SearchC: no route at (%d, %d)\n", mouse.x, mouse.y);
            return;
        }
		//----進行----
		switch (maze.route[0]) {
			//----前進----
		case KForward:
            {
                unsigned cells = KnownStraightCells(reached_goal, already_known);
                if (cells) {
                    DriveKnownStraight(cells, EXPLORE_KNOWN_STRAIGHT_SPEED_MM_S);
                    for (unsigned i = 1; i < cells; i++) AdvancePosition();
                } else DriveOneSection();
            } //半区画前進関数
			break;
			//----左90度回転----
		case KTurnL90:
			DriveTurnL90(); //左90度回転関数
			mouse.dir = (mouse.dir + 3) % 4; //左90度回転
			break;
			//----右90度回転----
		case KTurnR90:
			DriveTurnR90(); //右90度回転関数
			mouse.dir = (mouse.dir + 1) % 4; //右90度回転
			break;
			//----180度回転----
		case KTurn180:
			GetWallSensorInfo(); //センサーの壁情報を取得
			/*半区画減速走行をここに書く*/
			DriveHalfSectionBreak();
			if (sensor.wall_f && sensor.wall_l) {
				/*右旋回*/
				DriveRotateR90();
				/*位置合わせ*/
				DriveSetPosition();
				/*右旋回*/
				DriveRotateR90();
				/*位置合わせ*/
				DriveSetPosition();
			}
			else if (sensor.wall_f && sensor.wall_r) {
				/*左旋回*/
				DriveRotateL90();
				/*位置合わせ*/
				DriveSetPosition();
				/*左旋回*/
				DriveRotateL90();
				/*位置合わせ*/
				DriveSetPosition();
			}
			else {
				DriveRotateR180();
				/*180度旋回*/
			}
			/*半区画加速走行をここに書く*/
			DriveHalfSectionAccel();
			mouse.dir = (mouse.dir + 2) % 4; //180度回転
			break;
		default:
			break;
		}
		AdvancePosition(); //マウスの座標を進行方向に1区画進める
        already_known = maze.map[mouse.x][mouse.y].visited;
		GetSectionInfo(); //進んだ先の壁情報を取得
		MazeWriteWallMap(&maze, mouse.x, mouse.y, mouse.section); //進んだ先の壁情報を迷路マップに書き込む
	}
    // Finish the last half-cell into the start cell, then turn around.
    DriveHalfSectionBreak();
    DriveRotateR180();
    mouse.dir = (mouse.dir + 2) % 4;
    MazeWriteFlsh(&maze);
    printf("SearchC complete: shortest route confirmed, returned to start.\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：GetSectionInfo
// 内容：壁センサーから相対的な壁情報を取得し、現在のマウスの向きに応じた絶対方角の壁情報に変換・更新する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void GetSectionInfo(void) {
	//センサーの相対的な壁情報を取得
	GetWallSensorInfo(); //センサーの壁情報を取得
	//絶対的な方角の壁情報に変換する
	switch (mouse.dir) {
	case KNorth:
		mouse.section.north = sensor.wall_f;
		mouse.section.east = sensor.wall_r;
		mouse.section.south = false; //後ろの壁はセンサーで検知できないため、falseに設定
		mouse.section.west = sensor.wall_l;
		break;
	case KEast:
		mouse.section.north = sensor.wall_l;
		mouse.section.east = sensor.wall_f;
		mouse.section.south = sensor.wall_r;
		mouse.section.west = false; //後ろの壁はセンサーで検知できないため、falseに設定
		break;
	case KSouth:
		mouse.section.north = false; //後ろの壁はセンサーで検知できないため、falseに設定
		mouse.section.east = sensor.wall_l;
		mouse.section.south = sensor.wall_f;
		mouse.section.west = sensor.wall_r;
		break;
	case KWest:
		mouse.section.north = sensor.wall_r;
		mouse.section.east = false; //後ろの壁はセンサーで検知できないため、falseに設定
		mouse.section.south = sensor.wall_l;
		mouse.section.west = sensor.wall_f;
		break;
	default:
		break;
	}
	mouse.section.visited = true; //訪問済みに設定
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：AdvancePosition
// 内容：現在のマウスの向きに基づき、自己座標（x, y）を進行方向に1区画更新する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void AdvancePosition(void) {
	//マウスの座標を進行方向に1区画進める
	switch (mouse.dir) {
	case KNorth:
		mouse.y += 1;
		break;
	case KEast:
		mouse.x += 1;
		break;
	case KSouth:
		mouse.y -= 1;
		break;
	case KWest:
		mouse.x -= 1;
		break;
	default:
		break;
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：ConfirmRoute
// 内容：一次走行時において、次の予定動作の進行方向に壁が存在するか判定し、壁で遮られている場合は歩数マップと経路の再計算を行う
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void ConfirmRoute(void) {
	if (!mouse.second) {
		//進行方向に壁がある場合は、経路を再計算する
		bool is_blocked = false;
		switch (maze.route[mouse.r_cnt]) {
		case KForward:
			if (sensor.wall_f) is_blocked = true;
			break;
		case KTurnL90:
			if (sensor.wall_l) is_blocked = true;
			break;
		case KTurnR90:
			if (sensor.wall_r) is_blocked = true;
			break;
		case KTurn180:
			//180度ターンの場合は壁がない
			break;
		default:
			//不正な値の場合は壁があるとみなす
			is_blocked = true;
			break;
		}
		if (is_blocked) {
			//経路を再計算する
			MazeMakeStepMap(&maze, mouse.x, mouse.y, mouse.second); //迷路情報から歩数マップを作成
			MazeMakeRoute(&maze, mouse.x, mouse.y, mouse.dir); //歩数マップから経路を作成
			mouse.r_cnt = 0; //経路カウンタを初期化
		}
	}
}
//以下デバッグ用の関数群
void TestSearch(void) {
	int mode = 0;
	while (1) 
	{
		mode = select_mode(mode);
		switch (mode) {
			case 0:
				printf("Mode 0 selected\n");
				//迷路データの初期化
				MazeInit(&maze);
				break;
			case 1:
				printf("Mode 1 selected\n");
				//探索データの初期化
				SearchInit();
				break;
			case 2:
				printf("Mode 2 selected\n");
				//フラッシュから迷路データを読み込む
				MazeReadFlsh(&maze);
				break;
			case 3:
				printf("Mode 3 selected\n");
				//迷路データの表示
				OutputDebugMap(&maze);
				break;
			case 4:
				printf("Mode 4 selected\n");
				//経路データの表示
				OutputDebugRoute(&maze);
				break;
			case 5:
				printf("Mode 5 selected\n");
				//左手法でルートを作成
				MazeMakeRouteLeftHand(&maze, mouse.x, mouse.y, mouse.dir);
				break;
			case 6:
				printf("Mode 6 selected\n");
				//幅優先探索でルートを作成
				MazeMakeStepMap(&maze, mouse.x, mouse.y, mouse.second);
				MazeMakeRoute(&maze, mouse.x, mouse.y, mouse.dir);
				OutputDebugStepMap(&maze);
				OutputDebugRoute(&maze);
				break;
			case 7:
				printf("Mode 7 selected: Wall detour test\n");
				SetupWallDetourTest();
				break;
		}
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// SetupWallDetourTest
// (0,3) の北に壁を置き、経路が壁を避けることを確認する
//+++++++++++++++++++++++++++++++++++++++++++++++
void SetupWallDetourTest(void) {
	Section wall = { 0 };

	SearchInit();
	wall.north = true;
	wall.visited = true;
	MazeWriteWallMap(&maze, 0, 3, wall);

	OutputDebugMap(&maze);
	MazeMakeStepMap(&maze, mouse.x, mouse.y, mouse.second);
	MazeMakeRoute(&maze, mouse.x, mouse.y, mouse.dir);
	OutputDebugStepMap(&maze);
	OutputDebugRoute(&maze);
}
//+++++++++++++++++++++++++++++++++++++++++++++++
// OutputDebugMap
// 現在の迷路マップ情報とマウス・ゴールの位置をコンソールへ出力する
// 引数：MazeData* p_maze - 迷路データへのポインタ
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void OutputDebugMap(MazeData* p_maze) {
	printf("map:\n");

	for (int y = MAZE_SIZE - 1; y >= 0; y--) {
		//----[1] セル上部（北壁）----
		for (int x = 0; x < MAZE_SIZE; x++) {
			printf(p_maze->map[x][y].north ? "+---+" : "+   +");
		}
		printf("\n");

		//----[2] セル中央（西壁 / マウス・ゴール / 東壁）----
		for (int x = 0; x < MAZE_SIZE; x++) {
			// 左右の壁文字を決定
			char west = p_maze->map[x][y].west ? '|' : ' ';
			char east = p_maze->map[x][y].east ? '|' : ' ';

			// セル内部の文字列を決定（デフォルトは空白）
			const char* body = "   ";
			if (x == mouse.x && y == mouse.y) {
				body = "\x1b[42mMSE\x1b[49m"; // マウス位置（緑）
			}
			else if (x == p_maze->goal_x && y == p_maze->goal_y) {
				body = "\x1b[41mGOL\x1b[49m"; // ゴール位置（赤）
			}

			printf("%c%s%c", west, body, east);
		}
		printf("\n");

		//----[3] セル下部（南壁）----
		for (int x = 0; x < MAZE_SIZE; x++) {
			printf(p_maze->map[x][y].south ? "+---+" : "+   +");
		}
		printf("\n");
	}
}
//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：OutputDebugStepMap
// 内容：歩数マップをコンソールへ出力する
// 引数：MazeData* p_maze - 迷路データへのポインタ
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void OutputDebugStepMap(MazeData* p_maze) {
	printf("Step Map (goal: %d, %d):\n", p_maze->goal_x, p_maze->goal_y);
	for (int y = MAZE_SIZE - 1; y >= 0; y--) {
		for (int x = 0; x < MAZE_SIZE; x++) {
			if (p_maze->step_map[x][y] == MAX_ROUTE - 1) {
				printf(" --");
			} else {
				printf(" %02d", p_maze->step_map[x][y]);
			}
		}
		printf("\n");
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// 関数名：OutputDebugRoute
// 内容：現在の迷路経路データをコンソールへ出力する
// 引数：MazeData* p_maze - 迷路データへのポインタ
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void OutputDebugRoute(MazeData* p_maze) {
	printf("Route Data (count: %d):\n", p_maze->r_cnt);

	// maze.r_cnt に経路の総手数が格納されている前提で出力します
	// ※必要に応じてループ回数は MAX_ROUTE に変更してください
	for (int i = 0; i < p_maze->r_cnt; i++) {
		switch (p_maze->route[i]) {
		case KForward:
			printf("F "); // 直進
			break;
		case KTurnL90:
			printf("L "); // 左90度ターン
			break;
		case KTurnR90:
			printf("R "); // 右90度ターン
			break;
		case KTurn180:
			printf("U "); // Uターン(180度ターン)
			break;
		default:
			printf("- ");
			break;
		}
	}
	printf("\n");
}
